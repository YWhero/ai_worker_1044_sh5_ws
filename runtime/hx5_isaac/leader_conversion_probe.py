"""Read one passed synthetic Isaac episode through the official Cyclo converter.

Creates a new diagnostic JSON and scratch NPZ; official segment discovery may
also copy videos into that scratch directory. --full-v30 additionally writes
and validates a complete local dataset. It never commands a robot, records,
uploads, trains, or edits source bags.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import sys


def source_hashes(source):
    return {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in source.rglob('*') if p.is_file()}


def convert_v30(source, scratch, names):
    """Use the existing writer and audit its actual final artifacts."""
    import cv2
    import numpy as np
    import pyarrow as pa
    import pyarrow.parquet as pq
    from cyclo_data.converter.to_lerobot_v30 import V30ConversionConfig, RosbagToLerobotV30Converter
    original = source_hashes(source)
    dataset = scratch / 'lerobot_v30'
    assert not dataset.exists(), 'v3 output already exists'
    for variable in ('CYCLO_V30_DISABLE_SOURCE_AGGREGATE_CACHE', 'CYCLO_V30_DISABLE_DATA_AGGREGATE_CACHE'):
        os.environ[variable] = '1'
    os.environ['CYCLO_V30_POPULATE_SOURCE_AGGREGATE_CACHE'] = '0'
    os.environ['CYCLO_VIDEO_SYNC_STAGING_DIR'] = str(scratch / 'video-sync')
    os.environ.pop('CYCLO_VIDEO_SYNC_CLEAN_CACHE', None)
    os.environ['CYCLO_H264_ENCODER'] = 'libx264'
    os.environ['CYCLO_VIDEO_AGG_CAMERA_WORKERS'] = '1'
    config = V30ConversionConfig(repo_id='local/IsaacSkeletonContract20260918', output_dir=dataset,
                                 robot_type='ffw_sh5_rev1', fps=30, tactile_mode='separate_raw',
                                 use_videos=True, apply_trim=False, apply_exclude_regions=False,
                                 source_rosbags=[str(source)], enable_quality_report=True)
    try:
        converter = RosbagToLerobotV30Converter(config)
        assert converter.convert_multiple_rosbags([source]), 'official v3 converter returned false'
        info = json.loads((dataset / 'meta/info.json').read_text())
        assert info['codebase_version'] == 'v3.0' and info['total_episodes'] == 1
        table = pa.concat_tables([pq.ParquetFile(p).read() for p in sorted((dataset / 'data').rglob('*.parquet'))])
        length = table.num_rows
        assert length > 1 and info['total_frames'] == length
        summaries = {}
        for key in ('observation.state', 'action'):
            feature = info['features'][key]
            assert feature['shape'] == [54] and len(feature['names']) == 54 and set(feature['names']) == set(names)
            assert not any('gripper' in name for name in feature['names']), 'LG2 raw gripper leaked into v3 feature'
            values = np.asarray(table.column(key).to_pylist(), dtype=np.float32)
            assert values.shape == (length, 54) and np.isfinite(values).all() and np.any(np.abs(values) > .001)
            assert np.max(np.ptp(values, axis=0)) > .008, f'written {key} is constant'
            summaries[key] = {'shape': list(values.shape), 'varying_columns': int(np.count_nonzero(np.ptp(values, axis=0) > .001))}
        for side in ('left', 'right'):
            for prefix in ('observation.tactile', 'observation.tactile_baseline'):
                key = f'{prefix}.{side}'; feature = info['features'][key]
                assert feature['shape'] == [45] and len(feature['names']) == 45
                values = np.asarray(table.column(key).to_pylist(), dtype=np.float32)
                assert values.shape == (length, 45) and np.isfinite(values).all()
                summaries[key] = {'shape': list(values.shape), 'maximum': float(values.max())}
        stamps = np.asarray(table.column('timestamp').to_pylist(), dtype=float)
        assert np.isfinite(stamps).all() and np.all(np.diff(stamps) > 0)
        episodes = pa.concat_tables([pq.ParquetFile(p).read() for p in (dataset / 'meta/episodes').rglob('*.parquet')]).to_pylist()
        assert len(episodes) == 1 and episodes[0]['length'] == length
        videos = {}
        files = list((dataset / 'videos').rglob('*.mp4'))
        assert len(files) == 4, 'v3 export must have four final MP4 streams'
        for path in files:
            key = path.parent.parent.name
            feature = info['features'][key]
            expected = (376, 672, 3) if 'head' in key else (424, 240, 3)
            assert feature['shape'] == [3, expected[0], expected[1]], 'v3 camera metadata shape is incorrect'
            capture, count, shapes = cv2.VideoCapture(str(path)), 0, set()
            while True:
                ok, frame = capture.read()
                if not ok: break
                count += 1; shapes.add(tuple(frame.shape))
            capture.release()
            assert count == length and shapes == {expected}, 'v3 MP4 frame count/dimensions differ from data rows'
            assert episodes[0][f'videos/{key}/to_timestamp'] > episodes[0][f'videos/{key}/from_timestamp']
            videos[key] = {'path': str(path), 'decoded_frames': count, 'frame_shape': expected}
        return {'result': 'passed', 'dataset_dir': str(dataset), 'rows': length, 'features': summaries,
                'videos': videos, 'codebase_version': info['codebase_version'], 'source_hashes_unchanged': True,
                'quality_report_enabled': config.enable_quality_report}
    finally:
        assert source_hashes(source) == original, 'full conversion changed recorded source files'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-id', default='', help='Unique suffix for an additional probe; existing artifacts are preserved')
    parser.add_argument('--full-v30', action='store_true', help='Also export and validate one official LeRobot v3.0 scratch dataset')
    args = parser.parse_args()
    assert re.fullmatch(r'[A-Za-z0-9_-]*', args.run_id), 'invalid run-id'
    suffix = '-' + args.run_id if args.run_id else ''
    smoke_path = Path('/workspace/diagnostics/leader-recording-smoke.json')
    smoke = json.loads(smoke_path.read_text())
    assert smoke.get('result') == 'passed', 'run only after leader recording smoke has passed'
    root = Path('/workspace/rosbag2/Task_20260918_IsaacSkeletonContract20260918_MCAP')
    source = Path(smoke['episode_dir'])
    assert source.resolve().is_relative_to(root.resolve()), 'unexpected source episode'
    output = Path('/workspace/diagnostics') / f'leader-conversion-probe{suffix}.json'
    scratch = Path('/workspace/conversion-probes') / f'IsaacSkeletonContract20260918{suffix}'
    assert not output.exists() and not scratch.exists(), 'probe outputs already exist; refusing overwrite'
    assert (source / 'episode_info.json').is_file(), 'archived source metadata missing'
    # Official extraction normally writes a source-local pickle cache. Disable
    # it so this diagnostic leaves the recorded episode unchanged.
    os.environ['CYCLO_EXTRACT_CACHE_DISABLE'] = '1'
    os.environ['CYCLO_PREPARED_EPISODE_CACHE_DISABLE'] = '1'
    import numpy as np
    from cyclo_data.converter.base_converter import ConversionConfig, RosbagToLerobotConverterBase
    from cyclo_data.reader.frame_timestamps import load_frame_timestamps, build_frame_reuse_report
    sys.path.insert(0, '/hx5_isaac_runtime')
    from validate_integration import ARMS, HANDS

    report = {'result': 'failed', 'source_episode': str(source), 'scratch_output': str(scratch),
              'scope': ('Actual official LeRobot v3.0 local export, data/video metadata validation, and SIM timestamp mapping.'
                        if args.full_v30 else 'Actual official converter extraction and camera SIM timestamp mapping.') +
                       ' No upload, training, inference, physical leader, or grasp validation.'}
    try:
        config = ConversionConfig(repo_id='local/IsaacSkeletonContract20260918', output_dir=scratch,
                                  robot_type='ffw_sh5_rev1', fps=30, tactile_mode='separate_raw',
                                  use_videos=True, apply_trim=False, apply_exclude_regions=False)
        converter = RosbagToLerobotConverterBase(config)
        episode = converter._extract_joint_data(source, episode_index=0, trim_points=None, exclude_regions=[])
        assert episode is not None and episode.length > 1, 'official extraction yielded no usable rows'
        length = episode.length
        names = ARMS + HANDS
        for field in ('observation_state_names', 'action_names'):
            actual = getattr(episode, field)
            assert len(actual) == len(set(actual)) == 54 and set(actual) == set(names), f'{field} differs from SH5 54-joint policy schema'
            assert not any('gripper' in name for name in actual), 'LG2 gripper input leaked into HX5 policy dimension'
        state = np.asarray(episode.observation_state, dtype=np.float32)
        action = np.asarray(episode.action, dtype=np.float32)
        vectors = {'state': state, 'action': action}
        metrics = {}
        for kind, values in vectors.items():
            assert values.shape == (length, 54) and np.isfinite(values).all(), f'invalid extracted {kind} rows'
            assert np.any(np.abs(values) > .001) and np.max(np.ptp(values, axis=0)) > .008, f'{kind} is zero/constant'
            layout = episode.observation_state_names if kind == 'state' else episode.action_names
            ranges = np.ptp(values, axis=0)
            for side in ('l', 'r'):
                assert ranges[layout.index(f'arm_{side}_joint7')] > .008, f'{kind} arm {side} lost commanded variation'
                finger_columns = [layout.index(f'finger_{side}_joint{i}') for i in range(1, 21)]
                assert np.max(ranges[finger_columns]) > .005, f'{kind} hand {side} lost variation'
            metrics[kind] = {'shape': list(values.shape), 'varying_columns': int(np.count_nonzero(ranges > .001)),
                             'max_range_rad': float(np.max(ranges)), 'joint_names': layout}
        relative = np.asarray(episode.timestamps, dtype=np.float64)
        grid = np.rint(np.asarray(episode.grid_log_times_sec, dtype=np.float64) * 1e9).astype(np.int64)
        assert relative.shape == grid.shape == (length,) and np.isfinite(relative).all()
        assert np.all(np.diff(grid) > 0) and np.all(np.diff(relative) > 0), 'sample grid is not advancing'
        assert 0 < int(grid[0]) < int(grid[-1]) < 1000000000000000, 'sample grid appears to use wall time'
        assert set(episode.tactile) == set(episode.tactile_baseline) == {'left', 'right'}, 'both raw tactile sides required'
        arrays = {'state': state, 'action': action, 'relative_time_sec': relative, 'grid_sim_time_ns': grid}
        tactile_metrics = {}
        for side in ('left', 'right'):
            raw = np.asarray(episode.tactile[side], dtype=np.float32)
            baseline = np.asarray(episode.tactile_baseline[side], dtype=np.float32)
            assert raw.shape == baseline.shape == (length, 5, 3, 3), f'{side} tactile shape invalid'
            assert np.isfinite(raw).all() and np.isfinite(baseline).all(), f'{side} tactile is nonfinite'
            assert np.all((raw >= 0) & (raw <= 255)) and np.all(raw == np.rint(raw)), 'raw pressure bytes changed'
            assert np.all(baseline == baseline[0]), 'episode-local tactile baseline must remain fixed'
            arrays[f'tactile_{side}'], arrays[f'tactile_baseline_{side}'] = raw, baseline
            tactile_metrics[side] = {'shape': list(raw.shape), 'minimum': float(raw.min()), 'maximum': float(raw.max())}
        videos = converter._find_video_files(source)
        assert len(videos) == 4 and set(videos) == set(converter._camera_mapping.values()), 'official converter did not discover all four cameras'
        cameras = {}
        for camera, path in sorted(videos.items()):
            timestamps = load_frame_timestamps(path.with_name(path.stem + '_timestamps.parquet'), camera)
            assert timestamps.num_frames > 1 and timestamps.effective_time_source() == 'header', 'camera must have advancing SIM header timestamps'
            assert np.all(np.diff(timestamps.header_stamp_ns) >= 0), 'camera clock regressed'
            indices = timestamps.map_to_grid(grid, time_source='header')
            assert indices.shape == (length,) and np.all((indices >= 0) & (indices < timestamps.num_frames)), 'camera map index out of bounds'
            assert len(np.unique(indices)) > 1, 'camera mapping collapsed to one frame; check wall/SIM mismatch'
            mapped = timestamps.header_stamp_ns[indices]
            causal = grid >= timestamps.header_stamp_ns.min()
            assert np.any(causal) and np.all(mapped[causal] <= grid[causal]), 'camera mapping selected future frames'
            assert np.all(np.diff(indices) >= 0), 'camera frame mapping regressed'
            reuse = build_frame_reuse_report(indices, grid, timestamps, episode_index=0, camera=camera, fps=config.fps)
            arrays[f'camera_indices_{camera}'] = indices
            cameras[camera] = {'source_video': str(path), 'source_frames': timestamps.num_frames,
                               'target_rows': length, 'distinct_source_frames_used': int(len(np.unique(indices))),
                               'grid_range_ns': [int(grid[0]), int(grid[-1])],
                               'camera_header_range_ns': [int(timestamps.header_stamp_ns.min()), int(timestamps.header_stamp_ns.max())],
                               'max_causal_age_ms': float(np.max((grid[causal]-mapped[causal])/1e6)), 'frame_reuse': reuse}
        # Official segment discovery may already create this new output root.
        scratch.mkdir(parents=True, exist_ok=True)
        with (scratch / 'extracted_rows.npz').open('xb') as stream: np.savez_compressed(stream, **arrays)
        report.update(result='passed', extracted_rows=length, vectors=metrics, tactile=tactile_metrics,
                      cameras=cameras, grid_sim_time_ns=[int(grid[0]), int(grid[-1])],
                      staleness={k: v.to_dict() for k, v in converter._staleness_reports[0].items()},
                      source_cache_disabled=True, extracted_rows_npz=str(scratch / 'extracted_rows.npz'))
        if args.full_v30:
            report['full_v30'] = convert_v30(source, scratch, names)
    except Exception as error:
        report.update(result='failed', error=repr(error))
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as stream: json.dump(report, stream, indent=2); stream.write('\n')
    print(json.dumps(report, indent=2))
    return 0 if report['result'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
