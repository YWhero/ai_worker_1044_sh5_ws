#!/usr/bin/env python3
"""Persist a measured ROS snapshot as the Isaac-only SH5 startup pose.

The snapshot comes from /joint_states and odom -> base_link. The bridge's
documented odometry origin is the configured world spawn, not the world origin.
Official URDF limits are checked before any persistent file is changed.
Snapshots carry their original odometry_origin_spawn; legacy snapshots require
an explicit --metadata file from the time of capture.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import stat
import sys
import tempfile

import yaml

ROOT = Path(__file__).resolve().parents[2]
CELL = Path('/home/robotis-ai/workspaces/isaac_logistics_cell_ws')
sys.path.insert(0, str(ROOT/'src/hx5_isaac'))
from scene_builder import qmul, read_robot_description, POSITION_JOINT_NAMES, validate_initial_positions


def snapshot_to_pose(snapshot, metadata, description):
    if snapshot.get('odom_frame') != 'odom' or snapshot.get('base_frame') != 'base_link':
        raise ValueError('Expected the Isaac odom -> base_link measured pose')
    stamps = [snapshot['joint_stamp'], snapshot['odom_stamp']]
    if not all(math.isfinite(v) and v >= 0 for v in stamps) or abs(stamps[0]-stamps[1]) > .1:
        raise ValueError('Measured joint and base samples must be within 0.1 simulation seconds')
    measured = snapshot['joints']
    if not set(POSITION_JOINT_NAMES).issubset(measured):
        raise ValueError('Snapshot must contain all 57 arm/hand/head/lift positions')
    positions = validate_initial_positions(description, {n: measured[n] for n in POSITION_JOINT_NAMES})
    local = snapshot['base_position']
    xyzw = snapshot['base_orientation_xyzw']
    if 'odometry_origin_spawn' in snapshot:
        spawn = snapshot['odometry_origin_spawn']
    elif metadata is not None and 'spawn' in metadata:
        # Legacy snapshots need the scene metadata from the time of capture.
        spawn = metadata['spawn']
    else:
        raise ValueError('Legacy snapshots require explicit original --metadata or embedded odometry_origin_spawn')
    if len(local) != 3 or len(xyzw) != 4 or len(spawn) != 4 or not all(
            math.isfinite(v) for v in [*local, *xyzw, *spawn]):
        raise ValueError('Expected finite measured XYZ/quaternion and world spawn XYZ/yaw')
    norm = math.sqrt(sum(v*v for v in xyzw))
    if abs(norm-1.) > .001:
        raise ValueError('Measured quaternion must be normalized')
    yaw = math.radians(spawn[3]); c, s = math.cos(yaw), math.sin(yaw)
    world_position = [spawn[0]+c*local[0]-s*local[1],
                      spawn[1]+s*local[0]+c*local[1], spawn[2]+local[2]]
    local_q = tuple(v/norm for v in [xyzw[3], *xyzw[:3]])
    world_q = list(qmul((math.cos(yaw/2),0.,0.,math.sin(yaw/2)),local_q))
    w,x,y,z = world_q
    world_yaw = math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
    return {'metadata': {'source': 'Isaac measured /joint_states and /odom snapshot',
                         'ros_domain_id': 115, 'joint_stamp': stamps[0], 'odom_stamp': stamps[1],
                         'odometry_origin_spawn': spawn, 'world_position_m': world_position,
                         'world_orientation_wxyz': world_q, 'world_yaw_degrees': world_yaw,
                         'description': 'User-requested current pose; Isaac only. Hardware URDF limits checked.'},
            'initial_positions': positions}


def write_pose_and_layout(output, pose_content, layout_path, layout_content):
    """Stage both files before committing; restore the pose if layout commit fails."""
    if output.resolve() == layout_path.resolve():
        raise ValueError('Initial pose and layout must be separate files')
    temporary_paths = []

    def stage(path, content):
        fd, filename = tempfile.mkstemp(prefix=path.name+'.', suffix='.new', dir=path.parent)
        os.close(fd)
        temporary = Path(filename)
        temporary_paths.append(temporary)
        if isinstance(content, bytes):
            temporary.write_bytes(content)
        else:
            temporary.write_text(content)
        if path.exists():
            temporary.chmod(stat.S_IMODE(path.stat().st_mode))
        return temporary

    try:
        pose_staged = stage(output, pose_content)
        layout_staged = stage(layout_path, layout_content)
        # Prepare rollback before replacing either file, so recovery does not
        # depend on being able to write another temporary file after failure.
        rollback = stage(output, output.read_bytes()) if output.exists() else None
        pose_staged.replace(output)
        try:
            layout_staged.replace(layout_path)
        except BaseException:
            if rollback is None:
                output.unlink(missing_ok=True)
            else:
                rollback.replace(output)
            raise
    finally:
        for temporary in temporary_paths:
            temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--metadata', type=Path,
                        help='Original scene metadata; required only for legacy snapshots without embedded origin')
    parser.add_argument('--layout', type=Path, default=CELL/'config/layout.json')
    parser.add_argument('--urdf', type=Path, default=ROOT/'simulation/isaac/assets/sh5/robot.urdf')
    parser.add_argument('--output', type=Path, default=ROOT/'simulation/isaac/config/initial_pose.yaml')
    args = parser.parse_args()
    snapshot = json.loads(args.snapshot.read_text())
    if 'odometry_origin_spawn' not in snapshot and args.metadata is None:
        parser.error('Legacy snapshots require explicit original --metadata; the current scene may have a different origin')
    metadata = (json.loads(args.metadata.read_text())
                if 'odometry_origin_spawn' not in snapshot else None)
    pose = snapshot_to_pose(snapshot, metadata, read_robot_description(args.urdf))
    layout = json.loads(args.layout.read_text())
    data = pose['metadata']
    layout['robot'].update(position_m=data['world_position_m'], yaw_degrees=data['world_yaw_degrees'],
        orientation_wxyz=data['world_orientation_wxyz'], initial_pose_source=str(args.output.resolve()),
        initial_pose_provenance=data['description'])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_pose_and_layout(args.output, yaml.safe_dump(pose,sort_keys=False,allow_unicode=True),
                          args.layout, json.dumps(layout,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({'initial_pose':str(args.output), 'world_position':data['world_position_m'],
                      'world_yaw_degrees':data['world_yaw_degrees'], 'joints':len(pose['initial_positions'])}))


if __name__ == '__main__':
    main()
