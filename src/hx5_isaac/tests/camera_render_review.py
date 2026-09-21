#!/usr/bin/env python3
"""Opt-in RTX before/after review. Run with Isaac python.sh while GUI is stopped.

No ROS server or external robot commands are started. Stage edits are confined
to a temporary anonymous session layer; no scene, metadata, or source CAD is
saved. Baseline/proxy-purpose/aperture frames and a receipt are saved to --output.
"""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from camera_geometry import _rotate, clear_head_optical_apertures, head_visual_shells
from simulator import PhysicsRuntime


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', type=Path, required=True)
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path('/tmp/hx5-camera-render-review'))
    args, kit_args = parser.parse_known_args()
    # The wrapper appends isolated portable-root/Kit settings to argv. Leave
    # them visible to SimulationApp, while validating the custom script args.
    from scene_builder import validate_kit_args
    validate_kit_args(kit_args)
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'enable_cameras': True,
                         'fast_shutdown': True, 'shutdown_watchdog_timeout': 30.})
    receipt = {}; runtime = None
    try:
        import numpy as np
        from PIL import Image
        from pxr import Sdf, Usd, UsdGeom, UsdPhysics
        from isaacsim.sensors.camera import Camera
        args.output.mkdir(parents=True, exist_ok=True)
        metadata = json.loads(args.metadata.read_text())
        runtime = PhysicsRuntime(app, args.stage, metadata, cameras_enabled=True)
        # PhysX must publish the tensor-initialized joints to Fabric before
        # rendering; otherwise attached cameras see the zero-joint reset pose.
        for index in range(12):
            runtime.step(render=index % 4 == 0)
        cameras = metadata['cameras']
        before_colliders = {str(p.GetPath()): p.GetAttribute('physics:approximation').Get()
                            if p.GetAttribute('physics:approximation') else None
                            for p in runtime.stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)}
        before_mounts = {name: runtime.stage.GetPrimAtPath(spec['prim_path']).GetAttribute('xformOp:translate:hx5Initial').Get()
                         for name, spec in cameras.items()}
        original_source = args.stage.read_bytes()
        # All appearance experiments are authored in the session layer only.
        runtime.stage.SetEditTarget(runtime.stage.GetSessionLayer())
        overview = Camera(prim_path='/World/Cameras/Overview', name='review_overview', resolution=(640, 360), frequency=15)
        overview.initialize()
        all_cameras = {name: camera for name, (camera, _) in runtime.cameras.items()}
        all_cameras['overview'] = overview

        def capture(label):
            for _ in range(30):
                runtime.world.render()
            result = {}
            for name, camera in all_cameras.items():
                pixels = np.asarray(camera.get_rgb())
                if pixels.ndim != 3 or pixels.shape[2] < 3 or not pixels.size:
                    raise AssertionError(f'No actual RTX RGB frame for {name}/{label}')
                if pixels.dtype != np.uint8:
                    pixels = np.clip(pixels*255 if pixels.max() <= 1 else pixels, 0, 255).astype(np.uint8)
                pixels = pixels[:, :, :3]
                Image.fromarray(pixels).save(args.output/f'{label}-{name}.png')
                result[name] = pixels.copy()
            return result

        baseline = capture('baseline')
        positions = runtime.robot.get_joint_positions()
        receipt['measured_initial_head_positions'] = {name: float(positions[runtime.indices[name]])
                                                     for name in ('head_joint1', 'head_joint2')}
        receipt['head_camera_world_optical_axes'] = {}
        for name in ('head_left', 'head_right'):
            position, orientation = all_cameras[name].get_world_pose(camera_axes='ros')
            receipt['head_camera_world_optical_axes'][name] = {
                'position': position.tolist(), 'orientation_wxyz': orientation.tolist(),
                'forward': list(_rotate(orientation, (0, 0, 1)))}
        shells = head_visual_shells(runtime.stage, cameras)
        purposes = [UsdGeom.Imageable(p).GetPurposeAttr().Get() for p in shells]
        for shell in shells:
            UsdGeom.Imageable(shell).CreatePurposeAttr('proxy')
        for name in ('head_left', 'head_right'):
            product = runtime.stage.GetPrimAtPath(all_cameras[name].get_render_product_path())
            product.CreateAttribute('includedPurposes', Sdf.ValueTypeNames.TokenArray).Set(['default', 'render'])
        proxy = capture('proxy-purpose')
        proxy_differences = {name: float(np.mean(np.abs(proxy[name].astype(float)-baseline[name].astype(float))))
                             for name in all_cameras}
        receipt['proxy_purpose_mean_pixel_differences'] = proxy_differences
        # Do not deploy an authored but renderer-ignored purpose filter.
        receipt['proxy_purpose_has_camera_specific_effect'] = (
            min(proxy_differences[name] for name in ('head_left', 'head_right')) > 5
            and proxy_differences['overview'] < 2)
        for shell, purpose in zip(shells, purposes):
            UsdGeom.Imageable(shell).CreatePurposeAttr(purpose or 'default')
        for name in ('head_left', 'head_right'):
            product = runtime.stage.GetPrimAtPath(all_cameras[name].get_render_product_path())
            product.RemoveProperty('includedPurposes')
        receipt['visual_aperture_meshes'] = clear_head_optical_apertures(runtime.stage, cameras)
        assert receipt['visual_aperture_meshes'], 'No visual-only camera clearance authored'
        aperture = capture('aperture')
        receipt['aperture_mean_pixel_differences'] = {
            name: float(np.mean(np.abs(aperture[name].astype(float)-baseline[name].astype(float))))
            for name in all_cameras}
        after_colliders = {str(p.GetPath()): p.GetAttribute('physics:approximation').Get()
                           if p.GetAttribute('physics:approximation') else None
                           for p in runtime.stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)}
        after_mounts = {name: runtime.stage.GetPrimAtPath(spec['prim_path']).GetAttribute('xformOp:translate:hx5Initial').Get()
                        for name, spec in cameras.items()}
        receipt['colliders_unchanged'] = before_colliders == after_colliders
        receipt['camera_local_translations_unchanged'] = before_mounts == after_mounts
        receipt['stage_file_unchanged'] = original_source == args.stage.read_bytes()
        receipt['output'] = str(args.output)
        assert receipt['colliders_unchanged'] and receipt['camera_local_translations_unchanged'] and receipt['stage_file_unchanged']
        receipt['result'] = 'passed'
        overview.destroy()
        print(json.dumps(receipt), flush=True)
        (args.output/'receipt.json').write_text(json.dumps(receipt, indent=2))
    except Exception as error:
        receipt.update(result='failed', error=repr(error))
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output/'receipt.json').write_text(json.dumps(receipt, indent=2))
        raise
    finally:
        if runtime:
            runtime.world.stop()
        app.close()


if __name__ == '__main__':
    main()
