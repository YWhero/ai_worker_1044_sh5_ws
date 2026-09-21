#!/usr/bin/env python3
"""Read-only SH5 source JPEG/CameraInfo and HTTP preview quality receipt."""
import argparse
import io
import json
import math
import time
from pathlib import Path
from urllib.parse import urlencode
from urllib.request import urlopen

from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage

CAMERAS = {
    'head_left': ('/zed/zed_node/left/image_rect_color', 'zedm_left_camera_optical_frame'),
    'head_right': ('/zed/zed_node/right/image_rect_color', 'zedm_right_camera_optical_frame'),
    'wrist_left': ('/camera_left/camera_left/color/image_rect_raw', 'camera_l_color_optical_frame'),
    'wrist_right': ('/camera_right/camera_right/color/image_rect_raw', 'camera_r_color_optical_frame'),
}


def reference_tables(quality):
    out = io.BytesIO()
    Image.new('RGB', (8, 8)).save(out, format='JPEG', quality=quality, subsampling=0)
    return Image.open(io.BytesIO(out.getvalue())).quantization


def jpeg_details(data):
    image = Image.open(io.BytesIO(data))
    tables = getattr(image, 'quantization', {})
    return {'format': image.format, 'resolution': list(image.size), 'bytes': len(data),
            'quantization_tables': tables,
            'quality95_tables_match': tables == reference_tables(95),
            'matching_pil_quality': next((q for q in range(1, 101)
                                          if tables == reference_tables(q)), None)}


def header(message):
    return {'stamp': message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
            'frame_id': message.header.frame_id}


class Probe(Node):
    def __init__(self, directory):
        super().__init__('hx5_camera_quality_probe')
        self.directory = directory
        self.samples = {name: {'image_count': 0, 'info_count': 0} for name in CAMERAS}
        self.subscriptions_keep = []
        for name, (topic, _) in CAMERAS.items():
            info = topic.replace('image_rect_color', 'camera_info').replace('image_rect_raw', 'camera_info')
            self.subscriptions_keep.extend([
                self.create_subscription(CompressedImage, topic + '/compressed',
                    lambda msg, n=name: self.image(n, msg), qos_profile_sensor_data),
                self.create_subscription(CameraInfo, info,
                    lambda msg, n=name: self.info(n, msg), qos_profile_sensor_data)])

    def image(self, name, message):
        sample = self.samples[name]
        stamp = header(message)
        sample['image_count'] += 1
        sample['last_image_header'] = stamp
        if 'first_image_header' not in sample:
            data = bytes(message.data)
            (self.directory / f'{name}.jpg').write_bytes(data)
            sample['first_image_header'] = stamp
            try:
                sample['source_jpeg'] = jpeg_details(data)
            except Exception as error:
                sample['decode_error'] = str(error)

    def info(self, name, message):
        sample = self.samples[name]
        sample['info_count'] += 1
        sample['camera_info'] = {**header(message), 'resolution': [message.width, message.height],
            'distortion_model': message.distortion_model, 'd': list(message.d),
            'k': list(message.k), 'r': list(message.r), 'p': list(message.p)}


def official_errors(name, sample):
    failures = []
    jpeg = sample.get('source_jpeg', {})
    info = sample.get('camera_info', {})
    dimensions = [672, 376] if name.startswith('head') else [424, 240]
    fx = 367.0 if name.startswith('head') else 424 / (2 * math.tan(math.radians(43.5)))
    fy = 367.0 if name.startswith('head') else 240 / (2 * math.tan(math.radians(29)))
    tx = -23.121 if name == 'head_right' else 0.0
    for label, actual, expected in [('JPEG resolution', jpeg.get('resolution'), dimensions),
            ('CameraInfo resolution', info.get('resolution'), dimensions),
            ('image frame', sample.get('last_image_header', {}).get('frame_id'), CAMERAS[name][1]),
            ('CameraInfo frame', info.get('frame_id'), CAMERAS[name][1])]:
        if actual != expected: failures.append(f'{label}: {actual!r}, expected {expected!r}')
    for key, index, expected in [('k', 0, fx), ('k', 4, fy), ('p', 0, fx), ('p', 5, fy), ('p', 3, tx)]:
        array = info.get(key, [])
        if len(array) <= index or not math.isclose(array[index], expected, abs_tol=1e-5):
            failures.append(f'{key}[{index}] does not match {expected}')
    if not jpeg.get('quality95_tables_match'): failures.append('JPEG tables do not match quality95')
    fps = sample.get('image_frequency_sim_hz')
    if fps is None or not 28 <= fps <= 32: failures.append(f'simulation image frequency {fps}, expected [28,32]')
    return failures


def previews(directory, base):
    results = {}
    for name, (topic, _) in CAMERAS.items():
        url = base.rstrip('/') + '/snapshot?' + urlencode({'topic': topic + '/compressed'})
        deadline = time.monotonic() + 3
        while True:
            try:
                with urlopen(url, timeout=1) as response:
                    data = response.read()
                    details = jpeg_details(data)
                    details['frame_age_ms'] = response.headers.get('X-Frame-Age-Ms')
                (directory / f'preview_{name}.jpg').write_bytes(data)
                results[name] = details
                break
            except Exception as error:
                if time.monotonic() >= deadline:
                    results[name] = {'error': str(error)}
                    break
                time.sleep(.1)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=8)
    parser.add_argument('--domain-id', type=int, default=115)
    parser.add_argument('--preview-url', default='http://127.0.0.1:7886')
    parser.add_argument('--expect-official', action='store_true')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 60:
        parser.error('duration must be finite and in [1,60] seconds')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init(domain_id=args.domain_id)
    node = Probe(args.output_dir)
    start = time.monotonic()
    try:
        while time.monotonic() - start < args.duration:
            rclpy.spin_once(node, timeout_sec=.05)
        wall_duration = time.monotonic() - start
        for sample in node.samples.values():
            elapsed = (sample.get('last_image_header', {}).get('stamp', 0)
                       - sample.get('first_image_header', {}).get('stamp', 0))
            sample['image_frequency_sim_hz'] = (sample['image_count'] - 1) / elapsed if elapsed > 0 else None
            sample['first_to_last_image_sim_seconds'] = elapsed
            sample['image_frequency_wall_hz'] = sample['image_count'] / wall_duration
        failures = {name: official_errors(name, sample) for name, sample in node.samples.items()}
        report = {'domain_id': args.domain_id, 'wall_duration_seconds': wall_duration,
            'source': node.samples, 'official_checks': failures,
            'official_passed': not any(failures.values()),
            'preview': previews(args.output_dir, args.preview_url) if args.preview_url else {}}
        (args.output_dir / 'quality.json').write_text(json.dumps(report, indent=2))
        print(json.dumps({'official_passed': report['official_passed'], 'cameras': {
            name: {'count': sample['image_count'], 'sim_hz': sample['image_frequency_sim_hz'],
                   'jpeg': {k:v for k,v in sample.get('source_jpeg', {}).items() if k != 'quantization_tables'},
                   'k': sample.get('camera_info', {}).get('k'), 'p': sample.get('camera_info', {}).get('p')}
            for name, sample in node.samples.items()}, 'output': str(args.output_dir)}))
        return 1 if args.expect_official and not report['official_passed'] else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
