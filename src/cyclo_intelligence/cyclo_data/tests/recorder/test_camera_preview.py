import time
import threading
from http.server import ThreadingHTTPServer
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import urlopen
import cv2
import numpy as np
import pytest
from cyclo_data.camera_preview_node import Camera, PreviewSettings, encode_preview, handler_for


def jpeg(width, height):
    image = np.random.default_rng(1).integers(0, 255, (height, width, 3), dtype=np.uint8)
    return cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes()


def test_preview_resize_recompresses_without_modifying_source():
    original = jpeg(672, 376)
    before = bytes(original)
    preview = encode_preview(original)
    decoded = cv2.imdecode(np.frombuffer(preview, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape[:2] == (237, 424)
    assert len(preview) < len(original) / 3
    assert original == before


def test_small_wrist_image_is_not_upscaled():
    decoded = cv2.imdecode(np.frombuffer(encode_preview(jpeg(424, 240)), np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape[:2] == (240, 424)


def test_camera_keeps_latest_only_and_shares_encoded_result():
    camera = Camera()
    first = SimpleNamespace(data=jpeg(672, 376))
    newest = SimpleNamespace(data=jpeg(424, 240))
    camera.receive(first); camera.receive(newest)
    assert camera.latest is newest
    encoded, _ = camera.snapshot()
    assert camera.snapshot()[0] is encoded
    assert camera.latest.data is newest.data


def test_stale_camera_is_not_served_as_live():
    camera = Camera()
    with pytest.raises(TimeoutError): camera.snapshot()
    camera.receive(SimpleNamespace(data=jpeg(424, 240)))
    camera.received_at = time.monotonic() - 2
    with pytest.raises(TimeoutError): camera.snapshot()


def test_invalid_frame_does_not_replace_good_cached_preview():
    with pytest.raises(ValueError): encode_preview(b'not jpeg')


def test_encoding_time_does_not_halve_preview_refresh_rate(monkeypatch):
    import cyclo_data.camera_preview_node as preview
    clock = [10.0]
    calls = []
    monkeypatch.setattr(preview.time, 'monotonic', lambda: clock[0])
    def encode(data, settings=None):
        calls.append(data)
        clock[0] += .04
        return b'jpeg'
    monkeypatch.setattr(preview, 'encode_preview', encode)
    camera = Camera()
    camera.receive(SimpleNamespace(data=b'original'))
    camera.snapshot()
    clock[0] = 10.085
    camera.snapshot()
    assert calls == [b'original', b'original']


def test_configurable_preview_preserves_source_resolution_and_quality_option():
    original = jpeg(672, 376)
    high = encode_preview(original, PreviewSettings(max_width=1280, jpeg_quality=90))
    default = encode_preview(original)
    assert cv2.imdecode(np.frombuffer(high, np.uint8), cv2.IMREAD_COLOR).shape[:2] == (376, 672)
    assert len(high) > len(default) * 2


@pytest.mark.parametrize('kwargs', [
    {'max_width': 0}, {'max_width': 4097}, {'max_width': 424.5}, {'max_width': True},
    {'jpeg_quality': 0}, {'jpeg_quality': 101}, {'jpeg_quality': 90.5},
    {'preview_fps': 0}, {'preview_fps': 61}, {'preview_fps': float('nan')},
    {'preview_fps': float('inf')}, {'preview_fps': True},
])
def test_preview_settings_reject_unbounded_or_invalid_input(kwargs):
    with pytest.raises(ValueError): PreviewSettings(**kwargs)


def test_original_uses_newest_exact_bytes_without_changing_preview_cache(monkeypatch):
    import cyclo_data.camera_preview_node as preview
    clock = [10.0]
    calls = []
    monkeypatch.setattr(preview.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(preview, 'encode_preview',
                        lambda data, settings: calls.append(data) or b'preview')
    camera = Camera(settings=PreviewSettings(preview_fps=2))
    first, newest = jpeg(424, 240), jpeg(672, 376)
    camera.receive(SimpleNamespace(data=first))
    cached, _ = camera.snapshot()
    clock[0] += .1
    camera.receive(SimpleNamespace(data=bytearray(newest)))
    original, age = camera.snapshot('original')
    assert original == newest and age == 0
    assert camera.snapshot()[0] is cached
    assert calls == [first]
    assert camera.latest.data == newest


def test_original_requires_fresh_jpeg_and_never_serves_stale_cache():
    camera = Camera()
    with pytest.raises(TimeoutError): camera.snapshot('original')
    camera.receive(SimpleNamespace(data=jpeg(424, 240)))
    camera.snapshot()
    camera.received_at = time.monotonic() - 2
    with pytest.raises(TimeoutError): camera.snapshot('original')
    camera.receive(SimpleNamespace(data=b'not a JPEG'))
    with pytest.raises(ValueError): camera.snapshot('original')
    with pytest.raises(ValueError): camera.snapshot('unknown')


def test_preview_fps_controls_cache_interval(monkeypatch):
    import cyclo_data.camera_preview_node as preview
    clock = [10.0]
    calls = []
    monkeypatch.setattr(preview.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(preview, 'encode_preview',
                        lambda data, settings: calls.append(settings) or b'jpeg')
    settings = PreviewSettings(preview_fps=4)
    camera = Camera(settings=settings)
    camera.receive(SimpleNamespace(data=b'source'))
    camera.snapshot()
    clock[0] += .1
    camera.snapshot()
    assert calls == [settings]
    clock[0] += .2
    camera.snapshot()
    assert calls == [settings, settings]


def test_fresh_source_replaces_expired_preview_even_with_low_target_fps(monkeypatch):
    import cyclo_data.camera_preview_node as preview
    clock = [10.0]
    monkeypatch.setattr(preview.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(preview, 'encode_preview', lambda data, settings: data)
    camera = Camera(settings=PreviewSettings(preview_fps=.1))
    camera.receive(SimpleNamespace(data=b'first'))
    camera.snapshot()
    clock[0] += .8
    camera.receive(SimpleNamespace(data=b'newest'))
    assert camera.snapshot() == (b'newest', 0)


def test_http_original_preserves_jpeg_and_freshness_headers():
    camera = Camera()
    source = jpeg(672, 376)
    camera.receive(SimpleNamespace(data=source))
    topics = []
    node = SimpleNamespace(camera=lambda topic: topics.append(topic) or camera)
    server = ThreadingHTTPServer(('127.0.0.1', 0), handler_for(node))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f'http://127.0.0.1:{server.server_port}/snapshot?topic=/camera/compressed'
    try:
        with urlopen(base + '&quality=original', timeout=2) as response:
            assert response.read() == source
            assert response.headers['Content-Type'] == 'image/jpeg'
            assert response.headers['Cache-Control'] == 'no-store'
            assert response.headers['Access-Control-Expose-Headers'] == 'X-Frame-Age-Ms'
            assert float(response.headers['X-Frame-Age-Ms']) <= 750
        with pytest.raises(HTTPError) as error:
            urlopen(base + '&quality=wrong', timeout=2)
        assert error.value.code == 400
        assert topics == ['/camera/compressed']
        camera.received_at = time.monotonic() - 2
        with pytest.raises(HTTPError) as error:
            urlopen(base + '&quality=original', timeout=2)
        assert error.value.code == 503
    finally:
        server.shutdown(); server.server_close(); thread.join(timeout=2)
