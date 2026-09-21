"""Verify nominal camera projections agree with rendered physical optics."""
import importlib.util
import json
import math
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    'hx5_camera_profiles', Path(__file__).resolve().parents[1]/'camera_profiles.py')
profiles = importlib.util.module_from_spec(spec)
spec.loader.exec_module(profiles)


@pytest.mark.parametrize('profile', profiles.PROFILE_NAMES)
@pytest.mark.parametrize('name', profiles.CAMERA_NAMES)
def test_physical_optics_and_ros_pixel_projection_describe_same_view(name, profile):
    data = profiles.camera_profile(name, profile)
    width, height = data['resolution']
    assert width*data['focal_length_mm']/data['horizontal_aperture_mm'] == pytest.approx(data['fx'])
    assert height*data['focal_length_mm']/data['vertical_aperture_mm'] == pytest.approx(data['fy'])
    assert data['cx'] == width/2
    assert data['cy'] == height/2
    assert data['frequency_hz'] == 30
    assert data['jpeg_quality'] == 95 and data['jpeg_subsampling'] == 0
    assert data['source_url'].startswith('https://')
    # Pixel at the image boundary maps to the same ray in both models.
    assert width/(2*data['fx']) == pytest.approx(
        data['horizontal_aperture_mm']/(2*data['focal_length_mm']))
    assert height/(2*data['fy']) == pytest.approx(
        data['vertical_aperture_mm']/(2*data['focal_length_mm']))


def test_default_head_mode_matches_manufacturer_wvga_nominal_focal_length():
    data = profiles.camera_profile('head_left')
    assert data['resolution'] == [672, 376]
    assert data['fx'] == data['fy'] == 367
    assert data['focal_length_mm'] == pytest.approx(2.936)
    assert data['horizontal_aperture_mm'] == pytest.approx(5.376)
    assert data['vertical_aperture_mm'] == pytest.approx(3.008)
    assert data['calibration_kind'] == 'manufacturer_nominal_rectified'


@pytest.mark.parametrize('profile,fx', [('official', 367), ('hd720', 736)])
def test_right_stereo_projection_encodes_real_head_baseline(profile, fx):
    left = profiles.camera_profile('head_left', profile)
    right = profiles.camera_profile('head_right', profile)
    assert left['stereo_baseline_m'] == right['stereo_baseline_m'] == .063
    assert left['projection_tx'] == 0
    assert right['projection_tx'] == pytest.approx(-fx*.063)


@pytest.mark.parametrize('name', ['wrist_left', 'wrist_right'])
def test_d405_nominal_field_of_view_and_head_only_mode_scope(name):
    data = profiles.camera_profile(name)
    width, height = data['resolution']
    horizontal = math.degrees(2*math.atan(width/(2*data['fx'])))
    vertical = math.degrees(2*math.atan(height/(2*data['fy'])))
    assert horizontal == pytest.approx(87)
    assert vertical == pytest.approx(58)
    assert data == profiles.camera_profile(name, 'hd720')
    assert 'stereo_baseline_m' not in data
    assert data['projection_tx'] == 0


def test_hd720_head_retains_square_pixels_and_manufacturer_nominal_focal_length():
    data = profiles.camera_profile('head_left', 'hd720')
    assert data['resolution'] == [1280, 720]
    assert data['fx'] == data['fy'] == 736
    assert data['focal_length_mm'] == pytest.approx(2.944)
    assert data['horizontal_aperture_mm'] == pytest.approx(5.12)
    assert data['vertical_aperture_mm'] == pytest.approx(2.88)


@pytest.mark.parametrize('name,profile', [('other', 'official'), ('head_left', 'invalid'),
                                        ('wrist_right', 'invalid')])
def test_unknown_camera_or_profile_is_rejected(name, profile):
    with pytest.raises(ValueError, match='Unknown'):
        profiles.camera_profile(name, profile)


def test_returned_profile_cannot_mutate_future_camera_configuration():
    first = profiles.camera_profile('head_left')
    first['resolution'][0] = 1
    first['fx'] = 1
    assert profiles.camera_profile('head_left')['resolution'] == [672, 376]
    assert profiles.camera_profile('head_left')['fx'] == 367


@pytest.mark.parametrize('name', profiles.CAMERA_NAMES)
def test_generated_stage_lens_schema_matches_nominal_pixel_projection_and_stage_units(name):
    """Read the real importer output; physical mm must become USD camera units."""
    Usd = pytest.importorskip('pxr.Usd')
    from pxr import UsdGeom
    try:
        from pxr import Plug, PhysxSchema
    except ImportError:
        pass
    else:
        provider = Path(PhysxSchema.__file__).resolve().parents[2]
        Plug.Registry().RegisterPlugins(str(provider/'plugins/PhysxSchema/resources/plugInfo.json'))
    receipt = Path(__file__).resolve().parents[3]/'simulation/isaac/assets/sh5/scene_metadata.json'
    if not receipt.is_file():
        pytest.skip('Run the official scene importer before checking authored camera optics')
    metadata = json.loads(receipt.read_text())
    stage = Usd.Stage.Open(metadata['stage'])
    spec = metadata['cameras'][name]
    mode = 'hd720' if name.startswith('head_') and spec['resolution'] == [1280, 720] else 'official'
    expected = profiles.camera_profile(name, mode)
    prim = stage.GetPrimAtPath(spec['prim_path'])
    camera = UsdGeom.Camera(prim)
    assert camera
    # Camera apertures/focal length use tenths of a stage length unit.
    mm_to_camera_units = .001/(.1*UsdGeom.GetStageMetersPerUnit(stage))
    assert camera.GetFocalLengthAttr().Get() == pytest.approx(expected['focal_length_mm']*mm_to_camera_units)
    assert camera.GetHorizontalApertureAttr().Get() == pytest.approx(expected['horizontal_aperture_mm']*mm_to_camera_units)
    assert camera.GetVerticalApertureAttr().Get() == pytest.approx(expected['vertical_aperture_mm']*mm_to_camera_units)
    assert camera.GetFStopAttr().Get() == 0
    assert 'OmniLensDistortionOpenCvPinholeAPI' in prim.GetMetadata('apiSchemas').GetAppliedItems()
    assert prim.GetAttribute('omni:lensdistortion:model').Get() == 'opencvPinhole'
    prefix = 'omni:lensdistortion:opencvPinhole:'
    assert list(prim.GetAttribute(prefix+'imageSize').Get()) == expected['resolution']
    for attribute in ('fx', 'fy', 'cx', 'cy'):
        assert prim.GetAttribute(prefix+attribute).Get() == pytest.approx(expected[attribute])
    for attribute in ('k1', 'k2', 'p1', 'p2', 'k3', 'k4', 'k5', 'k6', 's1', 's2', 's3', 's4'):
        assert prim.GetAttribute(prefix+attribute).Get() == 0
