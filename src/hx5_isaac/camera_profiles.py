"""Manufacturer nominal camera optics for the SH5 simulation.

The values are nominal pinhole/rectified projections, not serial-number
factory calibration. Physical apertures are derived from the nominal pixel
focal lengths so rendered images and ROS CameraInfo describe the same view.
JPEG settings describe this simulation's transport, not camera hardware.
"""
from __future__ import annotations

import math

CAMERA_NAMES = ('head_left', 'head_right', 'wrist_left', 'wrist_right')
PROFILE_NAMES = ('official', 'hd720')
ZED_NOMINAL_SOURCE = (
    'https://support.stereolabs.com/hc/en-us/articles/'
    '360007395634-What-is-the-camera-focal-length-and-field-of-view')
D405_NOMINAL_SOURCE = 'https://www.realsenseai.com/product-family/d405-series/'


def camera_profile(name, profile='official'):
    """Return fresh metadata for one named SH5 camera and supported mode.

    ``official`` uses the workspace's head WVGA and wrist 424x240 modes.
    ``hd720`` changes only the head pair. Principal points are nominally
    centered in the image; distortion is omitted for this pinhole renderer.
    """
    if name not in CAMERA_NAMES:
        raise ValueError(f'Unknown SH5 camera: {name!r}; expected one of {CAMERA_NAMES}')
    if profile not in PROFILE_NAMES:
        raise ValueError(f'Unknown camera profile: {profile!r}; expected one of {PROFILE_NAMES}')

    if name.startswith('head_'):
        width, height, fx = ((672, 376, 367.0) if profile == 'official'
                             else (1280, 720, 736.0))
        fy = fx
        # WVGA pixels are 8 um and HD720 pixels 4 um after sensor binning.
        pixel_size_mm = 0.008 if profile == 'official' else 0.004
        focal = fx*pixel_size_mm
        horizontal_aperture = width*pixel_size_mm
        vertical_aperture = height*pixel_size_mm
        data = {
            'model': 'ZED Mini',
            'calibration_kind': 'manufacturer_nominal_rectified',
            'source_url': ZED_NOMINAL_SOURCE,
            'stereo_baseline_m': 0.063,
            'projection_tx': -fx*0.063 if name == 'head_right' else 0.0,
        }
    else:
        width, height = 424, 240
        fx = width/(2*math.tan(math.radians(87.0/2)))
        fy = height/(2*math.tan(math.radians(58.0/2)))
        focal = 3.0
        horizontal_aperture = focal*width/fx
        vertical_aperture = focal*height/fy
        data = {
            'model': 'RealSense D405',
            'calibration_kind': 'nominal_pinhole',
            'source_url': D405_NOMINAL_SOURCE,
            'projection_tx': 0.0,
        }
    data.update({
        'resolution': [width, height],
        'frequency_hz': 30,
        'jpeg_quality': 95,
        'jpeg_subsampling': 0,
        'fx': fx, 'fy': fy,
        'cx': width/2.0, 'cy': height/2.0,
        'focal_length_mm': focal,
        'horizontal_aperture_mm': horizontal_aperture,
        'vertical_aperture_mm': vertical_aperture,
    })
    return data
