import {
  buildInitialPoseSyncTaskInfo,
  getConversionCommandRequestFields,
  getRecordCommandServiceTimeoutMs,
  getTaskInfoInferenceHz,
  transformReplayDataResult,
} from './useRosServiceCaller';

describe('getConversionCommandRequestFields', () => {
  test.each(['start_inference', 'resume_inference'])(
    'omits conversion-only tactile fields from %s',
    (command) => {
      expect(getConversionCommandRequestFields(command, {
        tactileMode: 'separate_raw',
        selectedTactileTopics: ['/left/tactile'],
      })).toEqual({});
    },
  );

  test('keeps tactile settings on convert_mp4', () => {
    expect(getConversionCommandRequestFields('convert_mp4', {
      tactileMode: 'separate_raw',
      selectedTactileTopics: ['/left/tactile'],
      tactileBaselineSamples: 32,
      cameraRotations: { cam_left: 90 },
      imageResize: { height: 376, width: 672 },
    })).toMatchObject({
      tactile_mode: 'separate_raw',
      selected_tactile_topics: ['/left/tactile'],
      tactile_baseline_samples: 32,
      camera_rotation_keys: ['cam_left'],
      camera_rotation_values: [90],
      image_resize_height: 376,
      image_resize_width: 672,
    });
  });
});

describe('buildInitialPoseSyncTaskInfo', () => {
  test('converts UI settings to ROS task info fields', () => {
    expect(buildInitialPoseSyncTaskInfo({
      initialPoseSync: true,
      initialPoseSyncDurationS: 7.5,
    })).toEqual({
      initial_pose_sync: true,
      initial_pose_sync_duration_s: 7.5,
    });
  });

  test('uses safe defaults for legacy UI state', () => {
    expect(buildInitialPoseSyncTaskInfo()).toEqual({
      initial_pose_sync: false,
      initial_pose_sync_duration_s: 5.0,
    });
  });
});

describe('getRecordCommandServiceTimeoutMs', () => {
  test('keeps the response channel open for checkpoint switching', () => {
    expect(getRecordCommandServiceTimeoutMs('switch_inference_model')).toBe(0);
  });
  test('does not time out recording save commands', () => {
    expect(getRecordCommandServiceTimeoutMs('stop_segment')).toBe(0);
    expect(getRecordCommandServiceTimeoutMs('finish_episode')).toBe(0);
    expect(getRecordCommandServiceTimeoutMs('stop_inference_record')).toBe(0);
  });

  test('keeps the shorter default for ordinary commands', () => {
    expect(getRecordCommandServiceTimeoutMs('refresh_topics')).toBe(10000);
  });

  test('allows 30 seconds for inference start and resume', () => {
    expect(getRecordCommandServiceTimeoutMs('start_inference')).toBe(30000);
    expect(getRecordCommandServiceTimeoutMs('resume_inference')).toBe(30000);
  });

  test('allows callers to override the service timeout', () => {
    expect(getRecordCommandServiceTimeoutMs('stop_segment', {
      serviceTimeoutMs: 45000,
    })).toBe(45000);
  });
});

describe('getTaskInfoInferenceHz', () => {
  test('uses 30 Hz when an ACT task leaves the field blank', () => {
    expect(getTaskInfoInferenceHz({
      serviceType: 'lerobot',
      policyType: 'act',
      inferenceHz: '',
    })).toBe(30);
  });

  test('does not apply the ACT default to T-Rex or explicit values', () => {
    expect(getTaskInfoInferenceHz({
      serviceType: 'lerobot',
      policyType: 'trex',
      inferenceHz: '',
    })).toBe(15);
    expect(getTaskInfoInferenceHz({
      serviceType: 'lerobot',
      policyType: 'act',
      inferenceHz: 20,
    })).toBe(20);
  });
});

describe('transformReplayDataResult', () => {
  test('preserves replay robot metadata for the 3D viewer', () => {
    const result = transformReplayDataResult(
      {
        success: true,
        robot_type: 'ffw_sh5_rev1',
        urdf_path: '/workspace/robot_configs/urdf/ffw_sh5_follower.urdf',
        end_effector_links: ['tool0'],
      },
      '/workspace/rosbag2/sh5/0'
    );

    expect(result.robot_type).toBe('ffw_sh5_rev1');
    expect(result.urdf_path).toBe(
      '/workspace/robot_configs/urdf/ffw_sh5_follower.urdf'
    );
    expect(result.end_effector_links).toEqual(['tool0']);
    expect(result.bag_path).toBe('/workspace/rosbag2/sh5/0');
  });
});
