import { act, fireEvent, render, screen, waitFor } from '@testing-library/react';

import JointPosePresetControls from './JointPosePresetControls';
import {
  listJointPosePresets,
  saveJointPosePreset,
} from '../../features/actionCanvas/jointPosePresetsApi';

const pose = {
  arm_l_joint1: 0.1,
  arm_r_joint1: -0.1,
  finger_l_joint1: 0.3,
  finger_l_joint20: 0.8,
};

jest.mock('react-hot-toast', () => ({
  success: jest.fn(),
  error: jest.fn(),
}));

jest.mock('../../hooks/useJointPoseCapture', () => () => ({
  state: 'stable',
  message: 'Follower pose is stable',
  capture: () => ({ ok: true, message: 'Follower pose captured', pose }),
  sourceTopics: ['/joint_states'],
}));

jest.mock('../../features/actionCanvas/jointPosePresetsApi', () => ({
  deleteJointPosePreset: jest.fn(),
  listJointPosePresets: jest.fn(),
  saveJointPosePreset: jest.fn(),
}));

beforeEach(() => {
  listJointPosePresets.mockResolvedValue({ presets: [] });
  saveJointPosePreset.mockResolvedValue({ ok: true });
});

test('captures HX5 hand event poses independently without adding arm target gates', async () => {
  const onApplyParams = jest.fn();
  await act(async () => render(
    <JointPosePresetControls
      nodeType="ArmStateGate"
      params={{
        detect_left_hand: 'true',
        left_hand_event_joints: 'finger_l_joint20, finger_l_joint1',
      }}
      robotType="ffw_sh5_rev1"
      onApplyParams={onApplyParams}
    />,
  ));
  expect(screen.getByRole('button', { name: 'Capture Current Pose' })).toBeDisabled();
  fireEvent.click(screen.getByRole('button', { name: 'Capture Closed Hand Pose' }));
  expect(onApplyParams).toHaveBeenLastCalledWith({
    left_hand_event_joints: 'finger_l_joint20, finger_l_joint1',
    left_hand_closed_positions: '0.8, 0.3',
  });
  fireEvent.click(screen.getByRole('button', { name: 'Capture Open Hand Pose' }));
  expect(onApplyParams).toHaveBeenLastCalledWith({
    left_hand_event_joints: 'finger_l_joint20, finger_l_joint1',
    left_hand_open_positions: '0.8, 0.3',
  });
});

test('captures selected follower joints and saves the reusable full snapshot', async () => {
  const onApplyParams = jest.fn();
  render(
    <JointPosePresetControls
      nodeType="JointControl"
      params={{
        enable_head: 'false',
        enable_arms: 'true',
        left_joint_names: 'arm_l_joint1',
        right_joint_names: 'arm_r_joint1',
        enable_lift: 'false',
      }}
      robotType="ffw_sg2_rev1"
      onApplyParams={onApplyParams}
    />,
  );

  fireEvent.click(screen.getByRole('button', { name: 'Capture Current Pose' }));
  expect(onApplyParams).toHaveBeenCalledWith({
    left_positions: '0.1',
    right_positions: '-0.1',
  });

  fireEvent.change(screen.getByLabelText('Pose preset name'), {
    target: { value: 'pick_ready' },
  });
  fireEvent.click(screen.getByRole('button', { name: /Save/ }));

  await waitFor(() => expect(saveJointPosePreset).toHaveBeenCalledWith({
    preset: expect.objectContaining({
      schema_version: 'cyclo_joint_pose_v1',
      name: 'pick_ready',
      robot_type: 'ffw_sg2_rev1',
      source_topic: '/joint_states',
      source_topics: ['/joint_states'],
      joints: pose,
    }),
    overwrite: false,
  }));
});
