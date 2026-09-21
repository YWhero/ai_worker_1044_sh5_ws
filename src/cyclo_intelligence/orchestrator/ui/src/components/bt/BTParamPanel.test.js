// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Seongwoo Kim

import { fireEvent, render, screen, within } from '@testing-library/react';
import BTParamPanel from './BTParamPanel';

let mockRobotType = 'ffw_sg2';

jest.mock('react-redux', () => ({
  useDispatch: () => jest.fn(),
  useSelector: (selector) => selector({ tasks: { robotType: mockRobotType } }),
}));

jest.mock('../FileBrowserModal', () => () => null);
jest.mock('./JointPosePresetControls', () => () => null);

const node = {
  id: 'bt_1',
  data: {
    label: 'JointControl_1',
    nodeType: 'JointControl',
    params: {
      enable_head: 'false',
      enable_arms: 'true',
      enable_lift: 'false',
      duration: '2.0',
    },
  },
};

beforeEach(() => {
  mockRobotType = 'ffw_sg2';
});

function fieldControl(name) {
  return screen.getByText(name, { selector: 'label' }).parentElement.querySelector(
    'input, select, textarea',
  );
}

test('commits parameter drafts while typing instead of waiting for blur', () => {
  const onParamChange = jest.fn();
  render(
    <BTParamPanel
      nodes={[node]}
      selectedNodeId={node.id}
      onParamChange={onParamChange}
      onNameChange={jest.fn()}
    />,
  );

  fireEvent.change(screen.getByDisplayValue('2.0'), { target: { value: '3.5' } });

  expect(onParamChange).toHaveBeenCalledTimes(1);
  expect(onParamChange).toHaveBeenCalledWith('bt_1', 'duration', '3.5');
});

test('commits a valid node name while typing instead of waiting for blur', () => {
  const onNameChange = jest.fn();
  render(
    <BTParamPanel
      nodes={[node]}
      selectedNodeId={node.id}
      onParamChange={jest.fn()}
      onNameChange={onNameChange}
    />,
  );

  fireEvent.change(screen.getByDisplayValue('JointControl_1'), {
    target: { value: 'CloseGripper' },
  });

  expect(onNameChange).toHaveBeenCalledTimes(1);
  expect(onNameChange).toHaveBeenCalledWith('bt_1', 'CloseGripper');
});

test('restores the pre-edit node name when Escape is pressed', () => {
  const onNameChange = jest.fn();
  render(
    <BTParamPanel
      nodes={[node]}
      selectedNodeId={node.id}
      onParamChange={jest.fn()}
      onNameChange={onNameChange}
    />,
  );

  const input = screen.getByDisplayValue('JointControl_1');
  fireEvent.focus(input);
  fireEvent.change(input, { target: { value: 'TemporaryName' } });
  fireEvent.keyDown(input, { key: 'Escape' });

  expect(input).toHaveValue('JointControl_1');
  expect(onNameChange).toHaveBeenLastCalledWith('bt_1', 'JointControl_1');
});

test('JointControl renders per-joint chips that keep positions aligned', () => {
  const onParamChange = jest.fn();
  const jointNode = {
    id: 'bt_2',
    data: {
      label: 'JointControl_2',
      nodeType: 'JointControl',
      params: {
        enable_arms: 'true',
        left_joint_names: 'arm_l_joint1, arm_l_joint2',
        left_positions: '0.1, 0.2',
        right_joint_names: '',
        right_positions: '',
        duration: '2.0',
      },
    },
  };
  render(
    <BTParamPanel
      nodes={[jointNode]}
      selectedNodeId={jointNode.id}
      onParamChange={onParamChange}
      onNameChange={jest.fn()}
    />,
  );

  // Each side renders the SG2 joint chips (left list + right list).
  expect(screen.getAllByRole('button', { name: 'arm_l_joint3' })).toHaveLength(1);
  expect(screen.getAllByRole('button', { name: 'arm_r_joint1' })).toHaveLength(1);

  // Deselecting a joint drops its position from the paired CSV.
  fireEvent.click(screen.getByRole('button', { name: 'arm_l_joint1' }));
  expect(onParamChange).toHaveBeenCalledWith('bt_2', 'left_joint_names', 'arm_l_joint2');
  expect(onParamChange).toHaveBeenCalledWith('bt_2', 'left_positions', '0.2');

  // Selecting a new joint appends it in canonical order with a 0.0 target.
  fireEvent.click(screen.getByRole('button', { name: 'arm_l_joint3' }));
  expect(onParamChange).toHaveBeenCalledWith(
    'bt_2', 'left_joint_names', 'arm_l_joint2, arm_l_joint3',
  );
  expect(onParamChange).toHaveBeenCalledWith('bt_2', 'left_positions', '0.2, 0.0');
});

test('JointControl joint chips are disabled while arms are disabled', () => {
  const jointNode = {
    id: 'bt_3',
    data: {
      label: 'JointControl_3',
      nodeType: 'JointControl',
      params: {
        enable_arms: 'false',
        left_joint_names: 'arm_l_joint1',
        left_positions: '0.0',
      },
    },
  };
  render(
    <BTParamPanel
      nodes={[jointNode]}
      selectedNodeId={jointNode.id}
      onParamChange={jest.fn()}
      onNameChange={jest.fn()}
    />,
  );

  expect(screen.getByRole('button', { name: 'arm_l_joint1' })).toBeDisabled();
});

test('legacy JointControl nodes without joint_names still get the chips', () => {
  const onParamChange = jest.fn();
  const legacyNode = {
    id: 'bt_4',
    data: {
      label: 'JointControl_4',
      nodeType: 'JointControl',
      params: {
        enable_arms: 'true',
        left_positions: '0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8',
        right_positions: '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0',
        duration: '2.0',
      },
    },
  };
  render(
    <BTParamPanel
      nodes={[legacyNode]}
      selectedNodeId={legacyNode.id}
      onParamChange={onParamChange}
      onNameChange={jest.fn()}
    />,
  );

  // The synthesized selection defaults to the full joint list (what the
  // engine does when names are omitted), so deselecting one joint keeps
  // the other joints' existing positions aligned.
  fireEvent.click(screen.getByRole('button', { name: 'gripper_l_joint1' }));
  expect(onParamChange).toHaveBeenCalledWith(
    'bt_4',
    'left_joint_names',
    'arm_l_joint1, arm_l_joint2, arm_l_joint3, arm_l_joint4, '
    + 'arm_l_joint5, arm_l_joint6, arm_l_joint7',
  );
  expect(onParamChange).toHaveBeenCalledWith(
    'bt_4', 'left_positions', '0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7',
  );
});

test('SH5 JointControl renders independent HX5 hand joint chips', () => {
  mockRobotType = 'ffw_sh5_rev1';
  const onParamChange = jest.fn();
  const handNode = {
    id: 'bt_sh5',
    data: {
      label: 'JointControl_SH5',
      nodeType: 'JointControl',
      params: {
        enable_arms: 'false',
        enable_hands: 'true',
        left_positions: '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0',
        left_hand_joint_names: 'finger_l_joint1',
        left_hand_positions: '0.1',
        right_hand_joint_names: '',
        right_hand_positions: '',
      },
    },
  };
  render(
    <BTParamPanel
      nodes={[handNode]}
      selectedNodeId={handNode.id}
      onParamChange={onParamChange}
      onNameChange={jest.fn()}
    />,
  );

  expect(screen.getByRole('button', { name: 'finger_l_joint20' })).toBeEnabled();
  expect(screen.getByRole('button', { name: 'arm_l_joint1' })).toBeDisabled();
  fireEvent.click(screen.getByRole('button', { name: 'finger_l_joint2' }));
  expect(onParamChange).toHaveBeenCalledWith(
    'bt_sh5', 'left_hand_joint_names', 'finger_l_joint1, finger_l_joint2',
  );
  expect(onParamChange).toHaveBeenCalledWith(
    'bt_sh5', 'left_hand_positions', '0.1, 0.0',
  );
});

test('SH5 migrates SG2 catalog defaults and exposes both forty-finger hand controls', () => {
  mockRobotType = 'ffw_sh5_rev1';
  const onParamChange = jest.fn();
  const legacy = {
    id: 'bt_sh5_legacy',
    data: {
      nodeType: 'JointControl',
      label: 'JointControl_SH5',
      params: {
        enable_arms: 'true',
        left_joint_names: 'arm_l_joint1, gripper_l_joint1',
        left_positions: '0.4, 0.9',
        right_joint_names: 'arm_r_joint1, gripper_r_joint1',
        right_positions: '0.5, 0.8',
      },
    },
  };
  render(<BTParamPanel nodes={[legacy]} selectedNodeId={legacy.id} onParamChange={onParamChange} />);
  expect(screen.queryByRole('button', { name: 'gripper_l_joint1' })).not.toBeInTheDocument();
  expect(onParamChange).toHaveBeenCalledWith(legacy.id, 'left_joint_names', 'arm_l_joint1');
  expect(onParamChange).toHaveBeenCalledWith(legacy.id, 'left_positions', '0.4');
  expect(screen.getByRole('button', { name: 'finger_r_joint20' })).toBeDisabled();
  fireEvent.click(fieldControl('enable_hands'));
  expect(screen.getByRole('button', { name: 'finger_r_joint20' })).toBeEnabled();
});

test('SH5 ArmStateGate exposes hand/head/lift targets and tactile controls without SG2 grippers', () => {
  mockRobotType = 'ffw_sh5_rev1';
  const onParamChange = jest.fn();
  const gate = { id: 'gate_sh5', data: { nodeType: 'ArmStateGate', label: 'ArmStateGate', params: {} } };
  render(<BTParamPanel nodes={[gate]} selectedNodeId={gate.id} onParamChange={onParamChange} />);
  expect(screen.queryByText('gripper_open_value', { selector: 'label' })).not.toBeInTheDocument();
  expect(screen.queryByText('detect_left_gripper', { selector: 'label' })).not.toBeInTheDocument();
  expect(screen.getByRole('button', { name: 'head_joint2' })).toBeEnabled();
  expect(screen.getByRole('button', { name: 'lift_joint' })).toBeEnabled();
  expect(screen.getAllByRole('button', { name: 'finger_l_joint20' })).toHaveLength(2);
  const armTargets = screen.getByRole('group', { name: 'Arm targets' });
  expect(within(armTargets).getAllByRole('button')).toHaveLength(14);
  expect(within(armTargets).queryByRole('button', { name: /finger_/ })).not.toBeInTheDocument();
  expect(within(screen.getByRole('group', { name: 'Hand targets' })).getAllByRole('button')).toHaveLength(40);
  expect(within(screen.getByRole('group', { name: 'Head targets' })).getAllByRole('button')).toHaveLength(2);
  expect(within(screen.getByRole('group', { name: 'Lift target' })).getAllByRole('button')).toHaveLength(1);
  expect(fieldControl('left_contact_condition')).toBeDisabled();
  fireEvent.click(fieldControl('detect_left_contact'));
  expect(fieldControl('left_contact_condition')).toBeEnabled();
  fireEvent.change(fieldControl('left_contact_condition'), { target: { value: 'released' } });
  expect(onParamChange).toHaveBeenCalledWith(gate.id, 'left_contact_condition', 'released');
});

test('opening a legacy mixed gate persists newly migrated hand pairs and preserves arm values', () => {
  mockRobotType = 'ffw_sh5_rev1';
  const onParamChange = jest.fn();
  const gate = { id: 'gate_mixed', data: { nodeType: 'ArmStateGate', label: 'ArmStateGate', params: {
    left_target_joints: 'arm_l_joint2, finger_l_joint20', left_target_positions: '0.12, 0.82',
    right_target_joints: 'finger_r_joint1', right_target_positions: '-0.3',
  } } };
  render(<BTParamPanel nodes={[gate]} selectedNodeId={gate.id} onParamChange={onParamChange} />);
  expect(fieldControl('left_target_joints')).toHaveValue('arm_l_joint2');
  expect(fieldControl('left_target_positions')).toHaveValue('0.12');
  expect(fieldControl('left_hand_target_joints')).toHaveValue('finger_l_joint20');
  expect(fieldControl('left_hand_target_positions')).toHaveValue('0.82');
  expect(onParamChange).toHaveBeenCalledWith(gate.id, 'left_hand_target_joints', 'finger_l_joint20');
  expect(onParamChange).toHaveBeenCalledWith(gate.id, 'left_hand_target_positions', '0.82');
  expect(onParamChange).toHaveBeenCalledWith(gate.id, 'right_target_joints', '');
  expect(onParamChange).toHaveBeenCalledWith(gate.id, 'right_hand_target_positions', '-0.3');
  expect(screen.queryByRole('alert')).not.toBeInTheDocument();
});

test('a conflicting legacy hand target keeps both values visible and requires manual resolution', () => {
  mockRobotType = 'ffw_sh5_rev1';
  const onParamChange = jest.fn();
  const gate = { id: 'gate_conflict', data: { nodeType: 'ArmStateGate', label: 'ArmStateGate', params: {
    left_target_joints: 'arm_l_joint1, finger_l_joint2', left_target_positions: '0.4, 0.8',
    left_hand_target_joints: 'finger_l_joint2', left_hand_target_positions: '0.2',
  } } };
  render(<BTParamPanel nodes={[gate]} selectedNodeId={gate.id} onParamChange={onParamChange} />);
  expect(screen.getByRole('alert')).toHaveTextContent('targets 0.8 in left_target_joints and 0.2 in left_hand_target_joints');
  expect(fieldControl('left_target_positions')).toHaveValue('0.4, 0.8');
  expect(fieldControl('left_hand_target_positions')).toHaveValue('0.2');
  const armTargets = screen.getByRole('group', { name: 'Arm targets' });
  expect(within(armTargets).queryByRole('button', { name: 'finger_l_joint2' })).not.toBeInTheDocument();
  fireEvent.click(within(armTargets).getByRole('button', { name: 'arm_l_joint3' }));
  expect(onParamChange).toHaveBeenCalledWith(gate.id, 'left_target_joints', 'arm_l_joint1, arm_l_joint3, finger_l_joint2');
  expect(onParamChange).toHaveBeenCalledWith(gate.id, 'left_target_positions', '0.4, 0.0, 0.8');
  expect(fieldControl('left_hand_target_positions')).toHaveValue('0.2');
});

test('hand event finger selection keeps both closed and open arrays aligned', () => {
  mockRobotType = 'ffw_sh5_rev1';
  const onParamChange = jest.fn();
  const gate = { id: 'gate_event', data: { nodeType: 'ArmStateGate', label: 'ArmStateGate', params: {
    detect_left_hand: 'true',
    left_hand_event_joints: 'finger_l_joint1, finger_l_joint2',
    left_hand_closed_positions: '0.5, 0.6',
    left_hand_open_positions: '0.1, 0.2',
  } } };
  render(<BTParamPanel nodes={[gate]} selectedNodeId={gate.id} onParamChange={onParamChange} />);
  const eventField = screen.getByText('left_hand_event_joints', { selector: 'label' }).parentElement;
  fireEvent.click(within(eventField).getByRole('button', { name: 'finger_l_joint1' }));
  expect(onParamChange).toHaveBeenCalledWith(gate.id, 'left_hand_event_joints', 'finger_l_joint2');
  expect(onParamChange).toHaveBeenCalledWith(gate.id, 'left_hand_closed_positions', '0.6');
  expect(onParamChange).toHaveBeenCalledWith(gate.id, 'left_hand_open_positions', '0.2');
});

test('treats legacy SendCommand nodes without a target as inference commands', () => {
  const legacyCommand = {
    id: 'bt_5',
    data: {
      label: 'SendCommand_1',
      nodeType: 'SendCommand',
      params: {
        command: 'STOP',
        model: 'lerobot:act',
        policy_path: '/workspace/model/lerobot/example',
      },
    },
  };

  render(
    <BTParamPanel
      nodes={[legacyCommand]}
      selectedNodeId={legacyCommand.id}
      onParamChange={jest.fn()}
      onNameChange={jest.fn()}
    />,
  );

  expect(fieldControl('target')).toHaveValue('INFERENCE');
  expect(fieldControl('command')).toHaveValue('STOP');
  expect(Array.from(fieldControl('command').options, (option) => option.value)).toEqual([
    'LOAD', 'RESUME', 'STOP', 'CLEAR',
  ]);
  expect(fieldControl('model')).toBeDisabled();
  expect(fieldControl('policy_path')).toBeDisabled();
});

test('switches SendCommand to Docker controls and enables only target, command, and model', () => {
  const onParamChange = jest.fn();
  const dockerCommand = {
    id: 'bt_6',
    data: {
      label: 'SendCommand_2',
      nodeType: 'SendCommand',
      params: {
        target: 'INFERENCE',
        command: 'LOAD',
        model: 'groot:n17',
        policy_path: '/workspace/model/groot/example',
        task_instruction: 'Pick up the object',
        inference_mode: 'robot',
        action_request_mode: 'sync',
        inference_hz: '10',
        control_hz: '100',
        chunk_align_window_s: '0.3',
        acceleration_mode: 'pytorch',
        acceleration_engine_path: '',
      },
    },
  };

  render(
    <BTParamPanel
      nodes={[dockerCommand]}
      selectedNodeId={dockerCommand.id}
      onParamChange={onParamChange}
      onNameChange={jest.fn()}
    />,
  );

  fireEvent.change(fieldControl('target'), { target: { value: 'DOCKER' } });

  expect(onParamChange).toHaveBeenCalledWith('bt_6', 'target', 'DOCKER');
  expect(onParamChange).toHaveBeenCalledWith('bt_6', 'command', 'START');
  expect(fieldControl('target')).toBeEnabled();
  expect(fieldControl('command')).toBeEnabled();
  expect(fieldControl('model')).toBeEnabled();
  expect(fieldControl('command')).toHaveValue('START');
  expect(Array.from(fieldControl('command').options, (option) => option.value)).toEqual([
    'START', 'STOP', 'RESTART',
  ]);
  [
    'policy_path',
    'task_instruction',
    'inference_mode',
    'action_request_mode',
    'inference_hz',
    'control_hz',
    'chunk_align_window_s',
    'acceleration_mode',
    'acceleration_engine_path',
  ].forEach((key) => expect(fieldControl(key)).toBeDisabled());
});

test('LOAD edits first pose sync and duration while RESUME disables the retained settings', () => {
  const onParamChange = jest.fn();
  const command = { id: 'sync_load', data: { nodeType: 'SendCommand', label: 'LOAD', params: {
    target: 'INFERENCE',
    command: 'LOAD',
    model: 'vitacformer:vitacformer',
  } } };
  render(<BTParamPanel nodes={[command]} selectedNodeId={command.id} onParamChange={onParamChange} />);
  const synchronize = fieldControl('initial_pose_sync');
  const duration = fieldControl('initial_pose_sync_duration_s');
  expect(synchronize).toHaveAttribute('type', 'checkbox');
  expect(synchronize).not.toBeChecked();
  expect(duration).toHaveAttribute('type', 'number');
  expect(duration).toHaveAttribute('min', '1');
  expect(duration).toHaveAttribute('max', '60');
  expect(duration).toHaveValue(5);
  fireEvent.click(synchronize);
  fireEvent.change(duration, { target: { value: '6.5' } });
  expect(onParamChange).toHaveBeenCalledWith(command.id, 'initial_pose_sync', 'true');
  expect(onParamChange).toHaveBeenCalledWith(command.id, 'initial_pose_sync_duration_s', '6.5');
  fireEvent.change(fieldControl('command'), { target: { value: 'RESUME' } });
  expect(synchronize).toBeDisabled();
  expect(duration).toBeDisabled();
  expect(synchronize).toBeChecked();
  expect(duration).toHaveValue(6.5);
  fireEvent.change(fieldControl('command'), { target: { value: 'LOAD' } });
  expect(synchronize).toBeEnabled();
  expect(duration).toBeEnabled();
});
