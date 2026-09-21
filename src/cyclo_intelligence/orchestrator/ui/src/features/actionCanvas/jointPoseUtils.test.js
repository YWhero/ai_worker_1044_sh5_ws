import {
  nodeParamsFromPose,
  normalizeJointPoseNodeParams,
  poseJointNamesForNode,
  handEventParamsFromPose,
  gateTargetDiagnostics,
  jointPoseProfileForRobot,
  SH5_LEFT_ARM_JOINTS,
  SH5_LEFT_HAND_JOINTS,
  SH5_RIGHT_ARM_JOINTS,
  SH5_RIGHT_HAND_JOINTS,
  SG2_LEFT_JOINTS,
  SG2_RIGHT_JOINTS,
} from './jointPoseUtils';

const pose = Object.fromEntries([
  ['head_joint1', 0.42],
  ['head_joint2', -0.1],
  ['lift_joint', -0.48],
  ...SG2_LEFT_JOINTS.map((name, index) => [name, 0.1 + index * 0.01]),
  ...SG2_RIGHT_JOINTS.map((name, index) => [name, -0.1 - index * 0.01]),
]);

test('maps a follower pose to enabled JointControl groups and selected joints', () => {
  const result = nodeParamsFromPose('JointControl', {
    enable_head: 'true',
    enable_arms: 'true',
    left_joint_names: 'arm_l_joint1, arm_l_joint3',
    right_joint_names: 'arm_r_joint2',
    enable_lift: 'true',
  }, pose);

  expect(result.missing).toEqual([]);
  expect(result.updates).toEqual({
    head_positions: '0.42, -0.1',
    left_positions: '0.1, 0.12',
    right_positions: '-0.11',
    lift_position: '-0.48',
  });
});

test('migrates SG2 catalog arm defaults to SH5 without waiting on nonexistent grippers', () => {
  const params = normalizeJointPoseNodeParams('JointControl', {
    enable_head: 'true',
    enable_lift: 'true',
    enable_arms: 'true',
    left_joint_names: SG2_LEFT_JOINTS.join(', '),
    left_positions: '0, 1, 2, 3, 4, 5, 6, 0.8',
    right_joint_names: SG2_RIGHT_JOINTS.join(', '),
    right_positions: '0, -1, -2, -3, -4, -5, -6, 0.9',
  }, 'ffw_sh5_rev1');

  expect(params.left_joint_names.split(', ')).toEqual(SH5_LEFT_ARM_JOINTS);
  expect(params.left_positions).toBe('0, 1, 2, 3, 4, 5, 6');
  expect(params.right_positions).toBe('0, -1, -2, -3, -4, -5, -6');
  expect(params.enable_hands).toBe('false');
  expect(poseJointNamesForNode('JointControl', params, 'ffw_sh5_rev1')).toHaveLength(17);

  const withHands = { ...params, enable_hands: 'true' };
  expect(poseJointNamesForNode('JointControl', withHands, 'ffw_sh5_rev1')).toHaveLength(57);
});

test('migrates a legacy eight-position arm without names and preserves SG2 tasks', () => {
  const legacy = { left_positions: '1, 2, 3, 4, 5, 6, 7, 8' };
  expect(normalizeJointPoseNodeParams('JointControl', legacy, 'ffw_sh5_rev1').left_positions)
    .toBe('1, 2, 3, 4, 5, 6, 7');
  const sg2 = normalizeJointPoseNodeParams('JointControl', legacy, 'ffw_sg2_rev1');
  expect(sg2.left_positions).toBe(legacy.left_positions);
  expect(sg2.left_joint_names).toContain('gripper_l_joint1');
});

test('captures hand-only, head, and lift gate targets without adding unrelated arm conditions', () => {
  const params = {
    left_hand_target_joints: 'finger_l_joint20',
    right_hand_target_joints: 'finger_r_joint1',
    head_target_joints: 'head_joint2',
    lift_target_joints: 'lift_joint',
  };
  const currentPose = { finger_l_joint20: 0.5, finger_r_joint1: -0.5, head_joint2: 0.1, lift_joint: -0.4 };
  expect(poseJointNamesForNode('ArmStateGate', params, 'ffw_sh5_rev1'))
    .toEqual(['finger_l_joint20', 'finger_r_joint1', 'head_joint2', 'lift_joint']);
  expect(nodeParamsFromPose('ArmStateGate', params, currentPose, 'ffw_sh5_rev1')).toEqual({
    missing: [],
    updates: {
      left_hand_target_positions: '0.5',
      right_hand_target_positions: '-0.5',
      head_target_positions: '0.1',
      lift_target_positions: '-0.4',
    },
  });
});

test('a contact-only gate does not silently add arm joint targets when capturing', () => {
  const params = { detect_left_contact: 'true' };
  expect(poseJointNamesForNode('ArmStateGate', params, 'ffw_sh5_rev1')).toEqual([]);
  expect(nodeParamsFromPose('ArmStateGate', params, pose, 'ffw_sh5_rev1').updates).toEqual({});
});

test('captures independent closed and open finger arrays in the selected event order', () => {
  const params = {
    detect_left_hand: 'true',
    detect_right_hand: 'false',
    left_hand_event_joints: 'finger_l_joint20, finger_l_joint1',
  };
  const currentPose = { finger_l_joint1: 0.2, finger_l_joint20: 0.8 };
  expect(handEventParamsFromPose(params, currentPose, 'ffw_sh5_rev1', 'closed').updates).toEqual({
    left_hand_event_joints: 'finger_l_joint20, finger_l_joint1',
    left_hand_closed_positions: '0.8, 0.2',
  });
  expect(handEventParamsFromPose(params, currentPose, 'ffw_sh5_rev1', 'open').updates)
    .toHaveProperty('left_hand_open_positions', '0.8, 0.2');
});

test('ArmStateGate capture selects both seven-joint arms when selection is empty', () => {
  const result = nodeParamsFromPose('ArmStateGate', {
    left_target_joints: '',
    left_target_positions: '',
    right_target_joints: '',
    right_target_positions: '',
  }, pose);

  expect(result.missing).toEqual([]);
  expect(result.updates.left_target_joints).not.toContain('gripper');
  expect(result.updates.right_target_joints).not.toContain('gripper');
  expect(result.updates.left_target_positions.split(', ')).toHaveLength(7);
  expect(result.updates.right_target_positions.split(', ')).toHaveLength(7);
});

test('reports missing selected joints without writing a misaligned CSV', () => {
  const result = nodeParamsFromPose('JointControl', {
    enable_head: 'false',
    enable_arms: 'true',
    left_joint_names: 'arm_l_joint1, unknown_joint',
    right_joint_names: '',
    enable_lift: 'false',
  }, pose);

  expect(result.missing).toEqual(['unknown_joint']);
  expect(result.updates.left_positions).toBeUndefined();
});

test('maps an SH5 follower pose to independent arm and HX5 hand fields', () => {
  const sh5Pose = Object.fromEntries([
    ...SH5_LEFT_ARM_JOINTS.map((name, index) => [name, 0.1 + index]),
    ...SH5_RIGHT_ARM_JOINTS.map((name, index) => [name, -0.1 - index]),
    ...SH5_LEFT_HAND_JOINTS.map((name, index) => [name, 0.01 * index]),
    ...SH5_RIGHT_HAND_JOINTS.map((name, index) => [name, -0.01 * index]),
  ]);
  const result = nodeParamsFromPose('JointControl', {
    enable_head: 'false',
    enable_arms: 'true',
    left_joint_names: 'arm_l_joint1',
    right_joint_names: 'arm_r_joint7',
    enable_hands: 'true',
    left_hand_joint_names: 'finger_l_joint1, finger_l_joint20',
    right_hand_joint_names: 'finger_r_joint2',
    enable_lift: 'false',
  }, sh5Pose, 'ffw_sh5_rev1');

  expect(result.missing).toEqual([]);
  expect(result.updates).toEqual({
    left_positions: '0.1',
    right_positions: '-6.1',
    left_hand_positions: '0, 0.19',
    right_hand_positions: '-0.01',
  });
});

test('SH5 ArmStateGate defaults to seven arm joints without fingers', () => {
  const sh5Pose = Object.fromEntries([
    ...SH5_LEFT_ARM_JOINTS.map((name) => [name, 0]),
    ...SH5_RIGHT_ARM_JOINTS.map((name) => [name, 0]),
  ]);
  const result = nodeParamsFromPose('ArmStateGate', {
    left_target_joints: '',
    right_target_joints: '',
  }, sh5Pose, 'ffw_sh5_rev1');

  expect(result.missing).toEqual([]);
  expect(result.updates.left_target_joints.split(', ')).toHaveLength(7);
  expect(result.updates.right_target_joints.split(', ')).toHaveLength(7);
  expect(result.updates.left_target_joints).not.toContain('finger_');
});

test('SH5 gate arm choices contain seven joints per side while SG2 keeps grippers', () => {
  const sh5 = jointPoseProfileForRobot('ffw_sh5_rev1');
  expect(sh5.leftGateOptions).toEqual(SH5_LEFT_ARM_JOINTS);
  expect(sh5.rightGateOptions).toEqual(SH5_RIGHT_ARM_JOINTS);
  expect(jointPoseProfileForRobot('ffw_sg2_rev1').leftGateOptions).toContain('gripper_l_joint1');
});

test('migrates mixed SH5 arm targets to hand/head/lift groups with their exact position pairs', () => {
  const legacy = {
    left_target_joints: 'finger_l_joint20, arm_l_joint3, head_joint2, lift_joint, finger_r_joint1',
    left_target_positions: '0.8200, -0.21, 0.32, -0.45, -0.73',
    right_target_joints: 'arm_r_joint7, finger_r_joint20',
    right_target_positions: '0.11, -0.91',
    detect_left_hand: 'true', left_hand_event_joints: 'finger_l_joint20',
    left_hand_closed_positions: '1.5', left_hand_open_positions: '0.0',
  };
  const migrated = normalizeJointPoseNodeParams('ArmStateGate', legacy, 'ffw_sh5_rev1');
  expect(migrated).toMatchObject({
    left_target_joints: 'arm_l_joint3', left_target_positions: '-0.21',
    right_target_joints: 'arm_r_joint7', right_target_positions: '0.11',
    left_hand_target_joints: 'finger_l_joint20', left_hand_target_positions: '0.8200',
    right_hand_target_joints: 'finger_r_joint1, finger_r_joint20', right_hand_target_positions: '-0.73, -0.91',
    head_target_joints: 'head_joint2', head_target_positions: '0.32',
    lift_target_joints: 'lift_joint', lift_target_positions: '-0.45',
    left_hand_closed_positions: '1.5', left_hand_open_positions: '0.0',
  });
  expect(legacy.left_target_joints).toContain('finger_l_joint20');
  expect(gateTargetDiagnostics(migrated, 'ffw_sh5_rev1')).toEqual([]);
  expect(normalizeJointPoseNodeParams('ArmStateGate', migrated, 'ffw_sh5_rev1')).toEqual(migrated);
});

test('merges matching duplicate values into existing dedicated targets in their original order', () => {
  const migrated = normalizeJointPoseNodeParams('ArmStateGate', {
    left_target_joints: 'finger_l_joint1, arm_l_joint2, finger_l_joint3',
    left_target_positions: '0.1000, 0.2, 0.3',
    left_hand_target_joints: 'finger_l_joint2, finger_l_joint1',
    left_hand_target_positions: '0.22, 0.1',
  }, 'ffw_sh5_rev1');
  expect(migrated.left_target_joints).toBe('arm_l_joint2');
  expect(migrated.left_target_positions).toBe('0.2');
  expect(migrated.left_hand_target_joints).toBe('finger_l_joint2, finger_l_joint1, finger_l_joint3');
  expect(migrated.left_hand_target_positions).toBe('0.22, 0.1, 0.3');
  expect(gateTargetDiagnostics(migrated, 'ffw_sh5_rev1')).toEqual([]);
});

test('preserves and diagnoses conflicting explicit hand and legacy arm targets without choosing a value', () => {
  const migrated = normalizeJointPoseNodeParams('ArmStateGate', {
    left_target_joints: 'arm_l_joint1, finger_l_joint2, finger_l_joint3',
    left_target_positions: '0.4, 0.8, 0.9',
    left_hand_target_joints: 'finger_l_joint2', left_hand_target_positions: '0.2',
  }, 'ffw_sh5_rev1');
  expect(migrated.left_target_joints).toBe('arm_l_joint1, finger_l_joint2');
  expect(migrated.left_target_positions).toBe('0.4, 0.8');
  expect(migrated.left_hand_target_joints).toBe('finger_l_joint2, finger_l_joint3');
  expect(migrated.left_hand_target_positions).toBe('0.2, 0.9');
  expect(gateTargetDiagnostics(migrated, 'ffw_sh5_rev1').join(' '))
    .toContain('targets 0.8 in left_target_joints and 0.2 in left_hand_target_joints');
  expect(normalizeJointPoseNodeParams('ArmStateGate', migrated, 'ffw_sh5_rev1')).toEqual(migrated);
});

test('does not invent missing positions when either legacy or dedicated target CSV lengths differ', () => {
  const badSource = { left_target_joints: 'arm_l_joint1, finger_l_joint1', left_target_positions: '0.4' };
  const migrated = normalizeJointPoseNodeParams('ArmStateGate', badSource, 'ffw_sh5_rev1');
  expect(migrated).toMatchObject(badSource);
  expect(migrated.left_hand_target_joints).toBe('');
  expect(gateTargetDiagnostics(migrated, 'ffw_sh5_rev1').join(' ')).toContain('2 joints and 1 positions');
  const badDestination = {
    left_target_joints: 'finger_l_joint1', left_target_positions: '0.4',
    left_hand_target_joints: 'finger_l_joint2', left_hand_target_positions: '',
  };
  expect(normalizeJointPoseNodeParams('ArmStateGate', badDestination, 'ffw_sh5_rev1')).toMatchObject(badDestination);
});

test('preserves SG2 arm/gripper gate targets and close/open conditions', () => {
  const legacy = {
    left_target_joints: 'arm_l_joint1, gripper_l_joint1', left_target_positions: '0.4, 0.8',
    detect_left_gripper: 'true', left_gripper_joint: 'gripper_l_joint1',
    gripper_closed_value: '0.8', gripper_open_value: '0.0',
  };
  const normalized = normalizeJointPoseNodeParams('ArmStateGate', legacy, 'ffw_sg2_rev1');
  expect(normalized).toMatchObject(legacy);
  expect(gateTargetDiagnostics(normalized, 'ffw_sg2_rev1')).toEqual([]);
});
