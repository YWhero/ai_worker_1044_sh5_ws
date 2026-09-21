export const SG2_LEFT_JOINTS = [
  'arm_l_joint1',
  'arm_l_joint2',
  'arm_l_joint3',
  'arm_l_joint4',
  'arm_l_joint5',
  'arm_l_joint6',
  'arm_l_joint7',
  'gripper_l_joint1',
];

export const SG2_RIGHT_JOINTS = [
  'arm_r_joint1',
  'arm_r_joint2',
  'arm_r_joint3',
  'arm_r_joint4',
  'arm_r_joint5',
  'arm_r_joint6',
  'arm_r_joint7',
  'gripper_r_joint1',
];

export const SG2_ARM_JOINTS = [...SG2_LEFT_JOINTS, ...SG2_RIGHT_JOINTS];

export const SG2_POSE_JOINTS = [
  'head_joint1',
  'head_joint2',
  ...SG2_ARM_JOINTS,
  'lift_joint',
];

export const SH5_LEFT_ARM_JOINTS = Array.from(
  { length: 7 },
  (_, index) => `arm_l_joint${index + 1}`,
);
export const SH5_RIGHT_ARM_JOINTS = Array.from(
  { length: 7 },
  (_, index) => `arm_r_joint${index + 1}`,
);
export const SH5_LEFT_HAND_JOINTS = Array.from(
  { length: 20 },
  (_, index) => `finger_l_joint${index + 1}`,
);
export const SH5_RIGHT_HAND_JOINTS = Array.from(
  { length: 20 },
  (_, index) => `finger_r_joint${index + 1}`,
);
export const SH5_POSE_JOINTS = [
  'head_joint1',
  'head_joint2',
  ...SH5_LEFT_ARM_JOINTS,
  ...SH5_RIGHT_ARM_JOINTS,
  ...SH5_LEFT_HAND_JOINTS,
  ...SH5_RIGHT_HAND_JOINTS,
  'lift_joint',
];

const SG2_PROFILE = Object.freeze({
  leftControlJoints: SG2_LEFT_JOINTS,
  rightControlJoints: SG2_RIGHT_JOINTS,
  leftHandJoints: [],
  rightHandJoints: [],
  leftGateJoints: SG2_LEFT_JOINTS.slice(0, 7),
  rightGateJoints: SG2_RIGHT_JOINTS.slice(0, 7),
  leftGateOptions: SG2_LEFT_JOINTS,
  rightGateOptions: SG2_RIGHT_JOINTS,
  poseJoints: SG2_POSE_JOINTS,
  stateTopics: [
    { name: '/joint_states', type: 'sensor_msgs/msg/JointState' },
  ],
});

const SH5_PROFILE = Object.freeze({
  leftControlJoints: SH5_LEFT_ARM_JOINTS,
  rightControlJoints: SH5_RIGHT_ARM_JOINTS,
  leftHandJoints: SH5_LEFT_HAND_JOINTS,
  rightHandJoints: SH5_RIGHT_HAND_JOINTS,
  leftGateJoints: SH5_LEFT_ARM_JOINTS,
  rightGateJoints: SH5_RIGHT_ARM_JOINTS,
  leftGateOptions: SH5_LEFT_ARM_JOINTS,
  rightGateOptions: SH5_RIGHT_ARM_JOINTS,
  poseJoints: SH5_POSE_JOINTS,
  stateTopics: [
    { name: '/joint_states', type: 'sensor_msgs/msg/JointState' },
    { name: '/arm_hand/joint_states', type: 'sensor_msgs/msg/JointState' },
  ],
});

export function normalizedPoseRobotType(robotType) {
  const value = String(robotType || '').trim();
  if (!value || value === 'ffw_sg2') return 'ffw_sg2_rev1';
  if (value === 'ffw_sh5') return 'ffw_sh5_rev1';
  return value;
}

export function jointPoseProfileForRobot(robotType) {
  const normalized = normalizedPoseRobotType(robotType).toLowerCase();
  return normalized.includes('sh5') || normalized.includes('hx5') ? SH5_PROFILE : SG2_PROFILE;
}

export function isHandPoseRobot(robotType) {
  return jointPoseProfileForRobot(robotType) === SH5_PROFILE;
}

export const GATE_TARGET_FIELDS = [
  ['left_target_joints', 'left_target_positions'],
  ['right_target_joints', 'right_target_positions'],
  ['left_hand_target_joints', 'left_hand_target_positions'],
  ['right_hand_target_joints', 'right_hand_target_positions'],
  ['head_target_joints', 'head_target_positions'],
  ['lift_target_joints', 'lift_target_positions'],
];

const SH5_GATE_GROUPS = [
  SH5_LEFT_ARM_JOINTS, SH5_RIGHT_ARM_JOINTS,
  SH5_LEFT_HAND_JOINTS, SH5_RIGHT_HAND_JOINTS,
  ['head_joint1', 'head_joint2'], ['lift_joint'],
];

function sameTargetPosition(left, right) {
  if (left === right) return true;
  return Number.isFinite(Number(left)) && Number.isFinite(Number(right))
    && Number(left) === Number(right);
}

// Older SH5 gates selected fingers in the arm target fields. Move only
// complete name/value pairs. Explicit dedicated targets take precedence;
// conflicting legacy pairs remain intact for diagnosis and manual resolution.
function migrateMixedGateTargets(params) {
  const destinationByJoint = new Map(SH5_GATE_GROUPS.slice(2).flatMap((names, index) => (
    names.map((name) => [name, GATE_TARGET_FIELDS[index + 2]])
  )));
  GATE_TARGET_FIELDS.slice(0, 2).forEach(([namesKey, positionsKey]) => {
    const names = csvParts(params[namesKey]);
    const positions = csvParts(params[positionsKey]);
    if (names.length !== positions.length) return;
    const retained = [];
    let moved = false;
    names.forEach((joint, index) => {
      const destination = destinationByJoint.get(joint);
      if (!destination) {
        retained.push(index);
        return;
      }
      const [targetNamesKey, targetPositionsKey] = destination;
      const targetNames = csvParts(params[targetNamesKey]);
      const targetPositions = csvParts(params[targetPositionsKey]);
      const existing = targetNames.indexOf(joint);
      if (targetNames.length !== targetPositions.length || (existing !== -1
          && !sameTargetPosition(targetPositions[existing], positions[index]))) {
        retained.push(index);
        return;
      }
      if (existing === -1) {
        params[targetNamesKey] = [...targetNames, joint].join(', ');
        params[targetPositionsKey] = [...targetPositions, positions[index]].join(', ');
      }
      moved = true;
    });
    if (moved) {
      params[namesKey] = retained.map((index) => names[index]).join(', ');
      params[positionsKey] = retained.map((index) => positions[index]).join(', ');
    }
  });
}

export function gateTargetDiagnostics(params = {}, robotType = '') {
  const profile = jointPoseProfileForRobot(robotType);
  const groups = [profile.leftGateOptions, profile.rightGateOptions,
    profile.leftHandJoints, profile.rightHandJoints, ['head_joint1', 'head_joint2'], ['lift_joint']];
  const messages = [];
  const targets = new Map();
  GATE_TARGET_FIELDS.forEach(([namesKey, positionsKey], groupIndex) => {
    const names = csvParts(params[namesKey]);
    const positions = csvParts(params[positionsKey]);
    if (names.length !== positions.length) {
      messages.push(`${namesKey}: ${names.length} joints and ${positions.length} positions. Values were preserved; fix the CSV lengths before running.`);
    }
    names.forEach((joint, index) => {
      if (targets.has(joint)) {
        const previous = targets.get(joint);
        messages.push(`${joint}: targets ${previous.position ?? '(missing)'} in ${previous.namesKey} and ${positions[index] ?? '(missing)'} in ${namesKey}. Both values were preserved; keep one target before running.`);
      } else {
        targets.set(joint, { namesKey, position: positions[index] });
      }
      if (!groups[groupIndex].includes(joint)) {
        messages.push(`${joint} does not belong to ${namesKey}. Move its name and position together to the matching target group.`);
      }
    });
  });
  return messages;
}

// The live catalog and saved tasks may still contain SG2 defaults. Remove
// only its two known gripper joints when adapting to SH5, keeping joint and
// position CSVs aligned. Unknown custom joints remain visible for diagnosis.
export function normalizeJointPoseNodeParams(nodeType, params = {}, robotType = '') {
  const profile = jointPoseProfileForRobot(robotType);
  const handRobot = isHandPoseRobot(robotType);
  const next = { ...params };

  const removeLegacyGrippers = (namesKey, positionsKey) => {
    const names = csvParts(next[namesKey]);
    const positions = csvParts(next[positionsKey]);
    const keep = names.map((name, index) => ({ name, index })).filter(
      ({ name }) => name !== 'gripper_l_joint1' && name !== 'gripper_r_joint1',
    );
    if (keep.length === names.length) return;
    next[namesKey] = keep.map(({ name }) => name).join(', ');
    if (positions.length === names.length) {
      next[positionsKey] = keep.map(({ index }) => positions[index]).join(', ');
    }
  };

  if (nodeType === 'JointControl') {
    [
      ['left_joint_names', 'left_positions', profile.leftControlJoints],
      ['right_joint_names', 'right_positions', profile.rightControlJoints],
    ].forEach(([namesKey, positionsKey, names]) => {
      if (!(namesKey in next)) {
        // Legacy FFW arm arrays include the SG2 gripper as the eighth value.
        const values = csvParts(next[positionsKey]);
        if (handRobot && values.length === 8) next[positionsKey] = values.slice(0, 7).join(', ');
        if (positionsKey in next || handRobot) next[namesKey] = names.join(', ');
      }
      if (handRobot) {
        removeLegacyGrippers(namesKey, positionsKey);
        if (!(positionsKey in next)) next[positionsKey] = names.map(() => '0.0').join(', ');
      }
    });
    if (handRobot) {
      if (!('enable_hands' in next)) next.enable_hands = 'false';
      [
        ['left_hand_joint_names', 'left_hand_positions', profile.leftHandJoints],
        ['right_hand_joint_names', 'right_hand_positions', profile.rightHandJoints],
      ].forEach(([namesKey, positionsKey, names]) => {
        if (!(namesKey in next)) next[namesKey] = names.join(', ');
        if (!(positionsKey in next)) next[positionsKey] = names.map(() => '0.0').join(', ');
      });
    }
  }

  if (nodeType === 'ArmStateGate') {
    GATE_TARGET_FIELDS.forEach(([namesKey, positionsKey]) => {
      if (!handRobot && namesKey.includes('hand')) return;
      if (!(namesKey in next)) next[namesKey] = '';
      if (!(positionsKey in next)) next[positionsKey] = '';
      if (handRobot) removeLegacyGrippers(namesKey, positionsKey);
    });
    if (handRobot) migrateMixedGateTargets(next);
    if (!('hold_sec' in next)) next.hold_sec = '0.0';
    if (!('state_max_age_sec' in next)) next.state_max_age_sec = '1.0';
    if (handRobot) {
      // A gripper scalar cannot describe a twenty-joint HX5 hand.
      next.detect_left_gripper = 'false';
      next.detect_right_gripper = 'false';
      ['left', 'right'].forEach((side) => {
        if (!(`detect_${side}_hand` in next)) next[`detect_${side}_hand`] = 'false';
        if (!(`detect_${side}_contact` in next)) next[`detect_${side}_contact`] = 'false';
        ['hand_event_joints', 'hand_closed_positions', 'hand_open_positions', 'contact_sensor_names'].forEach((field) => {
          if (!(`${side}_${field}` in next)) next[`${side}_${field}`] = '';
        });
        if (!(`${side}_contact_condition` in next)) next[`${side}_contact_condition`] = 'contact';
      });
      if (!('hand_threshold' in next)) next.hand_threshold = '0.05';
      if (!('contact_pressure_threshold' in next)) next.contact_pressure_threshold = '30.0';
      if (!('contact_min_sensors' in next)) next.contact_min_sensors = '1';
    }
  }
  return next;
}

export function csvParts(value) {
  return String(value || '')
    .split(',')
    .map((part) => part.trim())
    .filter(Boolean);
}

function enabled(value) {
  return value === true || value === 'true';
}

function formatPosition(value) {
  return Number(value).toFixed(6).replace(/\.?0+$/, '') || '0';
}

function positionsFor(jointNames, pose) {
  const missing = jointNames.filter((name) => !Number.isFinite(pose[name]));
  if (missing.length > 0) return { value: '', missing };
  return {
    value: jointNames.map((name) => formatPosition(pose[name])).join(', '),
    missing: [],
  };
}

function applyJointGroup(updates, missing, params, pose, namesKey, valuesKey) {
  const names = csvParts(params[namesKey]);
  const values = positionsFor(names, pose);
  if (names.length > 0 && values.missing.length === 0) {
    updates[valuesKey] = values.value;
  }
  missing.push(...values.missing);
}

export function poseJointNamesForNode(nodeType, params, robotType = '') {
  const profile = jointPoseProfileForRobot(robotType);
  const names = [];
  if (nodeType === 'JointControl') {
    if (enabled(params.enable_head)) names.push('head_joint1', 'head_joint2');
    if (enabled(params.enable_lift)) names.push('lift_joint');
    if (enabled(params.enable_arms)) {
      names.push(...csvParts(params.left_joint_names), ...csvParts(params.right_joint_names));
    }
    if (enabled(params.enable_hands)) {
      names.push(...csvParts(params.left_hand_joint_names), ...csvParts(params.right_hand_joint_names));
    }
  }
  if (nodeType === 'ArmStateGate') {
    GATE_TARGET_FIELDS.forEach(([namesKey]) => names.push(...csvParts(params[namesKey])));
    if (names.length === 0 && !gateHasEventConditions(params)) {
      names.push(...profile.leftGateJoints, ...profile.rightGateJoints);
    }
  }
  return [...new Set(names)];
}

export function nodeParamsFromPose(nodeType, params, pose, robotType = '') {
  const profile = jointPoseProfileForRobot(robotType);
  const updates = {};
  const missing = [];

  if (nodeType === 'JointControl') {
    if (enabled(params.enable_head)) {
      const head = positionsFor(['head_joint1', 'head_joint2'], pose);
      if (head.missing.length === 0) updates.head_positions = head.value;
      missing.push(...head.missing);
    }

    if (enabled(params.enable_arms)) {
      applyJointGroup(
        updates, missing, params, pose, 'left_joint_names', 'left_positions',
      );
      applyJointGroup(
        updates, missing, params, pose, 'right_joint_names', 'right_positions',
      );
    }

    if (enabled(params.enable_hands)) {
      applyJointGroup(
        updates, missing, params, pose,
        'left_hand_joint_names', 'left_hand_positions',
      );
      applyJointGroup(
        updates, missing, params, pose,
        'right_hand_joint_names', 'right_hand_positions',
      );
    }

    if (enabled(params.enable_lift)) {
      if (Number.isFinite(pose.lift_joint)) {
        updates.lift_position = formatPosition(pose.lift_joint);
      } else {
        missing.push('lift_joint');
      }
    }
  }

  if (nodeType === 'ArmStateGate') {
    const selected = GATE_TARGET_FIELDS.some(([namesKey]) => csvParts(params[namesKey]).length > 0);
    const targetParams = { ...params };
    if (!selected && !gateHasEventConditions(params)) {
      targetParams.left_target_joints = profile.leftGateJoints.join(', ');
      targetParams.right_target_joints = profile.rightGateJoints.join(', ');
      updates.left_target_joints = targetParams.left_target_joints;
      updates.right_target_joints = targetParams.right_target_joints;
    }
    GATE_TARGET_FIELDS.forEach(([namesKey, positionsKey]) => {
      applyJointGroup(updates, missing, targetParams, pose, namesKey, positionsKey);
    });
  }

  return {
    updates,
    missing: [...new Set(missing)],
  };
}

function gateHasEventConditions(params) {
  return [
    'detect_left_hand', 'detect_right_hand', 'detect_left_contact',
    'detect_right_contact', 'detect_left_gripper', 'detect_right_gripper',
  ].some((key) => enabled(params[key]));
}

export function handEventJointNames(params, robotType) {
  const profile = jointPoseProfileForRobot(robotType);
  return ['left', 'right'].flatMap((side) => {
    if (!enabled(params[`detect_${side}_hand`])) return [];
    const selected = csvParts(params[`${side}_hand_event_joints`]);
    return selected.length ? selected : profile[`${side}HandJoints`];
  });
}

export function handEventParamsFromPose(params, pose, robotType, event = 'closed') {
  const profile = jointPoseProfileForRobot(robotType);
  const updates = {};
  const missing = [];
  ['left', 'right'].forEach((side) => {
    if (!enabled(params[`detect_${side}_hand`])) return;
    const namesKey = `${side}_hand_event_joints`;
    const names = csvParts(params[namesKey]);
    const selected = names.length ? names : profile[`${side}HandJoints`];
    const positions = positionsFor(selected, pose);
    missing.push(...positions.missing);
    if (selected.length && positions.missing.length === 0) {
      updates[namesKey] = selected.join(', ');
      updates[`${side}_hand_${event}_positions`] = positions.value;
    }
  });
  return { updates, missing: [...new Set(missing)] };
}
