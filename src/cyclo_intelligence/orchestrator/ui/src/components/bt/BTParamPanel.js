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

import React, { useState, useEffect, useMemo, useRef } from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { MdClose, MdFolderOpen } from 'react-icons/md';
import FileBrowserModal from '../FileBrowserModal';
import JointPosePresetControls from './JointPosePresetControls';
import { setSelectedNodeId } from '../../features/actionCanvas/actionCanvasSlice';
import {
  csvParts,
  GATE_TARGET_FIELDS,
  gateTargetDiagnostics,
  isHandPoseRobot,
  jointPoseProfileForRobot,
  normalizeJointPoseNodeParams,
} from '../../features/actionCanvas/jointPoseUtils';
import { DEFAULT_PATHS } from '../../constants/paths';

const NUMBER_PARAMS = new Set([
  'duration', 'angle_deg', 'lift_position', 'control_hz', 'inference_hz',
  'chunk_align_window_s', 'max_iterations', 'joint_threshold',
  'gripper_closed_value', 'gripper_open_value', 'gripper_threshold',
  'timeout_sec', 'hand_threshold', 'contact_pressure_threshold',
  'contact_min_sensors', 'hold_sec', 'state_max_age_sec',
  'initial_pose_sync_duration_s',
]);

// joint-selection param → its paired positions param. Toggling a joint chip
// keeps the two CSVs aligned (new joints get 0.0, removed joints drop theirs).
const TARGET_POSITION_PARAM = {
  left_target_joints: 'left_target_positions',
  right_target_joints: 'right_target_positions',
  left_joint_names: 'left_positions',
  right_joint_names: 'right_positions',
  left_hand_joint_names: 'left_hand_positions',
  right_hand_joint_names: 'right_hand_positions',
  left_hand_target_joints: 'left_hand_target_positions',
  right_hand_target_joints: 'right_hand_target_positions',
  head_target_joints: 'head_target_positions',
  lift_target_joints: 'lift_target_positions',
  left_hand_event_joints: 'left_hand_closed_positions',
  right_hand_event_joints: 'right_hand_closed_positions',
};

// Node types whose joint params render as the chip selector.
const JOINT_SELECTOR_NODE_TYPES = new Set(['ArmStateGate', 'JointControl']);

const GATE_PARAM_SECTIONS = [
  ['Arm targets', ['left_target_joints', 'left_target_positions', 'right_target_joints', 'right_target_positions']],
  ['Hand targets', ['left_hand_target_joints', 'left_hand_target_positions', 'right_hand_target_joints', 'right_hand_target_positions']],
  ['Head targets', ['head_target_joints', 'head_target_positions']],
  ['Lift target', ['lift_target_joints', 'lift_target_positions']],
  ['Hand close/open events', ['detect_left_hand', 'left_hand_event_joints', 'left_hand_closed_positions', 'left_hand_open_positions', 'detect_right_hand', 'right_hand_event_joints', 'right_hand_closed_positions', 'right_hand_open_positions', 'hand_threshold']],
  ['Contact conditions', ['detect_left_contact', 'left_contact_sensor_names', 'left_contact_condition', 'detect_right_contact', 'right_contact_sensor_names', 'right_contact_condition', 'contact_pressure_threshold', 'contact_min_sensors']],
  ['Gripper close/open events', ['detect_left_gripper', 'left_gripper_joint', 'detect_right_gripper', 'right_gripper_joint', 'gripper_closed_value', 'gripper_open_value', 'gripper_threshold']],
  ['State and timing', ['joint_threshold', 'hold_sec', 'state_max_age_sec', 'timeout_sec']],
];

const GATE_TARGET_PARAM_KEYS = new Set(GATE_TARGET_FIELDS.flat());

// Nodes created before the per-joint params existed (or from a stale bt_node
// catalog) lack left/right_joint_names entirely, so the selector would never
// render for them. Synthesize the params with the full joint list — exactly
// what the engine does when the names are omitted — inserting each one just
// before its positions field so the selector renders above it.
function withJointSelectionDefaults(params, nodeType, robotType) {
  const normalized = normalizeJointPoseNodeParams(nodeType, params, robotType);
  if (nodeType !== 'JointControl') return normalized;
  const profile = jointPoseProfileForRobot(robotType);
  const next = {};
  const insertBefore = {
    left_positions: ['left_joint_names', profile.leftControlJoints.join(', ')],
    right_positions: ['right_joint_names', profile.rightControlJoints.join(', ')],
    left_hand_positions: [
      'left_hand_joint_names', profile.leftHandJoints.join(', '),
    ],
    right_hand_positions: [
      'right_hand_joint_names', profile.rightHandJoints.join(', '),
    ],
  };
  Object.entries(normalized).forEach(([key, value]) => {
    const missing = insertBefore[key];
    if (missing && !(missing[0] in next)) next[missing[0]] = normalized[missing[0]] ?? missing[1];
    if (!(key in next)) next[key] = value;
  });
  return next;
}

function getTargetJointOptions(robotType, key) {
  const profile = jointPoseProfileForRobot(robotType);
  const options = {
    left_target_joints: profile.leftGateOptions,
    right_target_joints: profile.rightGateOptions,
    left_joint_names: profile.leftControlJoints,
    right_joint_names: profile.rightControlJoints,
    left_hand_joint_names: profile.leftHandJoints,
    right_hand_joint_names: profile.rightHandJoints,
    left_hand_target_joints: profile.leftHandJoints,
    right_hand_target_joints: profile.rightHandJoints,
    left_hand_event_joints: profile.leftHandJoints,
    right_hand_event_joints: profile.rightHandJoints,
    head_target_joints: ['head_joint1', 'head_joint2'],
    lift_target_joints: ['lift_joint'],
  };
  return options[key] || [];
}

// Per-param helper text shown beneath the input. Keep these short — they
// render directly under the field as a small gray hint.
const HELP_TEXT = {
  max_iterations: '0 = loop forever',
  left_hand_target_positions: 'Radians, in the selected left finger joint order.',
  right_hand_target_positions: 'Radians, in the selected right finger joint order.',
  detect_left_hand: 'Wait for the selected left fingers to close, then reopen.',
  detect_right_hand: 'Wait for the selected right fingers to close, then reopen.',
  detect_left_contact: 'Require fresh left HandPressures feedback.',
  detect_right_contact: 'Require fresh right HandPressures feedback.',
  left_contact_sensor_names: 'Comma-separated sensor_name values; empty selects all five fingertip sensors.',
  right_contact_sensor_names: 'Comma-separated sensor_name values; empty selects all five fingertip sensors.',
  contact_pressure_threshold: 'Raw pressure sum per sensor. Tune to the simulated or physical sensor baseline.',
  contact_min_sensors: 'Minimum number of selected sensors satisfying the contact condition.',
  hold_sec: 'All enabled conditions must remain true for this duration.',
  state_max_age_sec: 'Reject joint and pressure feedback older than this duration.',
  initial_pose_sync: 'LOAD: align with the model starting pose before inference resumes.',
  initial_pose_sync_duration_s: 'Starting pose movement duration: 1–60 seconds.',
};

// Boolean params render as checkboxes.
const BOOL_PARAMS = new Set([
  'enable_head',
  'enable_arms',
  'enable_hands',
  'enable_lift',
  'detect_left_gripper',
  'detect_right_gripper',
  'detect_left_hand',
  'detect_right_hand',
  'detect_left_contact',
  'detect_right_contact',
  'initial_pose_sync',
]);

const SEND_COMMAND_TARGETS = ['INFERENCE', 'DOCKER'];

const SEND_COMMAND_COMMANDS_BY_TARGET = {
  INFERENCE: ['LOAD', 'RESUME', 'STOP', 'CLEAR'],
  DOCKER: ['START', 'STOP', 'RESTART'],
};

function normalizeSendCommandTarget(target) {
  return String(target || 'INFERENCE').trim().toUpperCase() === 'DOCKER'
    ? 'DOCKER'
    : 'INFERENCE';
}

function normalizeSendCommandParams(params, nodeType, robotType) {
  const next = withJointSelectionDefaults(params, nodeType, robotType);
  if (nodeType !== 'SendCommand') return next;

  const target = normalizeSendCommandTarget(next.target);
  const commands = SEND_COMMAND_COMMANDS_BY_TARGET[target];
  const requestedCommand = String(next.command || commands[0]).trim().toUpperCase();
  const command = commands.includes(requestedCommand) ? requestedCommand : commands[0];
  const remaining = Object.fromEntries(
    Object.entries(next).filter(([key]) => key !== 'target' && key !== 'command'),
  );
  if (!('initial_pose_sync' in remaining)) remaining.initial_pose_sync = 'false';
  if (!('initial_pose_sync_duration_s' in remaining)) remaining.initial_pose_sync_duration_s = '5.0';
  return { target, command, ...remaining };
}

// Enum params surface as <select> dropdowns. Keep value lists in sync with
// the Python action definitions (send_command.COMMAND_MAP).
const ENUM_PARAMS = {
  model: [
    'lerobot:act',
    'lerobot:tactile_act',
    'vitacformer:vitacformer',
    'lerobot:diffusion',
    'lerobot:smolvla',
    'lerobot:xvla',
    'lerobot:pi0',
    'lerobot:pi05',
    'lerobot:trex',
    'lerobot:fastwam',
    'groot:n17',
    'groot',
    'lerobot',
  ],
  inference_mode: ['simulation', 'robot'],
  action_request_mode: ['async', 'async_ordered', 'sync', 'sync_step'],
  acceleration_mode: ['pytorch', 'tensorrt_dit'],
  left_contact_condition: ['contact', 'released'],
  right_contact_condition: ['contact', 'released'],
};

function enumOptionsFor(nodeType, key, params) {
  if (nodeType === 'SendCommand') {
    if (key === 'target') return SEND_COMMAND_TARGETS;
    if (key === 'command') {
      return SEND_COMMAND_COMMANDS_BY_TARGET[
        normalizeSendCommandTarget(params.target)
      ];
    }
  }
  return ENUM_PARAMS[key];
}

// SendCommand inputs that are meaningful per command. Anything outside
// the set for the current command is rendered disabled — the value stays
// in params so flipping back to LOAD restores the user's earlier entries.
// 'command' itself is always editable.
const SEND_COMMAND_ACTIVE_FIELDS = {
  LOAD: new Set([
    'target', 'command', 'model', 'policy_path', 'task_instruction',
    'inference_mode', 'action_request_mode', 'inference_hz', 'control_hz',
    'chunk_align_window_s', 'acceleration_mode', 'acceleration_engine_path',
    'initial_pose_sync', 'initial_pose_sync_duration_s',
  ]),
  // Resume can re-condition language mid-run; output mode is fixed by LOAD.
  RESUME: new Set(['target', 'command', 'task_instruction']),
  STOP: new Set(['target', 'command']),
  CLEAR: new Set(['target', 'command']),
};

const SEND_COMMAND_DOCKER_ACTIVE_FIELDS = new Set(['target', 'command', 'model']);

// JointControl: each group's positions input is gated on its enable_*
// flag. enable_* toggles themselves + duration are always editable.
const truthy = (v) => v === true || v === 'true';

function isSendCommandFieldDisabled(nodeType, key, params) {
  if (nodeType !== 'SendCommand') return false;
  if (normalizeSendCommandTarget(params.target) === 'DOCKER') {
    return !SEND_COMMAND_DOCKER_ACTIVE_FIELDS.has(key);
  }
  const cmd = String(params.command || 'LOAD').toUpperCase();
  const active = SEND_COMMAND_ACTIVE_FIELDS[cmd];
  if (!active) return false;
  return !active.has(key);
}

function isJointControlFieldDisabled(nodeType, key, params) {
  if (nodeType !== 'JointControl') return false;
  if (key === 'head_positions') return !truthy(params.enable_head);
  if (
    key === 'left_positions' || key === 'right_positions'
    || key === 'left_joint_names' || key === 'right_joint_names'
  ) {
    return !truthy(params.enable_arms);
  }
  if (
    key === 'left_hand_positions' || key === 'right_hand_positions'
    || key === 'left_hand_joint_names' || key === 'right_hand_joint_names'
  ) {
    return !truthy(params.enable_hands);
  }
  if (key === 'lift_position') return !truthy(params.enable_lift);
  return false;  // enable_*, duration stay editable
}

function isArmStateGateFieldDisabled(nodeType, key, params) {
  if (nodeType !== 'ArmStateGate') return false;
  if (key === 'left_gripper_joint') {
    return !truthy(params.detect_left_gripper);
  }
  if (key === 'right_gripper_joint') {
    return !truthy(params.detect_right_gripper);
  }
  for (const side of ['left', 'right']) {
    if (key.startsWith(`${side}_hand_`) && !key.includes('_target_')) {
      return !truthy(params[`detect_${side}_hand`]);
    }
    if (key.startsWith(`${side}_contact_`)) {
      return !truthy(params[`detect_${side}_contact`]);
    }
  }
  if (key === 'hand_threshold') {
    return !truthy(params.detect_left_hand) && !truthy(params.detect_right_hand);
  }
  if (key === 'contact_pressure_threshold' || key === 'contact_min_sensors') {
    return !truthy(params.detect_left_contact) && !truthy(params.detect_right_contact);
  }
  return false;
}

function isFieldDisabled(nodeType, key, params) {
  return (
    isSendCommandFieldDisabled(nodeType, key, params) ||
    isJointControlFieldDisabled(nodeType, key, params) ||
    isArmStateGateFieldDisabled(nodeType, key, params)
  );
}

export default function BTParamPanel({
  nodes,
  selectedNodeId,
  onParamChange,
  onNameChange,
  onClose,
  variant = 'legacy',
}) {
  const dispatch = useDispatch();
  const robotType = useSelector((state) => state.tasks?.robotType || '');

  const selectedNode = nodes.find((n) => n.id === selectedNodeId);

  // Local param state — isolates keystrokes from parent re-renders (preserves cursor)
  const [localParams, setLocalParams] = useState({});
  // Local name buffer — same cursor-preservation trick as localParams.
  const [localName, setLocalName] = useState('');
  const nameAtFocusRef = useRef('');
  const suppressNextNameBlurRef = useRef(false);
  const [showPolicyBrowser, setShowPolicyBrowser] = useState(false);

  const policyBrowserPath = useMemo(() => {
    const model = String(localParams.model || '').toLowerCase();
    if (model.startsWith('groot')) return DEFAULT_PATHS.GROOT_CHECKPOINTS_PATH;
    if (model.includes('vitacformer')) {
      return DEFAULT_PATHS.VITACFORMER_CHECKPOINTS_PATH;
    }
    return DEFAULT_PATHS.LEROBOT_CHECKPOINTS_PATH;
  }, [localParams.model]);

  // Reset local state only when switching to a different node
  useEffect(() => {
    if (selectedNode) {
      const originalParams = selectedNode.data.params || {};
      const normalizedParams = normalizeSendCommandParams(
        originalParams,
        selectedNode.data.nodeType,
        robotType,
      );
      setLocalParams(normalizedParams);
      // Persist migrations of existing values, so a loaded SG2 task cannot
      // silently publish nonexistent gripper joints to an SH5 controller.
      Object.entries(normalizedParams).forEach(([key, value]) => {
        const migratedTarget = selectedNode.data.nodeType === 'ArmStateGate'
          && GATE_TARGET_PARAM_KEYS.has(key) && value !== '';
        if ((key in originalParams || migratedTarget) && value !== originalParams[key]) {
          onParamChange(selectedNodeId, key, value);
        }
      });
      setLocalName(selectedNode.data.label || '');
    }
    setShowPolicyBrowser(false);
    // Keep mid-edit cursor position stable; reset only when the selection changes.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [selectedNodeId, robotType]); // intentionally excludes selectedNode to avoid resetting mid-edit

  if (!selectedNode) return null;

  const { label, nodeType } = selectedNode.data;
  const paramEntries = Object.entries(localParams).filter(([key]) => {
    if (nodeType !== 'ArmStateGate' || !isHandPoseRobot(robotType)) return true;
    return !key.includes('gripper');
  });
  const gateDiagnostics = nodeType === 'ArmStateGate'
    ? gateTargetDiagnostics(localParams, robotType) : [];

  const commitName = () => {
    if (suppressNextNameBlurRef.current) {
      suppressNextNameBlurRef.current = false;
      return;
    }
    const trimmed = localName.trim();
    if (!trimmed) {
      // Reject empty — snap input back to current label.
      setLocalName(label);
      return;
    }
    if (trimmed !== label) {
      onNameChange?.(selectedNodeId, trimmed);
    }
  };

  const handleChange = (paramName, value) => {
    if (nodeType === 'SendCommand' && paramName === 'target') {
      const target = normalizeSendCommandTarget(value);
      const commands = SEND_COMMAND_COMMANDS_BY_TARGET[target];
      const currentCommand = String(localParams.command || '').trim().toUpperCase();
      const command = commands.includes(currentCommand) ? currentCommand : commands[0];
      setLocalParams((prev) => ({ ...prev, target, command }));
      onParamChange(selectedNodeId, 'target', target);
      if (command !== currentCommand) {
        onParamChange(selectedNodeId, 'command', command);
      }
      return;
    }
    setLocalParams((prev) => ({ ...prev, [paramName]: value }));
    // A Design Save click can happen before a blur-driven graph update reaches
    // the mission state. Keep the graph current while the field is focused so
    // the visible value is always the value that gets serialized.
    onParamChange(selectedNodeId, paramName, value);
  };

  const commitParam = (paramName, value) => {
    setLocalParams((prev) => ({ ...prev, [paramName]: value }));
    onParamChange(selectedNodeId, paramName, value);
  };

  const commitParams = (updates) => {
    setLocalParams((prev) => ({ ...prev, ...updates }));
    Object.entries(updates).forEach(([paramName, value]) => {
      onParamChange(selectedNodeId, paramName, value);
    });
  };

  const handlePolicyFolderSelect = (item) => {
    const fullPath = item?.full_path || '';
    if (fullPath) {
      commitParam('policy_path', fullPath);
    }
    setShowPolicyBrowser(false);
  };

  const commitTargetJointSelection = (paramName, nextJoints) => {
    const positionsParam = TARGET_POSITION_PARAM[paramName];
    const currentJoints = csvParts(localParams[paramName]);
    const currentPositions = csvParts(localParams[positionsParam]);
    const positionByJoint = new Map(
      currentJoints.map((joint, idx) => [
        joint,
        currentPositions[idx] || '0.0',
      ])
    );
    const nextPositions = nextJoints.map(
      (joint) => positionByJoint.get(joint) || '0.0'
    );
    const jointsValue = nextJoints.join(', ');
    const positionsValue = nextPositions.join(', ');

    const updates = { [paramName]: jointsValue, [positionsParam]: positionsValue };
    if (paramName.endsWith('_hand_event_joints')) {
      const openParam = paramName.replace('_event_joints', '_open_positions');
      const openPositions = csvParts(localParams[openParam]);
      const openByJoint = new Map(currentJoints.map((joint, index) => [joint, openPositions[index] || '0.0']));
      updates[openParam] = nextJoints.map((joint) => openByJoint.get(joint) || '0.0').join(', ');
    }
    commitParams(updates);
  };

  const renderTargetJointSelector = (key, value, disabled = false) => {
    const options = getTargetJointOptions(robotType, key);
    const selected = csvParts(value);
    const selectedSet = new Set(selected);
    const disabledCls = disabled
      ? ' !bg-[var(--mc-surface-hover)] !text-[var(--mc-text-subtle)] cursor-not-allowed'
      : '';
    const jointTextarea = (
      <textarea
        value={value}
        disabled={disabled}
        onChange={(e) => handleChange(key, e.target.value)}
        rows={String(value).length > 60 ? 3 : 1}
        className={`w-full px-2 py-1.5 border border-[var(--mc-border-strong)] rounded-lg text-sm bg-[var(--mc-surface)] text-[var(--mc-text)] focus:outline-none focus:ring-1 focus:ring-[var(--mc-accent)] resize-y${disabledCls}`}
      />
    );

    if (options.length === 0) {
      return jointTextarea;
    }

    return (
      <div className="space-y-2">
        <div className="flex flex-wrap gap-1.5">
          {options.map((jointName) => {
            const isSelected = selectedSet.has(jointName);
            return (
              <button
                key={jointName}
                type="button"
                disabled={disabled}
                aria-pressed={isSelected}
                onClick={() => {
                  const nextJoints = isSelected
                    ? selected.filter((joint) => joint !== jointName)
                    : [
                        ...options.filter((joint) => joint === jointName || selectedSet.has(joint)),
                        // Keep unresolved/custom entries when an unrelated
                        // chip changes; only an explicit edit removes them.
                        ...selected.filter((joint) => !options.includes(joint)),
                      ];
                  commitTargetJointSelection(key, nextJoints);
                }}
                className={`px-2 py-1 border rounded-lg text-xs transition-colors ${
                  isSelected
                    ? 'border-[var(--mc-accent)] bg-[var(--mc-accent-soft)] text-[var(--mc-accent)]'
                    : 'border-[var(--mc-border-strong)] bg-[var(--mc-surface)] text-[var(--mc-text-muted)] hover:bg-[var(--mc-surface-hover)]'
                } ${disabled ? 'opacity-50 cursor-not-allowed' : ''}`}
              >
                {jointName}
              </button>
            );
          })}
        </div>
        {jointTextarea}
      </div>
    );
  };

  const renderInput = (key, value, disabled = false) => {
    const disabledCls = disabled
      ? ' !bg-[var(--mc-surface-hover)] !text-[var(--mc-text-subtle)] cursor-not-allowed'
      : '';

    if (JOINT_SELECTOR_NODE_TYPES.has(nodeType) && TARGET_POSITION_PARAM[key]) {
      return renderTargetJointSelector(key, value, disabled);
    }

    const enumOptions = enumOptionsFor(nodeType, key, localParams);
    if (enumOptions) {
      return (
        <select
          value={value}
          disabled={disabled}
          onChange={(e) => handleChange(key, e.target.value)}
          className={`w-full px-2 py-1.5 border border-[var(--mc-border-strong)] rounded-lg text-sm bg-[var(--mc-surface)] text-[var(--mc-text)] focus:outline-none focus:ring-1 focus:ring-[var(--mc-accent)]${disabledCls}`}
        >
          {enumOptions.map((opt) => (
            <option key={opt} value={opt}>{opt}</option>
          ))}
        </select>
      );
    }

    if (BOOL_PARAMS.has(key)) {
      return (
        <label className={`flex items-center gap-2 ${disabled ? 'cursor-not-allowed text-[var(--mc-text-subtle)]' : 'cursor-pointer'}`}>
          <input
            type="checkbox"
            disabled={disabled}
            checked={value === 'true' || value === true}
            onChange={(e) => {
              const v = e.target.checked ? 'true' : 'false';
              handleChange(key, v);
            }}
            className="w-4 h-4 rounded border-[var(--mc-border-strong)] text-[var(--mc-accent)] focus:ring-[var(--mc-accent)]"
          />
          <span className="text-sm text-[var(--mc-text-muted)]">{value === 'true' || value === true ? 'true' : 'false'}</span>
        </label>
      );
    }

    if (nodeType === 'SendCommand' && key === 'policy_path') {
      return (
        <div className="flex flex-row items-start gap-2">
          <textarea
            value={value}
            disabled={disabled}
            onChange={(e) => handleChange(key, e.target.value)}
            rows={String(value).length > 60 ? 3 : 1}
            placeholder="Enter Policy Path or Repo ID"
            className={`flex-1 min-w-0 px-2 py-1.5 border border-[var(--mc-border-strong)] rounded-lg text-sm bg-[var(--mc-surface)] text-[var(--mc-text)] focus:outline-none focus:ring-1 focus:ring-[var(--mc-accent)] resize-y${disabledCls}`}
          />
          <button
            type="button"
            onClick={() => !disabled && setShowPolicyBrowser(true)}
            disabled={disabled}
            className="flex items-center justify-center w-8 h-8 text-[var(--mc-accent)] bg-[var(--mc-surface-2)] border border-[var(--mc-border-strong)] rounded-lg hover:bg-[var(--mc-surface-hover)] disabled:opacity-50 disabled:cursor-not-allowed shrink-0"
            aria-label="Browse for policy model folder"
            title="Browse for policy model folder"
          >
            <MdFolderOpen size={18} />
          </button>
        </div>
      );
    }

    if (NUMBER_PARAMS.has(key)) {
      return (
        <input
          type="number"
          step="any"
          min={key === 'initial_pose_sync_duration_s' ? 1 : undefined}
          max={key === 'initial_pose_sync_duration_s' ? 60 : undefined}
          value={value}
          disabled={disabled}
          onChange={(e) => handleChange(key, e.target.value)}
          className={`w-full px-2 py-1.5 border border-[var(--mc-border-strong)] rounded-lg text-sm bg-[var(--mc-surface)] text-[var(--mc-text)] focus:outline-none focus:ring-1 focus:ring-[var(--mc-accent)]${disabledCls}`}
        />
      );
    }

    return (
      <textarea
        value={value}
        disabled={disabled}
        onChange={(e) => handleChange(key, e.target.value)}
        rows={String(value).length > 60 ? 3 : 1}
        className={`w-full px-2 py-1.5 border border-[var(--mc-border-strong)] rounded-lg text-sm bg-[var(--mc-surface)] text-[var(--mc-text)] focus:outline-none focus:ring-1 focus:ring-[var(--mc-accent)] resize-y${disabledCls}`}
      />
    );
  };

  const renderParam = ([key, value]) => {
    const disabled = isFieldDisabled(nodeType, key, localParams);
    const help = HELP_TEXT[key];
    return (
      <div key={key}>
        <label className={`block text-xs font-medium mb-1 font-mono ${disabled ? 'text-[var(--mc-text-subtle)]' : 'text-[var(--mc-text-muted)]'}`}>
          {key}
        </label>
        {renderInput(key, value, disabled)}
        {help && !disabled && (
          <div className="mt-1 text-xs text-[var(--mc-text-subtle)]">{help}</div>
        )}
      </div>
    );
  };

  const renderGateParams = () => {
    const entriesByKey = new Map(paramEntries);
    const sections = GATE_PARAM_SECTIONS.map(([title, keys]) => [
      title, keys.filter((key) => entriesByKey.has(key)).map((key) => [key, entriesByKey.get(key)]),
    ]).filter(([, entries]) => entries.length > 0);
    const groupedKeys = new Set(GATE_PARAM_SECTIONS.flatMap(([, keys]) => keys));
    const otherEntries = paramEntries.filter(([key]) => !groupedKeys.has(key));
    if (otherEntries.length) sections.push(['Other parameters', otherEntries]);
    return sections.map(([title, entries]) => (
      <fieldset key={title} className="space-y-3 border-t border-[var(--mc-border)] pt-3">
        <legend className="text-xs font-semibold text-[var(--mc-text)] px-1">{title}</legend>
        {entries.map(renderParam)}
      </fieldset>
    ));
  };

  // The --mc-* tokens are scoped to .autonomy-studio-page; outside it (BT
  // Manager) the vars are undefined, so the panel chrome needs opaque
  // fallbacks or the canvas nodes show through.
  return (
    <div className="absolute right-0 top-0 bottom-0 w-[320px] bg-[var(--mc-surface-2,#ffffff)] border-l border-[var(--mc-border,#e5e7eb)] shadow-lg z-10 flex flex-col">
      {/* Header */}
      <div className="flex items-start justify-between px-4 py-3 border-b border-[var(--mc-border,#e5e7eb)]">
        <div className="flex-1 min-w-0 pr-2">
          <div className="text-xs text-[var(--mc-text-muted)] mb-1 font-mono">{nodeType}</div>
          <input
            type="text"
            value={localName}
            onFocus={() => {
              nameAtFocusRef.current = label;
            }}
            onChange={(e) => {
              const value = e.target.value;
              setLocalName(value);
              const trimmed = value.trim();
              if (trimmed) onNameChange?.(selectedNodeId, trimmed);
            }}
            onBlur={commitName}
            onKeyDown={(e) => {
              if (e.key === 'Enter') {
                e.currentTarget.blur();
              } else if (e.key === 'Escape') {
                const originalName = nameAtFocusRef.current || label;
                suppressNextNameBlurRef.current = true;
                setLocalName(originalName);
                onNameChange?.(selectedNodeId, originalName);
                e.currentTarget.blur();
              }
            }}
            className="w-full text-sm font-bold text-[var(--mc-text)] bg-transparent border-0 border-b border-transparent hover:border-[var(--mc-border-strong)] focus:border-[var(--mc-accent)] focus:outline-none px-0 py-0.5"
          />
        </div>
        <button
          onClick={() => (typeof onClose === 'function' ? onClose() : dispatch(setSelectedNodeId(null)))}
          className="p-1 rounded-lg hover:bg-[var(--mc-surface-hover)] text-[var(--mc-text-subtle)] hover:text-[var(--mc-text)] transition-colors"
        >
          <MdClose size={20} />
        </button>
      </div>

      {/* Params */}
      <div className="flex-1 overflow-y-auto px-4 py-3 space-y-3">
        {JOINT_SELECTOR_NODE_TYPES.has(nodeType) && (
          <JointPosePresetControls
            nodeType={nodeType}
            params={localParams}
            robotType={robotType}
            onApplyParams={commitParams}
          />
        )}
        {nodeType === 'ArmStateGate' && (
          <p className="text-xs text-[var(--mc-text-muted)]">
            All configured joint targets, hand events, and contact conditions must pass. Leave unused target lists empty.
          </p>
        )}
        {gateDiagnostics.length > 0 && (
          <div role="alert" className="text-xs rounded-lg border border-amber-400 bg-amber-50 text-amber-900 p-2 space-y-1">
            <p className="font-semibold">Resolve joint target selections before running.</p>
            {gateDiagnostics.map((message, index) => <p key={`${index}-${message}`}>{message}</p>)}
          </div>
        )}
        {paramEntries.length === 0 ? (
          <p className="text-sm text-[var(--mc-text-subtle)]">No parameters</p>
        ) : (
          nodeType === 'ArmStateGate' ? renderGateParams() : paramEntries.map(renderParam)
        )}
      </div>
      <FileBrowserModal
        isOpen={showPolicyBrowser}
        onClose={() => setShowPolicyBrowser(false)}
        onFileSelect={handlePolicyFolderSelect}
        title="Select policy model folder"
        selectButtonText="Select"
        allowDirectorySelect={true}
        allowFileSelect={false}
        initialPath={policyBrowserPath}
        defaultPath={policyBrowserPath}
        homePath={DEFAULT_PATHS.POLICY_CHECKPOINTS_PATH}
        variant={variant}
      />
    </div>
  );
}
