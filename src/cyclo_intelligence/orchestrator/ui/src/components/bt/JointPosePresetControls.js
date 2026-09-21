import React, { useCallback, useEffect, useMemo, useState } from 'react';
import toast from 'react-hot-toast';
import { MdDelete, MdRefresh, MdSave } from 'react-icons/md';

import {
  deleteJointPosePreset,
  listJointPosePresets,
  saveJointPosePreset,
} from '../../features/actionCanvas/jointPosePresetsApi';
import {
  nodeParamsFromPose,
  handEventJointNames,
  handEventParamsFromPose,
  isHandPoseRobot,
  normalizedPoseRobotType,
  poseJointNamesForNode,
} from '../../features/actionCanvas/jointPoseUtils';
import useJointPoseCapture from '../../hooks/useJointPoseCapture';

const PRESET_NAME_RE = /^[\w-]+$/u;

function statusClasses(state) {
  if (state === 'stable') return 'bg-emerald-500';
  if (state === 'moving' || state === 'settling') return 'bg-amber-500';
  return 'bg-red-500';
}

function HandEventPoseCaptureControls({ params, robotType, onApplyParams }) {
  const selectedJoints = useMemo(
    () => handEventJointNames(params, robotType),
    [params, robotType],
  );
  const captureState = useJointPoseCapture(selectedJoints.length > 0, robotType, selectedJoints);
  if (selectedJoints.length === 0) return null;

  const captureHandEvent = (event) => {
    const result = captureState.capture();
    if (!result.ok) {
      toast.error(result.message);
      return;
    }
    const mapped = handEventParamsFromPose(params, result.pose, robotType, event);
    if (mapped.missing.length > 0) {
      toast.error(`Missing joint states: ${mapped.missing.join(', ')}`);
      return;
    }
    onApplyParams(mapped.updates);
    toast.success(`Captured ${event} hand pose`);
  };

  return (
    <div className="border-t border-[var(--mc-border,#e5e7eb)] pt-3 space-y-2">
      <div className="text-xs font-semibold text-[var(--mc-text,#111827)]">Hand Close → Open Event</div>
      <div className="text-[11px] text-[var(--mc-text-muted,#6b7280)]">
        {captureState.message}. Hold the {selectedJoints.length} selected finger joints still at each pose.
      </div>
      <div className="flex gap-1.5">
        {['closed', 'open'].map((event) => (
          <button
            key={event}
            type="button"
            disabled={captureState.state !== 'stable'}
            onClick={() => captureHandEvent(event)}
            className="flex-1 rounded-lg border border-[var(--mc-accent,#2563eb)] px-2 py-1.5 text-xs text-[var(--mc-accent,#2563eb)] disabled:opacity-40"
          >
            Capture {event === 'closed' ? 'Closed' : 'Open'} Hand Pose
          </button>
        ))}
      </div>
    </div>
  );
}

export default function JointPosePresetControls({
  nodeType,
  params,
  robotType,
  onApplyParams,
}) {
  const selectedJoints = useMemo(
    () => poseJointNamesForNode(nodeType, params, robotType),
    [nodeType, params, robotType],
  );
  const captureState = useJointPoseCapture(selectedJoints.length > 0, robotType, selectedJoints);
  const normalizedRobotType = normalizedPoseRobotType(robotType);
  const [presets, setPresets] = useState([]);
  const [selectedName, setSelectedName] = useState('');
  const [presetName, setPresetName] = useState('');
  const [capturedPose, setCapturedPose] = useState(null);
  const [capturedAt, setCapturedAt] = useState('');
  const [busy, setBusy] = useState(false);

  const selectedPreset = useMemo(
    () => presets.find((preset) => preset.name === selectedName),
    [presets, selectedName],
  );

  const refreshPresets = useCallback(async (quiet = false) => {
    try {
      const result = await listJointPosePresets(normalizedRobotType);
      const next = result.presets || [];
      setPresets(next);
      setSelectedName((current) => (
        next.some((preset) => preset.name === current) ? current : ''
      ));
    } catch (error) {
      if (!quiet) toast.error(`Failed to load pose presets: ${error.message}`);
    }
  }, [normalizedRobotType]);

  useEffect(() => {
    refreshPresets(true);
    setCapturedPose(null);
    setCapturedAt('');
    setSelectedName('');
    setPresetName('');
  }, [refreshPresets]);

  const applyPose = (pose, successMessage) => {
    const result = nodeParamsFromPose(nodeType, params, pose, robotType);
    if (Object.keys(result.updates).length === 0) {
      toast.error('Enable a JointControl group or select ArmStateGate joints first');
      return false;
    }
    if (result.missing.length > 0) {
      toast.error(`Missing joint states: ${result.missing.join(', ')}`);
      return false;
    }
    onApplyParams(result.updates);
    toast.success(successMessage);
    return true;
  };

  const handleCapture = () => {
    const result = captureState.capture();
    if (!result.ok) {
      toast.error(result.message);
      return;
    }
    if (!applyPose(result.pose, 'Current follower pose captured')) return;
    const now = new Date().toISOString();
    setCapturedPose(result.pose);
    setCapturedAt(now);
  };

  const handleLoad = () => {
    if (!selectedPreset) {
      toast.error('Select a pose preset');
      return;
    }
    if (applyPose(selectedPreset.joints, `Loaded pose: ${selectedPreset.name}`)) {
      setCapturedPose(selectedPreset.joints);
      setCapturedAt(selectedPreset.captured_at || new Date().toISOString());
      setPresetName(selectedPreset.name);
    }
  };

  const persistPreset = async (overwrite) => {
    const name = presetName.trim();
    if (!PRESET_NAME_RE.test(name)) {
      toast.error('Use letters, numbers, underscores, or hyphens for the preset name');
      return;
    }
    if (!capturedPose) {
      toast.error('Capture or load a pose before saving');
      return;
    }

    setBusy(true);
    try {
      await saveJointPosePreset({
        preset: {
          schema_version: 'cyclo_joint_pose_v1',
          name,
          robot_type: normalizedRobotType,
          source_topic: '/joint_states',
          source_topics: captureState.sourceTopics,
          captured_at: capturedAt || new Date().toISOString(),
          joints: capturedPose,
        },
        overwrite,
      });
      await refreshPresets(true);
      setSelectedName(name);
      toast.success(`Saved pose: ${name}`);
    } catch (error) {
      if (error.status === 409 && !overwrite) {
        if (window.confirm(`Pose '${name}' already exists. Overwrite it?`)) {
          setBusy(false);
          await persistPreset(true);
          return;
        }
      } else {
        toast.error(`Failed to save pose: ${error.message}`);
      }
    } finally {
      setBusy(false);
    }
  };

  const handleDelete = async () => {
    if (!selectedPreset) return;
    if (!window.confirm(`Delete pose '${selectedPreset.name}'?`)) return;
    setBusy(true);
    try {
      await deleteJointPosePreset(normalizedRobotType, selectedPreset.name);
      setSelectedName('');
      if (presetName === selectedPreset.name) setPresetName('');
      await refreshPresets(true);
      toast.success(`Deleted pose: ${selectedPreset.name}`);
    } catch (error) {
      toast.error(`Failed to delete pose: ${error.message}`);
    } finally {
      setBusy(false);
    }
  };

  return (
    <section className="rounded-xl border border-[var(--mc-border-strong,#d1d5db)] bg-[var(--mc-surface,#ffffff)] p-3 space-y-3">
      <div>
        <div className="flex items-center justify-between gap-2">
          <div className="text-xs font-semibold text-[var(--mc-text,#111827)]">Current Pose</div>
          <div className="flex items-center gap-1.5 text-[11px] text-[var(--mc-text-muted,#6b7280)]">
            <span className={`h-2 w-2 rounded-full ${statusClasses(captureState.state)}`} />
            {captureState.message}
          </div>
        </div>
        <div className="mt-1 text-[11px] text-[var(--mc-text-subtle,#9ca3af)]">
          {selectedJoints.length > 0
            ? `Captures ${selectedJoints.length} selected ${normalizedRobotType} follower joints.`
            : 'Select joint targets to capture a pose.'}
          {' '}Sources: {captureState.sourceTopics?.join(', ')}
        </div>
      </div>

      <button
        type="button"
        onClick={handleCapture}
        disabled={selectedJoints.length === 0 || captureState.state !== 'stable' || busy}
        className="w-full rounded-lg bg-[var(--mc-accent,#2563eb)] px-3 py-2 text-xs font-semibold text-white hover:opacity-90 disabled:cursor-not-allowed disabled:opacity-40"
      >
        Capture Current Pose
      </button>

      {nodeType === 'ArmStateGate' && isHandPoseRobot(robotType) && (
        <HandEventPoseCaptureControls
          params={params}
          robotType={robotType}
          onApplyParams={onApplyParams}
        />
      )}

      <div className="border-t border-[var(--mc-border,#e5e7eb)] pt-3 space-y-2">
        <div className="flex items-center justify-between">
          <div className="text-xs font-semibold text-[var(--mc-text,#111827)]">Pose Presets</div>
          <button
            type="button"
            title="Refresh pose presets"
            aria-label="Refresh pose presets"
            onClick={() => refreshPresets(false)}
            disabled={busy}
            className="rounded p-1 text-[var(--mc-text-muted,#6b7280)] hover:bg-[var(--mc-surface-hover,#f3f4f6)] disabled:opacity-40"
          >
            <MdRefresh size={16} />
          </button>
        </div>

        <div className="flex gap-1.5">
          <select
            aria-label="Pose preset"
            value={selectedName}
            onChange={(event) => setSelectedName(event.target.value)}
            className="min-w-0 flex-1 rounded-lg border border-[var(--mc-border-strong,#d1d5db)] bg-[var(--mc-surface,#ffffff)] px-2 py-1.5 text-xs text-[var(--mc-text,#111827)]"
          >
            <option value="">Select preset…</option>
            {presets.map((preset) => (
              <option key={preset.name} value={preset.name}>{preset.name}</option>
            ))}
          </select>
          <button
            type="button"
            onClick={handleLoad}
            disabled={!selectedPreset || busy}
            className="rounded-lg border border-[var(--mc-border-strong,#d1d5db)] px-2 py-1.5 text-xs text-[var(--mc-text,#111827)] hover:bg-[var(--mc-surface-hover,#f3f4f6)] disabled:opacity-40"
          >
            Load
          </button>
          <button
            type="button"
            title="Delete selected pose"
            aria-label="Delete selected pose"
            onClick={handleDelete}
            disabled={!selectedPreset || busy}
            className="rounded-lg border border-[var(--mc-border-strong,#d1d5db)] px-2 text-[var(--mc-danger,#dc2626)] hover:bg-[var(--mc-surface-hover,#f3f4f6)] disabled:opacity-40"
          >
            <MdDelete size={16} />
          </button>
        </div>

        <div className="flex gap-1.5">
          <input
            aria-label="Pose preset name"
            value={presetName}
            onChange={(event) => setPresetName(event.target.value)}
            placeholder="preset_name"
            className="min-w-0 flex-1 rounded-lg border border-[var(--mc-border-strong,#d1d5db)] bg-[var(--mc-surface,#ffffff)] px-2 py-1.5 text-xs text-[var(--mc-text,#111827)]"
          />
          <button
            type="button"
            onClick={() => persistPreset(false)}
            disabled={!capturedPose || busy}
            className="flex items-center gap-1 rounded-lg border border-[var(--mc-accent,#2563eb)] px-2 py-1.5 text-xs font-medium text-[var(--mc-accent,#2563eb)] hover:bg-[var(--mc-accent-soft,#eff6ff)] disabled:opacity-40"
          >
            <MdSave size={15} /> Save
          </button>
        </div>
      </div>
    </section>
  );
}
