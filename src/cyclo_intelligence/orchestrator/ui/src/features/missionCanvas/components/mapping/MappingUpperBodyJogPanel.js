import { useCallback, useEffect, useMemo, useRef, useState } from "react";

import { useNavigationRosTopic } from "../../../../hooks/useNavigationRosTopic";
import { MISSION_TEXT_MUTED } from "../../lib/theme";
import { ActionButton, Panel, SessionRow } from "../primitives";
import {
  TELEOP_MESSAGE_TYPE,
  TELEOP_STOP,
  TELEOP_TOPIC,
  teleopTwist,
} from "./MappingTeleopPanel";

export const UPPER_BODY_MESSAGE_TYPE = "trajectory_msgs/msg/JointTrajectory";
export const HEAD_COMMAND_TOPIC = "/leader/joystick_controller_left/joint_trajectory";
export const LIFT_COMMAND_TOPIC = "/leader/joystick_controller_right/joint_trajectory";

export const AI_WORKER_CONTROLLER_LIMITS = {
  head_joint1: { lower: -0.2267, upper: 0.6901 },
  head_joint2: { lower: -0.3, upper: 0.3 },
  lift_joint: { lower: -0.5, upper: 0.0 },
};

const HEAD_JOINTS = ["head_joint1", "head_joint2"];
const LIFT_JOINTS = ["lift_joint"];
const REQUIRED_JOINTS = [...HEAD_JOINTS, ...LIFT_JOINTS];
const HEAD_DURATION_SECONDS = 0.5;
const LIFT_DURATION_SECONDS = 0.8;
const COMMAND_TIMEOUT_MS = 3000;
const HEAD_TOLERANCE = 0.015;
const LIFT_TOLERANCE = 0.008;
const NOOP = () => {};

function clamp(value, lower, upper) {
  return Math.min(Math.max(value, lower), upper);
}

function duration(seconds) {
  const sec = Math.floor(seconds);
  return {
    sec,
    nanosec: Math.round((seconds - sec) * 1_000_000_000),
  };
}

export function buildJointTrajectory(jointNames, positions, durationSeconds) {
  return {
    header: {
      stamp: { sec: 0, nanosec: 0 },
      frame_id: "",
    },
    joint_names: jointNames,
    points: [{
      positions,
      velocities: [],
      accelerations: [],
      effort: [],
      time_from_start: duration(durationSeconds),
    }],
  };
}

export function extractJointPositions(message) {
  const positions = {};
  (message?.name || []).forEach((name, index) => {
    const value = Number(message?.position?.[index]);
    if (REQUIRED_JOINTS.includes(name) && Number.isFinite(value)) {
      positions[name] = value;
    }
  });
  return positions;
}

export function hasBaseFeedback(odometry) {
  const velocity = odometry?.twist?.twist;
  return [velocity?.linear?.x, velocity?.linear?.y, velocity?.angular?.z]
    .every(Number.isFinite);
}

export function isBaseMoving(odometry) {
  if (!hasBaseFeedback(odometry)) return true;
  const { linear, angular } = odometry.twist.twist;
  return (
    Math.hypot(linear.x, linear.y) > 0.015
    || Math.abs(angular.z) > 0.03
  );
}

export function resolveAiWorkerJointLimits(robotDescription) {
  const limits = Object.fromEntries(
    Object.entries(AI_WORKER_CONTROLLER_LIMITS).map(([name, value]) => [name, { ...value }]),
  );
  if (!robotDescription || typeof DOMParser === "undefined") {
    return { limits, source: "AI Worker controller defaults" };
  }

  try {
    const document = new DOMParser().parseFromString(robotDescription, "application/xml");
    let found = false;
    Array.from(document.getElementsByTagName("joint")).forEach((joint) => {
      const name = joint.getAttribute("name");
      if (!limits[name]) return;
      const limit = joint.getElementsByTagName("limit")[0];
      const lower = Number(limit?.getAttribute("lower"));
      const upper = Number(limit?.getAttribute("upper"));
      if (!Number.isFinite(lower) || !Number.isFinite(upper) || lower >= upper) return;
      const intersection = {
        lower: Math.max(limits[name].lower, lower),
        upper: Math.min(limits[name].upper, upper),
      };
      if (intersection.lower >= intersection.upper) return;
      limits[name] = intersection;
      found = true;
    });
    return {
      limits,
      source: found ? "Robot model + controller limits" : "AI Worker controller defaults",
    };
  } catch (_error) {
    return { limits, source: "AI Worker controller defaults" };
  }
}

// Backward-compatible names for callers introduced with the SG2-only version.
export const SG2_CONTROLLER_LIMITS = AI_WORKER_CONTROLLER_LIMITS;
export const resolveSg2JointLimits = resolveAiWorkerJointLimits;

function targetReached(pose, pending) {
  const tolerance = pending.group === "Head" ? HEAD_TOLERANCE : LIFT_TOLERANCE;
  return pending.jointNames.every((name, index) => (
    Number.isFinite(pose[name])
    && Math.abs(pose[name] - pending.positions[index]) <= tolerance
  ));
}

function formatPosition(value, unit) {
  return Number.isFinite(value) ? `${value.toFixed(3)} ${unit}` : "Waiting";
}

export default function MappingUpperBodyJogPanel({
  disabled,
  mobileMotionActive = false,
  onBusyChange = NOOP,
  onMessage = NOOP,
  onPublishRosTopic,
}) {
  const [activated, setActivated] = useState(false);
  const [headStep, setHeadStep] = useState(0.05);
  const [liftStep, setLiftStep] = useState(0.02);
  const [pending, setPending] = useState(null);
  const [errorMessage, setErrorMessage] = useState("");
  const pendingRef = useRef(null);
  const latestRef = useRef(null);
  const lastOdometryRef = useRef(null);

  const { topicData: jointStateData } = useNavigationRosTopic("/joint_states", {
    staleMs: 1500,
    throttleMs: 50,
  });
  const { topicData: odometryData } = useNavigationRosTopic("/odom", {
    staleMs: 1500,
    throttleMs: 100,
  });
  const { topicData: robotDescriptionData } = useNavigationRosTopic("/robot_description");

  const pose = useMemo(
    () => extractJointPositions(jointStateData?.data),
    [jointStateData],
  );
  const hasHeadPose = HEAD_JOINTS.every((name) => Number.isFinite(pose[name]));
  const hasLiftPose = LIFT_JOINTS.every((name) => Number.isFinite(pose[name]));
  const hasOdometry = hasBaseFeedback(odometryData?.data);
  const baseMoving = mobileMotionActive || isBaseMoving(odometryData?.data);
  latestRef.current = { disabled, activated, pose, hasOdometry, mobileMotionActive };
  const { limits, source: limitSource } = useMemo(
    () => resolveAiWorkerJointLimits(robotDescriptionData?.data?.data),
    [robotDescriptionData],
  );

  useEffect(() => {
    onBusyChange(Boolean(pending));
    return () => onBusyChange(false);
  }, [onBusyChange, pending]);

  const clearPending = useCallback((message) => {
    if (!pendingRef.current) return;
    pendingRef.current = null;
    setPending(null);
    if (message) {
      setErrorMessage(message);
      onMessage(message);
    }
  }, [onMessage]);

  useEffect(() => () => { pendingRef.current = null; }, []);

  useEffect(() => {
    if (!pending || pending.phase !== "moving" || !targetReached(pose, pending)) return;
    onMessage(`${pending.group} adjustment complete`);
    clearPending();
  }, [clearPending, onMessage, pending, pose]);

  useEffect(() => {
    const command = pendingRef.current;
    if (!command) return;
    if (disabled || !activated || !hasOdometry || mobileMotionActive
      || !command.jointNames.every((name) => Number.isFinite(pose[name]))) {
      clearPending("Jog interrupted: check connection, joint feedback and base control before retrying.");
    }
  }, [activated, clearPending, disabled, hasOdometry, mobileMotionActive, pose]);

  useEffect(() => {
    if (lastOdometryRef.current === odometryData) return;
    lastOdometryRef.current = odometryData;
    const command = pendingRef.current;
    if (!command || command.phase !== "confirming") return;
    const current = latestRef.current;
    if (current.disabled || !current.activated || !current.hasOdometry
      || current.mobileMotionActive) return;
    if (isBaseMoving(odometryData?.data)) {
      command.stoppedSamples = 0;
      return;
    }
    command.stoppedSamples += 1;
    if (command.stoppedSamples < 2) return;
    const tolerance = command.group === "Head" ? HEAD_TOLERANCE : LIFT_TOLERANCE;
    if (!command.jointNames.every((name, index) => (
      Number.isFinite(current.pose[name])
      && Math.abs(current.pose[name] - command.startPositions[index]) <= tolerance
    ))) {
      clearPending("Joint position changed while stopping. Stop other joint control and retry.");
      return;
    }
    command.phase = "moving";
    setPending({ ...command });
    const send = async () => {
      try {
        await onPublishRosTopic(
          command.topic,
          UPPER_BODY_MESSAGE_TYPE,
          buildJointTrajectory(command.jointNames, command.positions, command.seconds),
        );
        if (pendingRef.current === command) onMessage(`${command.group} target sent`);
      } catch (error) {
        if (pendingRef.current === command) {
          clearPending(error instanceof Error && error.message ? error.message : `${command.group} command failed`);
        }
      }
    };
    send();
  }, [clearPending, odometryData, onMessage, onPublishRosTopic]);

  useEffect(() => {
    if (!pending) return undefined;
    const commandId = pending.id;
    const timer = window.setTimeout(() => {
      const command = pendingRef.current;
      if (command?.id !== commandId) return;
      clearPending(command.phase === "moving"
        ? `${command.group} did not reach the target. Check controller or leader input.`
        : "Base stop was not confirmed. No joint command was sent; check /odom and retry.");
    }, COMMAND_TIMEOUT_MS);
    return () => window.clearTimeout(timer);
  }, [clearPending, pending]);

  useEffect(() => {
    if (disabled) setActivated(false);
  }, [disabled]);

  const publishTarget = useCallback(async (group, topic, jointNames, positions, seconds) => {
    if (disabled || !activated || !hasOdometry || baseMoving || pendingRef.current) return;
    if (!jointNames.every((name) => Number.isFinite(pose[name]))) return;
    setErrorMessage("");
    const unchanged = jointNames.every(
      (name, index) => Math.abs(pose[name] - positions[index]) < 1e-6,
    );
    if (unchanged) {
      onMessage(`${group} is already at that limit or target`);
      return;
    }

    const command = {
      id: `${Date.now()}-${group}`,
      group,
      jointNames,
      positions,
      startPositions: jointNames.map((name) => pose[name]),
      topic,
      seconds,
      phase: "stopping",
      stoppedSamples: 0,
    };
    pendingRef.current = command;
    setPending(command);
    try {
      await onPublishRosTopic(
        TELEOP_TOPIC,
        TELEOP_MESSAGE_TYPE,
        teleopTwist(TELEOP_STOP),
      );
      if (pendingRef.current !== command) return;
      command.phase = "confirming";
      setPending({ ...command });
      onMessage("Waiting for fresh odometry to confirm the base has stopped");
    } catch (error) {
      if (pendingRef.current === command) {
        clearPending(error instanceof Error && error.message ? error.message : `${group} command failed`);
      }
    }
  }, [
    activated,
    baseMoving,
    disabled,
    clearPending,
    hasOdometry,
    onMessage,
    onPublishRosTopic,
    pose,
  ]);

  const jogHead = (pitchDelta, yawDelta) => {
    publishTarget(
      "Head",
      HEAD_COMMAND_TOPIC,
      HEAD_JOINTS,
      [
        clamp(pose.head_joint1 + pitchDelta, limits.head_joint1.lower, limits.head_joint1.upper),
        clamp(pose.head_joint2 + yawDelta, limits.head_joint2.lower, limits.head_joint2.upper),
      ],
      HEAD_DURATION_SECONDS,
    );
  };

  const centerYaw = () => {
    publishTarget(
      "Head",
      HEAD_COMMAND_TOPIC,
      HEAD_JOINTS,
      [pose.head_joint1, clamp(0, limits.head_joint2.lower, limits.head_joint2.upper)],
      HEAD_DURATION_SECONDS,
    );
  };

  const jogLift = (delta) => {
    publishTarget(
      "Lift",
      LIFT_COMMAND_TOPIC,
      LIFT_JOINTS,
      [clamp(pose.lift_joint + delta, limits.lift_joint.lower, limits.lift_joint.upper)],
      LIFT_DURATION_SECONDS,
    );
  };

  const controlsDisabled = disabled || !activated || !hasOdometry || baseMoving || Boolean(pending);
  const headDisabled = controlsDisabled || !hasHeadPose;
  const liftDisabled = controlsDisabled || !hasLiftPose;
  const status = disabled
    ? "Unavailable"
    : !hasHeadPose && !hasLiftPose
      ? "Waiting for /joint_states"
      : !hasOdometry
        ? "Waiting for valid /odom"
      : pending && pending.phase !== "moving"
        ? "Confirming base stop"
      : baseMoving
        ? "Stop the base first"
        : pending
          ? `Moving ${pending.group}`
          : activated
            ? "Active"
            : "Inactive";

  return (
    <Panel title="Upper Body Jog" className="grid gap-3 min-h-0 content-start overflow-auto">
      <div className="flex items-center justify-between gap-2">
        <div className="text-xs" style={{ color: MISSION_TEXT_MUTED }}>{status}</div>
        <ActionButton
          active={activated}
          disabled={disabled || Boolean(pending)}
          onClick={() => {
            const next = !activated;
            if (next) setErrorMessage("");
            setActivated(next);
            onMessage(`Upper body jog ${next ? "activated" : "deactivated"}`);
          }}
          variant="secondary"
        >
          {activated ? "Deactivate" : "Activate"}
        </ActionButton>
      </div>

      {errorMessage && (
        <div role="alert" className="text-xs leading-4 break-words" style={{ color: "var(--mc-danger)" }}>
          {errorMessage}
        </div>
      )}

      <div className="grid gap-3">
        <div className="grid gap-2">
          <div className="flex items-center justify-between gap-2">
            <span className="text-xs font-semibold">Head</span>
            <label className="flex items-center gap-2 text-xs" style={{ color: MISSION_TEXT_MUTED }}>
              Step
              <select
                aria-label="Head jog step"
                value={headStep}
                disabled={headDisabled}
                onChange={(event) => setHeadStep(Number(event.currentTarget.value))}
                className="h-8 rounded-md border px-2"
                style={{
                  color: "var(--mc-text)",
                  borderColor: "var(--mc-border-strong)",
                  backgroundColor: "var(--mc-surface)",
                }}
              >
                <option value="0.02">0.02 rad</option>
                <option value="0.05">0.05 rad</option>
                <option value="0.1">0.10 rad</option>
              </select>
            </label>
          </div>
          <div className="grid grid-cols-2 gap-2">
            <ActionButton disabled={headDisabled} onClick={() => jogHead(-headStep, 0)} variant="secondary">
              Look Up
            </ActionButton>
            <ActionButton disabled={headDisabled} onClick={() => jogHead(headStep, 0)} variant="secondary">
              Look Down
            </ActionButton>
          </div>
          <div className="grid grid-cols-3 gap-2">
            <ActionButton disabled={headDisabled} onClick={() => jogHead(0, headStep)} variant="secondary">
              Left
            </ActionButton>
            <ActionButton disabled={headDisabled} onClick={centerYaw} variant="secondary">
              Center
            </ActionButton>
            <ActionButton disabled={headDisabled} onClick={() => jogHead(0, -headStep)} variant="secondary">
              Right
            </ActionButton>
          </div>
        </div>

        <div className="grid gap-2 border-t pt-3" style={{ borderColor: "var(--mc-border)" }}>
          <div className="flex items-center justify-between gap-2">
            <span className="text-xs font-semibold">Lift</span>
            <label className="flex items-center gap-2 text-xs" style={{ color: MISSION_TEXT_MUTED }}>
              Step
              <select
                aria-label="Lift jog step"
                value={liftStep}
                disabled={liftDisabled}
                onChange={(event) => setLiftStep(Number(event.currentTarget.value))}
                className="h-8 rounded-md border px-2"
                style={{
                  color: "var(--mc-text)",
                  borderColor: "var(--mc-border-strong)",
                  backgroundColor: "var(--mc-surface)",
                }}
              >
                <option value="0.01">0.01 m</option>
                <option value="0.02">0.02 m</option>
                <option value="0.05">0.05 m</option>
              </select>
            </label>
          </div>
          <div className="grid grid-cols-2 gap-2">
            <ActionButton disabled={liftDisabled} onClick={() => jogLift(-liftStep)} variant="secondary">
              Down
            </ActionButton>
            <ActionButton disabled={liftDisabled} onClick={() => jogLift(liftStep)} variant="secondary">
              Up
            </ActionButton>
          </div>
        </div>

        <div className="grid grid-cols-3 gap-2 border-t pt-3" style={{ borderColor: "var(--mc-border)" }}>
          <SessionRow label="Pitch" value={formatPosition(pose.head_joint1, "rad")} stacked />
          <SessionRow label="Yaw" value={formatPosition(pose.head_joint2, "rad")} stacked />
          <SessionRow label="Lift" value={formatPosition(pose.lift_joint, "m")} stacked />
        </div>
        <div className="text-[11px] leading-4" style={{ color: MISSION_TEXT_MUTED }}>
          Limits: {limitSource}. Base motion is stopped and locked while a command is active.
          Keep leader head/lift input idle while using this panel.
        </div>
      </div>
    </Panel>
  );
}
