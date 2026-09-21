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

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { MISSION_TEXT_MUTED } from "../../lib/theme";
import { isTextInputTarget } from "../../lib/dom";
import { ActionButton, Panel, SessionRow } from "../primitives";

export const TELEOP_TOPIC = "/cmd_vel";

export const TELEOP_MESSAGE_TYPE = "geometry_msgs/msg/Twist";

export const TELEOP_REPEAT_MS = 200;

export const TELEOP_DEFAULT_LINEAR_SPEED = 0.4;

export const TELEOP_DEFAULT_LATERAL_SPEED = 0.2;

export const TELEOP_DEFAULT_ANGULAR_SPEED = 0.8;

export const TELEOP_STOP = { linearX: 0, linearY: 0, angularZ: 0 };

const NOOP = () => {};

export function clampNumber(value, min, max) {
  const number = Number(value);
  if (!Number.isFinite(number)) return min;
  return Math.min(Math.max(number, min), max);
}

export function teleopTwist({ linearX = 0, linearY = 0, angularZ = 0 }) {
  return {
    linear: { x: linearX, y: linearY, z: 0 },
    angular: { x: 0, y: 0, z: angularZ },
  };
}

export function TeleopButton({
  children,
  active = false,
  disabled = false,
  title,
  onStart,
  onStop,
}) {
  const handlePointerDown = (event) => {
    if (disabled) return;
    event.preventDefault();
    if (event.currentTarget.setPointerCapture) {
      event.currentTarget.setPointerCapture(event.pointerId);
    }
    onStart();
  };

  const handlePointerStop = (event) => {
    if (disabled) return;
    event.preventDefault();
    onStop();
  };

  return (
    <button
      type="button"
      disabled={disabled}
      title={title}
      aria-pressed={active ? true : undefined}
      onPointerDown={handlePointerDown}
      onPointerUp={handlePointerStop}
      onPointerCancel={handlePointerStop}
      onPointerLeave={handlePointerStop}
      className="h-12 w-14 text-[16px] font-bold transition-all active:translate-y-px disabled:opacity-45"
      style={{
        borderRadius: 13,
        border: `1px solid ${active ? "var(--mc-text)" : "var(--mc-border-strong)"}`,
        color: active ? "var(--mc-bg)" : "var(--mc-text)",
        backgroundColor: active ? "var(--mc-text)" : "var(--mc-surface-2)",
      }}
    >
      {children}
    </button>
  );
}

export default function MappingTeleopPanel({
  disabled,
  motionBlocked = false,
  onMotionActiveChange = NOOP,
  onPublish,
  onMessage,
}) {
  const [linearSpeed, setLinearSpeed] = useState(TELEOP_DEFAULT_LINEAR_SPEED);
  const [lateralSpeed, setLateralSpeed] = useState(TELEOP_DEFAULT_LATERAL_SPEED);
  const [angularSpeed, setAngularSpeed] = useState(TELEOP_DEFAULT_ANGULAR_SPEED);
  const [activated, setActivated] = useState(false);
  const [activeLabel, setActiveLabel] = useState("");
  const [errorMessage, setErrorMessage] = useState("");
  const activeMotionRef = useRef(null);
  const controlsDisabled = disabled || motionBlocked || !activated;

  const publishMotion = useCallback((motion) => {
    void onPublish(motion).catch((error) => {
      setActivated(false);
      setActiveLabel("");
      activeMotionRef.current = null;
      onMotionActiveChange(false);
      const message = error instanceof Error && error.message ? error.message : "Teleop publish failed";
      setErrorMessage(message);
      onMessage(message);
    });
  }, [onMessage, onMotionActiveChange, onPublish]);

  const stopTeleop = useCallback(() => {
    activeMotionRef.current = null;
    setActiveLabel("");
    onMotionActiveChange(false);
    publishMotion(TELEOP_STOP);
  }, [onMotionActiveChange, publishMotion]);

  const deactivateTeleop = useCallback(() => {
    activeMotionRef.current = null;
    setActiveLabel("");
    setActivated(false);
    onMotionActiveChange(false);
    publishMotion(TELEOP_STOP);
  }, [onMotionActiveChange, publishMotion]);

  const activateTeleop = useCallback(() => {
    if (disabled) return;
    setErrorMessage("");
    setActivated(true);
    setActiveLabel("");
    onMessage("Mobile teleop activated");
  }, [disabled, onMessage]);

  const startTeleop = useCallback((label, motion) => {
    if (controlsDisabled) return;
    setErrorMessage("");
    activeMotionRef.current = motion;
    setActiveLabel(label);
    onMotionActiveChange(true);
    publishMotion(motion);
  }, [controlsDisabled, onMotionActiveChange, publishMotion]);

  useEffect(() => {
    if (disabled || motionBlocked) {
      if (activated || activeMotionRef.current) {
        publishMotion(TELEOP_STOP);
      }
      activeMotionRef.current = null;
      setActiveLabel("");
      onMotionActiveChange(false);
      if (disabled) setActivated(false);
      return undefined;
    }
    const interval = window.setInterval(() => {
      if (activeMotionRef.current) {
        publishMotion(activeMotionRef.current);
      }
    }, TELEOP_REPEAT_MS);
    return () => window.clearInterval(interval);
  }, [activated, disabled, motionBlocked, onMotionActiveChange, publishMotion]);

  useEffect(() => () => {
    if (activeMotionRef.current) {
      void onPublish(TELEOP_STOP);
    }
    onMotionActiveChange(false);
  }, [onMotionActiveChange, onPublish]);

  const commandByKey = useMemo(() => ({
    w: {
      label: "W",
      motion: { linearX: linearSpeed, linearY: 0, angularZ: 0 },
    },
    s: {
      label: "S",
      motion: { linearX: -linearSpeed, linearY: 0, angularZ: 0 },
    },
    a: {
      label: "A",
      motion: { linearX: 0, linearY: 0, angularZ: angularSpeed },
    },
    d: {
      label: "D",
      motion: { linearX: 0, linearY: 0, angularZ: -angularSpeed },
    },
    q: {
      label: "Q",
      motion: { linearX: 0, linearY: lateralSpeed, angularZ: 0 },
    },
    e: {
      label: "E",
      motion: { linearX: 0, linearY: -lateralSpeed, angularZ: 0 },
    },
  }), [angularSpeed, lateralSpeed, linearSpeed]);

  const handleKeyDown = useCallback((event) => {
    if (controlsDisabled || event.repeat || isTextInputTarget(event.target)) return;
    const key = event.key === " " ? "space" : event.key.toLowerCase();
    if (key === "space") {
      event.preventDefault();
      stopTeleop();
      return;
    }
    const command = commandByKey[key];
    if (!command) return;
    event.preventDefault();
    startTeleop(command.label, command.motion);
  }, [commandByKey, controlsDisabled, startTeleop, stopTeleop]);

  const handleKeyUp = useCallback((event) => {
    if (controlsDisabled) return;
    const key = event.key.toLowerCase();
    if (!commandByKey[key]) return;
    if (isTextInputTarget(event.target) && !activeMotionRef.current) return;
    event.preventDefault();
    stopTeleop();
  }, [commandByKey, controlsDisabled, stopTeleop]);

  useEffect(() => {
    if (controlsDisabled) return undefined;
    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);
    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
    };
  }, [controlsDisabled, handleKeyDown, handleKeyUp]);

  const updateLinearSpeed = (value) => {
    const nextSpeed = clampNumber(value, 0.05, 1.2);
    setLinearSpeed(nextSpeed);
    if (activeMotionRef.current?.linearX) {
      const direction = Math.sign(activeMotionRef.current.linearX);
      const nextMotion = { linearX: direction * nextSpeed, linearY: 0, angularZ: 0 };
      activeMotionRef.current = nextMotion;
      publishMotion(nextMotion);
    }
  };

  const updateLateralSpeed = (value) => {
    const nextSpeed = clampNumber(value, 0.05, 0.6);
    setLateralSpeed(nextSpeed);
    if (activeMotionRef.current?.linearY) {
      const direction = Math.sign(activeMotionRef.current.linearY);
      const nextMotion = { linearX: 0, linearY: direction * nextSpeed, angularZ: 0 };
      activeMotionRef.current = nextMotion;
      publishMotion(nextMotion);
    }
  };

  const updateAngularSpeed = (value) => {
    const nextSpeed = clampNumber(value, 0.05, 2);
    setAngularSpeed(nextSpeed);
    if (activeMotionRef.current?.angularZ) {
      const direction = Math.sign(activeMotionRef.current.angularZ);
      const nextMotion = { linearX: 0, linearY: 0, angularZ: direction * nextSpeed };
      activeMotionRef.current = nextMotion;
      publishMotion(nextMotion);
    }
  };

  const speedControlStyle = {
    accentColor: "var(--vscode-button-background)",
  };

  return (
    <Panel title="Mobile Teleop" className="grid gap-3 min-h-0 content-start overflow-auto">
      <div className="flex items-center justify-between gap-2">
        <div className="text-xs" style={{ color: MISSION_TEXT_MUTED }}>
          {disabled
            ? "Unavailable"
            : motionBlocked
              ? "Paused while upper body moves"
              : activated
                ? "Active"
                : "Inactive"}
        </div>
        <ActionButton
          active={activated}
          disabled={disabled}
          onClick={activated ? deactivateTeleop : activateTeleop}
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
      <div
        role="group"
        aria-label="Mobile Teleop"
        tabIndex={controlsDisabled ? -1 : 0}
        className="grid gap-4 min-h-0 outline-none"
      >
        <div className="grid grid-cols-5 gap-2 justify-self-center">
          <div />
          <div />
          <TeleopButton
            active={activeLabel === "W"}
            disabled={controlsDisabled}
            title="Forward (W)"
            onStart={() => startTeleop("W", { linearX: linearSpeed, linearY: 0, angularZ: 0 })}
            onStop={stopTeleop}
          >
            W
          </TeleopButton>
          <div />
          <div />
          <TeleopButton
            active={activeLabel === "Q"}
            disabled={controlsDisabled}
            title="Strafe Left (Q)"
            onStart={() => startTeleop("Q", { linearX: 0, linearY: lateralSpeed, angularZ: 0 })}
            onStop={stopTeleop}
          >
            Q
          </TeleopButton>
          <TeleopButton
            active={activeLabel === "A"}
            disabled={controlsDisabled}
            title="Rotate Left (A)"
            onStart={() => startTeleop("A", { linearX: 0, linearY: 0, angularZ: angularSpeed })}
            onStop={stopTeleop}
          >
            A
          </TeleopButton>
          <TeleopButton
            disabled={controlsDisabled}
            title="Stop"
            onStart={stopTeleop}
            onStop={stopTeleop}
          >
            0
          </TeleopButton>
          <TeleopButton
            active={activeLabel === "D"}
            disabled={controlsDisabled}
            title="Rotate Right (D)"
            onStart={() => startTeleop("D", { linearX: 0, linearY: 0, angularZ: -angularSpeed })}
            onStop={stopTeleop}
          >
            D
          </TeleopButton>
          <TeleopButton
            active={activeLabel === "E"}
            disabled={controlsDisabled}
            title="Strafe Right (E)"
            onStart={() => startTeleop("E", { linearX: 0, linearY: -lateralSpeed, angularZ: 0 })}
            onStop={stopTeleop}
          >
            E
          </TeleopButton>
          <div />
          <div />
          <TeleopButton
            active={activeLabel === "S"}
            disabled={controlsDisabled}
            title="Backward (S)"
            onStart={() => startTeleop("S", { linearX: -linearSpeed, linearY: 0, angularZ: 0 })}
            onStop={stopTeleop}
          >
            S
          </TeleopButton>
          <div />
          <div />
        </div>

        <div className="grid gap-2 min-w-0">
          <label className="grid grid-cols-[52px_1fr_44px] items-center gap-2 text-xs">
            <span style={{ color: MISSION_TEXT_MUTED }}>Linear</span>
            <input
              type="range"
              min="0.05"
              max="1.2"
              step="0.05"
              value={linearSpeed}
              disabled={controlsDisabled}
              onChange={(event) => updateLinearSpeed(event.currentTarget.value)}
              style={speedControlStyle}
            />
            <span className="font-mono text-right">{linearSpeed.toFixed(2)}</span>
          </label>
          <label className="grid grid-cols-[52px_1fr_44px] items-center gap-2 text-xs">
            <span style={{ color: MISSION_TEXT_MUTED }}>Lateral</span>
            <input
              type="range"
              min="0.05"
              max="0.6"
              step="0.05"
              value={lateralSpeed}
              disabled={controlsDisabled}
              onChange={(event) => updateLateralSpeed(event.currentTarget.value)}
              style={speedControlStyle}
            />
            <span className="font-mono text-right">{lateralSpeed.toFixed(2)}</span>
          </label>
          <label className="grid grid-cols-[52px_1fr_44px] items-center gap-2 text-xs">
            <span style={{ color: MISSION_TEXT_MUTED }}>Angular</span>
            <input
              type="range"
              min="0.05"
              max="2"
              step="0.05"
              value={angularSpeed}
              disabled={controlsDisabled}
              onChange={(event) => updateAngularSpeed(event.currentTarget.value)}
              style={speedControlStyle}
            />
            <span className="font-mono text-right">{angularSpeed.toFixed(2)}</span>
          </label>
          <div className="grid grid-cols-2 gap-2">
            <SessionRow label="Topic" value={TELEOP_TOPIC} stacked />
            <SessionRow
              label="Command"
              value={disabled
                ? "Unavailable"
                : motionBlocked
                  ? "Upper body moving"
                  : activated
                    ? activeLabel || "Stop"
                    : "Inactive"}
              stacked
            />
          </div>
        </div>
      </div>
    </Panel>
  );
}
