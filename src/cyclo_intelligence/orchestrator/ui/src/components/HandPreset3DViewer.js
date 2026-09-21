import React, { useEffect, useState, useRef } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { Center, OrbitControls } from '@react-three/drei';
import { MdAdd, MdRemove } from 'react-icons/md';

import useUrdfRobot from '../hooks/useUrdfRobot';

const HAND_URDF_PATH = '/urdf/urdf/ffw_sh5_follower.urdf';
const RIGHT_HAND_BASE_LINK = 'hx5_d20_right_base';
const EXPECTED_HAND_VISUAL_MESHES = 21;

function isFingerEndMarker(object) {
  let current = object;
  while (current) {
    if (/^finger_end_r_link\d+$/.test(String(current.name || ''))) return true;
    current = current.parent;
  }
  return false;
}

function cloneHand(robot) {
  const sourceHand = robot?.links?.[RIGHT_HAND_BASE_LINK];
  if (!sourceHand) return null;

  const object = sourceHand.clone(true);
  const joints = {};
  object.traverse((child) => {
    if (/^finger_end_r_link\d+$/.test(String(child.name || ''))) {
      child.visible = false;
    }
    if (!child.isURDFJoint) return;
    const source = robot.joints?.[child.name];
    if (source) {
      child.jointType = source.jointType;
      child.axis = source.axis?.clone ? source.axis.clone() : source.axis;
      child.limit = { ...source.limit };
      child.ignoreLimits = source.ignoreLimits;
      child.jointValue = [...(source.jointValue || [])];
      child.mimicJoints = source.mimicJoints ? [...source.mimicJoints] : [];
    }
    joints[child.name] = child;
  });
  return { object, joints };
}

function useReadyHand(robot) {
  const [hand, setHand] = useState(null);
  const [timedOut, setTimedOut] = useState(false);

  useEffect(() => {
    setHand(null);
    setTimedOut(false);
    const sourceHand = robot?.links?.[RIGHT_HAND_BASE_LINK];
    if (!sourceHand) return undefined;

    let attempts = 0;
    let timer = null;
    const prepareWhenMeshesAreReady = () => {
      attempts += 1;
      let visualMeshCount = 0;
      sourceHand.traverse((child) => {
        if (child.isMesh && !isFingerEndMarker(child)) visualMeshCount += 1;
      });
      if (visualMeshCount >= EXPECTED_HAND_VISUAL_MESHES) {
        setHand(cloneHand(robot));
        if (timer) clearInterval(timer);
      } else if (attempts >= 80) {
        setTimedOut(true);
        if (timer) clearInterval(timer);
      }
    };

    prepareWhenMeshesAreReady();
    timer = setInterval(prepareWhenMeshesAreReady, 100);
    return () => {
      if (timer) clearInterval(timer);
    };
  }, [robot]);

  return { hand, timedOut };
}

function fingerIndexFromObject(object) {
  let current = object;
  while (current) {
    const match = String(current.name || '').match(/^finger_r_link(\d+)$/);
    if (match) return Math.floor((Number(match[1]) - 1) / 4);
    current = current.parent;
  }
  return null;
}

function easeInOut(value) {
  return value * value * (3 - 2 * value);
}

function pointerClientY(event) {
  const value = event?.clientY
    ?? event?.sourceEvent?.clientY
    ?? event?.nativeEvent?.clientY;
  return Number.isFinite(value) ? value : null;
}

function AnimatedHand({
  hand,
  jointNames,
  openPositions,
  targetPositions,
  poseSignature,
  animate,
  editable,
  onFingerPointerDown,
}) {
  const elapsedRef = useRef(0);

  useEffect(() => {
    elapsedRef.current = 0;
  }, [animate, poseSignature]);

  useFrame((_state, delta) => {
    if (!hand || !jointNames.length) return;
    let blend = 1;
    if (animate) {
      elapsedRef.current += Math.min(delta, 0.1);
      const cycle = (elapsedRef.current % 2.4) / 2.4;
      if (cycle < 0.4) blend = easeInOut(cycle / 0.4);
      else if (cycle < 0.68) blend = 1;
      else blend = easeInOut(1 - ((cycle - 0.68) / 0.32));
    }

    jointNames.forEach((jointName, index) => {
      const joint = hand.joints[jointName];
      if (!joint) return;
      const open = Number(openPositions[index]) || 0;
      const target = Number(targetPositions[index]) || 0;
      joint.setJointValue(open + blend * (target - open));
    });
  });

  return (
    <Center>
      <group rotation={[-Math.PI / 2, 0, 0]} scale={1.35}>
        <primitive
          object={hand.object}
          onPointerDown={(event) => {
            if (!editable || !onFingerPointerDown) return;
            const fingerIndex = fingerIndexFromObject(event.object);
            if (fingerIndex === null || fingerIndex < 0 || fingerIndex > 4) return;
            const clientY = pointerClientY(event);
            if (clientY === null) return;
            event.stopPropagation();
            onFingerPointerDown(fingerIndex, clientY);
          }}
        />
      </group>
    </Center>
  );
}

export default function HandPreset3DViewer({
  jointNames = [],
  openPositions = [],
  targetPositions = [],
  animate = false,
  editable = false,
  dragging = false,
  draggingFingerIndex = null,
  fingerLabels = [],
  fingerValues = [],
  onFingerPointerDown,
  className = '',
}) {
  const { robot, loading, error } = useUrdfRobot(HAND_URDF_PATH);
  const { hand, timedOut } = useReadyHand(robot);
  const controlsRef = useRef(null);
  const hasPose = (
    jointNames.length > 0
    && openPositions.length === jointNames.length
    && targetPositions.length === jointNames.length
  );
  const changeZoom = (scale) => {
    const controls = controlsRef.current;
    const camera = controls?.object;
    if (!controls || !camera) return;
    const offset = camera.position.clone().sub(controls.target);
    const currentDistance = offset.length();
    const nextDistance = Math.max(0.18, Math.min(1.2, currentDistance * scale));
    if (currentDistance <= 0) return;
    camera.position.copy(
      controls.target.clone().add(offset.multiplyScalar(nextDistance / currentDistance))
    );
    camera.updateProjectionMatrix();
    controls.update();
  };

  return (
    <div
      className={`relative overflow-hidden rounded-2xl border border-slate-700 bg-slate-950 ${className}`}
      data-testid="hand-preset-3d-viewer"
    >
      <Canvas
        camera={{ position: [0.36, 0.27, 0.36], fov: 38, near: 0.005, far: 10 }}
        dpr={[1, 1.5]}
        gl={{ antialias: true, alpha: false, powerPreference: 'low-power' }}
        style={{ background: 'radial-gradient(circle at 50% 40%, #1e3a5f 0%, #0f172a 62%)' }}
      >
        <ambientLight intensity={1.1} />
        <directionalLight position={[2, 3, 4]} intensity={2.2} />
        <directionalLight position={[-3, 1, -2]} intensity={0.8} color="#93c5fd" />
        {hand && hasPose && (
          <AnimatedHand
            hand={hand}
            jointNames={jointNames}
            openPositions={openPositions}
            targetPositions={targetPositions}
            poseSignature={targetPositions.join(',')}
            animate={animate}
            editable={editable}
            onFingerPointerDown={onFingerPointerDown}
          />
        )}
        <OrbitControls
          ref={controlsRef}
          makeDefault
          enabled={!dragging}
          enablePan={false}
          minDistance={0.16}
          maxDistance={1.2}
          dampingFactor={0.08}
        />
      </Canvas>

      <div className="pointer-events-none absolute left-3 top-3 rounded-lg bg-black/45 px-2.5 py-1.5 text-[11px] font-medium text-white/80 backdrop-blur-sm">
        {editable
          ? dragging
            ? 'Move up to close · down to open'
            : 'Custom pose motion preview'
          : animate
            ? 'Grasp motion preview'
            : '3D grasp preview'}
      </div>
      {editable && (
        <div className="absolute inset-x-3 bottom-3 z-10 rounded-xl border border-white/10 bg-slate-950/75 p-2 shadow-xl backdrop-blur-md">
          <div className="mb-1.5 flex items-center justify-between px-1 text-[10px] font-medium text-slate-300">
            <span>Drag a finger handle</span>
            <span>↑ close · ↓ open</span>
          </div>
          <div className="grid grid-cols-5 gap-1.5">
            {fingerLabels.map((label, index) => (
              <button
                key={label}
                type="button"
                onPointerDown={(event) => {
                  event.preventDefault();
                  event.stopPropagation();
                  const clientY = pointerClientY(event);
                  if (clientY !== null) onFingerPointerDown?.(index, clientY);
                }}
                className={`touch-none select-none rounded-lg border px-1 py-2 text-center transition-colors ${
                  draggingFingerIndex === index
                    ? 'border-indigo-300 bg-indigo-500 text-white'
                    : 'border-white/10 bg-white/10 text-slate-100 hover:border-indigo-300 hover:bg-indigo-500/60'
                } cursor-ns-resize`}
                aria-label={`Drag ${label} finger`}
                title={`Drag ${label} up or down`}
              >
                <span className="block text-sm leading-none">↕</span>
                <span className="mt-1 block truncate text-[9px] font-semibold">{label}</span>
                <span className="mt-0.5 block font-mono text-[9px] opacity-75">
                  {Math.round((Number(fingerValues[index]) || 0) * 100)}%
                </span>
              </button>
            ))}
          </div>
        </div>
      )}
      <div className="absolute right-3 top-3 z-10 flex overflow-hidden rounded-lg border border-white/10 bg-black/50 shadow-lg backdrop-blur-sm">
        <button
          type="button"
          onClick={() => changeZoom(1.25)}
          className="flex h-9 w-10 items-center justify-center text-white/80 transition-colors hover:bg-white/15 hover:text-white focus:outline-none focus:ring-2 focus:ring-inset focus:ring-indigo-400"
          aria-label="Zoom out hand preview"
          title="Zoom out"
        >
          <MdRemove size={20} />
        </button>
        <div className="w-px bg-white/10" />
        <button
          type="button"
          onClick={() => changeZoom(0.8)}
          className="flex h-9 w-10 items-center justify-center text-white/80 transition-colors hover:bg-white/15 hover:text-white focus:outline-none focus:ring-2 focus:ring-inset focus:ring-indigo-400"
          aria-label="Zoom in hand preview"
          title="Zoom in"
        >
          <MdAdd size={20} />
        </button>
      </div>
      {(loading || (!hand && !timedOut)) && !error && (
        <div className="pointer-events-none absolute inset-0 flex items-center justify-center bg-slate-950/65 text-xs font-medium text-slate-300">
          Loading SH5 hand…
        </div>
      )}
      {error && (
        <div className="pointer-events-none absolute inset-0 flex items-center justify-center bg-slate-950/80 px-6 text-center text-xs text-rose-300">
          Could not load the SH5 hand model.
        </div>
      )}
      {timedOut && !error && (
        <div className="pointer-events-none absolute inset-0 flex items-center justify-center bg-slate-950/80 px-6 text-center text-xs text-rose-300">
          The SH5 hand meshes did not finish loading. Refresh the page and try again.
        </div>
      )}
      {!loading && !error && !hasPose && (
        <div className="pointer-events-none absolute inset-0 flex items-center justify-center bg-slate-950/70 px-6 text-center text-xs text-slate-300">
          Restart the ai_worker preset node to enable 3D pose data.
        </div>
      )}
    </div>
  );
}
