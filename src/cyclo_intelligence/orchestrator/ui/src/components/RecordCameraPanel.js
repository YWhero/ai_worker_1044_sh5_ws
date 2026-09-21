import React, { useCallback, useEffect, useRef, useState } from 'react';
import { useSelector } from 'react-redux';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import { RecordPhase } from '../constants/taskPhases';
import RecordCameraPreview from './RecordCameraPreview';

const LABELS = {
  cam_left_head: 'Head L',
  cam_right_head: 'Head R',
  cam_left_wrist: 'Wrist L',
  cam_right_wrist: 'Wrist R',
};
const ORDER = Object.keys(LABELS);
const buttonClass = 'rounded-full border px-3 py-1 text-xs disabled:opacity-40 disabled:cursor-not-allowed';

export default function RecordCameraPanel({ isActive = true }) {
  const robotType = useSelector((state) => state.tasks.robotType);
  const phase = useSelector((state) => state.tasks.recordStatus.recordPhase);
  const cameraTopics = useSelector((state) => state.tasks.recordingMonitor?.cameraTopics || []);
  const { callService } = useRosServiceCaller();
  const [config, setConfig] = useState(null);
  const [pending, setPending] = useState(false);
  const [error, setError] = useState('');
  const generation = useRef(0);
  const inFlight = useRef(false);

  const load = useCallback(async (enabledCameras) => {
    if (!robotType || inFlight.current) return;
    const current = generation.current;
    inFlight.current = true;
    setPending(true);
    setError('');
    try {
      const result = await callService(
        '/data/recording/cameras', 'interfaces/srv/RecordingCameras',
        { robot_type: robotType, apply: enabledCameras !== undefined, enabled_cameras: enabledCameras || [] }
      );
      if (generation.current !== current) return;
      if (!result?.success) throw new Error(result?.message || 'Camera settings unavailable');
      setConfig({ ...result, robotType });
    } catch (err) {
      if (generation.current === current) {
        // Re-read after a failed/uncertain write before allowing further edits.
        setConfig(null);
        setError(err.message || 'Camera settings unavailable');
      }
    } finally {
      if (generation.current === current) {
        inFlight.current = false;
        setPending(false);
      }
    }
  }, [callService, robotType]);

  useEffect(() => {
    generation.current += 1;
    inFlight.current = false;
    setConfig(null);
    setPending(false);
    setError('');
    if (isActive && robotType) load();
    return () => { generation.current += 1; };
  }, [isActive, robotType, load]);

  // Refresh server locks and selections after recording/saving ends.
  useEffect(() => {
    if (isActive && phase === RecordPhase.READY) load();
  }, [isActive, phase, load]);

  const currentConfig = config?.robotType === robotType ? config : null;
  const enabled = currentConfig?.enabled_cameras || [];
  const cameras = (currentConfig?.camera_names || []).map((name, index) => ({
    name,
    topic: currentConfig.image_topics[index],
    rotation: currentConfig.rotation_degrees[index] || 0,
  })).sort((a, b) => {
    const rank = (name) => ORDER.includes(name) ? ORDER.indexOf(name) : ORDER.length;
    return rank(a.name) - rank(b.name);
  });
  const locked = phase !== RecordPhase.READY || currentConfig?.locked;
  const disabled = pending || locked || !currentConfig || !isActive;
  const toggle = (name) => load(enabled.includes(name)
    ? enabled.filter((camera) => camera !== name)
    : [...enabled, name]);

  return (
    <section aria-label="Recording cameras" className="w-full h-full flex flex-col min-h-0 gap-2 px-1">
      <div className="flex flex-wrap items-center gap-2 shrink-0">
        <span className="text-sm font-semibold text-gray-700">Cameras {enabled.length}/{cameras.length}</span>
        {cameras.map(({ name }) => (
          <button key={name} type="button" role="switch" aria-checked={enabled.includes(name)}
            aria-label={LABELS[name] || name} disabled={disabled} onClick={() => toggle(name)}
            className={`${buttonClass} ${enabled.includes(name) ? 'bg-blue-100 border-blue-300 text-blue-800' : 'bg-gray-100 text-gray-500'}`}>
            {LABELS[name] || name} {enabled.includes(name) ? 'ON' : 'OFF'}
          </button>
        ))}
        <button type="button" className={buttonClass} disabled={disabled || enabled.length === cameras.length}
          onClick={() => load(cameras.map(({ name }) => name))}>All ON</button>
        <button type="button" className={buttonClass} disabled={disabled || enabled.length === 0}
          onClick={() => load([])}>All OFF</button>
        <button type="button" className={buttonClass} disabled={pending || !robotType || !isActive}
          onClick={() => load()}>Refresh</button>
        <span className="text-xs text-gray-500">
          {pending ? 'Applying…' : locked ? 'Locked while recording / saving' : 'ON cameras are previewed and recorded. Changes saved automatically.'}
        </span>
      </div>
      {error && <div role="alert" className="text-sm text-red-600">{error} — Refresh to retry.</div>}
      <div className="text-xs text-gray-500 shrink-0">Preview: up to 12 fps · Recording: original camera quality and frame rate</div>
      {!robotType && <p className="text-sm text-gray-500">Select a robot to configure cameras.</p>}
      {currentConfig && enabled.length === 0 && <p className="text-sm text-amber-700">All cameras OFF. Only non-camera data will be recorded.</p>}
      <div className="flex-1 min-h-0 grid gap-2 overflow-auto" style={{
        gridTemplateColumns: `repeat(${cameras.length > 1 ? 2 : 1}, minmax(0, 1fr))`,
        gridTemplateRows: `repeat(${Math.max(1, Math.ceil(cameras.length / 2))}, minmax(0, 1fr))`,
      }}>
        {cameras.map(({ name, topic, rotation }) => {
          const on = enabled.includes(name);
          const status = cameraTopics.find((item) => item.cameraName === name || item.name === topic);
          const live = status && Number(status.rateHz) > 0 && status.status !== 2;
          return (
            <div key={name} className="relative min-h-0 min-w-0 rounded-2xl overflow-hidden bg-gray-100" title={topic}>
              {on ? <RecordCameraPreview topic={topic} rotationDegrees={rotation} isActive={isActive} />
                : <div className="h-full flex items-center justify-center text-gray-400 text-sm">Camera OFF</div>}
              <div className="absolute bottom-2 left-2 rounded bg-black/60 px-2 py-1 text-xs text-white">
                {LABELS[name] || name} · {on ? live ? `${Number(status.rateHz).toFixed(1)} Hz` : 'Waiting for frames' : 'OFF'}
              </div>
            </div>
          );
        })}
      </div>
    </section>
  );
}
