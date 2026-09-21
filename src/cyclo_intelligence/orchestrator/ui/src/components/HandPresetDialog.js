import React, { useEffect, useMemo, useState } from 'react';
import clsx from 'clsx';
import toast from 'react-hot-toast';
import {
  MdAdd,
  MdClose,
  MdDelete,
  MdEdit,
  MdSave,
  MdTune,
  MdUndo,
  MdViewInAr,
} from 'react-icons/md';

import {
  buildCustomHandPresetRequest,
  buildHandPresetDeleteRequest,
  buildHandPresetRequest,
  buildHandPresetRestoreRequest,
  buildHandPresetUpdateRequest,
  HAND_PRESET_CUSTOM_SAVE_SERVICE,
  HAND_PRESET_DELETE_SERVICE,
  HAND_PRESET_RESTORE_SERVICE,
  HAND_PRESET_SERVICE,
  HAND_PRESET_SERVICE_TYPE,
  HAND_PRESET_UPDATE_SERVICE,
} from '../constants/handPresetProtocol';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';
import HandPreset3DViewer from './HandPreset3DViewer';

const SIDES = [
  { id: 'left', label: 'Left' },
  { id: 'both', label: 'Both' },
  { id: 'right', label: 'Right' },
];

function clamp(value) {
  return Math.max(0, Math.min(1, value));
}

function customTargetPositions(editor, curls) {
  const open = editor?.previewOpenPositions || [];
  const closed = editor?.previewClosedPositions || [];
  if (!open.length || open.length !== closed.length) return [];
  const jointsPerControl = open.length / curls.length;
  if (!Number.isInteger(jointsPerControl)) return [];
  return open.map((openPosition, index) => {
    const controlIndex = Math.min(curls.length - 1, Math.floor(index / jointsPerControl));
    return openPosition + curls[controlIndex] * (closed[index] - openPosition);
  });
}

function FingerCurlValues({ labels, values, exact = false, compact = false }) {
  if (!Array.isArray(values) || values.length !== 5) return null;
  return (
    <div>
      {!compact && (
        <div className="mb-1.5 flex items-center text-[10px] font-semibold uppercase tracking-wider text-gray-400 dark:text-slate-500">
          Finger closure
          <span className="ml-auto normal-case tracking-normal">
            {exact ? 'Saved values' : 'Approx. from joint pose'}
          </span>
        </div>
      )}
      <div className="grid grid-cols-5 gap-1">
        {values.map((value, index) => (
          <div
            key={labels[index] || index}
            className={clsx(
              'rounded-md text-center',
              compact
                ? 'bg-black/5 px-0.5 py-1 dark:bg-white/5'
                : 'bg-indigo-50 px-1 py-1.5 dark:bg-indigo-950/50'
            )}
          >
            <div className={clsx(
              'truncate font-semibold opacity-65',
              compact ? 'text-[8px]' : 'text-[9px]'
            )}>
              {compact ? (labels[index] || '').slice(0, 3) : labels[index]}
            </div>
            <div className={clsx(
              'font-mono font-bold',
              compact ? 'text-[9px]' : 'text-xs text-indigo-700 dark:text-indigo-300'
            )}>
              {Math.round(value * 100)}%
            </div>
          </div>
        ))}
      </div>
    </div>
  );
}

export default function HandPresetDialog({ status, isOnline, onClose }) {
  const { callService } = useRosServiceCaller();
  const [mode, setMode] = useState('presets');
  const [side, setSide] = useState('both');
  const [pendingPresetId, setPendingPresetId] = useState(null);
  const [saving, setSaving] = useState(false);
  const [managing, setManaging] = useState(false);
  const [deleting, setDeleting] = useState(false);
  const [previewPresetId, setPreviewPresetId] = useState(null);
  const [hoveredPresetId, setHoveredPresetId] = useState(null);
  const [customName, setCustomName] = useState('Custom grasp');
  const [customDescription, setCustomDescription] = useState('');
  const [curls, setCurls] = useState([0.75, 0.75, 0.35, 0.35, 0.35]);
  const [dragState, setDragState] = useState(null);
  const [editingPresetId, setEditingPresetId] = useState(null);
  const [editName, setEditName] = useState('');
  const [editDescription, setEditDescription] = useState('');
  const [editNote, setEditNote] = useState('');
  const [confirmDelete, setConfirmDelete] = useState(false);
  const [undoDeletion, setUndoDeletion] = useState(null);
  const [undoSeconds, setUndoSeconds] = useState(0);
  const [restoring, setRestoring] = useState(false);

  const presets = useMemo(() => status?.presets || [], [status?.presets]);
  const availableSides = useMemo(() => {
    const enabledSides = status?.enabledSides || [];
    const sides = [...enabledSides];
    if (enabledSides.includes('left') && enabledSides.includes('right')) {
      sides.splice(1, 0, 'both');
    }
    return new Set(sides);
  }, [status]);

  useEffect(() => {
    if (status && !availableSides.has(side)) setSide(status.enabledSides[0]);
  }, [availableSides, side, status]);

  useEffect(() => {
    if (!presets.length) return;
    if (!presets.some((preset) => preset.id === previewPresetId)) {
      const initialId = status?.rightPresetId ?? status?.leftPresetId ?? presets[0].id;
      setPreviewPresetId(initialId);
    }
  }, [presets, previewPresetId, status]);

  useEffect(() => {
    if (!dragState) return undefined;
    const handlePointerMove = (event) => {
      event.preventDefault();
      const delta = (dragState.startY - event.clientY) / 140;
      setCurls((values) => values.map((value, index) => (
        index === dragState.fingerIndex
          ? clamp(dragState.startValue + delta)
          : value
      )));
    };
    const stopDragging = () => setDragState(null);
    window.addEventListener('pointermove', handlePointerMove);
    window.addEventListener('pointerup', stopDragging);
    window.addEventListener('pointercancel', stopDragging);
    return () => {
      window.removeEventListener('pointermove', handlePointerMove);
      window.removeEventListener('pointerup', stopDragging);
      window.removeEventListener('pointercancel', stopDragging);
    };
  }, [dragState]);

  useEffect(() => {
    if (
      editingPresetId !== null
      && !presets.some((preset) => preset.id === editingPresetId)
    ) {
      setEditingPresetId(null);
    }
  }, [editingPresetId, presets]);

  useEffect(() => {
    if (!undoDeletion) return undefined;
    const updateCountdown = () => {
      const remaining = Math.max(
        0,
        Math.ceil((undoDeletion.expiresAt - Date.now()) / 1000)
      );
      setUndoSeconds(remaining);
      if (remaining === 0) setUndoDeletion(null);
    };
    updateCountdown();
    const timer = setInterval(updateCountdown, 250);
    return () => clearInterval(timer);
  }, [undoDeletion]);

  const previewPreset = presets.find((preset) => preset.id === previewPresetId) || presets[0];
  const editor = status?.customEditor;
  const controlLabels = editor?.controlLabels
    || ['Thumb', 'Index', 'Middle', 'Ring', 'Little'];
  const customPositions = useMemo(
    () => customTargetPositions(editor, curls),
    [curls, editor]
  );
  const preview = mode === 'custom' ? {
    name: customName || 'Untitled custom grasp',
    description: '3D-only preview. Save it first, then apply it from the preset list.',
    openPositions: editor?.previewOpenPositions || [],
    targetPositions: customPositions,
  } : {
    name: previewPreset?.name || 'Select a preset',
    description: previewPreset?.description || 'Hover over a preset to preview its motion.',
    openPositions: previewPreset?.previewOpenPositions || [],
    targetPositions: previewPreset?.previewPositions || [],
  };

  const isActivePreset = (presetId) => {
    if (side === 'left') return status?.leftPresetId === presetId;
    if (side === 'right') return status?.rightPresetId === presetId;
    return status?.leftPresetId === presetId && status?.rightPresetId === presetId;
  };

  const selectPreset = async (presetId) => {
    if (pendingPresetId !== null) return;
    setPreviewPresetId(presetId);
    if (!isOnline) {
      toast.error('Preset node is offline. Preview is available, but the robot cannot be changed.');
      return;
    }
    setPendingPresetId(presetId);
    try {
      const response = await callService(
        HAND_PRESET_SERVICE,
        HAND_PRESET_SERVICE_TYPE,
        buildHandPresetRequest(side, presetId),
        3000
      );
      if (!response?.result?.successful) {
        throw new Error(response?.result?.reason || 'Preset node rejected the request');
      }
      toast.success(`${side.toUpperCase()} hand preset ${presetId}`);
    } catch (error) {
      toast.error(`Preset change failed: ${error.message || error}`);
    } finally {
      setPendingPresetId(null);
    }
  };

  const saveCustomPreset = async () => {
    if (!isOnline || !editor || saving) return;
    setSaving(true);
    try {
      const response = await callService(
        HAND_PRESET_CUSTOM_SAVE_SERVICE,
        HAND_PRESET_SERVICE_TYPE,
        buildCustomHandPresetRequest(customName, customDescription, curls),
        3000
      );
      if (!response?.result?.successful) {
        throw new Error(response?.result?.reason || 'Preset node rejected the custom pose');
      }
      toast.success(`${response.result.reason}. Select it from Presets to apply.`);
      setMode('presets');
    } catch (error) {
      toast.error(`Custom preset save failed: ${error.message || error}`);
    } finally {
      setSaving(false);
    }
  };

  const openPresetEditor = (preset) => {
    setPreviewPresetId(preset.id);
    setHoveredPresetId(null);
    setEditingPresetId(preset.id);
    setEditName(preset.name);
    setEditDescription(preset.description || '');
    setEditNote(preset.note || '');
    setConfirmDelete(false);
  };

  const updatePreset = async () => {
    if (!isOnline || managing || editingPresetId === null) return;
    setManaging(true);
    try {
      const response = await callService(
        HAND_PRESET_UPDATE_SERVICE,
        HAND_PRESET_SERVICE_TYPE,
        buildHandPresetUpdateRequest(
          editingPresetId,
          editName,
          editDescription,
          editNote
        ),
        3000
      );
      if (!response?.result?.successful) {
        throw new Error(response?.result?.reason || 'Preset node rejected the update');
      }
      toast.success(response.result.reason);
      setEditingPresetId(null);
    } catch (error) {
      toast.error(`Preset update failed: ${error.message || error}`);
    } finally {
      setManaging(false);
    }
  };

  const deletePreset = async () => {
    if (!isOnline || deleting || editingPresetId === null) return;
    const deletedPreset = editingPreset;
    setDeleting(true);
    try {
      const response = await callService(
        HAND_PRESET_DELETE_SERVICE,
        HAND_PRESET_SERVICE_TYPE,
        buildHandPresetDeleteRequest(editingPresetId),
        3000
      );
      if (!response?.result?.successful) {
        throw new Error(response?.result?.reason || 'Preset node rejected the deletion');
      }
      setUndoDeletion({
        id: deletedPreset.id,
        name: deletedPreset.name,
        expiresAt: Date.now() + 10000,
      });
      setUndoSeconds(10);
      setPreviewPresetId(null);
      setEditingPresetId(null);
    } catch (error) {
      toast.error(`Preset delete failed: ${error.message || error}`);
    } finally {
      setDeleting(false);
      setConfirmDelete(false);
    }
  };

  const undoPresetDeletion = async () => {
    if (!undoDeletion || restoring) return;
    setRestoring(true);
    try {
      const response = await callService(
        HAND_PRESET_RESTORE_SERVICE,
        HAND_PRESET_SERVICE_TYPE,
        buildHandPresetRestoreRequest(undoDeletion.id),
        3000
      );
      if (!response?.result?.successful) {
        throw new Error(response?.result?.reason || 'Preset node rejected the undo');
      }
      toast.success(response.result.reason);
      setPreviewPresetId(undoDeletion.id);
      setUndoDeletion(null);
    } catch (error) {
      toast.error(`Preset undo failed: ${error.message || error}`);
      setUndoDeletion(null);
    } finally {
      setRestoring(false);
    }
  };

  const loadPresetIntoCustom = (preset) => {
    if (!editor || preset?.editorCurls?.length !== 5) {
      toast.error('This preset does not provide five-finger custom values.');
      return;
    }
    setCurls([...preset.editorCurls]);
    setCustomName(`${preset.name} copy`.slice(0, 48));
    setCustomDescription(preset.description || '');
    setMode('custom');
    setHoveredPresetId(null);
    setEditingPresetId(null);
  };

  const editingPreset = presets.find((preset) => preset.id === editingPresetId) || null;

  return (
    <div
      className="fixed inset-0 z-50 flex items-center justify-center bg-slate-950/55 p-4 backdrop-blur-[2px] dark:bg-black/75"
      onMouseDown={(event) => {
        if (event.target === event.currentTarget) onClose();
      }}
    >
      <div
        role="dialog"
        aria-modal="true"
        aria-labelledby="hand-preset-dialog-title"
        className="relative flex max-h-[94vh] w-full max-w-6xl flex-col overflow-hidden rounded-2xl border border-gray-200 bg-white shadow-2xl dark:border-slate-700 dark:bg-slate-900"
      >
        {undoDeletion && (
          <div className="absolute left-1/2 top-3 z-50 flex w-[min(92%,520px)] -translate-x-1/2 items-center gap-3 rounded-xl border border-emerald-300 bg-white px-4 py-3 text-gray-800 shadow-2xl dark:border-emerald-700 dark:bg-slate-800 dark:text-slate-100">
            <div className="flex h-9 w-9 shrink-0 items-center justify-center rounded-full bg-emerald-100 text-emerald-700 dark:bg-emerald-950 dark:text-emerald-300">
              <MdUndo size={20} />
            </div>
            <div className="min-w-0 flex-1">
              <div className="truncate text-sm font-semibold">Deleted “{undoDeletion.name}”</div>
              <div className="text-xs text-gray-500 dark:text-slate-400">Undo available for {undoSeconds}s</div>
            </div>
            <button
              type="button"
              disabled={restoring}
              onClick={undoPresetDeletion}
              className="rounded-lg bg-emerald-600 px-3 py-2 text-xs font-bold text-white hover:bg-emerald-700 disabled:opacity-50"
              aria-label="Undo preset deletion"
            >
              {restoring ? 'Restoring…' : 'Undo'}
            </button>
          </div>
        )}
        <header className="flex shrink-0 items-center gap-4 border-b border-gray-200 bg-gradient-to-r from-indigo-50 via-white to-blue-50 px-6 py-4 dark:border-slate-700 dark:from-slate-800 dark:via-slate-900 dark:to-indigo-950/80">
          <div className="flex h-11 w-11 items-center justify-center rounded-xl bg-indigo-600 text-white shadow-sm dark:bg-indigo-500">
            <MdViewInAr size={24} />
          </div>
          <div>
            <h2 id="hand-preset-dialog-title" className="text-xl font-bold text-gray-900 dark:text-slate-100">
              Hand preset studio
            </h2>
            <p className="text-sm text-gray-500 dark:text-slate-400">
              Preview a grasp in 3D or create a custom finger pose.
            </p>
          </div>
          <div className="ml-auto flex items-center gap-2">
            <span className={clsx('h-2.5 w-2.5 rounded-full', isOnline ? 'bg-emerald-500' : 'bg-gray-300')} />
            <span className={clsx(
              'hidden text-xs font-semibold sm:inline',
              isOnline ? 'text-emerald-700 dark:text-emerald-400' : 'text-gray-500 dark:text-slate-400'
            )}>
              {isOnline ? 'Preset node online' : 'Waiting for preset node'}
            </span>
            <button
              type="button"
              onClick={onClose}
              className="ml-2 flex h-9 w-9 items-center justify-center rounded-lg text-gray-500 hover:bg-gray-100 hover:text-gray-800 focus:outline-none focus:ring-2 focus:ring-indigo-300 dark:text-slate-400 dark:hover:bg-slate-800 dark:hover:text-white"
              aria-label="Close hand preset selector"
            >
              <MdClose size={22} />
            </button>
          </div>
        </header>

        <div className="grid shrink-0 grid-cols-2 border-b border-gray-200 bg-white dark:border-slate-700 dark:bg-slate-900">
          <button
            type="button"
            onClick={() => setMode('presets')}
            className={clsx(
              'flex h-12 items-center justify-center gap-2 border-b-2 text-sm font-semibold transition-colors',
              mode === 'presets'
                ? 'border-indigo-600 text-indigo-700 dark:border-indigo-400 dark:text-indigo-300'
                : 'border-transparent text-gray-500 hover:bg-gray-50 dark:text-slate-400 dark:hover:bg-slate-800'
            )}
          >
            <MdViewInAr size={18} /> Registered presets
          </button>
          <button
            type="button"
            onClick={() => setMode('custom')}
            disabled={!editor}
            className={clsx(
              'flex h-12 items-center justify-center gap-2 border-b-2 text-sm font-semibold transition-colors disabled:cursor-not-allowed disabled:opacity-40',
              mode === 'custom'
                ? 'border-indigo-600 text-indigo-700 dark:border-indigo-400 dark:text-indigo-300'
                : 'border-transparent text-gray-500 hover:bg-gray-50 dark:text-slate-400 dark:hover:bg-slate-800'
            )}
            title={editor ? 'Create a custom preset' : 'Restart ai_worker to enable custom presets'}
          >
            <MdAdd size={19} /> Create custom
          </button>
        </div>

        <div className="min-h-0 flex-1 overflow-y-auto bg-gray-50/80 p-5 dark:bg-slate-950/80">
          <div className="grid gap-5 lg:grid-cols-[minmax(0,1.1fr)_minmax(420px,0.9fr)]">
            <section className="min-w-0">
              <HandPreset3DViewer
                jointNames={status?.previewJointNames || []}
                openPositions={preview.openPositions}
                targetPositions={preview.targetPositions}
                animate={mode === 'custom' ? !dragState : hoveredPresetId !== null}
                editable={mode === 'custom'}
                dragging={Boolean(dragState)}
                draggingFingerIndex={dragState?.fingerIndex ?? null}
                fingerLabels={editor?.controlLabels || []}
                fingerValues={curls}
                onFingerPointerDown={(fingerIndex, startY) => setDragState({
                  fingerIndex,
                  startY,
                  startValue: curls[fingerIndex],
                })}
                className="h-[430px]"
              />
              <div className="mt-3 rounded-xl border border-gray-200 bg-white px-4 py-3 dark:border-slate-700 dark:bg-slate-900">
                <div className="flex items-center gap-3">
                  <div className="min-w-0 flex-1 truncate font-semibold text-gray-900 dark:text-slate-100">{preview.name}</div>
                  {mode === 'presets' && previewPreset?.editorCurls?.length === 5 && (
                    <button
                      type="button"
                      onClick={() => loadPresetIntoCustom(previewPreset)}
                      className="flex shrink-0 items-center gap-1.5 rounded-lg bg-indigo-100 px-3 py-2 text-xs font-semibold text-indigo-700 hover:bg-indigo-200 dark:bg-indigo-950 dark:text-indigo-300 dark:hover:bg-indigo-900"
                      aria-label={`Load hand preset ${previewPreset.id} into custom editor`}
                    >
                      <MdTune size={16} /> Load into Custom
                    </button>
                  )}
                </div>
                <div className="mt-1 text-sm leading-5 text-gray-500 dark:text-slate-400">
                  {preview.description}
                </div>
                {mode === 'presets' && (
                  <div className="mt-3 border-t border-gray-100 pt-3 dark:border-slate-700">
                    <FingerCurlValues
                      labels={controlLabels}
                      values={previewPreset?.editorCurls}
                      exact={previewPreset?.editorCurlsExact}
                    />
                  </div>
                )}
                {mode === 'presets' && previewPreset?.note && (
                  <div className="mt-2 rounded-lg bg-amber-50 px-3 py-2 text-xs leading-5 text-amber-800 dark:bg-amber-950/50 dark:text-amber-200">
                    <span className="font-semibold">Note:</span> {previewPreset.note}
                  </div>
                )}
              </div>
            </section>

            {mode === 'presets' ? (
              <section className="min-w-0">
                <div className="grid grid-cols-2 gap-3">
                  <div className="rounded-xl border border-gray-200 bg-white px-3 py-2.5 dark:border-slate-700 dark:bg-slate-900">
                    <div className="text-[10px] font-semibold uppercase tracking-wider text-gray-400 dark:text-slate-500">Left hand</div>
                    <div className="mt-1 flex items-baseline gap-2">
                      <span className="font-mono text-xl font-bold text-gray-900 dark:text-white">{status?.leftPresetId ?? '—'}</span>
                      <span className="truncate text-xs text-indigo-600 dark:text-indigo-300">
                        {presets.find((preset) => preset.id === status?.leftPresetId)?.name || ''}
                      </span>
                    </div>
                  </div>
                  <div className="rounded-xl border border-gray-200 bg-white px-3 py-2.5 dark:border-slate-700 dark:bg-slate-900">
                    <div className="text-[10px] font-semibold uppercase tracking-wider text-gray-400 dark:text-slate-500">Right hand</div>
                    <div className="mt-1 flex items-baseline gap-2">
                      <span className="font-mono text-xl font-bold text-gray-900 dark:text-white">{status?.rightPresetId ?? '—'}</span>
                      <span className="truncate text-xs text-indigo-600 dark:text-indigo-300">
                        {presets.find((preset) => preset.id === status?.rightPresetId)?.name || ''}
                      </span>
                    </div>
                  </div>
                </div>

                <div className="mt-4 text-xs font-semibold uppercase tracking-wider text-gray-500 dark:text-slate-400">Apply to</div>
                <div className="mt-2 grid grid-cols-3 gap-2 rounded-xl border border-gray-200 bg-white p-1.5 dark:border-slate-700 dark:bg-slate-900">
                  {SIDES.map((option) => (
                    <button
                      key={option.id}
                      type="button"
                      disabled={!isOnline || !availableSides.has(option.id)}
                      onClick={() => setSide(option.id)}
                      className={clsx(
                        'h-10 rounded-lg text-sm font-semibold transition-colors disabled:cursor-not-allowed disabled:opacity-35',
                        side === option.id
                          ? 'bg-indigo-600 text-white shadow-sm'
                          : 'text-gray-600 hover:bg-indigo-50 hover:text-indigo-700 dark:text-slate-300 dark:hover:bg-indigo-950/60 dark:hover:text-indigo-300'
                      )}
                      aria-label={`${option.id} hand preset target`}
                    >
                      {option.label}
                    </button>
                  ))}
                </div>

                <div className="mt-4 flex items-center">
                  <div className="text-xs font-semibold uppercase tracking-wider text-gray-500 dark:text-slate-400">Presets</div>
                  <div className="ml-auto text-xs text-gray-400 dark:text-slate-500">Hover to animate · Click to apply</div>
                </div>
                <div className="mt-2 grid max-h-[360px] grid-cols-2 gap-2 overflow-y-auto pr-1">
                  {presets.map((preset) => {
                    const active = isActivePreset(preset.id);
                    const previewing = previewPresetId === preset.id;
                    return (
                      <div
                        key={preset.id}
                        onMouseEnter={() => {
                          setPreviewPresetId(preset.id);
                          setHoveredPresetId(preset.id);
                        }}
                        onMouseLeave={() => setHoveredPresetId(null)}
                        className={clsx(
                          'relative min-h-[152px] overflow-hidden rounded-xl border text-left transition-all',
                          active
                            ? 'border-indigo-500 bg-indigo-600 text-white shadow-md shadow-indigo-200/70 dark:shadow-none'
                            : previewing
                              ? 'border-indigo-400 bg-indigo-50 text-indigo-800 dark:border-indigo-500 dark:bg-indigo-950/70 dark:text-indigo-200'
                              : 'border-gray-200 bg-white text-gray-700 hover:-translate-y-0.5 hover:border-indigo-300 dark:border-slate-700 dark:bg-slate-900 dark:text-slate-200'
                        )}
                      >
                        <button
                          type="button"
                          disabled={pendingPresetId !== null}
                          aria-disabled={!isOnline || pendingPresetId !== null}
                          onFocus={() => setPreviewPresetId(preset.id)}
                          onClick={() => selectPreset(preset.id)}
                          className="block min-h-[152px] w-full p-3 pr-11 text-left disabled:cursor-not-allowed disabled:opacity-45"
                          aria-label={`Apply hand preset ${preset.id}`}
                        >
                          <div className="flex items-center gap-2">
                            <span className={clsx(
                              'rounded-md px-2 py-1 font-mono text-xs font-bold',
                              active ? 'bg-white/15' : 'bg-indigo-100 dark:bg-indigo-950'
                            )}>
                              {pendingPresetId === preset.id ? '…' : preset.id}
                            </span>
                            <span className="min-w-0 truncate text-sm font-bold">{preset.name}</span>
                            {preset.custom && (
                              <span className="rounded-full bg-emerald-500/15 px-1.5 py-0.5 text-[9px] font-bold uppercase text-emerald-600 dark:text-emerald-300">Custom</span>
                            )}
                          </div>
                          <p className="mt-2 line-clamp-2 text-xs leading-4 opacity-75">
                            {preset.description || 'No description provided.'}
                          </p>
                          <div className="mt-2">
                            <FingerCurlValues
                              labels={controlLabels}
                              values={preset.editorCurls}
                              exact={preset.editorCurlsExact}
                              compact
                            />
                          </div>
                        </button>
                        <button
                          type="button"
                          onClick={() => openPresetEditor(preset)}
                          className={clsx(
                            'absolute right-2 top-2 flex h-8 w-8 items-center justify-center rounded-lg transition-colors focus:outline-none focus:ring-2 focus:ring-indigo-300',
                            active
                              ? 'bg-white/15 text-white hover:bg-white/25'
                              : 'bg-gray-100 text-gray-500 hover:bg-indigo-100 hover:text-indigo-700 dark:bg-slate-800 dark:text-slate-300 dark:hover:bg-indigo-950'
                          )}
                          aria-label={`Edit hand preset ${preset.id}`}
                          title="Edit name, description, or note"
                        >
                          <MdEdit size={16} />
                        </button>
                      </div>
                    );
                  })}
                </div>
                <div className="mt-3 rounded-xl border border-amber-200 bg-amber-50 px-3 py-2.5 text-xs leading-5 text-amber-800 dark:border-amber-700/60 dark:bg-amber-950/50 dark:text-amber-200">
                  Release the selected mini-leader trigger before applying a preset.
                </div>
              </section>
            ) : (
              <section className="rounded-2xl border border-gray-200 bg-white p-4 dark:border-slate-700 dark:bg-slate-900">
                <div className="flex items-center justify-between">
                  <div>
                    <h3 className="font-bold text-gray-900 dark:text-slate-100">Create custom grasp</h3>
                    <p className="mt-0.5 text-xs text-gray-500 dark:text-slate-400">The 3D hand repeatedly opens and closes to the exact percentages below.</p>
                  </div>
                  <div className="flex gap-1">
                    <button type="button" onClick={() => setCurls([0, 0, 0, 0, 0])} className="rounded-lg bg-gray-100 px-2.5 py-1.5 text-[11px] font-semibold text-gray-600 hover:bg-gray-200 dark:bg-slate-800 dark:text-slate-300">Open</button>
                    <button type="button" onClick={() => setCurls([1, 1, 0, 0, 0])} className="rounded-lg bg-gray-100 px-2.5 py-1.5 text-[11px] font-semibold text-gray-600 hover:bg-gray-200 dark:bg-slate-800 dark:text-slate-300">Pinch</button>
                    <button type="button" onClick={() => setCurls([1, 1, 1, 1, 1])} className="rounded-lg bg-gray-100 px-2.5 py-1.5 text-[11px] font-semibold text-gray-600 hover:bg-gray-200 dark:bg-slate-800 dark:text-slate-300">Close</button>
                  </div>
                </div>

                <div className="mt-4 space-y-3">
                  {(editor?.controlLabels || ['Thumb', 'Index', 'Middle', 'Ring', 'Little']).map((label, index) => (
                    <label key={label} className="block">
                      <div className="mb-1 flex items-center text-xs">
                        <span className="font-semibold text-gray-700 dark:text-slate-200">{label}</span>
                        <span className="ml-auto font-mono text-gray-400 dark:text-slate-500">{Math.round(curls[index] * 100)}%</span>
                      </div>
                      <input
                        type="range"
                        min="0"
                        max="100"
                        value={Math.round(curls[index] * 100)}
                        onChange={(event) => {
                          const value = Number(event.target.value) / 100;
                          setCurls((values) => values.map((current, currentIndex) => (
                            currentIndex === index ? value : current
                          )));
                        }}
                        className="h-2 w-full cursor-pointer appearance-none rounded-lg bg-gray-200 accent-indigo-600 dark:bg-slate-700 dark:accent-indigo-400"
                        aria-label={`${label} curl`}
                      />
                    </label>
                  ))}
                </div>

                <div className="mt-5 border-t border-gray-200 pt-4 dark:border-slate-700">
                  <label className="block text-xs font-semibold text-gray-700 dark:text-slate-200" htmlFor="custom-preset-name">Preset name</label>
                  <input
                    id="custom-preset-name"
                    value={customName}
                    maxLength={48}
                    onChange={(event) => setCustomName(event.target.value)}
                    className="mt-1.5 h-10 w-full rounded-lg border border-gray-300 bg-white px-3 text-sm text-gray-900 outline-none focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200 dark:border-slate-600 dark:bg-slate-950 dark:text-slate-100 dark:focus:ring-indigo-900"
                  />
                  <label className="mt-3 block text-xs font-semibold text-gray-700 dark:text-slate-200" htmlFor="custom-preset-description">Description</label>
                  <textarea
                    id="custom-preset-description"
                    value={customDescription}
                    maxLength={160}
                    rows={2}
                    placeholder="What is this grasp useful for?"
                    onChange={(event) => setCustomDescription(event.target.value)}
                    className="mt-1.5 w-full resize-none rounded-lg border border-gray-300 bg-white px-3 py-2 text-sm text-gray-900 outline-none focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200 dark:border-slate-600 dark:bg-slate-950 dark:text-slate-100 dark:focus:ring-indigo-900"
                  />
                  <button
                    type="button"
                    disabled={!isOnline || !editor || saving || !customName.trim()}
                    onClick={saveCustomPreset}
                    className="mt-3 flex h-11 w-full items-center justify-center gap-2 rounded-lg bg-indigo-600 text-sm font-semibold text-white shadow-sm hover:bg-indigo-700 disabled:cursor-not-allowed disabled:opacity-45 dark:bg-indigo-500 dark:hover:bg-indigo-400"
                    aria-label="Save custom hand preset"
                  >
                    <MdSave size={18} /> {saving ? 'Saving…' : 'Save custom preset'}
                  </button>
                  <p className="mt-2 text-[11px] leading-4 text-gray-400 dark:text-slate-500">
                    Saving does not move the robot. The pose is stored by ai_worker and appears as a new preset card.
                  </p>
                </div>
              </section>
            )}
          </div>
        </div>

        {editingPreset && (
          <div className="absolute inset-0 z-30 flex items-center justify-center bg-slate-950/60 p-4 backdrop-blur-[2px]">
            <section className="w-full max-w-lg rounded-2xl border border-gray-200 bg-white p-5 shadow-2xl dark:border-slate-700 dark:bg-slate-900">
              <div className="flex items-start gap-3">
                <div>
                  <div className="text-xs font-bold uppercase tracking-wider text-indigo-600 dark:text-indigo-300">Preset {editingPreset.id}</div>
                  <h3 className="mt-1 text-lg font-bold text-gray-900 dark:text-slate-100">Edit preset details</h3>
                </div>
                <button
                  type="button"
                  onClick={() => setEditingPresetId(null)}
                  className="ml-auto flex h-9 w-9 items-center justify-center rounded-lg text-gray-500 hover:bg-gray-100 dark:text-slate-400 dark:hover:bg-slate-800"
                  aria-label="Close preset editor"
                >
                  <MdClose size={20} />
                </button>
              </div>

              <label className="mt-4 block text-xs font-semibold text-gray-700 dark:text-slate-200" htmlFor="edit-preset-name">Name</label>
              <input
                id="edit-preset-name"
                value={editName}
                maxLength={48}
                onChange={(event) => setEditName(event.target.value)}
                className="mt-1.5 h-10 w-full rounded-lg border border-gray-300 bg-white px-3 text-sm text-gray-900 outline-none focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200 dark:border-slate-600 dark:bg-slate-950 dark:text-slate-100 dark:focus:ring-indigo-900"
              />
              <label className="mt-3 block text-xs font-semibold text-gray-700 dark:text-slate-200" htmlFor="edit-preset-description">Description</label>
              <textarea
                id="edit-preset-description"
                value={editDescription}
                maxLength={160}
                rows={2}
                onChange={(event) => setEditDescription(event.target.value)}
                className="mt-1.5 w-full resize-none rounded-lg border border-gray-300 bg-white px-3 py-2 text-sm text-gray-900 outline-none focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200 dark:border-slate-600 dark:bg-slate-950 dark:text-slate-100 dark:focus:ring-indigo-900"
              />
              <label className="mt-3 block text-xs font-semibold text-gray-700 dark:text-slate-200" htmlFor="edit-preset-note">Operator note</label>
              <textarea
                id="edit-preset-note"
                value={editNote}
                maxLength={240}
                rows={3}
                placeholder="Add an object, task, or usage memo."
                onChange={(event) => setEditNote(event.target.value)}
                className="mt-1.5 w-full resize-none rounded-lg border border-gray-300 bg-white px-3 py-2 text-sm text-gray-900 outline-none focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200 dark:border-slate-600 dark:bg-slate-950 dark:text-slate-100 dark:focus:ring-indigo-900"
              />

              <div className="mt-5 flex gap-2">
                <button
                  type="button"
                  disabled={!isOnline || managing || !editName.trim()}
                  onClick={updatePreset}
                  className="flex h-11 flex-1 items-center justify-center gap-2 rounded-lg bg-indigo-600 text-sm font-semibold text-white hover:bg-indigo-700 disabled:cursor-not-allowed disabled:opacity-45 dark:bg-indigo-500 dark:hover:bg-indigo-400"
                  aria-label="Save preset details"
                >
                  <MdSave size={18} /> {managing ? 'Saving…' : 'Save changes'}
                </button>
                <button
                  type="button"
                  disabled={!isOnline || deleting || !editingPreset.deletable}
                  onClick={() => setConfirmDelete(true)}
                  className="flex h-11 items-center justify-center gap-2 rounded-lg border border-rose-200 px-4 text-sm font-semibold text-rose-600 hover:bg-rose-50 disabled:cursor-not-allowed disabled:opacity-40 dark:border-rose-800 dark:text-rose-300 dark:hover:bg-rose-950/50"
                  aria-label={`Delete hand preset ${editingPreset.id}`}
                  title={editingPreset.deleteReason || 'Delete this preset'}
                >
                  <MdDelete size={18} /> Delete
                </button>
              </div>
              {!editingPreset.deletable && (
                <p className="mt-2 text-xs leading-5 text-amber-700 dark:text-amber-300">
                  {editingPreset.deleteReason
                    || 'Deletion information is unavailable. Restart the updated ai_worker preset node.'}
                </p>
              )}
              {confirmDelete && (
                <div className="mt-3 rounded-xl border border-rose-200 bg-rose-50 p-3 dark:border-rose-800 dark:bg-rose-950/50">
                  <p className="text-sm font-semibold text-rose-800 dark:text-rose-200">
                    Delete preset {editingPreset.id}?
                  </p>
                  <p className="mt-1 text-xs text-rose-600 dark:text-rose-300">The affected hand safely switches to Open. You can undo for 10 seconds afterward.</p>
                  <div className="mt-3 flex justify-end gap-2">
                    <button type="button" onClick={() => setConfirmDelete(false)} className="rounded-lg px-3 py-2 text-xs font-semibold text-gray-600 hover:bg-white dark:text-slate-300 dark:hover:bg-slate-800">Cancel</button>
                    <button type="button" disabled={deleting} onClick={deletePreset} className="rounded-lg bg-rose-600 px-3 py-2 text-xs font-semibold text-white hover:bg-rose-700 disabled:opacity-50" aria-label="Confirm delete hand preset">
                      {deleting ? 'Deleting…' : 'Yes, delete'}
                    </button>
                  </div>
                </div>
              )}
            </section>
          </div>
        )}

        <footer className="flex shrink-0 items-center border-t border-gray-200 bg-white px-6 py-3 text-xs text-gray-400 dark:border-slate-700 dark:bg-slate-900 dark:text-slate-500">
          <span>Preset poses and custom files are owned by ai_worker.</span>
          <span className="ml-auto hidden font-mono text-[11px] sm:inline">/leader/hand_preset/set</span>
        </footer>
      </div>
    </div>
  );
}
