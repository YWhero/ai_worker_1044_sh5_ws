import React, { useRef, useState } from 'react';
import { shallowEqual, useDispatch, useSelector } from 'react-redux';
import { MdFolderOpen, MdSkipNext } from 'react-icons/md';
import FileBrowserModal from './FileBrowserModal';
import { DEFAULT_PATHS } from '../constants/paths';
import { InferencePhase } from '../constants/taskPhases';
import {
  markLocalTaskInfoEdited,
  selectInferenceTaskInfo,
  setInferenceModelSwitch,
  setInferenceTaskInfo,
} from '../features/tasks/taskSlice';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';

const normalizePath = (path) => String(path || '').trim().replace(/\/+$/, '');

export default function InferenceModelSwitch() {
  const dispatch = useDispatch();
  const info = useSelector(selectInferenceTaskInfo, shallowEqual);
  const robotType = useSelector((state) => state.tasks.robotType);
  const phase = useSelector((state) => state.tasks.inferenceStatus.inferencePhase);
  const state = useSelector((store) => store.tasks.inferenceModelSwitch);
  const { sendRecordCommand } = useRosServiceCaller();
  const [showBrowser, setShowBrowser] = useState(false);
  const inFlight = useRef(false);
  const { nextPolicyPath, busy, preloadBusy, usePreload, preloadedPath, needsClear, message, error } = state;

  if (robotType !== 'ffw_sh5_rev1' || info.serviceType !== 'vitacformer') return null;

  const nextPath = normalizePath(nextPolicyPath);
  const active = [InferencePhase.INFERENCING, InferencePhase.PAUSED].includes(phase);
  const preparingAllowed = [InferencePhase.READY, InferencePhase.PAUSED].includes(phase);
  const blocked = busy || preloadBusy || needsClear;
  const differentTarget = nextPath && nextPath !== normalizePath(info.policyPath);
  const preloaded = Boolean(nextPath && nextPath === normalizePath(preloadedPath));
  const canSwitch = active && !blocked && differentTarget && (!usePreload || preloaded);
  const canPreload = preparingAllowed && !blocked && differentTarget && !preloaded;
  const changePath = (path) => dispatch(setInferenceModelSwitch({
    nextPolicyPath: path, message: '', error: '',
  }));

  const handlePreload = async (release = false) => {
    if (inFlight.current || (release ? !preparingAllowed || blocked : !canPreload)) return;
    inFlight.current = true;
    dispatch(setInferenceModelSwitch({
      preloadBusy: true, preloadedPath: '', error: '',
      message: release ? 'Releasing preloaded model…' : 'Preloading and warming up next model…',
    }));
    try {
      const result = await sendRecordCommand(release ? 'clear_preloaded_model' : 'preload_inference_model', {
        policyPath: nextPath, inferenceMode: info.inferenceMode || 'simulation',
      });
      if (!result?.success) throw new Error(result?.message || 'Model preparation failed');
      dispatch(setInferenceModelSwitch({
        preloadedPath: release ? '' : nextPath,
        message: result.message || (release ? 'Preloaded model released' : 'Preload ready; warmup complete'),
      }));
    } catch (err) {
      dispatch(setInferenceModelSwitch({ preloadedPath: '', message: '', error: err.message || String(err) }));
    } finally {
      inFlight.current = false;
      dispatch(setInferenceModelSwitch({ preloadBusy: false }));
    }
  };

  const handleSwitch = async () => {
    if (!canSwitch || inFlight.current) return;
    inFlight.current = true;
    let receivedResponse = false;
    dispatch(setInferenceModelSwitch({ busy: true, error: '',
      message: usePreload ? 'Activating preloaded model…' : 'Pausing and loading next model…' }));
    try {
      const result = await sendRecordCommand(usePreload ? 'switch_preloaded_model' : 'switch_inference_model', {
        policyPath: nextPath,
        inferenceMode: info.inferenceMode || 'simulation',
      });
      receivedResponse = Boolean(result);
      // The next model may have loaded even if its first motion failed preflight.
      // Display that authoritative path so Start resumes the correct model.
      if (result?.loaded_policy_path) {
        dispatch(setInferenceTaskInfo({ policyPath: result.loaded_policy_path }));
        dispatch(markLocalTaskInfoEdited({ source: 'inference' }));
      }
      dispatch(setInferenceModelSwitch({
        preloadedPath: usePreload && normalizePath(result?.loaded_policy_path) === nextPath
          ? normalizePath(info.policyPath) : '',
      }));
      // Lifecycle comes from the status topic: a Stop may have published a
      // newer PAUSED status after this response was constructed.
      if (!result?.success) throw new Error(result?.message || 'Model switch failed');
      dispatch(setInferenceModelSwitch({
        message: result.message || 'Next model is running',
        error: '',
        nextPolicyPath: '',
      }));
    } catch (err) {
      // Do not fabricate READY or retry a motion command after transport loss.
      dispatch(setInferenceModelSwitch({
        message: '', error: err.message || String(err), needsClear: !receivedResponse,
        ...(!receivedResponse ? { preloadedPath: '' } : {}),
      }));
    } finally {
      inFlight.current = false;
      dispatch(setInferenceModelSwitch({ busy: false }));
    }
  };

  const browserPath = DEFAULT_PATHS.VITACFORMER_CHECKPOINTS_PATH;
  return (
    <section className="mb-4 rounded-lg border border-indigo-200 bg-indigo-50 p-3" aria-label="Manual model switching">
      <h3 className="text-sm font-semibold text-indigo-950">Manual model switching</h3>
      <p className="mt-1 text-xs text-gray-600">
        Run model 1 using Policy Path above. When pouring is complete, switch to model 2 here.
      </p>
      <div className="mt-2 text-xs text-gray-600 break-all">
        Current model: <span className="font-medium">{needsClear ? 'Unconfirmed after connection loss' : info.policyPath || 'Select model 1 above'}</span>
      </div>
      <label htmlFor="next-inference-model" className="mt-3 block text-xs font-semibold text-gray-700">Next model path</label>
      <div className="mt-1 flex gap-1">
        <input
          id="next-inference-model"
          className="min-w-0 flex-1 rounded border border-gray-300 px-2 py-1.5 text-xs disabled:opacity-50"
          value={nextPolicyPath}
          onChange={(event) => changePath(event.target.value)}
          disabled={blocked}
          placeholder="/workspace/model/lerobot/…/model_2"
        />
        <button type="button" aria-label="Browse next model" disabled={blocked}
          className="rounded border border-gray-300 bg-white px-2 disabled:opacity-50"
          onClick={() => setShowBrowser(true)}><MdFolderOpen /></button>
      </div>
      <label className="mt-3 flex items-center gap-2 text-xs font-semibold text-gray-700">
        <input type="checkbox" checked={Boolean(usePreload)} disabled={blocked}
          onChange={(event) => dispatch(setInferenceModelSwitch({ usePreload: event.target.checked, message: '', error: '' }))} />
        Preload next model for fast switching
      </label>
      {usePreload && <div className="mt-2 text-xs text-gray-600">
        <p>Preload before Start, or press Stop to pause first. Both models stay in memory.</p>
        <div className="mt-2 flex gap-2">
          <button type="button" disabled={!canPreload} onClick={() => handlePreload()}
            className="rounded border border-indigo-300 bg-white px-2 py-1.5 disabled:opacity-40">
            {preloadBusy ? 'Preparing model…' : 'Preload next model'}
          </button>
          <button type="button" disabled={!preparingAllowed || blocked} onClick={() => handlePreload(true)}
            className="rounded border border-gray-300 bg-white px-2 py-1.5 disabled:opacity-40">Release preload</button>
        </div>
        {preloaded && <p className="mt-2 font-semibold text-green-700">Preload ready · warmup complete</p>}
        {!preloaded && <p className="mt-2">Preload the selected next model to enable fast switching.</p>}
        {preloadedPath && !preloaded && <p className="mt-1 break-all">Model retained in memory: {preloadedPath}</p>}
      </div>}
      <button type="button" disabled={!canSwitch} onClick={handleSwitch}
        className="mt-3 flex w-full items-center justify-center gap-1 rounded-md bg-indigo-600 px-3 py-2 text-sm font-semibold text-white disabled:cursor-not-allowed disabled:opacity-40">
        <MdSkipNext size={20} />{busy ? 'Switching model…' : 'Switch to next model'}
      </button>
      <p className="mt-2 text-xs text-gray-600">
        {usePreload ? 'Switches without loading weights; computes a fresh action and checks it before resuming' : 'Motion pauses while loading, then resumes'}
        {' '}in the current {info.inferenceMode === 'robot' ? 'Real Robot' : '3D Sim'} mode.
        {' '}Stop cancels automatic resume. Tactile calibration is retained.
      </p>
      {!active && !busy && <p className="mt-2 text-xs text-gray-600">Start model 1 to enable switching.</p>}
      {message && <p role="status" className="mt-2 text-xs text-indigo-800">{message}</p>}
      {error && <p role="alert" className="mt-2 text-xs text-red-700">{error}</p>}
      {needsClear && <p className="mt-2 text-xs text-red-700">Reconnect, press Stop, then Clear before setting up a new run.</p>}
      <FileBrowserModal isOpen={showBrowser} onClose={() => setShowBrowser(false)}
        onFileSelect={(item) => { changePath(item?.full_path || ''); setShowBrowser(false); }}
        title="Select next ViTacFormer model" selectButtonText="Select"
        allowDirectorySelect allowFileSelect={false}
        initialPath={browserPath} defaultPath={browserPath} homePath={DEFAULT_PATHS.POLICY_CHECKPOINTS_PATH} />
    </section>
  );
}
