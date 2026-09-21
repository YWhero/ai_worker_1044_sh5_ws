import { configureStore } from '@reduxjs/toolkit';
import { act, fireEvent, render, screen, waitFor } from '@testing-library/react';
import { Provider } from 'react-redux';
import InferenceModelSwitch from './InferenceModelSwitch';
import taskReducer, { setInferenceStatus } from '../features/tasks/taskSlice';
import { InferencePhase } from '../constants/taskPhases';
import { useRosServiceCaller } from '../hooks/useRosServiceCaller';

jest.mock('../hooks/useRosServiceCaller', () => ({ useRosServiceCaller: jest.fn() }));
jest.mock('./FileBrowserModal', () => () => null);

function setup({ phase = InferencePhase.INFERENCING, robot = 'ffw_sh5_rev1', service = 'vitacformer', send } = {}) {
  const initial = taskReducer(undefined, { type: '@@INIT' });
  const store = configureStore({
    reducer: { tasks: taskReducer },
    preloadedState: { tasks: {
      ...initial,
      robotType: robot,
      inferenceStatus: { ...initial.inferenceStatus, inferencePhase: phase },
      inferenceTaskInfo: {
        ...initial.inferenceTaskInfo, serviceType: service, policyType: 'vitacformer',
        policyPath: '/models/part1', inferenceMode: 'robot',
      },
      inferenceModelSwitch: { ...initial.inferenceModelSwitch, nextPolicyPath: '/models/part2' },
    } },
  });
  const sendRecordCommand = send || jest.fn().mockResolvedValue({
    success: true, message: 'Next model is running', loaded_policy_path: '/models/part2',
    inference_phase: InferencePhase.INFERENCING,
  });
  useRosServiceCaller.mockReturnValue({ sendRecordCommand });
  render(<Provider store={store}><InferenceModelSwitch /></Provider>);
  return { store, sendRecordCommand };
}

test('one click switches using a separate target without prematurely changing current model', async () => {
  let complete;
  const send = jest.fn(() => new Promise((resolve) => { complete = resolve; }));
  const { store } = setup({ send });
  fireEvent.click(screen.getByRole('button', { name: 'Switch to next model' }));
  fireEvent.click(screen.getByRole('button', { name: 'Switching model…' }));
  expect(send).toHaveBeenCalledTimes(1);
  expect(send).toHaveBeenCalledWith('switch_inference_model', { policyPath: '/models/part2', inferenceMode: 'robot' });
  expect(store.getState().tasks.inferenceTaskInfo.policyPath).toBe('/models/part1');
  expect(store.getState().tasks.inferenceModelSwitch.busy).toBe(true);
  await act(async () => complete({ success: true, loaded_policy_path: '/models/part2', message: 'Next model is running' }));
  expect(store.getState().tasks.inferenceTaskInfo.policyPath).toBe('/models/part2');
  expect(store.getState().tasks.inferenceModelSwitch.busy).toBe(false);
  expect(screen.getByRole('status')).toHaveTextContent('Next model is running');
});

test('failed resume displays the already loaded second model and an inline error', async () => {
  const { store } = setup({ send: jest.fn().mockResolvedValue({
    success: false, loaded_policy_path: '/models/part2', message: 'Next model loaded but remains paused: first action rejected',
  }) });
  fireEvent.click(screen.getByRole('button', { name: 'Switch to next model' }));
  expect(await screen.findByRole('alert')).toHaveTextContent('first action rejected');
  expect(store.getState().tasks.inferenceTaskInfo.policyPath).toBe('/models/part2');
  expect(screen.getByRole('button', { name: 'Switch to next model' })).toBeDisabled();
});

test('transport failure neither retries nor invents READY or a new loaded model', async () => {
  const send = jest.fn().mockRejectedValue(new Error('WebSocket disconnected'));
  const { store } = setup({ send });
  fireEvent.click(screen.getByRole('button', { name: 'Switch to next model' }));
  expect(await screen.findByRole('alert')).toHaveTextContent('WebSocket disconnected');
  expect(send).toHaveBeenCalledTimes(1);
  expect(store.getState().tasks.inferenceTaskInfo.policyPath).toBe('/models/part1');
  expect(store.getState().tasks.inferenceStatus.inferencePhase).toBe(InferencePhase.INFERENCING);
  expect(store.getState().tasks.inferenceModelSwitch.needsClear).toBe(true);
  expect(screen.getByRole('button', { name: 'Switch to next model' })).toBeDisabled();
});

test.each([InferencePhase.READY, InferencePhase.LOADING])('cannot switch in inactive phase %s', (phase) => {
  setup({ phase });
  expect(screen.getByRole('button', { name: 'Switch to next model' })).toBeDisabled();
});

test('rejects the same path including a trailing slash', () => {
  setup();
  fireEvent.change(screen.getByLabelText('Next model path'), { target: { value: ' /models/part1/ ' } });
  expect(screen.getByRole('button', { name: 'Switch to next model' })).toBeDisabled();
});

test.each([
  { robot: 'ffw_sg2_rev1' }, { service: 'lerobot' },
])('does not offer unsupported switching: %j', (options) => {
  setup(options);
  expect(screen.queryByRole('region', { name: 'Manual model switching' })).not.toBeInTheDocument();
});

test('Stop cancellation result keeps the loaded second model visible', async () => {
  const { store } = setup({ send: jest.fn().mockResolvedValue({
    success: true, loaded_policy_path: '/models/part2', message: 'Next model loaded; automatic resume cancelled',
    inference_phase: InferencePhase.PAUSED,
  }) });
  fireEvent.click(screen.getByRole('button', { name: 'Switch to next model' }));
  await waitFor(() => expect(store.getState().tasks.inferenceTaskInfo.policyPath).toBe('/models/part2'));
  expect(screen.getByRole('status')).toHaveTextContent('automatic resume cancelled');
});

test('preload before Start warms once and enables only the cached switch after model 1 starts', async () => {
  let complete;
  const send = jest.fn(() => new Promise((resolve) => { complete = resolve; }));
  const { store } = setup({ phase: InferencePhase.READY, send });
  fireEvent.click(screen.getByRole('checkbox', { name: 'Preload next model for fast switching' }));
  fireEvent.click(screen.getByRole('button', { name: 'Preload next model' }));
  fireEvent.click(screen.getByRole('button', { name: 'Preparing model…' }));
  expect(send).toHaveBeenCalledTimes(1);
  expect(send).toHaveBeenCalledWith('preload_inference_model', { policyPath: '/models/part2', inferenceMode: 'robot' });
  expect(store.getState().tasks.inferenceModelSwitch.preloadBusy).toBe(true);
  expect(store.getState().tasks.inferenceTaskInfo.policyPath).toBe('/models/part1');
  await act(async () => complete({ success: true, message: 'ready' }));
  expect(screen.getByText('Preload ready · warmup complete')).toBeInTheDocument();
  expect(screen.getByRole('button', { name: 'Switch to next model' })).toBeDisabled();
  act(() => store.dispatch(setInferenceStatus({ inferencePhase: InferencePhase.LOADING })));
  act(() => store.dispatch(setInferenceStatus({ inferencePhase: InferencePhase.INFERENCING })));
  expect(screen.getByRole('button', { name: 'Switch to next model' })).toBeEnabled();
  send.mockResolvedValueOnce({ success: true, loaded_policy_path: '/models/part2' });
  fireEvent.click(screen.getByRole('button', { name: 'Switch to next model' }));
  await waitFor(() => expect(store.getState().tasks.inferenceModelSwitch.busy).toBe(false));
  expect(send).toHaveBeenLastCalledWith('switch_preloaded_model', { policyPath: '/models/part2', inferenceMode: 'robot' });
  expect(store.getState().tasks.inferenceModelSwitch.preloadedPath).toBe('/models/part1');
});

test('cannot preload during live inference or switch before preload is ready', () => {
  setup();
  fireEvent.click(screen.getByRole('checkbox', { name: 'Preload next model for fast switching' }));
  expect(screen.getByRole('button', { name: 'Preload next model' })).toBeDisabled();
  expect(screen.getByRole('button', { name: 'Switch to next model' })).toBeDisabled();
});

test('changing target and unloading invalidate fast-switch readiness', async () => {
  const { store } = setup({ phase: InferencePhase.PAUSED, send: jest.fn().mockResolvedValue({ success: true }) });
  fireEvent.click(screen.getByRole('checkbox', { name: 'Preload next model for fast switching' }));
  fireEvent.click(screen.getByRole('button', { name: 'Preload next model' }));
  await screen.findByText('Preload ready · warmup complete');
  fireEvent.change(screen.getByLabelText('Next model path'), { target: { value: '/models/part3' } });
  expect(screen.getByRole('button', { name: 'Switch to next model' })).toBeDisabled();
  fireEvent.change(screen.getByLabelText('Next model path'), { target: { value: '/models/part2' } });
  expect(screen.getByRole('button', { name: 'Switch to next model' })).toBeEnabled();
  act(() => store.dispatch(setInferenceStatus({ inferencePhase: InferencePhase.READY })));
  expect(store.getState().tasks.inferenceModelSwitch.preloadedPath).toBe('');
});

test('failed preload never marks a candidate ready or starts inference', async () => {
  const send = jest.fn().mockResolvedValue({ success: false, message: 'CUDA out of memory' });
  const { store } = setup({ phase: InferencePhase.PAUSED, send });
  fireEvent.click(screen.getByRole('checkbox', { name: 'Preload next model for fast switching' }));
  fireEvent.click(screen.getByRole('button', { name: 'Preload next model' }));
  expect(await screen.findByRole('alert')).toHaveTextContent('CUDA out of memory');
  expect(store.getState().tasks.inferenceModelSwitch.preloadedPath).toBe('');
  expect(store.getState().tasks.inferenceTaskInfo.policyPath).toBe('/models/part1');
  expect(store.getState().tasks.inferenceStatus.inferencePhase).toBe(InferencePhase.PAUSED);
  expect(send).toHaveBeenCalledTimes(1);
});

test('release frees the spare without clearing the current model', async () => {
  const send = jest.fn().mockResolvedValue({ success: true });
  const { store } = setup({ phase: InferencePhase.PAUSED, send });
  fireEvent.click(screen.getByRole('checkbox', { name: 'Preload next model for fast switching' }));
  fireEvent.click(screen.getByRole('button', { name: 'Preload next model' }));
  await screen.findByText('Preload ready · warmup complete');
  fireEvent.click(screen.getByRole('button', { name: 'Release preload' }));
  await waitFor(() => expect(store.getState().tasks.inferenceModelSwitch.preloadBusy).toBe(false));
  expect(send).toHaveBeenLastCalledWith('clear_preloaded_model', { policyPath: '/models/part2', inferenceMode: 'robot' });
  expect(store.getState().tasks.inferenceModelSwitch.preloadedPath).toBe('');
  expect(store.getState().tasks.inferenceTaskInfo.policyPath).toBe('/models/part1');
});
