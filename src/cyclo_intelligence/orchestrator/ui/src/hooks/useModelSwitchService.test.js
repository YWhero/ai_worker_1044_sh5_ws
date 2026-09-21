import React from 'react';
import { configureStore } from '@reduxjs/toolkit';
import { act, renderHook } from '@testing-library/react';
import { Provider } from 'react-redux';
import ROSLIB from 'roslib';
import taskReducer from '../features/tasks/taskSlice';
import PageType from '../constants/pageType';
import rosConnectionManager from '../utils/rosConnectionManager';
import { useRosServiceCaller } from './useRosServiceCaller';

jest.mock('../utils/rosConnectionManager', () => ({
  __esModule: true, default: { getConnection: jest.fn() },
}));

function setup() {
  const initial = taskReducer(undefined, { type: '@@INIT' });
  const store = configureStore({ reducer: {
    tasks: taskReducer,
    training: () => ({ trainingInfo: {} }),
    editDataset: () => ({}),
    ui: () => ({ currentPage: PageType.RECORD }),
    ros: () => ({ rosbridgeUrl: 'ws://test' }),
  }, preloadedState: { tasks: {
    ...initial,
    inferenceTaskInfo: {
      ...initial.inferenceTaskInfo, policyPath: '/models/part1',
      serviceType: 'vitacformer', policyType: 'vitacformer', inferenceMode: 'robot',
    },
  } } });
  const ros = new ROSLIB.Ros();
  ros.isConnected = true;
  ros.callOnConnection = jest.fn();
  rosConnectionManager.getConnection.mockResolvedValue(ros);
  const wrapper = ({ children }) => <Provider store={store}>{children}</Provider>;
  const hook = renderHook(() => useRosServiceCaller(), { wrapper });
  return { ros, store, hook };
}

beforeEach(() => {
  jest.spyOn(console, 'log').mockImplementation(() => {});
  jest.spyOn(console, 'error').mockImplementation(() => {});
});
afterEach(() => jest.restoreAllMocks());

test.each([
  ['switch_inference_model', 26], ['preload_inference_model', 27],
  ['clear_preloaded_model', 28], ['switch_preloaded_model', 29],
])('%s sends target and loaded mode from inference even after navigating to Record', async (command, commandId) => {
  const { ros, store, hook } = setup();
  let pending;
  await act(async () => {
    pending = hook.result.current.sendRecordCommand(command, { policyPath: ' /models/part2 ' });
  });
  const call = ros.callOnConnection.mock.calls[0][0];
  expect(call.timeout).toBe(0);
  expect(call.args.command).toBe(commandId);
  expect(call.args.task_info).toMatchObject({
    policy_path: '/models/part2', task_type: 'inference', service_type: 'vitacformer',
    inference_mode: 'robot', tags: ['inference_mode:robot'],
  });
  expect(store.getState().tasks.inferenceTaskInfo.policyPath).toBe('/models/part1');
  const response = { success: true, loaded_policy_path: '/models/part2', inference_phase: 2 };
  ros.emit(call.id, { result: true, values: response });
  await expect(pending).resolves.toMatchObject(response);
  expect(ros.listeners('close')).toHaveLength(0);
});

test('disconnect releases a long switch call and its listeners without retrying', async () => {
  const { ros, hook } = setup();
  let pending;
  await act(async () => {
    pending = hook.result.current.sendRecordCommand('switch_inference_model', { policyPath: '/models/part2' });
  });
  const call = ros.callOnConnection.mock.calls[0][0];
  const rejected = expect(pending).rejects.toThrow('ROS connection closed');
  ros.emit('close');
  await rejected;
  expect(ros.listeners(call.id)).toHaveLength(0);
  expect(ros.listeners('close')).toHaveLength(0);
  expect(ros.callOnConnection).toHaveBeenCalledTimes(1);
});
