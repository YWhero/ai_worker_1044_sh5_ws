import { configureStore } from '@reduxjs/toolkit';
import { fireEvent, render, screen } from '@testing-library/react';
import { Provider } from 'react-redux';
import InferenceModelSelector from './InferenceModelSelector';
import taskReducer, {
  selectInferenceTaskInfo,
} from '../features/tasks/taskSlice';

const renderSelector = () => {
  const store = configureStore({
    reducer: { tasks: taskReducer },
  });
  render(
    <Provider store={store}>
      <InferenceModelSelector />
    </Provider>
  );
  return store;
};

describe('InferenceModelSelector policy rate defaults', () => {
  test('starts ACT at 30 Hz and applies defaults per selected policy', () => {
    const store = renderSelector();
    const selector = screen.getByRole('combobox');

    expect(selectInferenceTaskInfo(store.getState())).toMatchObject({
      inferenceHz: 30,
      actionRequestMode: 'sync',
    });

    fireEvent.change(selector, { target: { value: 'lerobot:trex' } });
    expect(selectInferenceTaskInfo(store.getState())).toMatchObject({
      serviceType: 'lerobot',
      policyType: 'trex',
      inferenceHz: 15,
      actionRequestMode: 'async',
    });

    fireEvent.change(selector, { target: { value: 'lerobot:act' } });
    expect(selectInferenceTaskInfo(store.getState())).toMatchObject({
      serviceType: 'lerobot',
      policyType: 'act',
      inferenceHz: 30,
      actionRequestMode: 'sync',
    });

    fireEvent.change(selector, { target: { value: 'lerobot:fastwam' } });
    expect(selectInferenceTaskInfo(store.getState())).toMatchObject({
      serviceType: 'lerobot',
      policyType: 'fastwam',
      inferenceHz: 15,
      actionRequestMode: 'async',
    });
    expect(screen.getByRole('option', { name: 'FastWAM' })).toBeEnabled();

    fireEvent.change(selector, { target: { value: 'lerobot:tactile_act' } });
    expect(selectInferenceTaskInfo(store.getState())).toMatchObject({
      serviceType: 'lerobot',
      policyType: 'tactile_act',
      inferenceHz: 30,
      actionRequestMode: 'async_ordered',
    });

    fireEvent.change(selector, { target: { value: 'vitacformer:vitacformer' } });
    expect(selectInferenceTaskInfo(store.getState())).toMatchObject({
      serviceType: 'vitacformer',
      policyType: 'vitacformer',
      inferenceHz: 30,
      actionRequestMode: 'async',
    });
  });
});
