import { act, render, screen, waitFor } from '@testing-library/react';
import ROSLIB from 'roslib';
import rosConnectionManager from '../utils/rosConnectionManager';
import TactileHandsPanel from './TactileHandsPanel';
import { extractRawFingerPressures } from '../hooks/useTactilePressureSubscription';

const mockSubscriptions = {};
const mockUnsubscribe = jest.fn();
jest.mock('react-redux', () => ({
  useSelector: (select) => select({ ros: { rosbridgeUrl: 'ws://test' } }),
}));
jest.mock('../utils/rosConnectionManager', () => ({
  getConnection: jest.fn(),
}));
jest.mock('roslib', () => ({
  Topic: jest.fn(),
}));

beforeEach(() => {
  Object.keys(mockSubscriptions).forEach((key) => delete mockSubscriptions[key]);
  mockUnsubscribe.mockClear();
  rosConnectionManager.getConnection.mockResolvedValue({});
  ROSLIB.Topic.mockImplementation(({ name }) => ({
    subscribe: (callback) => { mockSubscriptions[name] = callback; },
    unsubscribe: mockUnsubscribe,
  }));
});

test('maps named fingers and preserves all nine raw cell values and labels', () => {
  const values = [0, 1, 2, 3, 4, 5, 6, 7, 255];
  const fingers = extractRawFingerPressures({ sensors: [
    { sensor_name: 'finger_r_sensor5', pressure_values: values, pressure_names: ['p1'] },
    { sensor_name: 'finger_r_sensor1', pressure_values: new Uint8Array(values) },
  ] });
  expect(fingers[4].values).toEqual(values);
  expect(fingers[0].values).toEqual(values);
  expect(fingers[4].pressureNames[0]).toBe('p1');
  expect(fingers[1]).toBeNull();
});

test('decodes rosbridge byte arrays without averaging', () => {
  const values = [0, 1, 2, 3, 4, 5, 6, 7, 255];
  const fingers = extractRawFingerPressures({ sensors: [{
    sensor_name: 'finger_l_sensor2',
    pressure_values: btoa(String.fromCharCode(...values)),
  }] });
  expect(fingers[1].values).toEqual(values);
});

test('invalid or missing cells retain their indices and do not become zero', () => {
  const fingers = extractRawFingerPressures({ sensors: [{
    sensor_name: 'finger_l_sensor1', pressure_values: [0, NaN, 17, null, Infinity],
  }] });
  expect(fingers[0].values).toEqual([0, null, 17, null, null, null, null, null, null]);
  expect(extractRawFingerPressures({ sensors: [{ pressure_values: '!!!' }] })[0].values)
    .toEqual(Array(9).fill(null));
});

test('shows the first raw frame immediately and follows changes without baseline or smoothing', async () => {
  const { unmount } = render(<TactileHandsPanel />);
  await waitFor(() => expect(mockSubscriptions['/right_hand/finger_pressures']).toBeDefined());
  const callback = mockSubscriptions['/right_hand/finger_pressures'];
  act(() => callback({ sensors: [{
    sensor_name: 'finger_r_sensor1', pressure_values: [255, 0, 1, 2, 3, 4, 5, 6, 7],
  }] }));
  expect(screen.getByText('Tactile · RAW')).toBeInTheDocument();
  expect(screen.getByLabelText('right Thumb cell 1: 255')).toHaveTextContent('255');
  expect(screen.getByLabelText('right Thumb cell 2: 0')).toHaveTextContent('0');
  expect(screen.getByLabelText('left Thumb cell 1: unavailable')).toHaveTextContent('—');
  expect(screen.getByText('Peak 255')).toBeInTheDocument();
  expect(screen.queryByText('Calibrating')).not.toBeInTheDocument();
  act(() => callback({ sensors: [{ sensor_name: 'finger_r_sensor1', pressure_values: Array(9).fill(0) }] }));
  expect(screen.getByLabelText('right Thumb cell 1: 0')).toHaveTextContent('0');
  expect(screen.getByText('Peak 0')).toBeInTheDocument();
  unmount();
  expect(mockUnsubscribe).toHaveBeenCalledTimes(2);
});
