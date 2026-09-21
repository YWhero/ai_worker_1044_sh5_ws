import { act, renderHook, waitFor } from '@testing-library/react';
import useJointStateSubscription from './useJointStateSubscription';
import ROSLIB from 'roslib';
import rosConnectionManager from '../utils/rosConnectionManager';

jest.mock('react-redux', () => ({
  useSelector: (selector) => selector({ ros: { rosbridgeUrl: 'ws://robot.local/rosbridge/' } }),
}));
jest.mock('../utils/rosConnectionManager', () => ({
  __esModule: true,
  default: { getConnection: jest.fn() },
}));
jest.mock('roslib', () => ({
  __esModule: true,
  default: { Topic: jest.fn() },
}));

let topics;
const stateTopics = [
  { name: '/arm_hand/joint_states', type: 'sensor_msgs/msg/JointState' },
  { name: '/joint_states', type: 'sensor_msgs/msg/JointState' },
];

beforeEach(() => {
  jest.clearAllMocks();
  topics = [];
  rosConnectionManager.getConnection.mockResolvedValue({ connected: true });
  ROSLIB.Topic.mockImplementation((options) => {
    const topic = {
      ...options,
      subscribe: jest.fn((callback) => { topic.callback = callback; }),
      unsubscribe: jest.fn(),
    };
    topics.push(topic);
    return topic;
  });
});

test('draws measured arms, fingers, head and lift received on separate feedback topics', async () => {
  const update = jest.fn();
  const hook = renderHook(() => useJointStateSubscription(update, null, true, { stateTopics }));
  await waitFor(() => expect(topics).toHaveLength(2));
  expect(topics.map((topic) => topic.name)).toEqual(['/arm_hand/joint_states', '/joint_states']);
  expect(topics.every((topic) => topic.queue_length === 1)).toBe(true);

  const armNames = ['l', 'r'].flatMap((side) => (
    Array.from({ length: 7 }, (_, index) => `arm_${side}_joint${index + 1}`)
  ));
  const fingerNames = ['l', 'r'].flatMap((side) => (
    Array.from({ length: 20 }, (_, index) => `finger_${side}_joint${index + 1}`)
  ));
  const armHand = { name: [...armNames, ...fingerNames], position: Array(54).fill(0.2) };
  const auxiliary = { name: ['head_joint1', 'head_joint2', 'lift_joint'], position: [0.68637, 0, -0.0056] };

  // Separate topic throttles must not discard head/lift when both feedback
  // messages arrive during the same rendering interval.
  act(() => {
    topics[0].callback(armHand);
    topics[1].callback(auxiliary);
  });
  expect(update.mock.calls.map(([message]) => message)).toEqual([armHand, auxiliary]);
  hook.unmount();
  expect(topics.every((topic) => topic.unsubscribe.mock.calls.length === 1)).toBe(true);
});

test('state visualization subscribes to measured feedback rather than commanded trajectories', async () => {
  const hook = renderHook(() => useJointStateSubscription(jest.fn(), null, true, {
    stateTopics,
    actionTopics: [{ name: '/leader/head/joint_trajectory', type: 'trajectory_msgs/msg/JointTrajectory' }],
    enableActionPreview: true,
    visualizationSource: 'state',
  }));
  await waitFor(() => expect(topics).toHaveLength(2));
  expect(topics.every((topic) => topic.messageType === 'sensor_msgs/msg/JointState')).toBe(true);
  hook.unmount();
});

test('a disabled model does not create feedback subscriptions', async () => {
  renderHook(() => useJointStateSubscription(jest.fn(), null, false, { stateTopics }));
  await act(async () => {});
  expect(rosConnectionManager.getConnection).not.toHaveBeenCalled();
  expect(ROSLIB.Topic).not.toHaveBeenCalled();
});
