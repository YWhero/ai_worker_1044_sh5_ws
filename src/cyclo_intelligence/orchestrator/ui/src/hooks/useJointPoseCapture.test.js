import { act, renderHook } from '@testing-library/react';
import useJointPoseCapture, { evaluateJointPoseSamples } from './useJointPoseCapture';
import { SH5_LEFT_ARM_JOINTS, SH5_RIGHT_ARM_JOINTS, SH5_LEFT_HAND_JOINTS, SH5_RIGHT_HAND_JOINTS } from '../features/actionCanvas/jointPoseUtils';

let mockOnJointState;
let mockSubscriptionOptions;
jest.mock('./useJointStateSubscription', () => (callback, _action, _enabled, options) => {
  mockOnJointState = callback;
  mockSubscriptionOptions = options;
});

function samples(now, values) {
  return values.map((value, index) => ({
    at: now - (values.length - index - 1) * 150,
    joints: { arm_l_joint1: value, lift_joint: -0.4 },
  }));
}

test('reports a steady recent pose as stable', () => {
  const now = 10_000;
  expect(evaluateJointPoseSamples(samples(now, [0.1, 0.101, 0.1, 0.099]), now)).toEqual({
    state: 'stable',
    message: 'Follower pose is stable',
  });
});

test('only requires selected joints to remain stationary', () => {
  const now = 10_000;
  const messages = [0, 1, 2, 3].map((index) => ({
    at: now - 450 + index * 150,
    joints: { arm_l_joint1: 0.2, finger_r_joint20: index * 0.1 },
  }));
  expect(evaluateJointPoseSamples(messages, now, ['arm_l_joint1']).state).toBe('stable');
  expect(evaluateJointPoseSamples(messages, now, ['finger_r_joint20']).state).toBe('moving');
});

test('head messages cannot refresh a disconnected arm topic', () => {
  const now = 10_000;
  const messages = [
    { at: now - 2000, joints: { arm_l_joint1: 0.2 } },
    ...[0, 1, 2, 3].map((index) => ({
      at: now - 450 + index * 150,
      joints: { head_joint1: 0.1 },
    })),
  ];
  expect(evaluateJointPoseSamples(messages, now, ['arm_l_joint1']).state).toBe('stale');
  expect(evaluateJointPoseSamples(messages, now, ['head_joint1']).state).toBe('stable');
});

test('a single fresh arm sample is not multiplied by head samples', () => {
  const now = 10_000;
  const messages = [
    { at: now - 450, joints: { arm_l_joint1: 0.2 } },
    ...[0, 1, 2, 3].map((index) => ({
      at: now - 400 + index * 100,
      joints: { head_joint1: 0.1 },
    })),
  ];
  expect(evaluateJointPoseSamples(messages, now, ['arm_l_joint1']).state).toBe('settling');
  expect(evaluateJointPoseSamples(messages, now, ['lift_joint']).state).toBe('waiting');
});

test('rejects moving and stale pose samples', () => {
  const now = 10_000;
  expect(evaluateJointPoseSamples(samples(now, [0.1, 0.11, 0.13, 0.15]), now).state)
    .toBe('moving');
  expect(evaluateJointPoseSamples([{ at: now - 2_000, joints: { arm_l_joint1: 0.1 } }], now).state)
    .toBe('stale');
});

test('captures SH5 arms and forty fingers from /arm_hand/joint_states with separate head/lift samples', () => {
  jest.useFakeTimers();
  jest.setSystemTime(10_000);
  const armHandNames = [...SH5_LEFT_ARM_JOINTS, ...SH5_RIGHT_ARM_JOINTS, ...SH5_LEFT_HAND_JOINTS, ...SH5_RIGHT_HAND_JOINTS];
  const selected = [...armHandNames, 'head_joint1', 'head_joint2', 'lift_joint'];
  const { result, unmount } = renderHook(() => useJointPoseCapture(true, 'ffw_sh5_rev1', selected));
  expect(mockSubscriptionOptions.stateTopics.map((topic) => topic.name))
    .toEqual(['/joint_states', '/arm_hand/joint_states']);

  for (let index = 0; index < 5; index += 1) {
    act(() => {
      jest.advanceTimersByTime(100);
      mockOnJointState({ name: armHandNames, position: armHandNames.map((_, jointIndex) => jointIndex / 100) });
      mockOnJointState({ name: ['head_joint1', 'head_joint2', 'lift_joint'], position: [0.1, -0.1, -0.4] });
    });
  }
  expect(result.current.state).toBe('stable');
  const captured = result.current.capture();
  expect(captured.ok).toBe(true);
  expect(Object.keys(captured.pose)).toHaveLength(57);
  expect(captured.pose.finger_r_joint20).toBe(0.53);
  expect(captured.pose.lift_joint).toBe(-0.4);

  act(() => jest.advanceTimersByTime(2_000));
  expect(result.current.state).toBe('stale');
  expect(result.current.capture().ok).toBe(false);
  unmount();
  jest.useRealTimers();
});

test('shows an actionable missing-joint message and requires a nonempty selection', () => {
  expect(evaluateJointPoseSamples([], 10_000, []).message).toBe('Select joints to capture');
  expect(evaluateJointPoseSamples(samples(10_000, [0, 0, 0, 0]), 10_000, ['finger_l_joint20']))
    .toEqual({ state: 'waiting', message: 'Waiting for finger_l_joint20' });
});
