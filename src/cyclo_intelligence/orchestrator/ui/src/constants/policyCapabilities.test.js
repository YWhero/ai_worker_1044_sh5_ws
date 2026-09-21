import {
  DEFAULT_INFERENCE_HZ,
  defaultInferenceHz,
  requiredActionRequestMode,
  resolveActionRequestMode,
  requiresInstruction,
} from './policyCapabilities';

describe('policy capabilities', () => {
  test('uses the validated 30 Hz source-action rate for vanilla LeRobot ACT', () => {
    expect(defaultInferenceHz('lerobot', 'act')).toBe(30);
    expect(defaultInferenceHz(' LeRobot ', ' ACT ')).toBe(30);
  });

  test('uses the validated 30 Hz source-action rate for Tactile ACT', () => {
    expect(defaultInferenceHz('lerobot', 'tactile_act')).toBe(30);
    expect(requiresInstruction('lerobot', 'tactile_act')).toBe(false);
  });

  test('uses the SH5 ViTacFormer deployment contract', () => {
    expect(defaultInferenceHz('vitacformer', 'vitacformer')).toBe(30);
    expect(requiresInstruction('vitacformer', 'vitacformer')).toBe(false);
    expect(requiredActionRequestMode('vitacformer', 'vitacformer')).toBe('async');
    expect(resolveActionRequestMode('vitacformer', 'vitacformer', 'sync')).toBe('async');
  });

  test('keeps the generic UI default for T-Rex and unrelated policies', () => {
    expect(defaultInferenceHz('lerobot', 'trex')).toBe(DEFAULT_INFERENCE_HZ);
    expect(defaultInferenceHz('groot', 'n17')).toBe(DEFAULT_INFERENCE_HZ);
    expect(defaultInferenceHz('lerobot', 'unknown')).toBe(DEFAULT_INFERENCE_HZ);
  });

  test('keeps instruction capabilities independent from rate defaults', () => {
    expect(requiresInstruction('lerobot', 'act')).toBe(false);
    expect(requiresInstruction('lerobot', 'trex')).toBe(true);
    expect(requiresInstruction('lerobot', 'fastwam')).toBe(true);
  });

  test('uses the validated request mode for each ACT deployment', () => {
    expect(requiredActionRequestMode('lerobot', 'act')).toBe('sync');
    expect(requiredActionRequestMode('lerobot', 'tactile_act')).toBe('async_ordered');
    expect(resolveActionRequestMode('lerobot', 'tactile_act', 'sync')).toBe('async_ordered');
    expect(resolveActionRequestMode('lerobot', 'trex', 'async')).toBe('async');
    expect(resolveActionRequestMode('lerobot', 'trex', 'ordered_async')).toBe('async_ordered');
  });
});
