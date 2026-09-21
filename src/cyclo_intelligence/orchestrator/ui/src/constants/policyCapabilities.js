// Copyright 2025 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Whitelist of (service_type, policy_type) pairs that require a
// task_instruction. Missing entries fall back to
// DEFAULT_REQUIRES_INSTRUCTION (false → hide the field).
//
// Key shape: '<service_type>:<policy_type>' or '<service_type>:*'
// Exact key beats wildcard.
//
// Grow this map one line at a time as policies are validated end-to-end.
export const POLICY_REQUIRES_INSTRUCTION = {
  'groot:n17': true,
  'lerobot:act': false,
  'lerobot:tactile_act': false,
  'vitacformer:vitacformer': false,
  'lerobot:diffusion': false,
  'lerobot:smolvla': true,
  'lerobot:xvla': true,
  'lerobot:pi0': true,
  'lerobot:pi05': true,
  'lerobot:trex': true,
  'lerobot:fastwam': true,
};

export const DEFAULT_REQUIRES_INSTRUCTION = false;

// UI-side source-action-rate defaults. Keep these policy-specific: the
// generic 15 Hz fallback is still used by T-Rex and every other policy unless
// that policy is explicitly validated at another rate. In particular, the
// T-Rex deployment's fractional 1.25 Hz playback remains owned by its Docker
// runtime profile because TaskInfo.inference_hz is an integer ROS field.
export const POLICY_DEFAULT_INFERENCE_HZ = {
  'lerobot:act': 30,
  'lerobot:tactile_act': 30,
  'vitacformer:vitacformer': 30,
};

export const DEFAULT_INFERENCE_HZ = 15;

// Vanilla ACT uses its configured execution horizon in deterministic append
// order. SH5 TactileACT keeps its four-step closed-loop horizon and performs
// time-aligned decoder overlap ensembling at ordered asynchronous replans.
export const POLICY_REQUIRED_ACTION_REQUEST_MODE = {
  'lerobot:act': 'sync',
  'lerobot:tactile_act': 'async_ordered',
  'vitacformer:vitacformer': 'async',
};

export function requiresInstruction(serviceType, policyType) {
  const exact = `${serviceType}:${policyType}`;
  if (exact in POLICY_REQUIRES_INSTRUCTION) {
    return POLICY_REQUIRES_INSTRUCTION[exact];
  }
  const wild = `${serviceType}:*`;
  if (wild in POLICY_REQUIRES_INSTRUCTION) {
    return POLICY_REQUIRES_INSTRUCTION[wild];
  }
  return DEFAULT_REQUIRES_INSTRUCTION;
}

export function defaultInferenceHz(serviceType, policyType) {
  const service = String(serviceType ?? '').trim().toLowerCase();
  const policy = String(policyType ?? '').trim().toLowerCase();
  const exact = `${service}:${policy}`;
  return POLICY_DEFAULT_INFERENCE_HZ[exact] ?? DEFAULT_INFERENCE_HZ;
}

export function requiredActionRequestMode(serviceType, policyType) {
  const service = String(serviceType ?? '').trim().toLowerCase();
  const policy = String(policyType ?? '').trim().toLowerCase();
  return POLICY_REQUIRED_ACTION_REQUEST_MODE[`${service}:${policy}`] || '';
}

export function resolveActionRequestMode(serviceType, policyType, requestedMode) {
  const required = requiredActionRequestMode(serviceType, policyType);
  if (required) return required;
  const requested = String(requestedMode ?? '').trim().toLowerCase();
  if (requested === 'sync') return 'sync';
  if (requested === 'sync_step' || requested === 'step_sync') {
    return 'sync_step';
  }
  if (requested === 'async_ordered' || requested === 'ordered_async') {
    return 'async_ordered';
  }
  return 'async';
}
