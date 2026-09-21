import { useCallback, useEffect, useMemo, useRef, useState } from 'react';

import { jointPoseProfileForRobot } from '../features/actionCanvas/jointPoseUtils';
import useJointStateSubscription from './useJointStateSubscription';

const SAMPLE_WINDOW_MS = 500;
const STALE_AFTER_MS = 1500;
const MIN_SAMPLES = 4;
const MAX_REVOLUTE_SPREAD = 0.02;
const MAX_LIFT_SPREAD = 0.005;

function median(values) {
  const sorted = [...values].sort((left, right) => left - right);
  const middle = Math.floor(sorted.length / 2);
  if (sorted.length % 2) return sorted[middle];
  return (sorted[middle - 1] + sorted[middle]) / 2;
}

export function evaluateJointPoseSamples(samples, now = Date.now(), selectedJoints = null) {
  if (selectedJoints?.length === 0) return { state: 'waiting', message: 'Select joints to capture' };
  if (samples.length === 0) {
    return { state: 'waiting', message: 'Waiting for /joint_states' };
  }
  const names = selectedJoints ?? [...new Set(samples.flatMap((sample) => Object.keys(sample.joints)))];
  if (names.length === 0) return { state: 'waiting', message: 'Select joints to capture' };
  const recent = samples.filter((sample) => now - sample.at <= SAMPLE_WINDOW_MS);
  for (const name of names) {
    const latest = [...samples].reverse().find((sample) => Number.isFinite(sample.joints[name]));
    if (!latest) return { state: 'waiting', message: `Waiting for ${name}` };
    if (now - latest.at > STALE_AFTER_MS) {
      return { state: 'stale', message: `Joint state is stale: ${name}` };
    }
    const jointSamples = recent.filter((sample) => Number.isFinite(sample.joints[name]));
    if (jointSamples.length < MIN_SAMPLES
      || jointSamples[jointSamples.length - 1].at - jointSamples[0].at < SAMPLE_WINDOW_MS * 0.6) {
      return { state: 'settling', message: `Hold still; waiting for fresh ${name} samples` };
    }
    const values = jointSamples.map((sample) => sample.joints[name]);
    const spread = Math.max(...values) - Math.min(...values);
    if (spread > (name === 'lift_joint' ? MAX_LIFT_SPREAD : MAX_REVOLUTE_SPREAD)) {
      return { state: 'moving', message: `Selected joint is still moving: ${name}` };
    }
  }

  return { state: 'stable', message: 'Follower pose is stable' };
}

export default function useJointPoseCapture(enabled = true, robotType = '', selectedJoints = null) {
  const profile = jointPoseProfileForRobot(robotType);
  const supportedJoints = useMemo(
    () => new Set(profile.poseJoints),
    [profile.poseJoints],
  );
  const samplesRef = useRef([]);
  const [revision, setRevision] = useState(0);

  const handleJointState = useCallback((message) => {
    const next = {};
    (message.name || []).forEach((name, index) => {
      const position = message.position?.[index];
      if (supportedJoints.has(name) && Number.isFinite(position)) {
        next[name] = position;
      }
    });
    if (Object.keys(next).length === 0) return;

    const now = Date.now();
    samplesRef.current = [
      ...samplesRef.current.filter((sample) => now - sample.at <= STALE_AFTER_MS * 2),
      { at: now, joints: next },
    ];
    setRevision((value) => value + 1);
  }, [supportedJoints]);

  useEffect(() => {
    samplesRef.current = [];
    setRevision((value) => value + 1);
  }, [enabled, supportedJoints]);

  useJointStateSubscription(handleJointState, null, enabled, {
    stateTopics: profile.stateTopics,
    liveUpdateHz: 20,
  });

  useEffect(() => {
    if (!enabled) return undefined;
    const timer = window.setInterval(() => setRevision((value) => value + 1), 250);
    return () => window.clearInterval(timer);
  }, [enabled]);

  const status = evaluateJointPoseSamples(samplesRef.current, Date.now(), selectedJoints);
  if (status.message === 'Waiting for /joint_states') {
    status.message = `Waiting for ${profile.stateTopics.map((topic) => topic.name).join(' or ')}`;
  }

  const capture = useCallback(() => {
    const now = Date.now();
    const currentStatus = evaluateJointPoseSamples(samplesRef.current, now, selectedJoints);
    if (currentStatus.state !== 'stable') {
      return { ok: false, message: currentStatus.message, pose: {} };
    }
    const recent = samplesRef.current.filter(
      (sample) => now - sample.at <= SAMPLE_WINDOW_MS,
    );
    const pose = {};
    (selectedJoints ?? profile.poseJoints).forEach((name) => {
      const values = recent
        .map((sample) => sample.joints[name])
        .filter(Number.isFinite);
      if (values.length > 0) pose[name] = median(values);
    });
    return { ok: true, message: 'Follower pose captured', pose };
  }, [profile.poseJoints, selectedJoints]);

  return {
    ...status,
    capture,
    revision,
    receivedJointCount: new Set(samplesRef.current.flatMap((sample) => Object.keys(sample.joints))).size,
    sourceTopics: profile.stateTopics.map((topic) => topic.name),
  };
}
