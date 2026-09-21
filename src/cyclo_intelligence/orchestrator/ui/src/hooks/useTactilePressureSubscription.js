import { useEffect, useState } from 'react';
import { useSelector } from 'react-redux';
import ROSLIB from 'roslib';
import rosConnectionManager from '../utils/rosConnectionManager';

const MESSAGE_TYPE = 'robotis_interfaces/msg/HandPressures';
const UPDATE_INTERVAL_MS = 50;
const FINGER_COUNT = 5;
const TAXEL_COUNT = 9;

const HAND_TOPICS = {
  left: '/left_hand/finger_pressures',
  right: '/right_hand/finger_pressures',
};

function pressureArray(values) {
  if (Array.isArray(values) || ArrayBuffer.isView(values)) {
    return Array.from(values);
  }
  if (typeof values === 'string' && typeof atob === 'function') {
    try {
      return Array.from(atob(values), (character) => character.charCodeAt(0));
    } catch (_error) {
      return [];
    }
  }
  return [];
}

export function extractRawFingerPressures(message) {
  const fingers = Array(FINGER_COUNT).fill(null);
  const sensors = Array.isArray(message?.sensors) ? message.sensors : [];

  sensors.forEach((sensor, sensorIndex) => {
    const suffix = String(sensor?.sensor_name || '').match(/sensor(\d+)$/i);
    const fingerIndex = suffix ? Number(suffix[1]) - 1 : sensorIndex;
    if (fingerIndex < 0 || fingerIndex >= FINGER_COUNT) return;

    const values = pressureArray(sensor?.pressure_values);
    fingers[fingerIndex] = {
      sensorName: String(sensor?.sensor_name || `sensor${fingerIndex + 1}`),
      pressureNames: Array.from({ length: TAXEL_COUNT }, (_, index) => (
        String(sensor?.pressure_names?.[index] || `cell ${index + 1}`)
      )),
      // Preserve cell indices. Missing cells are unknown, not zero pressure.
      values: Array.from({ length: TAXEL_COUNT }, (_, index) => (
        typeof values[index] === 'number' && Number.isFinite(values[index])
          ? values[index]
          : null
      )),
    };
  });

  return fingers;
}

export default function useTactilePressureSubscription(enabled = true) {
  const rosbridgeUrl = useSelector((state) => state.ros.rosbridgeUrl);
  const [hands, setHands] = useState({ left: null, right: null });

  useEffect(() => {
    if (!enabled || !rosbridgeUrl) {
      setHands({ left: null, right: null });
      return undefined;
    }

    let cancelled = false;
    const subscriptions = [];

    const subscribe = (ros, side, topicName) => {
      const topic = new ROSLIB.Topic({
        ros,
        name: topicName,
        messageType: MESSAGE_TYPE,
        throttle_rate: UPDATE_INTERVAL_MS,
        queue_length: 1,
      });
      topic.subscribe((message) => {
        if (cancelled) return;
        const fingers = extractRawFingerPressures(message);
        setHands((current) => ({
          ...current,
          [side]: {
            fingers,
            receivedAt: Date.now(),
          },
        }));
      });
      subscriptions.push(topic);
    };

    const connect = async () => {
      try {
        const ros = await rosConnectionManager.getConnection(rosbridgeUrl);
        if (cancelled || !ros) return;
        Object.entries(HAND_TOPICS).forEach(([side, topicName]) => {
          subscribe(ros, side, topicName);
        });
      } catch (error) {
        if (!cancelled) {
          console.error('Tactile pressure ROS connection error:', error);
        }
      }
    };

    connect();

    return () => {
      cancelled = true;
      subscriptions.forEach((topic) => {
        try {
          topic.unsubscribe();
        } catch (_error) {}
      });
    };
  }, [enabled, rosbridgeUrl]);

  return hands;
}
