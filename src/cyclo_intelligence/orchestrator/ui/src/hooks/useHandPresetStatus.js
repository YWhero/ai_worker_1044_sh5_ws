import { useEffect, useState } from 'react';
import { useSelector } from 'react-redux';
import ROSLIB from 'roslib';

import {
  HAND_PRESET_STATUS_TOPIC,
  HAND_PRESET_STATUS_TYPE,
  parseHandPresetStatus,
} from '../constants/handPresetProtocol';
import rosConnectionManager from '../utils/rosConnectionManager';

export default function useHandPresetStatus(enabled = true) {
  const rosbridgeUrl = useSelector((state) => state.ros.rosbridgeUrl);
  const [status, setStatus] = useState(null);

  useEffect(() => {
    if (!enabled || !rosbridgeUrl) {
      setStatus(null);
      return undefined;
    }

    let cancelled = false;
    let topic = null;

    const subscribe = async () => {
      try {
        const ros = await rosConnectionManager.getConnection(rosbridgeUrl);
        if (cancelled || !ros) return;

        topic = new ROSLIB.Topic({
          ros,
          name: HAND_PRESET_STATUS_TOPIC,
          messageType: HAND_PRESET_STATUS_TYPE,
          queue_length: 1,
        });
        topic.subscribe((message) => {
          if (cancelled) return;
          const parsed = parseHandPresetStatus(message);
          if (parsed) {
            setStatus({ ...parsed, receivedAt: Date.now() });
          }
        });
      } catch (error) {
        if (!cancelled) {
          console.error('Hand preset status connection error:', error);
        }
      }
    };

    subscribe();

    return () => {
      cancelled = true;
      if (topic) {
        try {
          topic.unsubscribe();
        } catch (_error) {}
      }
    };
  }, [enabled, rosbridgeUrl]);

  return status;
}
