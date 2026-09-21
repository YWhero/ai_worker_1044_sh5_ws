import React, { useEffect, useRef, useState } from 'react';
import { useSelector } from 'react-redux';
import { CYCLO_CAMERA_PREVIEW_PORT } from '../config/runtimeConfig';
import { startLatestCameraPreview } from '../utils/latestCameraPreview';

export default function RecordCameraPreview({ topic, rotationDegrees = 0, isActive = true }) {
  const host = useSelector((state) => state.ros.rosHost);
  const canvas = useRef(null);
  const [status, setStatus] = useState('waiting');
  const [visible, setVisible] = useState(document.visibilityState !== 'hidden');
  useEffect(() => {
    const changed = () => setVisible(document.visibilityState !== 'hidden');
    document.addEventListener('visibilitychange', changed);
    return () => document.removeEventListener('visibilitychange', changed);
  }, []);
  useEffect(() => {
    setStatus('waiting');
    if (!isActive || !visible || !topic || !host) return undefined;
    return startLatestCameraPreview({
      url: `http://${host}:${CYCLO_CAMERA_PREVIEW_PORT}/snapshot?topic=${encodeURIComponent(topic)}`,
      draw: (bitmap) => {
        if (!canvas.current) return;
        const target = canvas.current;
        if (target.width !== bitmap.width || target.height !== bitmap.height) {
          target.width = bitmap.width;
          target.height = bitmap.height;
        }
        target.getContext('2d').drawImage(bitmap, 0, 0);
      },
      onStatus: setStatus,
    });
  }, [host, isActive, visible, topic]);
  const rotation = ((Number(rotationDegrees) % 360) + 360) % 360;
  const swaps = rotation === 90 || rotation === 270;
  return (
    <div className="w-full h-full relative overflow-hidden" style={{ containerType: 'size' }}>
      <canvas ref={canvas} aria-label={`Preview ${topic}`} style={{
        position: 'absolute', left: '50%', top: '50%', objectFit: 'contain',
        width: swaps ? '100cqh' : '100%', height: swaps ? '100cqw' : '100%',
        transform: `translate(-50%, -50%) rotate(${rotation}deg)`,
        visibility: status === 'live' ? 'visible' : 'hidden',
      }} />
      {status !== 'live' && <div className="absolute inset-0 flex items-center justify-center text-gray-500 text-sm">
        {isActive && visible ? 'Waiting for live preview…' : 'Preview paused'}
      </div>}
    </div>
  );
}
