export const PREVIEW_FPS = 12;
export const PREVIEW_TIMEOUT_MS = 750;

// At most one request + decode at a time. Slow connections reduce preview FPS
// instead of accumulating older frames in a continuous MJPEG stream.
export function startLatestCameraPreview({ url, draw, onStatus,
  request = fetch, decode = createImageBitmap, now = () => performance.now() }) {
  let stopped = false;
  let next;
  let deadline;
  let controller;

  const frame = async () => {
    if (stopped) return;
    const started = now();
    controller = new AbortController();
    const current = controller;
    deadline = setTimeout(() => current.abort(), PREVIEW_TIMEOUT_MS);
    let delay = 1000 / PREVIEW_FPS;
    try {
      const response = await request(url, { signal: current.signal, cache: 'no-store' });
      if (!response.ok) throw new Error('Preview unavailable');
      const bitmap = await decode(await response.blob());
      try {
        if (!stopped && !current.signal.aborted && now() - started < PREVIEW_TIMEOUT_MS) {
          draw(bitmap);
          onStatus('live');
        } else if (!stopped) {
          onStatus('waiting');
        }
      } finally {
        bitmap.close();
      }
    } catch (_) {
      if (!stopped) onStatus('waiting');
      delay = 250;
    } finally {
      clearTimeout(deadline);
      if (!stopped) next = setTimeout(frame, Math.ceil(Math.max(0, delay - (now() - started))));
    }
  };
  frame();
  return () => {
    stopped = true;
    clearTimeout(next);
    clearTimeout(deadline);
    controller?.abort();
  };
}
