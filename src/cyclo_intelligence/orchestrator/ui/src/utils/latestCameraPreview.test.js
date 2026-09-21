import { startLatestCameraPreview, PREVIEW_TIMEOUT_MS } from './latestCameraPreview';

const flush = async () => { for (let i = 0; i < 8; i += 1) await Promise.resolve(); };
beforeEach(() => jest.useFakeTimers());
afterEach(() => jest.useRealTimers());
const bitmap = () => ({ width: 424, height: 238, close: jest.fn() });
const response = { ok: true, blob: async () => ({}) };

test('waits for transfer and decode before requesting another frame', async () => {
  let resolve;
  const request = jest.fn(() => new Promise((done) => { resolve = done; }));
  const image = bitmap();
  const draw = jest.fn();
  const stop = startLatestCameraPreview({ url: '/preview', request, decode: async () => image, draw, onStatus: jest.fn() });
  jest.advanceTimersByTime(500);
  expect(request).toHaveBeenCalledTimes(1);
  resolve(response); await flush();
  expect(draw).toHaveBeenCalledTimes(1);
  expect(image.close).toHaveBeenCalledTimes(1);
  jest.advanceTimersByTime(1); await flush();
  expect(request).toHaveBeenCalledTimes(2);
  stop();
});

test('discards a late frame and closes its decoded image', async () => {
  let resolve;
  const image = bitmap();
  const request = jest.fn(() => new Promise((done) => { resolve = done; }));
  const draw = jest.fn(); const status = jest.fn();
  const stop = startLatestCameraPreview({ url: '/preview', request, decode: async () => image, draw, onStatus: status });
  jest.advanceTimersByTime(PREVIEW_TIMEOUT_MS + 1);
  expect(request.mock.calls[0][1].signal.aborted).toBe(true);
  resolve(response); await flush();
  expect(draw).not.toHaveBeenCalled();
  expect(status).toHaveBeenLastCalledWith('waiting');
  expect(image.close).toHaveBeenCalled();
  stop();
});

test('unmount aborts requests and prevents late drawing or retries', async () => {
  let resolve;
  const request = jest.fn(() => new Promise((done) => { resolve = done; }));
  const image = bitmap(); const draw = jest.fn();
  const stop = startLatestCameraPreview({ url: '/preview', request, decode: async () => image, draw, onStatus: jest.fn() });
  stop(); resolve(response); await flush();
  jest.advanceTimersByTime(2000);
  expect(request).toHaveBeenCalledTimes(1);
  expect(draw).not.toHaveBeenCalled();
  expect(image.close).toHaveBeenCalled();
});

test('limits fast responses to 12 fps and backs off on errors', async () => {
  const request = jest.fn().mockResolvedValue(response);
  const stop = startLatestCameraPreview({ url: '/preview', request, decode: async () => bitmap(), draw: jest.fn(), onStatus: jest.fn() });
  await flush();
  jest.advanceTimersByTime(80); await flush();
  expect(request).toHaveBeenCalledTimes(1);
  jest.advanceTimersByTime(4); await flush();
  expect(request).toHaveBeenCalledTimes(2);
  stop();
});
