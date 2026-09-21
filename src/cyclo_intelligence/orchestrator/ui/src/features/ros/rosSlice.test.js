function loadSlice(config = {}) {
  window.__CYCLO_CONFIG__ = config;
  jest.resetModules();
  return require('./rosSlice');
}

afterEach(() => {
  delete window.__CYCLO_CONFIG__;
  jest.resetModules();
});

test('resolves the initial direct target and updates it when the ROS host changes', () => {
  const { default: reducer, setRosHost } = loadSlice({ rosbridgePort: 7390 });
  const initial = reducer(undefined, { type: '@@init' });
  expect(initial.rosbridgeUrl).toBe(`ws://${window.location.hostname}:7390`);
  const updated = reducer(initial, setRosHost('robot.local'));
  expect(updated.rosHost).toBe('robot.local');
  expect(updated.rosbridgeUrl).toBe('ws://robot.local:7390');
});

test('initializes the Isaac proxy before rendering and retains the UI origin on host changes', () => {
  const { default: reducer, setRosHost } = loadSlice({
    rosbridgePort: 7890, rosbridgePath: '/rosbridge/',
  });
  const expected = `ws://${window.location.host}/rosbridge/`;
  const initial = reducer(undefined, { type: '@@init' });
  expect(initial.rosbridgeUrl).toBe(expected);
  const updated = reducer(initial, setRosHost('internal-robot'));
  expect(updated.rosHost).toBe('internal-robot');
  expect(updated.rosbridgeUrl).toBe(expected);
});

test('preserves explicit connection overrides and connection error state', () => {
  const { default: reducer, setRosbridgeUrl, setConnectionError, resetConnection } = loadSlice({
    rosbridgePath: '/rosbridge/',
  });
  const override = reducer(undefined, setRosbridgeUrl('wss://explicit.local/robot/'));
  expect(override.rosbridgeUrl).toBe('wss://explicit.local/robot/');
  const failed = reducer(override, setConnectionError('connection refused'));
  expect(failed.connectionError).toBe('connection refused');
  const reset = reducer(failed, resetConnection());
  expect(reset.connectionError).toBeNull();
  expect(reset.rosbridgeUrl).toBe(override.rosbridgeUrl);
});
