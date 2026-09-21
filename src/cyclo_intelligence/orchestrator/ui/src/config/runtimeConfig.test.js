function loadConfig(config = {}) {
  window.__CYCLO_CONFIG__ = config;
  jest.resetModules();
  return require('./runtimeConfig');
}

afterEach(() => {
  delete window.__CYCLO_CONFIG__;
  jest.resetModules();
});

test('keeps the direct rosbridge port for profiles without a proxy path', () => {
  const config = loadConfig({ rosbridgePort: 7390 });
  expect(config.CYCLO_ROSBRIDGE_PATH).toBe('');
  expect(config.buildRosbridgeUrl('robot.local', {
    protocol: 'http:', host: 'forwarded.local:8080',
  })).toBe('ws://robot.local:7390');
  expect(config.buildRosbridgeUrl('robot.local', {
    protocol: 'https:', host: 'forwarded.local:8443',
  })).toBe('wss://robot.local:7390');
});

test('uses the current UI origin for the Isaac proxy including a forwarded port', () => {
  const config = loadConfig({ rosbridgePort: 7890, rosbridgePath: '/rosbridge/' });
  expect(config.buildRosbridgeUrl('internal-robot', {
    protocol: 'http:', host: 'remote.local:7880',
  })).toBe('ws://remote.local:7880/rosbridge/');
  expect(config.buildRosbridgeUrl('different-host', {
    protocol: 'https:', host: 'remote.local:8443',
  })).toBe('wss://remote.local:8443/rosbridge/');
});

test('normalizes an absolute proxy path and supports multiple simple segments', () => {
  const config = loadConfig({ rosbridgePath: '/robot/rosbridge' });
  expect(config.CYCLO_ROSBRIDGE_PATH).toBe('/robot/rosbridge/');
  expect(config.normalizeRosbridgePath(' /rosbridge/ ')).toBe('/rosbridge/');
  expect(config.normalizeRosbridgePath('/robot-1/ros_bridge')).toBe('/robot-1/ros_bridge/');
});

test.each(['', '/', '//remote/rosbridge', 'https://remote/rosbridge', '/path?x=1', '/path;bad', '/path/../rosbridge'])
  ('rejects a non-proxy path %s and retains the direct target', (rosbridgePath) => {
    const config = loadConfig({ rosbridgePath });
    expect(config.CYCLO_ROSBRIDGE_PATH).toBe('');
    expect(config.buildRosbridgeUrl('robot', { protocol: 'http:' })).toBe('ws://robot:7090');
  });

test('supports IPv6 and an empty host without creating a malformed URL', () => {
  const config = loadConfig({ rosbridgePort: 7890 });
  expect(config.buildRosbridgeUrl('::1', { protocol: 'http:' })).toBe('ws://[::1]:7890');
  expect(config.buildRosbridgeUrl('[::1]', { protocol: 'http:' })).toBe('ws://[::1]:7890');
  expect(config.buildRosbridgeUrl('', { protocol: 'http:' })).toBe('');
});

test('does not fall back to a private direct port when a proxy has no UI origin', () => {
  const config = loadConfig({ rosbridgePath: '/rosbridge/' });
  expect(config.buildRosbridgeUrl('internal-robot', null)).toBe('');
});
