"""Restore the SH5 and all 72 payloads through an acknowledged PhysX reset."""
import argparse
import json
import subprocess
import urllib.request

CYCLO = 'cyclo_intelligence_1044_hx5_isaac'
AI = 'ai_worker_1044_hx5_isaac'
SETUP = ('source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; '
         'source /workspace/ros_transport/install/local_setup.bash; ')
PROBE = '''
import json,time,rclpy
from interfaces.msg import RecordingStatus,InferenceStatus
rclpy.init();n=rclpy.create_node('isaac_reset_idle_check');out={};subs=[]
for key,topic,typ,field in [('recording','/data/recording/status',RecordingStatus,'record_phase'),
                          ('inference','/task/inference_status',InferenceStatus,'inference_phase')]:
    subs.append(n.create_subscription(typ,topic,lambda m,k=key,f=field:out.update({k:getattr(m,f)}),10))
until=time.monotonic()+3
while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.1)
out['recording_publishers']=n.count_publishers('/data/recording/status')
n.destroy_node();rclpy.shutdown();print(json.dumps(out))
'''
RESET = '''
import json,time,rclpy
import sys
from std_srvs.srv import Trigger
rclpy.init();n=rclpy.create_node('isaac_reset_request');c=n.create_client(Trigger,sys.argv[1])
if not c.wait_for_service(timeout_sec=4):raise RuntimeError('Isaac reset bridge unavailable')
f=c.call_async(Trigger.Request());until=time.monotonic()+20
while not f.done() and time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.1)
if not f.done():raise RuntimeError('Isaac reset response timed out')
r=f.result();print(json.dumps({'success':r.success,'message':r.message}))
n.destroy_node();rclpy.shutdown()
if not r.success:raise RuntimeError(r.message)
'''


def api(path):
    with urllib.request.urlopen('http://127.0.0.1:7880/api/' + path, timeout=8) as response:
        return json.load(response)


def check_idle():
    leader = subprocess.run(['docker', 'inspect', '--format', '{{.State.Running}}',
        'lg2_leader_1044_hx5_isaac'], capture_output=True, text=True)
    if leader.returncode == 0 and leader.stdout.strip() == 'true':
        raise RuntimeError('Stop physical Skeleton Leader with runtime/hx5_isaac.sh leader-stop before resetting/stopping Isaac')
    if api('navigation/status')['is_up'] or api('services/bt_node/status')['state'] != 'down':
        raise RuntimeError('Stop Mapping/Navigation and turn off Task Engine before resetting')
    output = subprocess.check_output(['docker', 'exec', CYCLO, 'bash', '--noprofile', '--norc', '-c',
        SETUP + 'exec python3 -c "$1"', 'bash', PROBE], text=True, timeout=12)
    state = json.loads(output.strip().splitlines()[-1])
    if not state['recording_publishers'] or state.get('recording') is None:
        raise RuntimeError('Recording status heartbeat unavailable; start Isaac and press GUI Play '
                           'so /clock and Cyclo status advance before checking readiness')
    if state['recording'] != 0:
        raise RuntimeError('Save/discard recording and return to READY before resetting')
    if state.get('inference') not in (None, 0):
        raise RuntimeError('STOP and CLEAR loaded inference before resetting')
    for backend in ('vitacformer', 'lerobot', 'groot'):
        result = subprocess.run(['docker', 'inspect', '--format', '{{.State.Running}}',
            f'{backend}_server_1044_hx5_isaac'], capture_output=True, text=True)
        if result.returncode == 0 and result.stdout.strip() == 'true' and state.get('inference') != 0:
            raise RuntimeError(f'{backend} is running without an unloaded inference heartbeat')
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Check readiness without resetting')
    parser.add_argument('--stop', action='store_true', help='Stop this Isaac process after checking readiness')
    args = parser.parse_args()
    state = check_idle()
    if args.check:
        print(json.dumps({'ready': True, 'status': state}))
        return
    setup = 'source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; '
    subprocess.run(['docker', 'exec', AI, 'bash', '--noprofile', '--norc', '-c',
        setup + 'exec python3 -c "$1" "$2"', 'bash', RESET,
        '/simulation/shutdown' if args.stop else '/simulation/reset'], check=True, timeout=28)
    if args.stop:
        print('SH5 Isaac process stopped; containers and saved data are preserved.')
    else:
        print('Initial SH5/HX5 pose and all payload placements restored; localize again before running a mission.')


if __name__ == '__main__':
    main()
