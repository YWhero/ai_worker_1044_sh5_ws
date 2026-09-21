#!/usr/bin/env python3
"""Restore the HX5 workcell by relaunching Gazebo, keeping Cyclo/data intact.

Use a relaunch rather than resetting gz_ros2_control's live entity handles.
Mapping, tasks and inference must be idle; unsaved recordings are rejected.
"""
import argparse
import json
import subprocess
import urllib.request

AI = 'ai_worker_1044_hx5_sim'
CYCLO = 'cyclo_intelligence_1044_hx5_sim'
SETUP = 'source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; '

STATUS_PROBE = '''
import json, time
import rclpy
from interfaces.msg import RecordingStatus, InferenceStatus
rclpy.init()
node=rclpy.create_node('hx5_reset_idle_check')
received={}
topics={'recording': ('/data/recording/status', RecordingStatus, 'record_phase'),
        'inference': ('/task/inference_status', InferenceStatus, 'inference_phase')}
subscriptions=[]
for key,(topic,kind,field) in topics.items():
    subscriptions.append(node.create_subscription(kind,topic,
        lambda msg,k=key,f=field: received.update({k:getattr(msg,f)}),10))
deadline=time.monotonic()+3
while time.monotonic()<deadline:
    rclpy.spin_once(node,timeout_sec=0.1)
result={key:{'publishers':node.count_publishers(topic),'phase':received.get(key)}
        for key,(topic,_,_) in topics.items()}
node.destroy_node()
rclpy.shutdown()
print(json.dumps(result))
'''

STOP_GAZEBO = '''
from pathlib import Path
import json,os,signal,time
matches=[]
processes={}
for path in Path('/proc').glob('[0-9]*/cmdline'):
    try:
        args=path.read_bytes().decode().split('\\0')
        pid=int(path.parent.name)
        stat=(path.parent/'stat').read_text().rsplit(')',1)[1].split()
        processes[pid]=(int(stat[1]),stat[19],args)
        if (len(args)>1 and Path(args[0]).name.startswith('python')
            and args[1].endswith('/bin/ros2') and 'launch' in args
            and 'hx5_simulation' in args and 'workcell.launch.py' in args):
            matches.append((pid,args))
    except (OSError,UnicodeError): pass
if len(matches)!=1: raise RuntimeError(f'Expected one HX5 Gazebo launcher, found {len(matches)}')
pid,args=matches[0]
options=args[args.index('workcell.launch.py')+1:]
options=[x for x in options if x]
# Gazebo server/GUI create their own process groups. Track descendants before
# launch exits, and collect exact orphan Gazebo processes from earlier launches
# in this dedicated simulation container (never use a host-wide pkill).
tracked={pid}
tracked.update(p for p,(_,_,argv) in processes.items()
               if argv and argv[0] in ('gz sim server','gz sim gui'))
while True:
    children={p for p,(parent,_,_) in processes.items() if parent in tracked}
    if children<=tracked: break
    tracked.update(children)
def alive(p):
    try:
        stat=Path(f'/proc/{p}/stat').read_text().rsplit(')',1)[1].split()
        return stat[0]!='Z' and stat[19]==processes[p][1]
    except OSError: return False
def signal_remaining(kind):
    for p in tracked:
        if alive(p):
            try: os.kill(p,kind)
            except ProcessLookupError: pass
def wait_remaining(seconds):
    deadline=time.monotonic()+seconds
    while any(alive(p) for p in tracked) and time.monotonic()<deadline: time.sleep(0.1)
signal_remaining(signal.SIGINT)
wait_remaining(8)
signal_remaining(signal.SIGTERM)
wait_remaining(3)
signal_remaining(signal.SIGKILL)
wait_remaining(1)
if any(alive(p) for p in tracked):
    raise RuntimeError('Gazebo did not stop cleanly; no second simulator started')
print(json.dumps(options))
'''


def api(path):
    with urllib.request.urlopen('http://127.0.0.1:7380/api/'+path,timeout=8) as response:
        return json.load(response)


def check_idle():
    nav=api('navigation/status')
    engine=api('services/bt_node/status')
    if nav['is_up'] or engine['state']!='down':
        raise RuntimeError('Stop Mapping/Navigation and turn off Task Engine before resetting')
    command=['docker','exec',CYCLO,'bash','--noprofile','--norc','-c',
             SETUP+'exec python3 -c "$1"','bash',STATUS_PROBE]
    output=subprocess.check_output(command,text=True,timeout=12)
    status=json.loads(output.strip().splitlines()[-1])
    recording=status['recording']
    if recording['publishers']==0 or recording['phase']!=0:
        raise RuntimeError('Recording status is unavailable or not READY; save/discard recording first')
    inference=status['inference']
    # The orchestrator can advertise this topic without publishing until a
    # backend exists. The backend checks below still reject every running
    # policy container, including one whose heartbeat is unavailable.
    if inference['phase'] not in (None,0):
        raise RuntimeError('Clear loaded/running inference before resetting')
    for name in ('lg2_leader_1044_hx5_sim','lerobot_server_1044_hx5_sim',
                 'vitacformer_server_1044_hx5_sim','groot_server_1044_hx5_sim'):
        result=subprocess.run(['docker','inspect','--format','{{.State.Running}}',name],
                              text=True,capture_output=True,check=False)
        if result.returncode==0 and result.stdout.strip()=='true':
            raise RuntimeError(f'Stop {name} before resetting; command publishers must be idle')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--initial-pose', choices=('inference', 'navigation', 'task'),
                        help='Override the previous launch pose; otherwise keep its options')
    args=parser.parse_args()
    check_idle()
    if args.check:
        print('HX5 workcell reset preflight passed')
        return
    options=json.loads(subprocess.check_output(
        ['docker','exec',AI,'python3','-c',STOP_GAZEBO],text=True,timeout=16))
    if args.initial_pose:
        options=[option for option in options if not option.startswith('initial_pose:=')]
        options.append('initial_pose:='+args.initial_pose)
    subprocess.run(['docker','exec','-d',AI,'bash','--noprofile','--norc','-c',
        SETUP+'exec flock -n -o /tmp/hx5-gazebo.lock ros2 launch hx5_simulation '
        'workcell.launch.py "$@" > /tmp/hx5-gazebo-reset.log 2>&1',
        'bash',*options],check=True)
    print('HX5 workcell relaunch started; log: ai_worker_1044_hx5_sim:/tmp/hx5-gazebo-reset.log')
    print('Localize again before running a mission; maps/tasks/presets/recordings are preserved.')


if __name__=='__main__':
    main()
