#!/usr/bin/env python3
"""Exercise operator interlocks and watchdogs without calling a control service."""
import argparse
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import rclpy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from sm_carry_real_guard import CarryGuard


def main():
    p=argparse.ArgumentParser();p.add_argument('--assets',required=True,type=Path);args=p.parse_args()
    rclpy.init()
    with tempfile.TemporaryDirectory() as directory:
        guard=CarryGuard(args.assets,Path(directory)/'guard.jsonl')
        calls=[]
        guard.switch=lambda activate,callback=None: (calls.append(activate) or True)
        def joy(*pressed):
            message=Joy();message.buttons=[int(i in pressed) for i in range(16)];message.axes=[0.]*4
            guard.joy(message)
        def activity(standby,walking):
            guard.activity(SimpleNamespace(controllers=[SimpleNamespace(name=name,state=SimpleNamespace(id=state))
                for name,state in [('state_estimator',3),('standby_controller',standby),('walking_controller',walking)]]))
        def ready():
            guard.joint_q=guard.first_q.copy();guard.last_joint=time.monotonic();guard.last_joy=time.monotonic()
            guard.last_debug=time.monotonic();guard.stop_pending=False
            guard.last_stop=-float('inf')
            guard.last_orientation=time.monotonic();guard.gravity=guard.reference_gravity[0].copy()
        try:
            activity(2,2);ready()
            joy(0,5);assert not calls, 'Tracking accepted before preparation'
            joy();joy(0,4);assert calls[-1]==['standby_controller']
            activity(3,2);ready();joy();joy(0,5)
            assert calls[-1]==['standby_controller'], 'Tracking accepted before standby settles'
            guard.standby_since=time.monotonic()-3;joy();guard.last_joint=time.monotonic()-1;joy(0,5)
            assert calls[-1]==['standby_controller'], 'Tracking accepted with stale joints'
            ready();guard.joint_q=guard.first_q+.5;joy();joy(0,5)
            assert calls[-1]==['standby_controller'], 'Tracking accepted away from first pose'
            ready();joy();joy(0,5);assert calls[-1]==['walking_controller']
            activity(2,3)
            row=np.zeros(773);row[0]=len(guard.reference_gravity)-1;row[11:14]=guard.reference_gravity[-1]
            message=Float64MultiArray();message.data=row.tolist();guard.debug(message)
            assert guard.completed and calls[-1]==['walking_controller'], 'End frame unexpectedly stops policy'
            ready();guard.tick();assert calls[-1]==['walking_controller']
            joy(1);assert calls[-1]==[], 'B did not stop all motor controllers'
            activity(2,2);ready();joy();joy(0,4);activity(3,2)
            guard.standby_since=time.monotonic()-3;ready();joy();joy(0,5)
            assert calls[-1]==['walking_controller'], 'Could not rearm after B and preparation'
            activity(2,3);ready();guard.last_debug=time.monotonic()-1;guard.tick()
            assert calls[-1]==[], 'Stale policy did not stop'
            guard.stop_requested=False;ready();guard.last_joy=time.monotonic()-1;guard.tick()
            assert calls[-1]==[], 'Stale remote did not stop'
            guard.stop_requested=False;ready();guard.last_joint=time.monotonic()-1;guard.tick()
            assert calls[-1]==[], 'Stale joints did not stop'
            guard.requested=False;guard.stop_requested=False;activity(3,2);ready()
            guard.standby_since=time.monotonic()-3;guard.tracking_enabled=False
            count=len(calls);joy();joy(0,5)
            assert len(calls)==count, 'Tracking lock did not block R1+A'
            guard.tracking_enabled=True;ready();guard.last_orientation=time.monotonic()-1
            joy();joy(0,5);assert len(calls)==count, 'Stale orientation did not block tracking'
            print('SM_CARRY_GUARD_PASS: preparation, pose gate, freshness, full completion, B, rearm, watchdogs')
        finally:
            guard.file.close();guard.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
