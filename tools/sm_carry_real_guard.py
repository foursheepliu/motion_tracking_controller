#!/usr/bin/env python3
"""Operator-triggered carry trials; full reference + last-frame holding, no time cap."""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from controller_manager_msgs.msg import ControllerManagerActivity
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage

from prepare_sm_carry import rotation


class CarryGuard(Node):
    def __init__(self, assets, output, standby_config=None, tracking_enabled=True):
        super().__init__('sm_carry_operator_guard')
        reference = json.loads((assets/'reference.json').read_text())
        self.names = reference['joint_names']
        self.tracking_enabled=tracking_enabled
        self.first_q = np.array(reference['joint_pos'][0])
        self.prepare_q = self.first_q.copy()
        if standby_config is not None:
            standby=yaml.safe_load(standby_config.read_text())['standby_controller']['ros__parameters']
            if standby['joint_names'] != self.names: raise ValueError('Standby joint order mismatch')
            self.prepare_q=np.array(standby['default_position'])
        self.last_orientation=-math.inf;self.gravity=None
        self.reference_gravity = -rotation(reference['pelvis_quat_wxyz'])[:, 2, :]
        self.file = output.open('a', buffering=1)
        self.states = {}; self.neutral = False; self.previous_buttons = set()
        self.joint_q = None; self.last_joint = self.last_joy = self.last_debug = -math.inf
        self.standby_since = None; self.requested = False; self.stop_requested = False
        self.request_time = None; self.stop_pending = False; self.last_stop = -math.inf
        self.completed = False; self.last_frame = -1
        self.client = self.create_client(SwitchController, '/controller_manager/switch_controller')
        activity_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                  reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(ControllerManagerActivity, '/controller_manager/activity', self.activity, activity_qos)
        self.create_subscription(Joy, '/joy', self.joy, qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.joints, qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, '/sm_carry/debug', self.debug, 100)
        self.create_subscription(TFMessage, '/tf', self.transforms, 100)
        self.create_timer(.02, self.tick)
        self.event('WAITING_FOR_OPERATOR', reference_frames=len(self.reference_gravity),
                   reference_seconds=len(self.reference_gravity)/50, duration_limit=None,
                   end_behavior='hold_last_frame',tracking_enabled=self.tracking_enabled,
                   B='damping', L1_A='prepare', R1_A='tracking')

    def event(self, event, **values):
        line = json.dumps(dict(event=event, wall_time=time.time(), **values))
        print(line, flush=True); self.file.write(line+'\n')

    def activity(self, message):
        previous = self.states
        self.states = {c.name:c.state.id for c in message.controllers}
        if self.states != previous:
            self.event('CONTROLLERS', states=self.states)
        if self.states.get('standby_controller') == 3:
            if self.standby_since is None: self.standby_since = time.monotonic()
        else: self.standby_since = None
        if self.stop_requested and self.states.get('walking_controller') == 3:
            self.stop('late_activation_after_stop')
        if self.requested and self.states.get('walking_controller') != 3 and previous.get('walking_controller') == 3:
            self.requested = False; self.event('TRACKING_INACTIVE')

    def joints(self, message):
        values = dict(zip(message.name, message.position))
        if all(name in values for name in self.names):
            q = np.array([values[name] for name in self.names])
            if np.isfinite(q).all(): self.joint_q=q; self.last_joint=time.monotonic()

    def transforms(self, message):
        for transform in message.transforms:
            if transform.child_frame_id.lstrip('/') != 'pelvis':continue
            q=transform.transform.rotation
            value=np.array([q.w,q.x,q.y,q.z])
            if np.isfinite(value).all() and np.linalg.norm(value)>.5:
                self.gravity=-rotation(value)[2,:];self.last_orientation=time.monotonic()

    def switch(self, activate, callback=None):
        if not self.client.service_is_ready():
            self.event('SERVICE_UNAVAILABLE'); return False
        request = SwitchController.Request()
        request.activate_controllers = activate
        request.deactivate_controllers = [n for n in ['walking_controller','standby_controller'] if n not in activate]
        request.strictness = 1; request.activate_asap = True; request.timeout.sec = 1
        future = self.client.call_async(request)
        if callback: future.add_done_callback(callback)
        return True

    def response(self, future):
        try:
            ok = future.result().ok; self.event('SWITCH_RESPONSE', ok=ok)
            if not ok: self.stop('switch_failed')
        except Exception as error:
            self.event('SWITCH_ERROR', error=str(error)); self.stop('switch_error')

    def joy(self, message):
        now=time.monotonic(); self.last_joy=now
        buttons={i for i,b in enumerate(message.buttons) if b}
        old=self.previous_buttons; self.previous_buttons=buttons
        if not buttons:
            if not self.neutral: self.event('NEUTRAL_RECEIVED')
            self.neutral=True
        if 1 in buttons:
            self.stop('B_pressed'); return
        if {0,4} <= buttons and not {0,4} <= old:
            if not self.neutral: self.event('PREPARE_REJECTED_NO_NEUTRAL'); return
            if self.states.get('state_estimator') != 3 or now-self.last_joint > .10:
                self.event('PREPARE_REJECTED_STATE_NOT_READY'); return
            if self.switch(['standby_controller'], self.response):
                self.requested=False; self.stop_requested=False; self.standby_since=now
                self.event('PREPARE_REQUEST')
        if not ({0,5} <= buttons and not {0,5} <= old): return
        reasons=[]
        if not self.tracking_enabled:reasons.append('tracking_disabled_pending_ground_support')
        if not self.neutral: reasons.append('no_neutral')
        if self.states.get('state_estimator') != 3: reasons.append('state_estimator_inactive')
        if self.states.get('standby_controller') != 3 or self.states.get('walking_controller') != 2:
            reasons.append('not_in_standby')
        if self.standby_since is None or now-self.standby_since < 2.5: reasons.append('prepare_not_settled')
        if now-self.last_joint > .10: reasons.append('joint_feedback_stale')
        if any(not math.isfinite(x) or abs(x)>.1 for x in message.axes): reasons.append('sticks_not_neutral')
        error = float(np.abs(self.joint_q-self.prepare_q).max()) if self.joint_q is not None else math.inf
        reference_error=float(np.abs(self.joint_q-self.first_q).max()) if self.joint_q is not None else math.inf
        if error > .20: reasons.append('preparation_joint_error')
        if reference_error > .60: reasons.append('reference_pose_too_far')
        if now-self.last_orientation > .10: reasons.append('pelvis_orientation_stale')
        elif np.linalg.norm(self.gravity-self.reference_gravity[0])>.35: reasons.append('initial_pelvis_orientation_error')
        if self.requested: reasons.append('already_requested')
        if reasons:
            self.event('TRACKING_REJECTED', reasons=reasons, max_joint_error=error,reference_pose_error=reference_error); return
        if self.switch(['walking_controller'], self.response):
            self.requested=True; self.stop_requested=False; self.request_time=now
            self.last_debug=now; self.completed=False; self.last_frame=-1
            self.event('TRACKING_REQUEST_FULL_REFERENCE', max_joint_error=error,reference_pose_error=reference_error)

    def debug(self, message):
        row=np.asarray(message.data)
        if not self.requested and self.states.get('walking_controller') != 3: return
        self.last_debug=time.monotonic()
        if len(row)!=773 or not np.isfinite(row).all(): self.stop('invalid_policy_telemetry'); return
        frame=int(row[0]); self.last_frame=frame
        idx=min(max(frame,0),len(self.reference_gravity)-1)
        gravity_error=float(np.linalg.norm(row[11:14]-self.reference_gravity[idx]))
        if gravity_error > .65:
            self.stop('pelvis_orientation_deviation'); return
        if not self.completed and frame>=len(self.reference_gravity)-1:
            self.completed=True; self.event('REFERENCE_COMPLETED_HOLDING_LAST_FRAME', frame=frame)
        self.file.write(json.dumps(dict(event='POLICY',values=row.tolist()))+'\n')

    def stop(self, reason):
        self.stop_requested=True
        now=time.monotonic()
        if self.stop_pending or now-self.last_stop < .1: return
        self.last_stop=now
        def done(future):
            self.stop_pending=False
            try: self.event('STOP_RESPONSE',ok=future.result().ok)
            except Exception as error: self.event('STOP_ERROR',error=str(error))
        if self.switch([],done):
            self.stop_pending=True; self.event('STOP_REQUEST',reason=reason,frame=self.last_frame)

    def tick(self):
        now=time.monotonic()
        active=self.states.get('walking_controller')==3
        if self.stop_requested and active:
            self.stop('stop_retry'); return
        if not (self.requested or active): return
        if now-self.last_joy > .30: self.stop('joy_feedback_stale')
        elif now-self.last_joint > .10: self.stop('joint_feedback_stale')
        elif now-self.last_debug > .15 and (active or now-self.request_time > 1.): self.stop('policy_feedback_stale')
        elif self.requested and not active and now-self.request_time > 1.: self.stop('activation_timeout')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--assets',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--standby-config',type=Path)
    p.add_argument('--tracking-disabled',action='store_true')
    args,ros_args=p.parse_known_args();args.output.parent.mkdir(parents=True,exist_ok=True)
    rclpy.init(args=ros_args);node=CarryGuard(args.assets,args.output,args.standby_config,not args.tracking_disabled)
    try:rclpy.spin(node)
    finally:
        node.file.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__': main()
