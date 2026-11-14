"""Staged G1 hardware startup. No motor controller activates until operator input."""
import json
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    assets=Path(LaunchConfiguration('assets').perform(context)).expanduser().resolve()
    output=Path(LaunchConfiguration('output').perform(context)).expanduser().resolve()
    interface=LaunchConfiguration('network_interface').perform(context)
    tracking_enabled=LaunchConfiguration('enable_tracking').perform(context).lower()=='true'
    if not interface or not all(c.isalnum() or c in '_.:-' for c in interface):
        raise ValueError('Invalid network interface')
    output.mkdir(parents=True,exist_ok=True)
    own=Path(get_package_share_directory('motion_tracking_controller'))
    description=Path(get_package_share_directory('unitree_description'))
    control=json.loads((assets/'control.json').read_text())
    reference=json.loads((assets/'reference.json').read_text())
    if reference['joint_names']!=control['joint_names']:raise ValueError('Joint order mismatch')
    params=yaml.safe_load((own/'config/g1/sm_carry_controllers.yaml').read_text())
    original=yaml.safe_load((own/'config/g1/controllers.yaml').read_text())
    # Keep the previously tested neutral standby; the reference's first pose is not a static balance controller.
    params['standby_controller']=original['standby_controller']
    params['walking_controller']['ros__parameters'].update({
        'policy.path':str(assets/'actor.onnx'),'sm_carry.control_path':str(assets/'control.json'),
        'sm_carry.reference_path':str(assets/'reference.json'),'sm_carry.align_heading':True,'sm_carry.debug':True})
    generated=output/'controllers.real.generated.yaml'
    generated.write_text(yaml.safe_dump(params))
    robot={'robot_description':Command(['xacro ',str(description/'urdf/g1/robot.xacro'),
                                        ' robot_type:=g1 simulation:=false network_interface:=',interface])}
    control_node=Node(package='controller_manager',executable='ros2_control_node',output='screen',respawn=False,
        parameters=[robot,str(generated),{'hardware_components_initial_state.unconfigured':['UnitreeSdk2']}],
        on_exit=Shutdown(reason='Carry hardware controller manager exited'))
    publisher=Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[robot],output='screen')
    loader=Node(package='controller_manager',executable='spawner',output='screen',
        arguments=['state_estimator','standby_controller','walking_controller','--inactive','--param-file',str(generated)])
    hardware=ExecuteProcess(cmd=['bash','-c','ros2 control set_hardware_component_state UnitreeSdk2 inactive && '
                                 'ros2 control set_hardware_component_state UnitreeSdk2 active'],output='screen')
    estimator=ExecuteProcess(cmd=['ros2','control','switch_controllers','--activate','state_estimator',
                                  '--strict','--activate-asap'],output='screen')
    guard=ExecuteProcess(cmd=['python3',str(own/'tools/sm_carry_real_guard.py'),'--assets',str(assets),
                             '--output',str(output/'operator_and_policy.jsonl'),'--standby-config',str(generated)]
                             + ([] if tracking_enabled else ['--tracking-disabled']),output='screen')
    emergency=ExecuteProcess(cmd=['ros2','control','switch_controllers','--deactivate','walking_controller',
                                  'standby_controller','--best-effort','--activate-asap'],output='screen')
    def after_load(event, _context):
        return [hardware] if event.returncode==0 else [Shutdown(reason='Carry controller loading failed')]
    def after_hardware(event, _context):
        return [estimator] if event.returncode==0 else [Shutdown(reason='Carry hardware activation failed')]
    return [control_node,publisher,
            RegisterEventHandler(OnProcessExit(target_action=loader,on_exit=after_load)),
            RegisterEventHandler(OnProcessExit(target_action=hardware,on_exit=after_hardware)),
            RegisterEventHandler(OnProcessExit(target_action=guard,on_exit=[emergency])),
            RegisterEventHandler(OnProcessExit(target_action=emergency,on_exit=[Shutdown(reason='Carry guard exited')])),
            loader,guard]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('assets'),DeclareLaunchArgument('output'),
                              DeclareLaunchArgument('enable_tracking',default_value='false'),
                              DeclareLaunchArgument('network_interface',default_value='eth0'),
                              OpaqueFunction(function=setup)])
