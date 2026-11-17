"""SuperMimic carry policy in MuJoCo only. No real-hardware backend is launched."""
import json
import os
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, Shutdown, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    assets = Path(LaunchConfiguration('assets').perform(context)).expanduser().resolve()
    control = json.loads((assets/'control.json').read_text())
    staged = LaunchConfiguration('staged_start').perform(context).lower() == 'true'
    own = Path(get_package_share_directory('motion_tracking_controller'))
    description = Path(get_package_share_directory('unitree_description'))
    robot = {'robot_description': Command(['xacro ',str(description/'urdf/g1/robot.xacro'),
                                          ' robot_type:=g1 simulation:=mujoco'])}
    parameters = str(own/'config/g1/sm_carry_controllers.yaml')
    overrides = {'walking_controller': {'ros__parameters': {
        'policy.path':str(assets/'actor.onnx'),
        'sm_carry.control_path':str(assets/'control.json'),
        'sm_carry.reference_path':str(assets/'reference.json'),
        'sm_carry.align_heading':True,
        'sm_carry.debug':True}}}
    # Keep generated configuration in the caller's asset directory, not a shared /tmp filename.
    import yaml
    merged = yaml.safe_load(Path(parameters).read_text())
    merged['walking_controller']['ros__parameters'].update(overrides['walking_controller']['ros__parameters'])
    merged['standby_controller']['ros__parameters'].update({
        'joint_names':control['joint_names'], 'default_position':json.loads((assets/'reference.json').read_text())['joint_pos'][0],
        'kp':control['kp'],'kd':control['kd']})
    if staged:
        real_standby = yaml.safe_load((own/'config/g1/controllers.yaml').read_text())['standby_controller']['ros__parameters']
        merged['standby_controller']['ros__parameters'] = real_standby
    generated = assets/'controllers.generated.yaml'
    generated.write_text(yaml.safe_dump(merged))
    # A local XML copy keeps the installed robot description untouched. A fixed
    # trackcom camera follows the simulated robot without mouse-based selection.
    tree = ET.parse(description/'mjcf/g1.xml')
    compiler = tree.getroot().find('compiler')
    compiler.set('meshdir', str((description/'mjcf'/compiler.get('meshdir', '')).resolve()))
    world = tree.getroot().find('worldbody')
    ET.SubElement(world, 'camera', name='sm_carry_follow', mode='trackcom',
                  pos='1.8 -2.4 1.5', xyaxes='0.8 0.6 0 -0.12 0.16 0.98')
    armatures = dict(zip(control['joint_names'],control['armature']))
    efforts = dict(zip(control['joint_names'],control.get('effort_limits_nm',[])))
    for joint in world.iter('joint'):
        if joint.get('name') in armatures:
            joint.set('armature', str(armatures[joint.get('name')]))
        if joint.get('name') in efforts:
            joint.set('actuatorfrclimited','true')
            joint.set('actuatorfrcrange',' '.join(str(x) for x in efforts[joint.get('name')]))
    model_xml = assets/'sim.generated.xml'
    tree.write(model_xml)
    model_file = '/'+os.path.relpath(model_xml,description)
    startup = assets/'startup.yaml'
    if staged:
        startup_data = yaml.safe_load(startup.read_text())
        startup_data['mujoco_sim_ros2_node']['ros__parameters']['startup_controllers'] = ['state_estimator','standby_controller']
        startup_data['mujoco_sim_ros2_node']['ros__parameters']['initial_joint_positions'] = real_standby['default_position']
        startup = assets/'startup.staged.generated.yaml'
        startup.write_text(yaml.safe_dump(startup_data))
    simulator = Node(package='mujoco_sim_ros2',executable='mujoco_sim',output='screen',
        parameters=[{'model_package':'unitree_description','model_file':model_file,
                     'physics_plugins':['mujoco_ros2_control::MujocoRos2ControlPlugin'], 'use_sim_time':True},
                    robot,str(generated),str(startup)],
        on_exit=Shutdown(reason='SM carry MuJoCo simulation exited'))
    publisher = Node(package='robot_state_publisher',executable='robot_state_publisher',
                     parameters=[robot,{'use_sim_time':True}],output='screen')
    spawner = Node(package='controller_manager',executable='spawner',output='screen',
                   arguments=['state_estimator','walking_controller','--param-file',str(generated)])
    if staged:
        loader = Node(package='controller_manager',executable='spawner',output='screen',
            arguments=['state_estimator','standby_controller','--param-file',str(generated)])
        inactive = Node(package='controller_manager',executable='spawner',output='screen',
            arguments=['walking_controller','--inactive','--param-file',str(generated)])
        guard = ExecuteProcess(cmd=['python3',str(own/'tools/sm_carry_real_guard.py'),'--assets',str(assets),
                                    '--output',str(assets/'guard.sim.jsonl'),'--standby-config',str(generated)],output='screen')
        return [simulator,publisher,RegisterEventHandler(OnProcessExit(target_action=loader,on_exit=[inactive])),
                TimerAction(period=float(LaunchConfiguration('controller_start_delay').perform(context)),actions=[loader]),guard]
    return [simulator,publisher,TimerAction(period=float(LaunchConfiguration('controller_start_delay').perform(context)),actions=[spawner])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('assets',description='Prepared SM carry asset directory'),
        DeclareLaunchArgument('controller_start_delay',default_value='0.0'),
        DeclareLaunchArgument('staged_start',default_value='false'),
        OpaqueFunction(function=setup)])
