import React, { useState, useRef, useEffect, useCallback, useMemo } from 'react';
import { Activity, ArrowRightLeft, Box, X, FileText, Sliders, Cpu, Database, Settings, ChevronDown, ChevronUp, Info, Palette, Layers } from 'lucide-react';

// ═══════════════════════════════════════════════════════════════
//  SYSTEM DATA
// ═══════════════════════════════════════════════════════════════
const systemData = {
  nodes: {
    'A':      { title: 'User / Script / GUI', type: 'Interface & Config', description: 'The entry point for triggering robot actions. Users edit task definitions, write G-code files, and invoke launch files to start either the simulation or hardware pipeline. Also represents static configuration parameters injected at launch.', actions_taken: ['Edits task JSON files (config/examples/)', 'Writes .gcode files for CNC-style paths', 'Launches delta_robot_spawn.launch.py (sim)', 'Launches delta_robot.launch.py (hardware)', 'Launches gcode_json_tools.launch.py (task runners)'], files: ['config/examples/example_task.json', 'config/examples/circle_test.gcode', 'config/delta_config.yaml', 'config/ros2_controllers.yaml'], parameters: ['json_units (meters|mm)', 'default_duration_s', 'motion_rate_hz', 'home_x_mm / home_y_mm / home_z_mm'] },
    'B':      { title: 'json_task_sequencer.py', type: 'Python ROS2 Node', description: 'Reads high-level task descriptions from JSON and executes them sequentially. Supports batched trajectory mode (PlayCustomTrajectory) for high-throughput, or falls back to point-by-point MoveToPoint. Supported actions: move, home, wait, tilt, spin, suction.', services_called: ['delta_motion_planner/move_to_point', 'delta_motion_planner/play_custom_trajectory'], parameters: ['move_to_point_service', 'play_custom_trajectory_service', 'json_units (meters|mm)', 'default_duration_s', 'motion_rate_hz', 'home_x/y/z_mm'], files: ['delta_robot/json_task_sequencer.py'], actions_taken: ['Parses JSON → action sequence', 'Supports: move, home, wait, tilt, spin, suction', 'Batches waypoints into PlayCustomTrajectory', 'Falls back to MoveToPoint per-waypoint'] },
    'B2':     { title: 'gcode_parser.py', type: 'Python ROS2 Node', description: 'G-code interpreter for CNC-style motion commands. Parses standard G-code commands and converts them to delta robot motions. Uses batched PlayCustomTrajectory when available for smooth continuous paths.', services_called: ['delta_motion_planner/move_to_point', 'delta_motion_planner/play_custom_trajectory'], parameters: ['move_to_point_service', 'play_custom_trajectory_service', 'default_units (mm)', 'home_z_mm', 'min_move_time_s', 'default_speed_mm_s', 'motion_rate_hz'], files: ['delta_robot/gcode_parser.py'], actions_taken: ['G0/G1 — Linear rapid/feed moves', 'G28 — Home to reference position', 'G90/G91 — Absolute/Relative positioning', 'G20/G21 — Inch/Millimetre units'] },
    'C':      { title: 'motion_planner (C++)', type: 'C++ ROS2 Node', description: 'The core trajectory generation engine. Receives high-level move commands, delegates IK/FK math to kinematics, and publishes motor commands on a separate execution thread. Supports 5 demo trajectories and custom point-list trajectories via PlayCustomTrajectory. Publishes to both simulation (JointTrajectory) and hardware (DeltaJoints) simultaneously.', topics_published: ['delta_motors/set_joints (DeltaJoints)', 'delta_motors/set_joint_vels (DeltaJointVels)', '/joint_trajectory_controller/joint_trajectory'], services_provided: ['play_demo_trajectory', 'move_to_point', 'move_to_configuration', 'motion_demo (start/stop auto-demo)', 'play_custom_trajectory'], services_called: ['delta_kinematics/delta_ik', 'delta_kinematics/delta_fk', 'delta_kinematics/convert_to_joint_trajectory', 'delta_kinematics/convert_to_joint_vel_trajectory'], parameters: ['traj_step_ms (default 10 ms)'], threads: ['TrajectoryExecutor — detached std::thread for non-blocking', 'Init Timer — deferred until kinematics online', 'Demo Timer — 32s auto-demo cycle'], files: ['src/motion_planner.cpp', 'include/motion_planner.hpp'], actions_taken: ['Demo: circle — XY circle at Z=-180mm, R=40mm', 'Demo: pringle — saddle-shaped 3D path', 'Demo: axes — sequential X/Y/Z axis translations', 'Demo: up_down — 5 cycle vertical oscillation', 'Demo: scan — snake-scan from CSV file'] },
    'D':      { title: 'delta_kinematics (C++)', type: 'C++ ROS2 Node', description: 'Provides FK, IK, Jacobian computation, trajectory conversion, and velocity mapping for the Delta Robot parallel kinematics. Uses Eigen3 for Jacobian matrix operations. Publishes periodic RobotConfig messages at 50 Hz combining joint angles, velocities, and FK-derived end-effector position. Initialises joints at safe 45° angle to prevent FK failure on startup.', services_provided: ['delta_kinematics/delta_fk — Forward Kinematics', 'delta_kinematics/delta_ik — Inverse Kinematics', 'convert_to_joint_trajectory — EE path → joint path', 'convert_to_joint_vel_trajectory — EE vel → joint vel'], services_called: ['delta_motors/set_joint_limits'], topics_published: ['delta_robot/robot_config (RobotConfig @ 50Hz)'], topics_subscribed: ['delta_motors/motor_position_feedback (DeltaJoints)', 'delta_motors/motor_velocity_feedback (DeltaJointVels)'], parameters: ['base_triangle_side_length (360.27 mm)', 'end_effector_side_length (138.56 mm)', 'active_link_length (105.0 mm)', 'passive_link_length (218.75 mm)', 'passive_link_width (42.85 mm)', 'joint_min (-10° / -0.175 rad)', 'joint_max (90° / π/2 rad)', 'max_joint_velocity (11.1 rad/s ≈ 106 RPM)', 'robot_config_freq (50 Hz)'], files: ['src/kinematics.cpp', 'include/kinematics.hpp', 'config/delta_config.yaml'], actions_taken: ['FK: 3-sphere intersection → end-effector xyz', 'IK: Geometric per-leg angle solving (YZ plane)', 'Jacobian: Jθ⁻¹ · Jp velocity mapping', 'Gradient: forward/central/backward differencing', 'Aux angles: auxiliary passive link angles'] },
    'M':      { title: 'motor_control_node.py', type: 'Python ROS2 Node', description: 'Hardware serial driver for real ST3215 servo motors via Waveshare ESP32 serial bridge. Supports both binary (0x7E SETN frames with XOR CRC) and text (SET/PING/SCAN) protocols. 3 bicep motors (IDs 1-3) for delta XYZ and 2 end-effector motors (IDs 4-5) for tilt/spin. 50 Hz feedback loop with read-fail watchdog. Auto-detects serial ports (/dev/ttyUSB*, /dev/ttyACM*).', topics_subscribed: ['delta_motors/set_joints (DeltaJoints)', 'delta_motors/set_joint_vels (DeltaJointVels)'], topics_published: ['delta_motors/motor_position_feedback (DeltaJoints)', 'delta_motors/motor_velocity_feedback (DeltaJointVels)', '/servo/target (Float32MultiArray)', '/servo/actual (Float32MultiArray)'], services_provided: ['delta_motors/set_joint_limits'], parameters: ['device_name (/dev/ttyUSB0)', 'baudrate (500000)', 'bicep moving_speed / moving_acc', 'ee_moving_speed / ee_moving_acc', 'max_write_retries (3)', 'use_binary_bridge (true)', 'stream_feedback_period_ms (20)', 'read_fail_watchdog_limit (5)'], files: ['delta_robot/motor_control_node.py'], actions_taken: ['Binary protocol: 0x7E SOF + SETN cmd + XOR CRC', 'Text protocol: SET / PING / SCAN / TORQUE / LIST', 'Motor position: 0–4095 ticks (12-bit, 2048=center)', 'Conversion: θ·(4096/2π) + 2048 = motor ticks', 'Auto-port detection: /dev/ttyUSB* + /dev/ttyACM*', 'Streamed feedback: FBP id=N pos=P speed=S'] },
    'E':      { title: 'ros2_control', type: 'Controller Manager', description: 'Executes trajectories on the Gazebo simulation hardware interface via the joint_trajectory_controller plugin. Manages 7 joints at 250 Hz update rate: 3 bicep (jbf1-3) with PID gains p=100, i=1, d=10, plus 4 end-effector joints (Bevelj1, Bevelj2, Tj1, BeveljEE).', topics_subscribed: ['/joint_trajectory_controller/joint_trajectory (JointTrajectory)'], parameters: ['update_rate: 250 Hz', 'joints: jbf1, jbf2, jbf3, Bevelj1, Bevelj2, Tj1, BeveljEE', 'command_interfaces: position', 'state_interfaces: position, velocity', 'PID gains (jbf1-3): p=100, i=1, d=10'], files: ['config/ros2_controllers.yaml'] },
    'H':      { title: 'Gazebo / Sim Physics', type: 'Simulation Environment', description: 'Ignition Gazebo Harmonic (gz-sim-8) calculates the physics, collisions, and dynamics of the Delta robot SDF model. Acts as the digital twin. Provides full 3D rendering, collision detection, and joint/link state publication.', topics_published: ['/clock', '/tf', '/tf_static', '/model/delta_robot/...'], files: ['worlds/empty.sdf', 'delta_robot_description/models/model.sdf', 'delta_robot_description/meshes/'], actions_taken: ['Physics simulation at real-time factor', 'SDF model with custom meshes', 'ros2_control hardware interface integration'] },
    'G':      { title: 'joint_state_broadcaster', type: 'ROS2 Controller', description: 'Reads simulated hardware state from ros2_control registers and publishes /joint_states continuously. Broadcasts position and velocity for all 7 joints.', topics_published: ['/joint_states (sensor_msgs/JointState)'], parameters: ['Broadcasts: position + velocity for all joints'] },
    'F':      { title: 'ros_gz_bridge', type: 'Bridge Node', description: 'Translates messages between Ignition/Gazebo transport and standard ROS2 topics. Configured via YAML for selective topic bridging with QoS overrides.', topics_bridged: ['/joint_states', '/clock (rosgz_interfaces/Clock)', '/tf (tf2_msgs/TFMessage)', '/tf_static (tf2_msgs/TFMessage)'], files: ['config/ros_gz_bridge.yaml'], parameters: ['config_file', 'qos: tf_static → transient_local durability'] },
    'I':      { title: 'joint_state_bridge.py', type: 'Python ROS2 Node', description: 'Subscribes to /joint_states, extracts bicep joints (jbf1-3), applies angle correction (raw < -0.5 → raw + π), publishes DeltaJoints + DeltaJointVels. Only active when use_sim_feedback=true. Bridges the gap between ros2_control joint naming and delta_motors topic convention.', topics_subscribed: ['/joint_states (sensor_msgs/JointState)'], topics_published: ['delta_motors/motor_position_feedback (DeltaJoints)', 'delta_motors/motor_velocity_feedback (DeltaJointVels)'], parameters: ['Joint names: jbf1, jbf2, jbf3', 'Correction: raw < -0.5 → raw + π'], files: ['delta_robot/joint_state_bridge.py'] },
    'J':      { title: 'RViz2', type: 'Visualization GUI', description: 'Standard ROS2 3D visualization tool. Renders the robot URDF/SDF model in real-time using /tf and /joint_states. Pre-configured with delta_robot.rviz layout.', topics_subscribed: ['/joint_states', '/tf', '/tf_static', '/clock'], files: ['config/delta_robot.rviz'] },
    'K':      { title: 'System Launch & Config', type: 'Launch / YAML Configs', description: 'Two main launch pipelines: delta_robot_spawn.launch.py (simulation — spawns Gazebo, RViz, controllers, 3D plotter) and delta_robot.launch.py (hardware — kinematics + motor control). Third: gcode_json_tools.launch.py (task runners).', files: ['delta_robot_sim/launch/delta_robot_spawn.launch.py', 'delta_robot/launch/delta_robot.launch.py', 'delta_robot/launch/gcode_json_tools.launch.py', 'config/ros2_controllers.yaml', 'config/delta_config.yaml', 'config/ros_gz_bridge.yaml'], actions_taken: ['Sim pipeline: Gazebo + RViz + controllers + bridge + plotter', 'HW pipeline: kinematics + motion_planner + motor_control', 'Task pipeline: json_sequencer or gcode_parser'] },
    'P1':     { title: 'see_motors.py (Plotter)', type: 'Python Visualization Tool', description: 'Live matplotlib plotter for real servo feedback. Displays target vs actual motor positions in real-time. Computes cross-correlation lag, RMSE, peak error, and estimated max RPM for motor performance analysis.', topics_subscribed: ['/servo/target (Float32MultiArray)', '/servo/actual (Float32MultiArray)'], files: ['delta_robot/see_motors.py'], actions_taken: ['Real-time target vs actual position plots', 'Cross-correlation lag computation', 'RMSE and peak error stats', 'Estimated max RPM calculation'] },
    'P2':     { title: 'plotter3d.py', type: 'Python ROS2 Node', description: '3D end-effector position plotter. Subscribes to robot_config at 50 Hz and renders the end-effector path in 3D using matplotlib. Useful for visualising trajectory quality and workspace utilisation.', topics_subscribed: ['delta_robot/robot_config (RobotConfig)'], files: ['delta_robot_sim/scripts/plotter3d.py'], actions_taken: ['3D scatter/line plot of EE path', 'Real-time matplotlib animation', 'Workspace boundary visualization'] },
    'IFACES': { title: 'deltarobot_interfaces', type: 'ROS2 Interface Package', description: 'Custom message and service definitions shared across all packages. 3 messages for joint state communication and 10 services covering kinematics, motion planning, and hardware configuration.', files: ['msg/DeltaJoints.msg — θ1, θ2, θ3 [rad]', 'msg/DeltaJointVels.msg — θ̇1, θ̇2, θ̇3 [rad/s]', 'msg/RobotConfig.msg — angles + vels + EE position', 'srv/DeltaFK.srv — joint angles → EE xyz', 'srv/DeltaIK.srv — EE xyz → joint angles', 'srv/ConvertToJointTrajectory.srv', 'srv/ConvertToJointVelTrajectory.srv', 'srv/MoveToPoint.srv', 'srv/MoveToConfiguration.srv', 'srv/MotionDemo.srv', 'srv/PlayDemoTrajectory.srv', 'srv/PlayCustomTrajectory.srv — Point[] + step_ms', 'srv/SetJointLimits.srv — min/max rad + max vel'], actions_taken: ['3 custom message types', '10 custom service definitions', 'Shared compile-time contracts across all packages'] }
  },
  edges: {
    'e1':     { title: 'JSON Tasks & Config', type: 'Service Calls', description: 'User triggers task execution via JSON files and provides static JSON config parameters.' },
    'e1_b2':  { title: 'G-code & Config',     type: 'Service Calls', description: 'User triggers execution of paths defined in G-code and config parameters.' },
    'e12':    { title: 'Launch Execution',    type: 'Process', description: 'User launches the system via ros2 launch files that orchestrate all node startup.' },
    'e3':     { title: 'Motion Services',     type: 'Service Calls', description: 'Sequencer calls move_to_point or play_custom_trajectory on the Motion Planner.' },
    'e3b':    { title: 'G-code → Planner',    type: 'Service Calls', description: 'G-code parser converts G0/G1 moves into service calls to the Motion Planner.' },
    'e4':     { title: 'Math & Kinematics',   type: 'Service Calls', description: 'Motion Planner delegates IK/FK computation and trajectory conversion to Kinematics.' },
    'e5b':    { title: 'Traj → Hardware',     type: 'Topic', description: 'Motor commands published as DeltaJoints to the hardware motor control node.' },
    'e5':     { title: 'Traj → Gazebo',       type: 'Topic', description: 'JointTrajectory messages published to the simulation joint_trajectory_controller.' },
    'e6':     { title: 'Hardware State',      type: 'Internal Control', description: 'ros2_control reads simulated joint state from Gazebo hardware interface.' },
    'e7':     { title: '/joint_states',       type: 'Topic', description: 'Joint state broadcaster publishes position and velocity for all 7 joints.' },
    'e8':     { title: 'Sim Feedback',        type: 'Topics', description: 'Joint State Bridge converts /joint_states to DeltaJoints feedback for kinematics.' },
    'e8b':    { title: 'HW Feedback',         type: 'Serial + Topics', description: 'Motor control reads serial position feedback and publishes DeltaJoints to kinematics.' },
    'e9':     { title: 'Sim Transport',       type: 'Topics', description: 'Gazebo publishes physics state via Ignition transport topics.' },
    'e10':    { title: 'Sim → ROS2 Bridge',   type: 'Bridge', description: 'ros_gz_bridge translates Ignition transport to ROS2 topics for RViz and other nodes.' },
    'e11':    { title: 'Viz Data',            type: 'Topics', description: 'Bridge forwards /tf and /joint_states to the Joint State Bridge for processing.' },
    'e13':    { title: 'Config Distribution', type: 'Parameters', description: 'Launch files distribute delta_config.yaml parameters to kinematics and other nodes.' },
    'e14':    { title: 'Servo Plotter',       type: 'Topics', description: 'Motor control publishes /servo/target and /servo/actual for the servo performance plotter.' },
    'e15':    { title: 'EE Plotter',          type: 'Topic', description: 'Kinematics publishes RobotConfig at 50Hz for the 3D end-effector position plotter.' },
    'e16':    { title: 'Interface Contracts', type: 'Compile-time', description: 'All packages depend on deltarobot_interfaces for shared message/service type definitions.' },
    'e17':    { title: 'Joint Limits',        type: 'Service Call', description: 'Kinematics calls set_joint_limits on motor control to configure hardware angle bounds.' }
  }
};

const NODE_W = 160;
const NODE_H = 56;

// ═══════════════════════════════════════════════════════════════
//  DETERMINISTIC LAYOUT (Tightened Horizontal Span)
// ═══════════════════════════════════════════════════════════════
// Col centers: 150, 430, 710, 990, 1270
const nodesLayout = {
  'A':      { x: 460,  y: 80,   color: '#059669', label: 'User / GUI', typeText: 'Interface' },
  'K':      { x: 250,  y: 240,  color: '#475569', label: 'Launch & Config', typeText: 'System Files' },

  'B2':     { x: 460,  y: 240,  color: '#2563eb', label: 'G-code Parser', typeText: 'Python Node' },
  'B':      { x: 670,  y: 240,  color: '#2563eb', label: 'Task Sequencer', typeText: 'Python Node' },

  'IFACES': { x: 780, y: 440,  color: '#6366f1', label: 'Interfaces Pkg', typeText: 'Msgs / Srvs' },
  'C':      { x: 460,  y: 440,  color: '#4f46e5', label: 'Motion Planner', typeText: 'C++ Core' },

  'M':      { x: 160,  y: 600,  color: '#b45309', label: 'Motor Control', typeText: 'Serial Driver' },
  'D':      { x: 460,  y: 600,  color: '#4f46e5', label: 'Kinematics', typeText: 'C++ Math' },
  'E':      { x: 780,  y: 600,  color: '#7c3aed', label: 'ROS2 Control', typeText: 'Controller' },
  'H':      { x: 1000, y: 600,  color: '#d97706', label: 'Gazebo Physics', typeText: 'Simulation' },

  'G':      { x: 780,  y: 780,  color: '#7c3aed', label: 'JS Broadcaster', typeText: 'Broadcaster' },
  'F':      { x: 1000, y: 780,  color: '#0891b2', label: 'ROS GZ Bridge', typeText: 'Middleware' },

  'P1':     { x: 160,  y: 980,  color: '#db2777', label: 'Servo Plotter', typeText: 'Matplotlib' },
  'P2':     { x: 460,  y: 980,  color: '#db2777', label: '3D EE Plotter', typeText: 'Matplotlib' },
  'I':      { x: 780,  y: 980,  color: '#2563eb', label: 'JS Bridge (Py)', typeText: 'Sim Filter' },
  'J':      { x: 1000, y: 980,  color: '#db2777', label: 'RViz2', typeText: 'Visualization' },

};

// ═══════════════════════════════════════════════════════════════
//  DOMAIN GROUPS (Bounding Boxes)
// ═══════════════════════════════════════════════════════════════
const domainGroups = [
  { id: 'g_task', title: 'Task & Configuration Layer', color: '#0ea5e9', nodes: ['A', 'K', 'B2', 'B'] },
  { id: 'g_core', title: 'Motion & Kinematics Engine', color: '#6366f1', nodes: ['C', 'D'] },
  { id: 'g_hw', title: 'Physical Hardware Interface', color: '#f59e0b', nodes: ['M', 'P1'] },
  { id: 'g_sim', title: 'Gazebo Digital Twin', color: '#8b5cf6', nodes: ['E', 'H', 'G', 'F'] },
  { id: 'g_viz', title: 'Telemetry & Visualization', color: '#ec4899', nodes: ['P2', 'I', 'J'] },
  { id: 'g_iface', title: 'ROS 2 Contracts', color: '#14b8a6', nodes: ['IFACES'] }
];

const edgesRaw = [
  // Top layer carefully offsets lines from node 'A' to prevent crossing text overlaps
  { id: 'e1_b2', from: 'A', fs: 'bottom', fo: 0, to: 'B2', ts: 'top', toOff: 0, label: 'G-code & Config', t: 0.5 },
  { id: 'e1',    from: 'A', fs: 'bottom', fo: 20, to: 'B', ts: 'top', toOff: 0, label: 'JSON Tasks & Config', t: 0.5 },
  { id: 'e12',   from: 'A', fs: 'bottom', fo: -20, to: 'K', ts: 'top', toOff: 0, label: 'Launches', t: 0.5 },
  
  { id: 'e3b', from: 'B2', fs: 'bottom', fo: 0, to: 'C', ts: 'top', toOff: 0, label: 'G-code → Plan', t: 0.35 },
  { id: 'e3',  from: 'B', fs: 'bottom', fo: 0, to: 'C', ts: 'top', toOff: 30, label: 'Motion Srv', t: 0.375 },
  { id: 'e16', from: 'IFACES', fs: 'left', fo: 0, to: 'C', ts: 'right', toOff: 0, label: 'Msg/Srv Types' },
  { id: 'e13', from: 'K', fs: 'bottom', fo: 0, to: 'D', ts: 'top', toOff: -60, label: 'Config', t: 0.215 },

  { id: 'e4',  from: 'C', fs: 'bottom', fo: 0, to: 'D', ts: 'top', toOff: 0, label: 'IK/FK Math' },
  { id: 'e5b', from: 'C', fs: 'bottom', fo: -30, to: 'M', ts: 'top', toOff: 0, label: 'Traj → Motors', t: 0.465 },
  { id: 'e5',  from: 'C', fs: 'bottom', fo: 30, to: 'E', ts: 'top', toOff: 0, label: 'Traj → Gazebo', t: 0.465 },

  { id: 'e6',  from: 'E', fs: 'bottom', fo: 0, to: 'G', ts: 'top', toOff: 0, label: 'HW State' },
  { id: 'e7',  from: 'G', fs: 'bottom', fo: 0, to: 'I', ts: 'top', toOff: -20, label: '/joint_states', t: 0.35 },
  { id: 'e9',  from: 'H', fs: 'bottom', fo: 0, to: 'F', ts: 'top', toOff: 0, label: 'Sim Topics' },
  { id: 'e10', from: 'F', fs: 'bottom', fo: 20, to: 'J', ts: 'top', toOff: 0, label: 'Bridge → RViz', t: 0.365 },
  { id: 'e11', from: 'F', fs: 'bottom', fo: -20, to: 'I', ts: 'top', toOff: 20, label: 'Viz Data', t: 0.36 },

  { id: 'e14', from: 'M', fs: 'bottom', fo: 0, to: 'P1', ts: 'top', toOff: 0, label: 'Servo Data' },
  { id: 'e15', from: 'D', fs: 'bottom', fo: 0, to: 'P2', ts: 'top', toOff: 0, label: 'EE Config' },

  { id: 'e8',  from: 'I', fs: 'left', fo: 0, to: 'D', ts: 'right', toOff: 0, label: 'Sim Feedback', t: 0.4 },
  { id: 'e8b', from: 'M', fs: 'right', fo: 15, to: 'D', ts: 'left', toOff: 15, label: 'HW Feedback' },
  { id: 'e17', from: 'D', fs: 'left', fo: -15, to: 'M', ts: 'right', toOff: -15, label: 'Joint Limits' }
];

function generateEdges() {
  return edgesRaw.map(e => {
    const p1 = nodesLayout[e.from];
    const p2 = nodesLayout[e.to];
    const hw = NODE_W / 2;
    const hh = NODE_H / 2;

    let x1 = p1.x + (e.fs === 'left' ? -hw : e.fs === 'right' ? hw : e.fo);
    let y1 = p1.y + (e.fs === 'top' ? -hh : e.fs === 'bottom' ? hh : e.fo);

    let x2 = p2.x + (e.ts === 'left' ? -hw : e.ts === 'right' ? hw : e.toOff);
    let y2 = p2.y + (e.ts === 'top' ? -hh : e.ts === 'bottom' ? hh : e.toOff);

    let c1x = x1, c1y = y1;
    let c2x = x2, c2y = y2;
    const CV = 80;

    if (e.fs === 'top') c1y -= CV;
    if (e.fs === 'bottom') c1y += CV;
    if (e.fs === 'left') c1x -= CV;
    if (e.fs === 'right') c1x += CV;

    if (e.ts === 'top') c2y -= CV;
    if (e.ts === 'bottom') c2y += CV;
    if (e.ts === 'left') c2x -= CV;
    if (e.ts === 'right') c2x += CV;

    const t = e.t || 0.5;
    const mt = 1 - t;
    const lx = mt * mt * mt * x1 + 3 * mt * mt * t * c1x + 3 * mt * t * t * c2x + t * t * t * x2;
    const ly = mt * mt * mt * y1 + 3 * mt * mt * t * c1y + 3 * mt * t * t * c2y + t * t * t * y2;

    return { ...e, path: `M ${x1} ${y1} C ${c1x} ${c1y}, ${c2x} ${c2y}, ${x2} ${y2}`, lx, ly };
  });
}

const edgesData = generateEdges();

// ═══════════════════════════════════════════════════════════════
//  FLYOUT PANEL
// ═══════════════════════════════════════════════════════════════
const DetailBadge = ({ items, icon, colorClass }) => {
  if (!items || items.length === 0) return null;
  return (
    <div className="flex flex-wrap gap-1.5 mt-2">
      {items.map((item, idx) => (
        <span key={idx} className={`px-2 py-1 bg-slate-800 text-xs border rounded-md shadow-sm flex items-center gap-1 ${colorClass}`}>
          {icon} {item}
        </span>
      ))}
    </div>
  );
};

const DetailSection = ({ title, items, icon, colorClass }) => {
  if (!items || items.length === 0) return null;
  return (
    <div className="mt-4">
      <h4 className="text-xs font-bold uppercase tracking-wider text-slate-400 mb-2">{title}</h4>
      <DetailBadge items={items} icon={icon} colorClass={colorClass} />
    </div>
  );
};

function FlyoutPanel({ selectedId, onClose }) {
  if (!selectedId) return null;
  const item = systemData.nodes[selectedId] || systemData.edges[selectedId];
  if (!item) return null;
  const isNode = !!systemData.nodes[selectedId];

  return (
    <div className="absolute top-3 bottom-3 right-3 w-[400px] max-w-[88vw] bg-slate-900/95 backdrop-blur-xl rounded-2xl border border-slate-700/60 shadow-2xl z-30 flex flex-col overflow-hidden"
         style={{ animation: 'flyIn 0.25s ease-out' }}>
      <div className="flex items-start justify-between p-5 pb-2 border-b border-slate-800/50">
        <div>
          <div className="flex items-center gap-2 text-[10px] font-black uppercase tracking-widest text-slate-400 mb-1">
            {isNode ? <Box size={12} className="text-blue-400"/> : <ArrowRightLeft size={12} className="text-emerald-400"/>}
            {item.type}
          </div>
          <h2 className="text-lg font-black text-white leading-tight">{item.title}</h2>
        </div>
        <button onClick={onClose} className="shrink-0 ml-2 bg-slate-800 hover:bg-slate-700 text-white p-1.5 rounded-lg border border-slate-600"><X size={14} /></button>
      </div>
      <div className="flex-1 overflow-y-auto p-5 pt-3 custom-scrollbar">
        <div className="text-sm text-slate-300 leading-relaxed mb-5 bg-slate-800/30 p-4 rounded-xl border border-slate-700/50 italic">
          {item.description || item.desc}
        </div>
        {isNode && (
          <div>
            <DetailSection title="Files" items={item.files} icon={<FileText size={12}/>} colorClass="text-emerald-300 border-emerald-900/50" />
            <DetailSection title="Parameters" items={item.parameters} icon={<Sliders size={12}/>} colorClass="text-amber-300 border-amber-900/50" />
            <DetailSection title="Actions & Threads" items={[...(item.actions||[]),...(item.threads||[]),...(item.actions_taken||[])]} icon={<Cpu size={12}/>} colorClass="text-rose-300 border-rose-900/50" />
            <div className="pt-2 mt-2 border-t border-slate-800/50">
              <DetailSection title="Subscribed Topics" items={item.topics_subscribed} icon={<Database size={12}/>} colorClass="text-sky-300 border-sky-900/50" />
              <DetailSection title="Published Topics" items={item.topics_published} icon={<Database size={12}/>} colorClass="text-sky-300 border-sky-900/50" />
              <DetailSection title="Topics Bridged" items={item.topics_bridged} icon={<Database size={12}/>} colorClass="text-sky-300 border-sky-900/50" />
              <DetailSection title="Services Provided" items={item.services_provided} icon={<Settings size={12}/>} colorClass="text-purple-300 border-purple-900/50" />
              <DetailSection title="Services Called" items={item.services_called} icon={<Settings size={12}/>} colorClass="text-purple-300 border-purple-900/50" />
            </div>
          </div>
        )}
      </div>
    </div>
  );
}

// ═══════════════════════════════════════════════════════════════
//  MAIN APP COMPONENT
// ═══════════════════════════════════════════════════════════════
export default function App() {
  const [selectedId, setSelectedId] = useState(null);
  const [showHwSpecs, setShowHwSpecs] = useState(false);
  const [showLegend, setShowLegend] = useState(false);
  const containerRef = useRef(null);
  const svgRef = useRef(null);
  
  // Perfectly calibrated default viewbox to fit the new tighter layout
  const [viewBox, setViewBox] = useState({ x: -50, y: -20, w: 1550, h: 1100 });
  const [isPanning, setIsPanning] = useState(false);
  const panStart = useRef({ x: 0, y: 0, vbx: 0, vby: 0 });

  // Pan & Zoom Logic
  const handleWheel = useCallback((e) => {
    e.preventDefault();
    const f = e.deltaY > 0 ? 1.1 : 1 / 1.1;
    const svg = svgRef.current; if (!svg) return;
    const r = svg.getBoundingClientRect();
    const ptX = viewBox.x + (e.clientX - r.left) / r.width * viewBox.w;
    const ptY = viewBox.y + (e.clientY - r.top) / r.height * viewBox.h;

    setViewBox(vb => {
      const nw = vb.w * f, nh = vb.h * f;
      if (nw < 300 || nw > 6000) return vb;
      return { x: ptX - (ptX - vb.x) * f, y: ptY - (ptY - vb.y) * f, w: nw, h: nh };
    });
  }, [viewBox]);

  const handlePointerDown = useCallback((e) => {
    if (e.target.tagName === 'svg' || e.target.tagName === 'rect' || e.target.tagName === 'path' && e.target.getAttribute('fill') === 'none') {
      setIsPanning(true);
      panStart.current = { x: e.clientX, y: e.clientY, vbx: viewBox.x, vby: viewBox.y };
      setSelectedId(null);
    }
  }, [viewBox]);

  const handlePointerMove = useCallback((e) => {
    if (!isPanning) return;
    const svg = svgRef.current; if (!svg) return;
    const r = svg.getBoundingClientRect();
    const dx = (e.clientX - panStart.current.x) / r.width * viewBox.w;
    const dy = (e.clientY - panStart.current.y) / r.height * viewBox.h;
    setViewBox(vb => ({ ...vb, x: panStart.current.vbx - dx, y: panStart.current.vby - dy }));
  }, [isPanning, viewBox.w, viewBox.h]);

  const handlePointerUp = useCallback(() => setIsPanning(false), []);

  useEffect(() => {
    const el = containerRef.current; if (!el) return;
    el.addEventListener('wheel', handleWheel, { passive: false });
    return () => el.removeEventListener('wheel', handleWheel);
  }, [handleWheel]);

  // Highlighting Logic
  const activeNodes = useMemo(() => {
    if (!selectedId) return null;
    if (nodesLayout[selectedId]) return [selectedId];
    const edge = edgesData.find(e => e.id === selectedId);
    return edge ? [edge.from, edge.to] : [];
  }, [selectedId]);

  const activeEdges = useMemo(() => {
    if (!selectedId) return null;
    if (systemData.edges[selectedId]) return [selectedId];
    return edgesData.filter(e => e.from === selectedId || e.to === selectedId).map(e => e.id);
  }, [selectedId]);

  return (
    <div className="h-screen w-full bg-slate-950 font-sans text-slate-200 overflow-hidden relative">
      <div 
        ref={containerRef} 
        className="absolute inset-0 bg-slate-900 select-none"
        onPointerDown={handlePointerDown} 
        onPointerMove={handlePointerMove}
        onPointerUp={handlePointerUp} 
        onPointerLeave={handlePointerUp}
        style={{ cursor: isPanning ? 'grabbing' : 'grab' }}
      >
        <div className="absolute inset-0 pointer-events-none opacity-15" style={{ backgroundImage: 'radial-gradient(circle, #475569 1px, transparent 1px)', backgroundSize: '28px 28px' }} />

        <svg ref={svgRef} viewBox={`${viewBox.x} ${viewBox.y} ${viewBox.w} ${viewBox.h}`} className="w-full h-full absolute inset-0" preserveAspectRatio="xMidYMid meet">
          <defs>
            <marker id="arrow-dim" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto-start-reverse">
              <polygon points="0 0, 6 3, 0 6" fill="#475569" />
            </marker>
            <marker id="arrow-act" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto-start-reverse">
              <polygon points="0 0, 6 3, 0 6" fill="#94a3b8" />
            </marker>
            <marker id="arrow-sel" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto-start-reverse">
              <polygon points="0 0, 6 3, 0 6" fill="#e2e8f0" />
            </marker>
            <style>{`
              @keyframes fl{from{stroke-dashoffset:20}to{stroke-dashoffset:0}}
              .fl-s{stroke-dasharray:5 5;animation:fl .6s linear infinite}
            `}</style>
          </defs>

          <rect x={viewBox.x} y={viewBox.y} width={viewBox.w} height={viewBox.h} fill="transparent" />

          {/* DOMAIN GROUPS (Bounding Boxes) */}
          {domainGroups.map(g => {
            const xs = g.nodes.map(n => nodesLayout[n].x);
            const ys = g.nodes.map(n => nodesLayout[n].y);
            
            // Adjusted padding: -60 top padding guarantees a beautiful gap below the title text
            const minX = Math.min(...xs) - NODE_W / 2 - 25;
            const maxX = Math.max(...xs) + NODE_W / 2 + 25;
            const minY = Math.min(...ys) - NODE_H / 2 - 60; 
            const maxY = Math.max(...ys) + NODE_H / 2 + 25;
            
            const w = maxX - minX;
            const h = maxY - minY;

            // Box stays active if NO node is selected, OR if a node inside it is selected
            const isAct = activeNodes === null || activeNodes.some(n => g.nodes.includes(n));

            return (
              <g key={g.id} opacity={isAct ? 1 : 0.15} className="transition-opacity duration-300 pointer-events-none">
                <rect x={minX} y={minY} width={w} height={h} rx={20} fill={`${g.color}0a`} stroke={g.color} strokeWidth={1.5} strokeDasharray="8 8" />
                <foreignObject x={minX + 15} y={minY + 15} width={w - 30} height={30}>
                  <div className="flex items-start gap-1.5 font-bold uppercase tracking-widest text-[10px] leading-tight" style={{ color: g.color }}>
                    <Layers size={14} className="shrink-0 mt-[1px]" /> 
                    <span>{g.title}</span>
                  </div>
                </foreignObject>
              </g>
            );
          })}

          {/* EDGES */}
          {edgesData.map(e => {
            const isSel = selectedId === e.id;
            const isAct = activeEdges?.includes(e.id);
            const opacity = (activeEdges === null || isAct) ? 1 : 0.1;
            const color = isSel ? '#e2e8f0' : (isAct ? '#94a3b8' : '#475569');
            const mk = isSel ? 'url(#arrow-sel)' : (isAct ? 'url(#arrow-act)' : 'url(#arrow-dim)');

            return (
              <g key={e.id} opacity={opacity} onClick={(ev) => { ev.stopPropagation(); setSelectedId(e.id); }} className="cursor-pointer transition-opacity duration-300">
                <path d={e.path} fill="none" stroke="transparent" strokeWidth={20} />
                <path d={e.path} fill="none" stroke={color} strokeWidth={isSel ? 2.5 : 1.5} markerEnd={mk} className={isSel ? "fl-s" : ""} strokeDasharray={e.custom ? "4 4" : "none"} />
                <foreignObject x={e.lx - 60} y={e.ly - 10} width="120" height="20" className="pointer-events-none overflow-visible">
                  <div className="flex items-center justify-center w-full h-full">
                    <span className={`px-1.5 py-0.5 text-[8px] font-bold tracking-wide rounded-full whitespace-nowrap
                      ${isSel ? 'bg-white text-slate-900 shadow-[0_0_8px_rgba(255,255,255,0.5)]' : 'bg-slate-950/80 border border-slate-700/60 text-slate-400'}`}>
                      {e.label}
                    </span>
                  </div>
                </foreignObject>
              </g>
            );
          })}

          {/* NODES */}
          {Object.entries(nodesLayout).map(([id, n]) => {
            const isSel = selectedId === id;
            const isAct = activeNodes === null || activeNodes.includes(id);
            
            return (
              <g key={id} opacity={isAct ? 1 : 0.2} onClick={(ev) => { ev.stopPropagation(); setSelectedId(id); }} className="cursor-pointer transition-opacity duration-300">
                <foreignObject x={n.x - NODE_W / 2} y={n.y - NODE_H / 2} width={NODE_W} height={NODE_H} className="overflow-visible pointer-events-none">
                  <div className={`w-full h-full rounded-xl flex flex-col items-center justify-center p-2 text-center shadow-lg transition-all duration-200 pointer-events-auto
                    ${isSel ? 'ring-2 ring-white shadow-[0_0_20px_rgba(255,255,255,0.35)] scale-110' : 'hover:ring-2 hover:ring-slate-400 hover:scale-105'}
                  `} style={{ backgroundColor: n.color, border: '1px solid rgba(255,255,255,0.18)' }}>
                    <div className="font-bold text-white text-[11px] leading-tight px-1 drop-shadow-md">{n.label}</div>
                    <div className="text-[8px] text-white/60 mt-0.5 uppercase tracking-widest font-bold">{n.typeText}</div>
                  </div>
                </foreignObject>
              </g>
            );
          })}
        </svg>
      </div>

      {/* Controls Overlay */}
      <div className="absolute top-3 left-3 z-20 flex flex-col gap-1.5">
        <button onClick={() => setViewBox(vb => ({ ...vb, x: vb.x + vb.w*0.05, y: vb.y + vb.h*0.05, w: vb.w/1.1, h: vb.h/1.1 }))}
          className="bg-slate-800/90 hover:bg-slate-700 text-white w-8 h-8 rounded-lg flex items-center justify-center text-sm font-bold border border-slate-600 shadow-lg">+</button>
        <button onClick={() => setViewBox(vb => ({ ...vb, x: vb.x - vb.w*0.05, y: vb.y - vb.h*0.05, w: vb.w*1.1, h: vb.h*1.1 }))}
          className="bg-slate-800/90 hover:bg-slate-700 text-white w-8 h-8 rounded-lg flex items-center justify-center text-sm font-bold border border-slate-600 shadow-lg">−</button>
        <button onClick={() => setViewBox({ x: -50, y: -20, w: 1550, h: 1100 })}
          className="bg-slate-800/90 hover:bg-slate-700 text-white w-8 h-8 rounded-lg flex items-center justify-center text-[9px] font-bold border border-slate-600 shadow-lg" title="Reset">⟲</button>
      </div>

      {!selectedId && (
        <div className="absolute top-3 left-1/2 -translate-x-1/2 z-20 bg-slate-900/90 backdrop-blur-xl px-5 py-2.5 rounded-xl border border-slate-700/50 shadow-xl pointer-events-none">
          <h1 className="text-base font-black text-white flex items-center gap-2 tracking-tight">
            <Activity className="text-emerald-500" size={20} /> Delta Robot Architecture
          </h1>
          <p className="text-[10px] text-slate-400 mt-0.5 text-center">{Object.keys(nodesLayout).length} nodes · {edgesRaw.length} edges · Click to inspect</p>
        </div>
      )}

      {selectedId && (
        <button onClick={() => setSelectedId(null)}
          className="absolute top-3 left-1/2 -translate-x-1/2 z-20 bg-slate-800/90 hover:bg-slate-700 text-white px-4 py-2 rounded-full backdrop-blur-md shadow-xl flex items-center gap-2 text-xs font-bold border border-slate-600">
          <X size={14} /> Clear Selection
        </button>
      )}

      <FlyoutPanel selectedId={selectedId} onClose={() => setSelectedId(null)} />

      {/* Hardware Specs Panel - Bottom Left */}
      <div className="absolute bottom-3 left-3 z-20" style={{ animation: 'flyIn 0.3s ease-out' }}>
        <button onClick={() => setShowHwSpecs(v => !v)}
          className="bg-slate-900/90 backdrop-blur-xl text-white px-3 py-2 rounded-xl border border-slate-700/50 shadow-xl flex items-center gap-2 text-xs font-bold hover:bg-slate-800/90 transition-colors">
          <Info size={14} className="text-amber-400" />
          Hardware Specs
          {showHwSpecs ? <ChevronDown size={12} /> : <ChevronUp size={12} />}
        </button>
        {showHwSpecs && (
          <div className="mt-1.5 bg-slate-900/95 backdrop-blur-xl rounded-xl border border-slate-700/50 shadow-2xl p-4 w-[280px]" style={{ animation: 'flyIn 0.2s ease-out' }}>
            <h3 className="text-[10px] font-black uppercase tracking-widest text-slate-400 mb-3">Robot Specifications</h3>
            <div className="space-y-2.5 text-[11px]">
              <div className="flex justify-between">
                <span className="text-slate-400">Servos</span>
                <span className="text-amber-300 font-semibold">ST3215 × 5</span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-400">Protocol</span>
                <span className="text-amber-300 font-semibold">STS Bus · 500 kbaud</span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-400">Driver</span>
                <span className="text-amber-300 font-semibold">ESP32 (USB Serial)</span>
              </div>
              <div className="h-px bg-slate-700/50 my-1" />
              <div className="flex justify-between">
                <span className="text-slate-400">Base ▲</span>
                <span className="text-blue-300 font-semibold">360.27 mm</span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-400">End-Effector ▲</span>
                <span className="text-blue-300 font-semibold">138.56 mm</span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-400">Active Link</span>
                <span className="text-blue-300 font-semibold">105.0 mm</span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-400">Passive Link</span>
                <span className="text-blue-300 font-semibold">218.75 mm</span>
              </div>
              <div className="h-px bg-slate-700/50 my-1" />
              <div className="flex justify-between">
                <span className="text-slate-400">Joint Range</span>
                <span className="text-emerald-300 font-semibold">-10° to 90°</span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-400">Max Velocity</span>
                <span className="text-emerald-300 font-semibold">11.1 rad/s (106 RPM)</span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-400">Platform</span>
                <span className="text-emerald-300 font-semibold">ROS 2 Jazzy</span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-400">Simulation</span>
                <span className="text-emerald-300 font-semibold">Gazebo Harmonic</span>
              </div>
            </div>
          </div>
        )}
      </div>

      {/* Quick Stats Bar - Bottom Center */}
      <div className="absolute bottom-3 left-1/2 -translate-x-1/2 z-20 bg-slate-900/90 backdrop-blur-xl rounded-xl border border-slate-700/50 shadow-xl px-1 py-1.5 flex gap-1 pointer-events-none">
        {[
          { label: 'Packages', value: '4', color: 'text-blue-400' },
          { label: 'Messages', value: '3', color: 'text-emerald-400' },
          { label: 'Services', value: '10', color: 'text-purple-400' },
          { label: 'Joints', value: '7', color: 'text-amber-400' },
          { label: 'Demos', value: '5', color: 'text-pink-400' },
        ].map(s => (
          <div key={s.label} className="flex flex-col items-center px-2.5 py-1">
            <span className={`text-sm font-black ${s.color}`}>{s.value}</span>
            <span className="text-[8px] text-slate-400 uppercase tracking-wider font-bold mt-0.5">{s.label}</span>
          </div>
        ))}
      </div>

      {/* Legend Panel - Bottom Right */}
      <div className="absolute bottom-3 right-3 z-20">
        <button onClick={() => setShowLegend(v => !v)}
          className="bg-slate-900/90 backdrop-blur-xl text-white px-3 py-2 rounded-xl border border-slate-700/50 shadow-xl flex items-center gap-2 text-xs font-bold hover:bg-slate-800/90 transition-colors">
          <Palette size={14} className="text-purple-400" />
          Legend
          {showLegend ? <ChevronDown size={12} /> : <ChevronUp size={12} />}
        </button>
        {showLegend && (
          <div className="mt-1.5 bg-slate-900/95 backdrop-blur-xl rounded-xl border border-slate-700/50 shadow-2xl p-4 w-[200px]" style={{ animation: 'flyIn 0.2s ease-out' }}>
            <h3 className="text-[10px] font-black uppercase tracking-widest text-slate-400 mb-3">Node Types</h3>
            <div className="space-y-2">
              {[
                { color: '#2563eb', label: 'Python ROS2 Node' },
                { color: '#4f46e5', label: 'C++ ROS2 Node' },
                { color: '#7c3aed', label: 'Controller' },
                { color: '#b45309', label: 'Hardware Driver' },
                { color: '#d97706', label: 'Simulation' },
                { color: '#db2777', label: 'Visualization' },
                { color: '#0891b2', label: 'Bridge / Middleware' },
                { color: '#059669', label: 'External Interface' },
                { color: '#0f766e', label: 'Configuration' },
                { color: '#475569', label: 'System / Launch' },
                { color: '#6366f1', label: 'Interface Package' },
              ].map(l => (
                <div key={l.label} className="flex items-center gap-2">
                  <div className="w-3 h-3 rounded-sm shrink-0" style={{ backgroundColor: l.color }} />
                  <span className="text-[10px] text-slate-300 font-medium">{l.label}</span>
                </div>
              ))}
            </div>
          </div>
        )}
      </div>

      <style dangerouslySetInnerHTML={{__html: `
        @keyframes flyIn{from{opacity:0;transform:translateY(10px)}to{opacity:1;transform:translateY(0)}}
        .custom-scrollbar::-webkit-scrollbar{width:5px}
        .custom-scrollbar::-webkit-scrollbar-track{background:transparent}
        .custom-scrollbar::-webkit-scrollbar-thumb{background:#334155;border-radius:10px}
        .custom-scrollbar::-webkit-scrollbar-thumb:hover{background:#475569}
      `}} />
    </div>
  );
}