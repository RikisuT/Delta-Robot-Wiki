import React, { useState } from 'react';
import { Activity, ArrowRightLeft, Box, Settings, Play, Info, X, FileText, Sliders, Zap, Cpu, Database } from 'lucide-react';

// --- Ultra-Detailed System Parsed Data ---
const systemData = {
  nodes: {
    'A': {
      title: 'User / Script / GUI',
      type: 'External Interface',
      description: 'The entry point for triggering robot actions. Edits task definitions and launches the core system via launch files.',
      actions_taken: ['Edits example_task.json', 'Launches delta_robot_spawn.launch.py']
    },
    'B_CONF': {
      title: 'Task & Parameters',
      type: 'Configuration',
      description: 'Contains the JSON sequence definition and node-specific parameters for the sequencer.',
      files: ['example_task.json'],
      parameters: ['task_path', 'wait_time', 'execution_mode']
    },
    'B': {
      title: 'json_task_sequencer.py',
      type: 'Python ROS2 Node',
      description: 'Reads high-level task descriptions formatted in JSON and acts as the state machine. Calls the motion planner iteratively.',
      services_called: ['/PlayDemoTrajectory', '/PlayCustomTrajectory', '/MoveToPoint', '/MoveToConfiguration']
    },
    'C': {
      title: 'motion_planner',
      type: 'C++ ROS2 Node',
      description: 'The core trajectory generation engine. Handles action logic, threading, and coordinates closely with kinematics and hardware controllers.',
      topics_published: ['/DeltaJoints', '/DeltaJointVels', '/joint_trajectory_controller/joint_trajectory'],
      services_provided: ['/PlayDemoTrajectory', '/PlayCustomTrajectory', '/MoveToPoint', '/MoveToConfiguration'],
      services_called: ['/ConvertToJointTrajectory', '/ConvertToJointVelTrajectory', '/DeltaIK', '/DeltaFK'],
      parameters: ['traj_step_ms', 'joint_limits', 'velocity_scaling'],
      actions: ['CancelTrajectory (Action Server)'],
      threads: ['TrajectoryExecutor (Execution Thread)']
    },
    'D': {
      title: 'delta_kinematics',
      type: 'C++ ROS2 Node',
      description: 'Provides complex mathematical transformations specific to the parallel kinematics of a Delta Robot. Pure math, no state.',
      services_provided: ['/ConvertToJointTrajectory', '/ConvertToJointVelTrajectory', '/DeltaIK', '/DeltaFK'],
      parameters: ['link_lengths', 'joint_offsets', 'base_radius', 'end_effector_radius']
    },
    'E': {
      title: 'ros2_control',
      type: 'Controller Manager',
      description: 'Executes trajectories on hardware/simulation interfaces. Contains the joint_trajectory_controller plugin.',
      topics_subscribed: ['/joint_trajectory_controller/joint_trajectory', '/DeltaJoints', '/DeltaJointVels'],
      topics_published: ['/joint_states'],
      parameters: ['update_rate', 'controller_gains']
    },
    'H': {
      title: 'Gazebo / Sim Physics',
      type: 'Simulation Environment',
      description: 'Calculates the physics, collisions, and dynamics of the Delta robot. Acts as the digital twin.',
      topics_published: ['/clock', '/tf', '/tf_static', '/model/...']
    },
    'G': {
      title: 'joint_state_broadcaster',
      type: 'ROS2 Controller',
      description: 'Reads the simulated hardware state directly from ros2_control registers and publishes it continuously.',
      topics_published: ['/joint_states']
    },
    'F': {
      title: 'ros_gz_bridge',
      type: 'Bridge Node',
      description: 'Crucial middleware component that translates messages between Ignition/Gazebo transport layer and standard ROS2.',
      topics_bridged: ['/joint_states', '/clock', '/tf', '/tf_static', '/model/...'],
      files: ['ros_gz_bridge.yaml']
    },
    'I': {
      title: 'joint_state_bridge.py',
      type: 'Python ROS2 Node',
      description: 'Custom bridge that subscribes to raw /joint_states and extracts/formats specific active joints of the Delta robot.',
      topics_subscribed: ['/joint_states'],
      topics_published: ['/DeltaJoints', '/DeltaJointVels'],
      parameters: ['joint_names', 'correction_fn']
    },
    'J': {
      title: 'RViz2',
      type: 'Visualization GUI',
      description: 'The standard ROS2 3D visualization tool. Renders the robot model moving in real-time.',
      topics_subscribed: ['/DeltaJoints', '/DeltaJointVels', '/joint_states', '/tf', '/tf_static', '/clock'],
      files: ['delta_robot.rviz']
    },
    'K': {
      title: 'System Launch & Config',
      type: 'Launch / YAML Configs',
      description: 'The backbone files that start the system, load parameters, and configure hardware interfaces.',
      files: ['delta_robot_spawn.launch.py', 'ros2_controllers.yaml', 'delta_config.yaml', 'sensors_config.yaml']
    }
  },
  edges: {
    'e1': { title: 'Task Commands', type: 'Service Calls', desc: 'User triggers operations which the sequencer translates into /PlayDemoTrajectory, /MoveToPoint, etc.' },
    'e2': { title: 'Task Loading', type: 'File I/O', desc: 'Sequencer reads example_task.json and loads execution parameters.' },
    'e3': { title: 'Motion Services', type: 'Service Calls', desc: 'Sequencer orchestrates the Planner using MoveToPoint, MoveToConfiguration, PlayDemo, PlayCustom.' },
    'e4': { title: 'Math & Kinematics', type: 'Service Calls', desc: 'Planner delegates heavy math (IK, FK, Trajectory Interpolation) to the delta_kinematics node.' },
    'e5': { title: 'Trajectory Execution', type: 'Topic', desc: 'Planner publishes calculated paths to /joint_trajectory_controller/joint_trajectory for execution.' },
    'e6': { title: 'Hardware State', type: 'Internal Control', desc: 'ros2_control reads state from the Gazebo plugin and passes it to the joint_state_broadcaster.' },
    'e7': { title: 'Raw Joint States', type: 'Topic', desc: 'Broadcaster publishes all standard ROS2 /joint_states.' },
    'e8': { title: 'Feedback Loop', type: 'Topics', desc: 'Bridge filters states into /DeltaJoints and /DeltaJointVels back to the Planner and ros2_control.' },
    'e9': { title: 'Sim Transport', type: 'Topics', desc: 'Gazebo natively publishes /clock, /tf, and /tf_static inside the simulation layer.' },
    'e10': { title: 'Sim to ROS2 Bridge', type: 'Bridge', desc: 'ros_gz_bridge translates Gazebo topics into standard ROS2 topics for RViz.' },
    'e11': { title: 'Processed Viz Data', type: 'Topics', desc: '/DeltaJoints and /DeltaJointVels are routed to RViz for accurate digital twin rendering.' },
    'e12': { title: 'Launch Execution', type: 'Process', desc: 'User invokes the primary launch files to start the ROS2 graph.' },
    'e13': { title: 'Config Distribution', type: 'Parameters', desc: 'Launch file distributes YAML configs (ros2_controllers, delta_config, etc.) to respective nodes.' }
  }
};

// --- Map Coordinates & Layout (Expanded Grid) ---
// Canvas is conceptually 1400x800
const nodesLayout = [
  { id: 'A', label: 'User / GUI', x: 120, y: 250, color: '#059669', typeText: 'Interface' },
  { id: 'K', label: 'Launch & Config', x: 120, y: 480, color: '#475569', typeText: 'System Files' },
  { id: 'B_CONF', label: 'Task Configs', x: 360, y: 100, color: '#0f766e', typeText: 'JSON / Params' },
  { id: 'B', label: 'Task Sequencer', x: 360, y: 250, color: '#2563eb', typeText: 'Python Node' },
  { id: 'C', label: 'Motion Planner', x: 640, y: 250, color: '#4f46e5', typeText: 'C++ Core Logic' },
  { id: 'D', label: 'Delta Kinematics', x: 640, y: 80, color: '#4f46e5', typeText: 'C++ Math Engine' },
  { id: 'E', label: 'ROS2 Control', x: 920, y: 250, color: '#7c3aed', typeText: 'Controller' },
  { id: 'H', label: 'Gazebo Physics', x: 1180, y: 250, color: '#d97706', typeText: 'Simulation' },
  { id: 'G', label: 'JS Broadcaster', x: 920, y: 420, color: '#7c3aed', typeText: 'Broadcaster' },
  { id: 'F', label: 'ROS GZ Bridge', x: 1180, y: 420, color: '#0891b2', typeText: 'Middleware' },
  { id: 'I', label: 'JS Bridge (Py)', x: 640, y: 420, color: '#2563eb', typeText: 'Filter Node' },
  { id: 'J', label: 'RViz2', x: 640, y: 620, color: '#db2777', typeText: 'Visualization' },
];

const edgesLayout = [
  { id: 'e1', source: 'A', target: 'B', d: "M 210 250 L 270 250", label: "Task Triggers", lx: 240, ly: 250 },
  { id: 'e2', source: 'B_CONF', target: 'B', d: "M 360 130 L 360 220", label: "Reads JSON/Params", lx: 360, ly: 175 },
  { id: 'e3', source: 'B', target: 'C', d: "M 450 250 L 550 250", label: "Motion Services", lx: 500, ly: 250 },
  { id: 'e4', source: 'C', target: 'D', d: "M 620 220 L 620 110", label: "IK/FK/Traj Math", lx: 620, ly: 165 },
  { id: 'e5', source: 'C', target: 'E', d: "M 730 250 L 830 250", label: "/joint_trajectory", lx: 780, ly: 250 },
  { id: 'e6', source: 'E', target: 'G', d: "M 920 280 L 920 390", label: "Hardware State", lx: 920, ly: 335 },
  { id: 'e7', source: 'G', target: 'I', d: "M 830 420 L 730 420", label: "/joint_states", lx: 780, ly: 420 },
  { id: 'e8', source: 'I', target: 'C', d: "M 660 390 L 660 280", label: "Feedback Loop", lx: 660, ly: 335 },
  { id: 'e9', source: 'H', target: 'F', d: "M 1180 280 L 1180 390", label: "Sim Topics", lx: 1180, ly: 335 },
  { id: 'e10', source: 'F', target: 'J', d: "M 1180 450 L 1180 620 L 730 620", label: "Sim Bridge to UI", lx: 955, ly: 620 },
  { id: 'e11', source: 'I', target: 'J', d: "M 640 450 L 640 590", label: "/DeltaJoints", lx: 640, ly: 520 },
  { id: 'e12', source: 'A', target: 'K', d: "M 120 280 L 120 450", label: "User Launches", lx: 120, ly: 365 },
  { id: 'e13', source: 'K', target: 'C', d: "M 210 480 L 480 480 L 480 270 L 550 270", label: "Config Injection", lx: 480, ly: 375 },
];


// --- UI Components ---

const DetailBadge = ({ items, icon, colorClass }) => {
  if (!items || items.length === 0) return null;
  return (
    <div className="flex flex-wrap gap-2 mt-2">
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
    <div className="mt-5">
      <h4 className="text-xs font-bold uppercase tracking-wider text-slate-400 mb-2 flex items-center gap-2">
        {title}
      </h4>
      <DetailBadge items={items} icon={icon} colorClass={colorClass} />
    </div>
  );
};

export default function App() {
  const [selectedId, setSelectedId] = useState(null);
  const [hoverId, setHoverId] = useState(null);

  const checkIsActive = (itemId) => {
    if (!selectedId) return true;
    if (selectedId === itemId) return true;

    const isNode = nodesLayout.some(n => n.id === selectedId);
    if (isNode) {
      return edgesLayout.some(e => 
        (e.source === selectedId && (e.id === itemId || e.target === itemId)) || 
        (e.target === selectedId && (e.id === itemId || e.source === itemId))
      );
    }

    const selectedEdge = edgesLayout.find(e => e.id === selectedId);
    if (selectedEdge) {
      return itemId === selectedEdge.source || itemId === selectedEdge.target;
    }

    return false;
  };

  return (
    <div className="flex flex-col md:flex-row h-screen w-full bg-slate-950 font-sans text-slate-200 overflow-hidden">
      
      {/* --- Visual Map Canvas --- */}
      <div className="flex-grow relative border-b md:border-b-0 md:border-r border-slate-800 bg-slate-900 overflow-hidden select-none">
        
        {/* Background Grid */}
        <div className="absolute inset-0 pointer-events-none opacity-20" 
             style={{ backgroundImage: 'radial-gradient(circle, #475569 1px, transparent 1px)', backgroundSize: '24px 24px' }} />

        {/* Scaled SVG Canvas */}
        <svg viewBox="0 0 1350 750" className="w-full h-full absolute inset-0 cursor-crosshair">
          <defs>
            <marker id="arrowhead" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
              <polygon points="0 0, 10 3.5, 0 7" fill="#64748b" />
            </marker>
            <marker id="arrowhead-selected" markerWidth="10" markerHeight="7" refX="9" refY="3.5" orient="auto">
              <polygon points="0 0, 10 3.5, 0 7" fill="#f8fafc" />
            </marker>
            <style>
              {`
                @keyframes flow { from { stroke-dashoffset: 24; } to { stroke-dashoffset: 0; } }
                .flow-line { stroke-dasharray: 6 6; animation: flow 1.5s linear infinite; }
                .flow-line-selected { stroke-dasharray: 6 6; animation: flow 0.5s linear infinite; }
              `}
            </style>
          </defs>

          {/* Render Edges */}
          {edgesLayout.map((edge) => {
            const isActive = checkIsActive(edge.id);
            const isSelected = selectedId === edge.id;
            const isDimmed = selectedId && !isActive;

            return (
              <g key={edge.id} 
                 onClick={() => setSelectedId(edge.id)} 
                 onMouseEnter={() => setHoverId(edge.id)}
                 onMouseLeave={() => setHoverId(null)}
                 className="cursor-pointer transition-opacity duration-300"
                 style={{ opacity: isDimmed ? 0.15 : 1 }}>
                
                <path d={edge.d} fill="none" stroke="transparent" strokeWidth={30} />
                
                <path d={edge.d} fill="none" 
                      stroke={isSelected ? '#ffffff' : (isActive && selectedId ? '#94a3b8' : '#475569')}
                      strokeWidth={isSelected ? 3 : 2}
                      markerEnd={`url(#arrowhead${isSelected ? '-selected' : ''})`}
                      strokeLinejoin="round"
                      className={isSelected ? 'flow-line-selected' : 'flow-line'} />

                <foreignObject x={edge.lx - 100} y={edge.ly - 12} width="200" height="24" className="pointer-events-none overflow-visible">
                  <div className="flex items-center justify-center w-full h-full">
                    <span className={`px-2 py-0.5 text-[10px] font-semibold tracking-wide rounded-full whitespace-nowrap shadow-sm transition-colors
                      ${isSelected ? 'bg-white text-slate-900 shadow-[0_0_12px_rgba(255,255,255,0.6)]' : 'bg-slate-950 border border-slate-700 text-slate-300'}`}>
                      {edge.label}
                    </span>
                  </div>
                </foreignObject>
              </g>
            );
          })}

          {/* Render Nodes */}
          {nodesLayout.map((node) => {
            const isActive = checkIsActive(node.id);
            const isSelected = selectedId === node.id;
            const isDimmed = selectedId && !isActive;

            return (
              <g key={node.id} 
                 onClick={() => setSelectedId(node.id)}
                 onMouseEnter={() => setHoverId(node.id)}
                 onMouseLeave={() => setHoverId(null)}
                 className="cursor-pointer transition-opacity duration-300"
                 style={{ opacity: isDimmed ? 0.15 : 1 }}>
                 
                <foreignObject x={node.x - 90} y={node.y - 30} width="180" height="60" className="overflow-visible">
                  <div 
                    className={`w-full h-full rounded-xl flex flex-col items-center justify-center p-2 text-center transition-all duration-200 shadow-lg
                      ${isSelected ? 'ring-4 ring-white shadow-[0_0_25px_rgba(255,255,255,0.4)] scale-110 z-10' : 'hover:ring-2 hover:ring-slate-400 hover:scale-105 z-0'}
                    `}
                    style={{ backgroundColor: node.color, border: '1px solid rgba(255,255,255,0.15)' }}>
                    <div className="font-bold text-white text-sm leading-tight px-1 drop-shadow-md">
                      {node.label}
                    </div>
                    <div className="text-[10px] text-white/70 mt-1 uppercase tracking-widest font-bold">
                      {node.typeText}
                    </div>
                  </div>
                </foreignObject>
              </g>
            );
          })}
        </svg>

        {selectedId && (
          <button 
            onClick={() => setSelectedId(null)}
            className="absolute top-4 right-4 z-10 bg-slate-800/90 hover:bg-slate-700 text-white p-2 rounded-full backdrop-blur-md transition-colors shadow-xl flex items-center gap-2 pr-4 text-sm font-bold border border-slate-600">
            <X size={18} /> Clear Selection
          </button>
        )}
      </div>

      {/* --- Sidebar Info Panel --- */}
      <div className="w-full md:w-[450px] bg-slate-900 flex flex-col h-1/3 md:h-full relative shadow-[-10px_0_30px_rgba(0,0,0,0.5)] z-10 border-l border-slate-800">
        {!selectedId ? (
          <div className="p-8 h-full overflow-y-auto">
            <h2 className="text-2xl font-black text-white mb-4 flex items-center gap-3 tracking-tight">
              <Activity className="text-emerald-500" size={32} />
              Architecture Explorer
            </h2>
            <p className="text-slate-400 text-sm leading-relaxed mb-8">
              Welcome to the Ultra-Detailed interactive map of your Delta Robot ROS2 system. This visualizer parses your system logic, topics, actions, and configuration files into a connected data-flow graph.
            </p>
            
            <div className="bg-slate-800/40 border border-slate-700/50 rounded-2xl p-6 shadow-inner">
              <h3 className="text-xs font-bold text-slate-200 mb-4 flex items-center gap-2 uppercase tracking-widest">
                <Info size={16} className="text-blue-400"/> Quick Guide
              </h3>
              <ul className="text-sm text-slate-400 space-y-4">
                <li className="flex gap-3 items-start">
                  <Box className="shrink-0 text-slate-500 mt-0.5" size={18}/>
                  <span>Click on any colored <strong>Node Block</strong> to view its parameters, threads, actions, and endpoints.</span>
                </li>
                <li className="flex gap-3 items-start">
                  <ArrowRightLeft className="shrink-0 text-slate-500 mt-0.5" size={18}/>
                  <span>Click on any dashed <strong>Edge</strong> to see details about the topics or services flowing between nodes.</span>
                </li>
                <li className="flex gap-3 items-start">
                  <Zap className="shrink-0 text-amber-500 mt-0.5" size={18}/>
                  <span>Graph highlights active data paths to trace flows seamlessly.</span>
                </li>
              </ul>
            </div>
          </div>
        ) : (
          <div className="p-8 flex flex-col h-full overflow-y-auto custom-scrollbar">
            {(() => {
              const item = systemData.nodes[selectedId] || systemData.edges[selectedId];
              const isNode = !!systemData.nodes[selectedId];

              return (
                <div className="animate-in fade-in slide-in-from-right-4 duration-300">
                  <div className="mb-6">
                    <div className="flex items-center gap-2 text-xs font-black uppercase tracking-widest text-slate-400 mb-2">
                      {isNode ? <Box size={14} className="text-blue-400"/> : <ArrowRightLeft size={14} className="text-emerald-400"/>}
                      {item.type}
                    </div>
                    <h2 className="text-3xl font-black text-white leading-tight tracking-tight">{item.title}</h2>
                  </div>

                  <div className="text-sm text-slate-300 leading-relaxed mb-8 bg-slate-800/30 p-5 rounded-2xl border border-slate-700/50 shadow-inner italic">
                    {item.description || item.desc}
                  </div>

                  {isNode && (
                    <div className="space-y-2">
                      <DetailSection title="Associated Files" items={item.files} icon={<FileText size={14} />} colorClass="text-emerald-300 border-emerald-900/50" />
                      <DetailSection title="Node Parameters" items={item.parameters} icon={<Sliders size={14} />} colorClass="text-amber-300 border-amber-900/50" />
                      <DetailSection title="Actions & Threads" items={[...(item.actions || []), ...(item.threads || []), ...(item.actions_taken || [])]} icon={<Cpu size={14} />} colorClass="text-rose-300 border-rose-900/50" />
                      
                      <div className="pt-4 mt-4 border-t border-slate-800/50">
                        <DetailSection title="Subscribed Topics" items={item.topics_subscribed} icon={<Database size={14} />} colorClass="text-sky-300 border-sky-900/50" />
                        <DetailSection title="Published Topics" items={item.topics_published} icon={<Database size={14} />} colorClass="text-sky-300 border-sky-900/50" />
                        <DetailSection title="Services Provided" items={item.services_provided} icon={<Settings size={14} />} colorClass="text-purple-300 border-purple-900/50" />
                        <DetailSection title="Services Called" items={item.services_called} icon={<Settings size={14} />} colorClass="text-purple-300 border-purple-900/50" />
                      </div>
                    </div>
                  )}
                </div>
              );
            })()}
          </div>
        )}
      </div>
      
      {/* Custom Scrollbar Styles embedded */}
      <style dangerouslySetInnerHTML={{__html: `
        .custom-scrollbar::-webkit-scrollbar { width: 6px; }
        .custom-scrollbar::-webkit-scrollbar-track { background: transparent; }
        .custom-scrollbar::-webkit-scrollbar-thumb { background: #334155; border-radius: 10px; }
        .custom-scrollbar::-webkit-scrollbar-thumb:hover { background: #475569; }
      `}} />
    </div>
  );
}