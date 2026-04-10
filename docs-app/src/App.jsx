import { useMemo, useState } from 'react';
import Plot from 'react-plotly.js';
import {
  BookOpen,
  Boxes,
  Cable,
  Cpu,
  Gauge,
  GitFork,
  GraduationCap,
  Move3d,
  Radio,
  Route,
  Sparkles,
  Triangle,
  Waves,
} from 'lucide-react';
import './App.css';

const ROBOT = {
  SB: 261.30642,
  SP: 146.96152,
  L: 108,
  ELL: 174,
  H: 42.8467,
};

const tan30 = 1 / Math.sqrt(3);
const sin30 = 0.5;
const tan60 = Math.sqrt(3);
const cos120 = -0.5;
const sin120 = Math.sqrt(3) / 2;

const WB = (Math.sqrt(3) / 6) * ROBOT.SB;
const UP = (Math.sqrt(3) / 3) * ROBOT.SP;
const WP = (Math.sqrt(3) / 6) * ROBOT.SP;

const packageCards = [
  {
    name: 'delta_robot',
    tag: 'Core motion package',
    details:
      'C++ kinematics + motion planning, Python motor bridge, G-code parser, JSON task sequencer, and launch integration for sim/hardware.',
    highlights: [
      'Services: delta_fk, delta_ik, move_to_point, play_custom_trajectory',
      'Nodes: kinematics.cpp, motion_planner.cpp, motor_control_node.py',
      'Demo trajectories: circle, pringle, axes, up_down, scan',
    ],
  },
  {
    name: 'delta_robot_gui',
    tag: 'Operator GUI',
    details:
      'PyQt control center for Cartesian moves, G-code playback, and JSON sequence execution.',
    highlights: [
      'Launch: delta_robot_gui.launch.py',
      'Supports task-level teaching and replay workflows',
      'Bridges user intent to motion planner services',
    ],
  },
  {
    name: 'delta_robot_sim',
    tag: 'Digital twin',
    details:
      'Gazebo Harmonic simulation stack with ros2_control, bridge tooling, and trajectory visualization scripts.',
    highlights: [
      'Launch: delta_robot_spawn.launch.py',
      'Includes 3D plotting helper for end-effector path',
      'Feeds /joint_states and TF into full ROS graph',
    ],
  },
  {
    name: 'delta_robot_sensors',
    tag: 'Sensing and filtering',
    details:
      'IMU + ToF integration package with filtering and ROS topics for end-effector sensing and scan workflows.',
    highlights: [
      'BNO and VL53L1X nodes in C++',
      'Filter implementations and data collection tooling',
      'Supports surface scanning use cases',
    ],
  },
  {
    name: 'deltarobot_interfaces',
    tag: 'Message/service contracts',
    details:
      'Shared message and service definitions used by all compute, hardware, and GUI packages.',
    highlights: [
      'DeltaJoints, DeltaJointVels, RobotConfig',
      'Kinematics + planner service contracts',
      'Stable integration boundary across packages',
    ],
  },
  {
    name: 'delta_robot_description',
    tag: 'Robot model assets',
    details:
      'Robot model resources and supporting description package used by simulation and visualization tools.',
    highlights: [
      'Feeds sim and RViz representation',
      'Works with launch stack and controllers',
      'Complements delta_robot_sim pipeline',
    ],
  },
];

function fk(theta1, theta2, theta3) {
  const t = ((ROBOT.SB - ROBOT.SP) * tan30) / 2;

  const y1 = -(t + ROBOT.L * Math.cos(theta1));
  const z1 = -ROBOT.L * Math.sin(theta1);

  const y2 = (t + ROBOT.L * Math.cos(theta2)) * sin30;
  const x2 = y2 * tan60;
  const z2 = -ROBOT.L * Math.sin(theta2);

  const y3 = (t + ROBOT.L * Math.cos(theta3)) * sin30;
  const x3 = -y3 * tan60;
  const z3 = -ROBOT.L * Math.sin(theta3);

  const dnm = (y2 - y1) * x3 - (y3 - y1) * x2;
  if (Math.abs(dnm) < 1e-9) return null;

  const w1 = y1 * y1 + z1 * z1;
  const w2 = x2 * x2 + y2 * y2 + z2 * z2;
  const w3 = x3 * x3 + y3 * y3 + z3 * z3;

  const a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
  const b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2;

  const a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
  const b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2;

  const a = a1 * a1 + a2 * a2 + dnm * dnm;
  const b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
  const c = (b2 - y1 * dnm) ** 2 + b1 * b1 + dnm * dnm * (z1 * z1 - ROBOT.ELL * ROBOT.ELL);

  const d = b * b - 4 * a * c;
  if (d < 0) return null;

  const z = -0.5 * (b + Math.sqrt(d)) / a;
  const x = (a1 * z + b1) / dnm;
  const y = (a2 * z + b2) / dnm;

  if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) return null;
  return { x, y, z };
}

function ik(x, y, z) {
  if (Math.abs(z) < 1e-9) return null;

  function calcAngleYZ(x0, y0, z0) {
    const y1 = -0.5 * tan30 * ROBOT.SB;
    const yAdj = y0 - 0.5 * tan30 * ROBOT.SP;

    const a =
      (x0 * x0 + yAdj * yAdj + z0 * z0 + ROBOT.L * ROBOT.L - ROBOT.ELL * ROBOT.ELL - y1 * y1) /
      (2 * z0);
    const b = (y1 - yAdj) / z0;
    const disc = -((a + b * y1) ** 2) + ROBOT.L * (b * b * ROBOT.L + ROBOT.L);

    if (disc < 0) return null;

    const yj = (y1 - a * b - Math.sqrt(disc)) / (b * b + 1);
    const zj = a + b * yj;

    const theta = Math.atan(-zj / (y1 - yj)) + (yj > y1 ? Math.PI : 0);
    return theta;
  }

  const t1 = calcAngleYZ(x, y, z);
  if (t1 == null) return null;

  const t2 = calcAngleYZ(x * cos120 + y * sin120, y * cos120 - x * sin120, z);
  if (t2 == null) return null;

  const t3 = calcAngleYZ(x * cos120 - y * sin120, y * cos120 + x * sin120, z);
  if (t3 == null) return null;

  return { theta1: t1, theta2: t2, theta3: t3 };
}

function elbowPoints(theta1, theta2, theta3) {
  return [
    {
      x: 0,
      y: -WB - ROBOT.L * Math.cos(theta1) + UP,
      z: -ROBOT.L * Math.sin(theta1),
    },
    {
      x: (Math.sqrt(3) / 2) * (WB + ROBOT.L * Math.cos(theta2)) - ROBOT.SP / 2,
      y: 0.5 * (WB + ROBOT.L * Math.cos(theta2)) - WP,
      z: -ROBOT.L * Math.sin(theta2),
    },
    {
      x: -(Math.sqrt(3) / 2) * (WB + ROBOT.L * Math.cos(theta3)) + ROBOT.SP / 2,
      y: 0.5 * (WB + ROBOT.L * Math.cos(theta3)) - WP,
      z: -ROBOT.L * Math.sin(theta3),
    },
  ];
}

function platformJoints(x, y, z) {
  return [
    { x: x, y: y - WP, z },
    { x: x + ROBOT.SP / 2, y: y + WP / 2, z },
    { x: x - ROBOT.SP / 2, y: y + WP / 2, z },
  ];
}

function baseJoints() {
  return [
    { x: 0, y: -WB, z: 0 },
    { x: ROBOT.SB / 2, y: WB / 2, z: 0 },
    { x: -ROBOT.SB / 2, y: WB / 2, z: 0 },
  ];
}

function deg(rad) {
  return (rad * 180) / Math.PI;
}

function formatRad(rad) {
  return `${rad.toFixed(4)} rad (${deg(rad).toFixed(2)} deg)`;
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

export default function App() {
  const [x, setX] = useState(0);
  const [y, setY] = useState(0);
  const [z, setZ] = useState(-180);
  const [sampleDensity, setSampleDensity] = useState(13);

  const ikSolution = useMemo(() => ik(x, y, z), [x, y, z]);

  const robotTraces = useMemo(() => {
    if (!ikSolution) return [];

    const bases = baseJoints();
    const elbows = elbowPoints(ikSolution.theta1, ikSolution.theta2, ikSolution.theta3);
    const plate = platformJoints(x, y, z);

    const traces = [];

    traces.push({
      type: 'scatter3d',
      mode: 'lines+markers',
      x: [...bases.map((p) => p.x), bases[0].x],
      y: [...bases.map((p) => p.y), bases[0].y],
      z: [...bases.map((p) => p.z), bases[0].z],
      line: { color: '#f59e0b', width: 7 },
      marker: { size: 3, color: '#f59e0b' },
      name: 'Base triangle',
    });

    traces.push({
      type: 'scatter3d',
      mode: 'lines+markers',
      x: [...plate.map((p) => p.x), plate[0].x],
      y: [...plate.map((p) => p.y), plate[0].y],
      z: [...plate.map((p) => p.z), plate[0].z],
      line: { color: '#0ea5e9', width: 7 },
      marker: { size: 3, color: '#0ea5e9' },
      name: 'End-effector plate',
    });

    for (let i = 0; i < 3; i += 1) {
      traces.push({
        type: 'scatter3d',
        mode: 'lines',
        x: [bases[i].x, elbows[i].x],
        y: [bases[i].y, elbows[i].y],
        z: [bases[i].z, elbows[i].z],
        line: { color: '#fb7185', width: 9 },
        name: `Active link ${i + 1}`,
        showlegend: i === 0,
      });

      traces.push({
        type: 'scatter3d',
        mode: 'lines',
        x: [elbows[i].x, plate[i].x],
        y: [elbows[i].y, plate[i].y],
        z: [elbows[i].z, plate[i].z],
        line: { color: '#22d3ee', width: 5 },
        name: `Passive link ${i + 1}`,
        showlegend: i === 0,
      });
    }

    traces.push({
      type: 'scatter3d',
      mode: 'markers',
      x: [x],
      y: [y],
      z: [z],
      marker: { color: '#10b981', size: 7 },
      name: 'TCP',
    });

    return traces;
  }, [ikSolution, x, y, z]);

  const workspaceTrace = useMemo(() => {
    const minDeg = -10;
    const maxDeg = 90;
    const samples = Math.max(8, Math.min(20, sampleDensity));

    const points = [];
    for (let i = 0; i < samples; i += 1) {
      const t1d = minDeg + (i * (maxDeg - minDeg)) / (samples - 1);
      const t1 = (t1d * Math.PI) / 180;
      for (let j = 0; j < samples; j += 1) {
        const t2d = minDeg + (j * (maxDeg - minDeg)) / (samples - 1);
        const t2 = (t2d * Math.PI) / 180;
        for (let k = 0; k < samples; k += 1) {
          const t3d = minDeg + (k * (maxDeg - minDeg)) / (samples - 1);
          const t3 = (t3d * Math.PI) / 180;
          const p = fk(t1, t2, t3);
          if (p && p.z < -40) points.push(p);
        }
      }
    }

    return {
      type: 'scatter3d',
      mode: 'markers',
      x: points.map((p) => p.x),
      y: points.map((p) => p.y),
      z: points.map((p) => p.z),
      marker: {
        size: 2,
        opacity: 0.25,
        color: points.map((p) => p.z),
        colorscale: 'Turbo',
        colorbar: {
          title: 'Z (mm)',
          x: 1.02,
        },
      },
      name: `Workspace cloud (${points.length} pts)`,
    };
  }, [sampleDensity]);

  const reachable = Boolean(ikSolution);

  return (
    <div className="wiki-shell">
      <header className="hero">
        <div className="hero-glow" />
        <p className="eyebrow">DeltaRobot Documentation Hub</p>
        <h1>Major Project Workspace Wiki</h1>
        <p className="hero-copy">
          This wiki documents your current forked codebase in <code>/home/rikisu/major_project_ws2/src/</code>.
          It is separate from the upstream project page and is focused on your architecture, package updates,
          and learning-oriented explanations.
        </p>
        <div className="hero-badges">
          <span><GitFork size={14} /> Fork lineage acknowledged</span>
          <span><Boxes size={14} /> Package-level breakdown</span>
          <span><Move3d size={14} /> Live 3D kinematics lab</span>
        </div>
      </header>

      <main className="content-grid">
        <section className="panel full">
          <h2><BookOpen size={18} /> What Changed In Your Workspace</h2>
          <div className="bullet-grid">
            <article>
              <h3><Cpu size={16} /> Control stack extension</h3>
              <p>
                Your stack now combines trajectory services, JSON/G-code executors, and mixed C++/Python runtime
                nodes around <code>delta_robot</code>.
              </p>
            </article>
            <article>
              <h3><Cable size={16} /> Hardware pipeline specifics</h3>
              <p>
                Motor bridge details include serial protocol support, USB auto-detection, and feedback publication
                loops that synchronize with kinematics.
              </p>
            </article>
            <article>
              <h3><Radio size={16} /> Sensor path coverage</h3>
              <p>
                The <code>delta_robot_sensors</code> package captures IMU and ToF integration plus filtering context
                for scan-quality improvements.
              </p>
            </article>
            <article>
              <h3><Route size={16} /> Workspace-aware planning</h3>
              <p>
                The notebook and motion pipeline reflect workspace boundaries, cross-sections, and singularity-safe
                trajectory logic for practical operation.
              </p>
            </article>
          </div>
        </section>

        <section className="panel">
          <h2><Boxes size={18} /> Package Index</h2>
          <div className="card-stack">
            {packageCards.map((pkg) => (
              <article key={pkg.name} className="package-card">
                <div className="package-head">
                  <h3>{pkg.name}</h3>
                  <p>{pkg.tag}</p>
                </div>
                <p>{pkg.details}</p>
                <ul>
                  {pkg.highlights.map((item) => (
                    <li key={item}>{item}</li>
                  ))}
                </ul>
              </article>
            ))}
          </div>
        </section>

        <section className="panel">
          <h2><GraduationCap size={18} /> From Legacy Script To Advanced Notebook</h2>
          <div className="compare">
            <div>
              <h3>Legacy: delta_explain.py</h3>
              <p>
                Focused on per-arm geometric intuition, local 2D projection math, and matplotlib controls for
                stepping through IK logic.
              </p>
            </div>
            <div>
              <h3>Advanced: delta_kinematics.ipynb</h3>
              <p>
                Expands to configuration-space sampling, workspace cloud generation, convex hull reasoning, random
                in-workspace trajectory tests, and Jacobian-centered velocity mapping.
              </p>
            </div>
          </div>
          <p className="note">
            This wiki live lab is built from the upgraded notebook concepts, not just the older script visualization.
          </p>
        </section>

        <section className="panel full lab">
          <h2><Sparkles size={18} /> Live Delta Kinematics Lab</h2>
          <p className="lab-intro">
            Orbit, pan, and zoom this 3D scene to inspect robot posture and sampled workspace. Controls update IK in
            real time using the same analytical structure as the notebook.
          </p>

          <div className="controls-grid">
            <label>
              <span>X (mm)</span>
              <input
                type="range"
                min={-90}
                max={90}
                value={x}
                onChange={(e) => setX(Number(e.target.value))}
              />
              <input
                type="number"
                value={x}
                onChange={(e) => setX(clamp(Number(e.target.value), -120, 120))}
              />
            </label>

            <label>
              <span>Y (mm)</span>
              <input
                type="range"
                min={-90}
                max={90}
                value={y}
                onChange={(e) => setY(Number(e.target.value))}
              />
              <input
                type="number"
                value={y}
                onChange={(e) => setY(clamp(Number(e.target.value), -120, 120))}
              />
            </label>

            <label>
              <span>Z (mm)</span>
              <input
                type="range"
                min={-320}
                max={-80}
                value={z}
                onChange={(e) => setZ(Number(e.target.value))}
              />
              <input
                type="number"
                value={z}
                onChange={(e) => setZ(clamp(Number(e.target.value), -400, -20))}
              />
            </label>

            <label>
              <span>Workspace sample density</span>
              <input
                type="range"
                min={8}
                max={20}
                value={sampleDensity}
                onChange={(e) => setSampleDensity(Number(e.target.value))}
              />
              <input
                type="number"
                min={8}
                max={20}
                value={sampleDensity}
                onChange={(e) => setSampleDensity(clamp(Number(e.target.value), 8, 20))}
              />
            </label>
          </div>

          <div className="lab-status">
            <div className={reachable ? 'ok' : 'bad'}>
              <Triangle size={15} />
              {reachable ? 'Reachable IK solution' : 'Unreachable target for current geometry'}
            </div>
            <div>
              <Gauge size={15} />
              Joint limits sampled: -10 deg to 90 deg
            </div>
            <div>
              <Waves size={15} />
              Passive link length: {ROBOT.ELL} mm
            </div>
          </div>

          {reachable && (
            <div className="angles">
              <p>theta1: {formatRad(ikSolution.theta1)}</p>
              <p>theta2: {formatRad(ikSolution.theta2)}</p>
              <p>theta3: {formatRad(ikSolution.theta3)}</p>
            </div>
          )}

          <div className="plot-wrap">
            <Plot
              data={[workspaceTrace, ...robotTraces]}
              layout={{
                autosize: true,
                paper_bgcolor: '#05131a',
                plot_bgcolor: '#05131a',
                font: { color: '#d9f3ff', family: 'IBM Plex Sans, sans-serif' },
                margin: { l: 0, r: 0, b: 0, t: 0 },
                scene: {
                  xaxis: { title: 'X (mm)', gridcolor: '#16404f', zerolinecolor: '#2f7088' },
                  yaxis: { title: 'Y (mm)', gridcolor: '#16404f', zerolinecolor: '#2f7088' },
                  zaxis: { title: 'Z (mm)', gridcolor: '#16404f', zerolinecolor: '#2f7088' },
                  camera: { eye: { x: 1.35, y: 1.35, z: 0.95 } },
                  aspectmode: 'data',
                },
                legend: {
                  bgcolor: 'rgba(4, 20, 30, 0.8)',
                  bordercolor: '#1f495a',
                  borderwidth: 1,
                },
              }}
              useResizeHandler
              style={{ width: '100%', height: '560px' }}
              config={{ responsive: true, displaylogo: false }}
            />
          </div>
        </section>
      </main>
    </div>
  );
}
