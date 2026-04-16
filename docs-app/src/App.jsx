import React, { useState, useRef, useEffect, useCallback, useMemo } from 'react';
import { Activity, ArrowRightLeft, Box, X, FileText, Sliders, Cpu, Database, Settings, ChevronDown, ChevronUp, Info, Palette, Layers, BookOpen } from 'lucide-react';
import Plotly from 'plotly.js-dist-min';

// ═══════════════════════════════════════════════════════════════
//  KINEMATICS LEARNING COMPONENT (Following delta_explain.py style)
// ═══════════════════════════════════════════════════════════════
function KinematicsLearning() {
  const [pos, setPos] = useState({ x: 0, y: 0, z: -150 });
  const [activeArm, setActiveArm] = useState(0);
  const canvasRef = useRef(null);
  
  // Synchronized with major_project_ws2/src/delta_robot/config/delta_config.yaml
  const robotConstants = {
    R: 100,
    r: 32.5,
    L: 100,
    l: 263,
    SB: 346.41016151377545,
    SP: 112.58330249197702,
    jointMinDeg: -10,
    jointMaxDeg: 90,
    maxJointVelocity: 11.1,
    passiveLinkWidth: 42.84670
  };
  
  // IK solver from delta_explain.py - 2D circle intersection approach
  const solveIK = (x, y, z) => {
    const tan30 = 1 / Math.sqrt(3);
    
    const getAngles = (armIdx, x, y, z) => {
      const phi = (armIdx * 2 * Math.PI) / 3;
      const cosPhi = Math.cos(phi);
      const sinPhi = Math.sin(phi);
      
      // Step 1: Rotate coordinates to arm-specific frame
      const xPrime = x * cosPhi + y * sinPhi;
      const yPrime = -x * sinPhi + y * cosPhi;
      
      // Step 2: Virtual parameters
      const J = (robotConstants.R - robotConstants.r) - xPrime;
      const lEffSq = robotConstants.l * robotConstants.l - yPrime * yPrime;
      
      if (lEffSq < 0) return null;
      
      const lEff = Math.sqrt(lEffSq);
      
      // Step 3: Intersection logic - 2D circle intersection
      const d2 = J * J + z * z;
      const d = Math.sqrt(d2);
      
      if (d === 0) return null;
      
      const a = (robotConstants.L * robotConstants.L - lEff * lEff + d2) / (2 * d);
      const hSq = robotConstants.L * robotConstants.L - a * a;
      
      if (hSq < 0) return null;
      
      const h = Math.sqrt(hSq);
      
      // Step 4: Elbow coordinates in 2D plane
      const dx = -J / d;
      const dz = z / d;
      
      const e2dX = a * dx - h * dz;
      const e2dZ = a * dz + h * dx;
      
      // Step 5: Angle using atan2
      const theta = Math.atan2(e2dZ, e2dX);
      
      return theta * 180 / Math.PI;
    };
    
    const theta1 = getAngles(0, x, y, z);
    if (theta1 === null) return null;
    
    const theta2 = getAngles(1, x, y, z);
    if (theta2 === null) return null;
    
    const theta3 = getAngles(2, x, y, z);
    if (theta3 === null) return null;
    
    return [theta1, theta2, theta3];
  };

  const angles = solveIK(pos.x, pos.y, pos.z);
  const reachable = angles !== null;

  const setCameraToTopView = useCallback(() => {
    if (!canvasRef.current) return;
    Plotly.relayout(canvasRef.current, {
      'scene.camera': {
        eye: { x: 0, y: 0, z: 2.4 },
        center: { x: 0, y: 0, z: 0 },
        up: { x: 0, y: 1, z: 0 }
      }
    });
  }, []);
  
  // Draw 3D Plotly visualization
  useEffect(() => {
    if (!canvasRef.current) return;
    
    try {
      // Calculate elbow positions (forward kinematics)
      const elbows = [];
      if (reachable && angles) {
        for (let i = 0; i < 3; i++) {
          const phi = (i * 2 * Math.PI) / 3;
          const bx = robotConstants.R * Math.cos(phi);
          const by = robotConstants.R * Math.sin(phi);
          
          const theta = angles[i] * Math.PI / 180;
          const ex = bx + robotConstants.L * Math.cos(phi) * Math.cos(theta);
          const ey = by + robotConstants.L * Math.sin(phi) * Math.cos(theta);
          const ez = robotConstants.L * Math.sin(theta);
          elbows.push([ex, ey, ez]);
        }
      }
      
      // Build traces array
      const traces = [];
      
      // Base triangle
      const base_angles = [0, 2*Math.PI/3, 4*Math.PI/3, 0];
      const bx = base_angles.map(a => robotConstants.R * Math.cos(a));
      const by = base_angles.map(a => robotConstants.R * Math.sin(a));
      const bz = Array(4).fill(0);
      
      traces.push({
        x: bx,
        y: by,
        z: bz,
        mode: 'lines',
        line: { color: '#64748b', width: 6 },
        name: 'Base',
        type: 'scatter3d'
      });
      
      // Platform triangle (end effector)
      const ee_angles = [0, 2*Math.PI/3, 4*Math.PI/3, 0];
      const px = ee_angles.map(a => pos.x + robotConstants.r * Math.cos(a));
      const py = ee_angles.map(a => pos.y + robotConstants.r * Math.sin(a));
      const pz = Array(4).fill(pos.z);
      
      traces.push({
        x: px,
        y: py,
        z: pz,
        mode: 'lines',
        line: { color: '#10b981', width: 6 },
        name: 'Platform',
        type: 'scatter3d'
      });
      
      // Arm-specific plane (for visualization)
      const phi_plane = (activeArm * 2 * Math.PI) / 3;
      const planeX = [-250, 250, 250, -250, -250].map(r => r * Math.cos(phi_plane));
      const planeY = [-250, 250, 250, -250, -250].map(r => r * Math.sin(phi_plane));
      const planeZ = [-300, -300, 50, 50, -300];
      
      traces.push({
        x: planeX,
        y: planeY,
        z: planeZ,
        mode: 'lines',
        line: { color: 'rgba(168, 85, 247, 0.3)', width: 2 },
        name: `Arm ${activeArm + 1} Plane`,
        type: 'scatter3d'
      });
      
      // Biceps and Forearms
      if (elbows.length === 3) {
        for (let i = 0; i < 3; i++) {
          const phi = (i * 2 * Math.PI) / 3;
          const bx_i = robotConstants.R * Math.cos(phi);
          const by_i = robotConstants.R * Math.sin(phi);
          const px_i = pos.x + robotConstants.r * Math.cos(phi);
          const py_i = pos.y + robotConstants.r * Math.sin(phi);
          
          const bicepColor = i === activeArm ? '#22c55e' : '#ef4444';
          const forearmColor = i === activeArm ? '#0ea5e9' : '#06b6d4';
          const bicepWidth = i === activeArm ? 8 : 4;
          const forearmWidth = i === activeArm ? 6 : 3;
          
          // Bicep
          traces.push({
            x: [bx_i, elbows[i][0]],
            y: [by_i, elbows[i][1]],
            z: [0, elbows[i][2]],
            mode: 'lines',
            line: { color: bicepColor, width: bicepWidth },
            name: i === 0 ? 'Bicep' : undefined,
            showlegend: i === 0,
            type: 'scatter3d'
          });
          
          // Forearm
          traces.push({
            x: [elbows[i][0], px_i],
            y: [elbows[i][1], py_i],
            z: [elbows[i][2], pos.z],
            mode: 'lines',
            line: { color: forearmColor, width: forearmWidth },
            name: i === 0 ? 'Forearm' : undefined,
            showlegend: i === 0,
            type: 'scatter3d'
          });
        }
        
        // Y' perpendicular offset line (from joint to arm plane)
        {
          const phi = (activeArm * 2 * Math.PI) / 3;
          const cos_phi = Math.cos(phi);
          const sin_phi = Math.sin(phi);
          const x_prime = pos.x * cos_phi + pos.y * sin_phi;
          const y_prime = -pos.x * sin_phi + pos.y * cos_phi;
          
          // Joint in global coordinates
          const joint_x = pos.x + robotConstants.r * cos_phi;
          const joint_y = pos.y + robotConstants.r * sin_phi;
          
          // Perpendicular to arm (tangential to arm circle)
          const perp_x = -sin_phi;
          const perp_y = cos_phi;
          
          // Projection on arm plane: remove y' offset
          const plane_x = joint_x - y_prime * perp_x;
          const plane_y = joint_y - y_prime * perp_y;
          
          traces.push({
            x: [joint_x, plane_x],
            y: [joint_y, plane_y],
            z: [pos.z, pos.z],
            mode: 'lines',
            line: { color: '#a855f7', width: 3, dash: 'dash' },
            name: "y' (Offset)",
            type: 'scatter3d'
          });
        }
        
        // Elbow points
        traces.push({
          x: elbows.map(e => e[0]),
          y: elbows.map(e => e[1]),
          z: elbows.map(e => e[2]),
          mode: 'markers',
          marker: { size: 8, color: '#fbbf24' },
          name: 'Elbows',
          type: 'scatter3d'
        });
      }
      
      // End effector point
      traces.push({
        x: [pos.x],
        y: [pos.y],
        z: [pos.z],
        mode: 'markers',
        marker: { size: 12, color: reachable ? '#3b82f6' : '#ef4444' },
        name: 'EE',
        type: 'scatter3d'
      });
      
      // Camera angle based on active arm
      const armAngle = (activeArm * 2 * Math.PI) / 3 + Math.PI / 2;
      const camEye = {
        x: 1.5 * Math.cos(armAngle),
        y: 1.5 * Math.sin(armAngle),
        z: 1.2
      };
      
      const currentCamera = canvasRef.current?.layout?.scene?.camera;

      const layout = {
        title: `3D Delta Robot (Arm ${activeArm + 1})`,
        showlegend: true,
        uirevision: 'keep-camera',
        scene: {
          xaxis: { title: 'X (mm)', range: [-200, 200] },
          yaxis: { title: 'Y (mm)', range: [-200, 200] },
          zaxis: { title: 'Z (mm)', range: [-300, 50] },
          uirevision: 'keep-camera',
          aspectmode: 'cube',
          camera: currentCamera || undefined
        },
        margin: { l: 0, r: 0, t: 40, b: 0 },
        paper_bgcolor: '#0f172a',
        plot_bgcolor: '#1e293b',
        font: { color: '#cbd5e1', size: 12, family: 'Arial' },
        hovermode: 'closest'
      };
      
      const config = {
        responsive: true,
        displayModeBar: true,
        displaylogo: false,
        modeBarButtonsToRemove: ['lasso3d', 'select3d']
      };
      
      Plotly.react(canvasRef.current, traces, layout, config);
    } catch (e) {
      console.error('Plotly error:', e);
    }
  }, [pos, angles, reachable, robotConstants.R, robotConstants.r, robotConstants.L, robotConstants.l, activeArm]);

  // Update camera ONLY when activeArm changes, NOT when sliders move
  useEffect(() => {
    if (!canvasRef.current) return;
    
    const phi = (activeArm * 2 * Math.PI) / 3;
    // Camera looks along arm-plane normal so selected plane appears parallel to the view.
    const nx = Math.sin(phi);
    const ny = -Math.cos(phi);
    const camEye = {
      x: 1.7 * nx,
      y: 1.7 * ny,
      z: 0.9
    };
    
    Plotly.relayout(canvasRef.current, {
      'scene.camera': {
        eye: camEye,
        center: { x: 0, y: 0, z: 0 },
        up: { x: 0, y: 0, z: 1 }
      }
    });
  }, [activeArm]);

  return (
    <div className="h-full w-full flex flex-col bg-slate-950 overflow-hidden">
      {/* Header */}
      <div className="bg-gradient-to-r from-slate-900 via-slate-800 to-slate-900 border-b border-slate-700/50 p-4">
        <h2 className="text-2xl font-black text-white flex items-center gap-2"><BookOpen size={24} className="text-emerald-400" /> Inverse Kinematics Explorer</h2>
        <p className="text-slate-400 text-xs mt-1">major_project_ws2 (5-DOF extension) delta base IK: R={robotConstants.R}mm, r={robotConstants.r}mm, L={robotConstants.L}mm, l={robotConstants.l}mm</p>
      </div>

      {/* Main Content - 3 Column Layout */}
      <div className="flex-1 flex gap-4 overflow-hidden p-4">
        
        {/* LEFT: Controls */}
        <div className="w-72 flex flex-col gap-3 overflow-y-auto">
          {/* Position Controls */}
          <div className="bg-slate-800/40 backdrop-blur p-4 rounded-lg border border-slate-700/50">
            <h3 className="text-xs font-bold text-slate-300 mb-3">📍 End-Effector Position</h3>
            <div className="space-y-3">
              {[{k:'x', l:'X', min:'-150', max:'150'}, {k:'y', l:'Y', min:'-150', max:'150'}, {k:'z', l:'Z', min:'-300', max:'-50'}].map(({k, l, min, max}) => (
                <div key={k}>
                  <div className="flex justify-between mb-1">
                    <span className="text-xs text-slate-400">{l}</span>
                    <span className="text-xs font-bold text-blue-300">{pos[k].toFixed(0)} mm</span>
                  </div>
                  <input type="range" min={min} max={max} step="2" value={pos[k]} 
                    onChange={(e) => setPos({...pos, [k]: parseFloat(e.target.value)})}
                    className="w-full h-2 bg-slate-700 rounded cursor-pointer" />
                </div>
              ))}
            </div>
            <button onClick={() => setPos({x:0, y:0, z:-200})} className="w-full mt-3 px-3 py-2 bg-emerald-600 hover:bg-emerald-700 text-white text-xs font-semibold rounded transition-all">🏠 Home Position</button>
          </div>

          {/* Arm Selection */}
          <div className="bg-slate-800/40 backdrop-blur p-4 rounded-lg border border-slate-700/50">
            <h3 className="text-xs font-bold text-slate-300 mb-2">🦾 Select Arm</h3>
            <div className="flex gap-2 mb-2">
              {[0, 1, 2].map((i) => (
                <button key={i} onClick={() => setActiveArm(i)} className={`flex-1 px-2 py-2 rounded text-xs font-semibold transition-all ${activeArm === i ? 'bg-emerald-600 text-white' : 'bg-slate-700 text-slate-300 hover:bg-slate-600'}`}>
                  Arm {i + 1}
                </button>
              ))}
            </div>
            <button
              onClick={setCameraToTopView}
              className="w-full px-3 py-2 rounded text-xs font-semibold bg-sky-700 text-white hover:bg-sky-600 transition-all"
            >
              Top View
            </button>
          </div>

          {/* Robot Parameters */}
          <div className="bg-slate-800/40 backdrop-blur p-4 rounded-lg border border-slate-700/50">
            <h3 className="text-xs font-bold text-slate-300 mb-2">⚙️ Robot Geometry</h3>
            <div className="space-y-1 text-xs">
              <div className="flex justify-between"><span className="text-slate-400">Base radius (R):</span><span className="text-blue-300 font-mono">{robotConstants.R} mm</span></div>
              <div className="flex justify-between"><span className="text-slate-400">Platform (r):</span><span className="text-blue-300 font-mono">{robotConstants.r} mm</span></div>
              <div className="flex justify-between"><span className="text-slate-400">Base side (SB):</span><span className="text-blue-300 font-mono">{robotConstants.SB.toFixed(2)} mm</span></div>
              <div className="flex justify-between"><span className="text-slate-400">EE side (SP):</span><span className="text-blue-300 font-mono">{robotConstants.SP.toFixed(2)} mm</span></div>
              <div className="flex justify-between"><span className="text-slate-400">Bicep (L):</span><span className="text-blue-300 font-mono">{robotConstants.L} mm</span></div>
              <div className="flex justify-between"><span className="text-slate-400">Rod (l):</span><span className="text-blue-300 font-mono">{robotConstants.l} mm</span></div>
              <div className="flex justify-between"><span className="text-slate-400">Rod width (H):</span><span className="text-blue-300 font-mono">{robotConstants.passiveLinkWidth.toFixed(2)} mm</span></div>
            </div>
          </div>

          {/* Workspace Bounds */}
          <div className="bg-slate-800/40 backdrop-blur p-4 rounded-lg border border-slate-700/50">
            <h3 className="text-xs font-bold text-slate-300 mb-2">🤖 Workspace Bounds</h3>
            <div className="space-y-1 text-xs">
              <div className="flex justify-between"><span className="text-slate-400">X/Y Range:</span><span className="text-emerald-300 font-mono">±{(robotConstants.R + robotConstants.L).toFixed(0)} mm</span></div>
              <div className="flex justify-between"><span className="text-slate-400">Z Min:</span><span className="text-emerald-300 font-mono">-{(robotConstants.L + robotConstants.l).toFixed(0)} mm</span></div>
              <div className="flex justify-between"><span className="text-slate-400">Z Max:</span><span className="text-emerald-300 font-mono">0 mm</span></div>
              <div className="flex justify-between"><span className="text-slate-400">Joint range:</span><span className="text-emerald-300 font-mono">{robotConstants.jointMinDeg}°..{robotConstants.jointMaxDeg}°</span></div>
              <div className="flex justify-between"><span className="text-slate-400">Max joint vel:</span><span className="text-emerald-300 font-mono">{robotConstants.maxJointVelocity} rad/s</span></div>
              <div className={`mt-2 px-2 py-1 rounded text-center font-semibold ${reachable ? 'bg-emerald-900/30 text-emerald-300' : 'bg-red-900/30 text-red-300'}`}>
                {reachable ? '✓ Reachable' : '✗ Unreachable'}
              </div>
            </div>
          </div>
        </div>

        {/* CENTER: 3D Visualization */}
        <div className="flex-1 flex flex-col gap-4">
          <div ref={canvasRef} className="bg-slate-900/50 rounded-lg border border-slate-700/50 overflow-hidden flex-1" style={{minHeight: '400px'}}></div>
        </div>

        {/* RIGHT: Results */}
        <div className="w-96 flex flex-col gap-3 overflow-y-auto">
          {/* Active Arm Math State */}
          <div className="bg-slate-800/40 backdrop-blur p-4 rounded-lg border border-slate-700/50">
            <h3 className="text-xs font-bold text-slate-300 mb-3">⚙️ Arm {activeArm + 1} State</h3>
            {reachable && angles ? (
              <div className="space-y-2 text-xs font-mono">
                <div className="bg-slate-900/50 p-2 rounded border border-slate-600">
                  <span className="text-slate-400">Motor Angle:</span>
                  <div className="text-blue-300 font-bold">{angles[activeArm].toFixed(2)}°</div>
                </div>
                <div className="bg-slate-900/50 p-2 rounded border border-slate-600">
                  <span className="text-slate-400">Status:</span>
                  <div className="text-emerald-300">✓ Reachable</div>
                </div>
              </div>
            ) : (
              <div className="text-red-300 text-xs">✗ Target unreachable</div>
            )}
          </div>

          {/* Full Step-by-Step Calculations */}
          <div className="bg-slate-800/40 backdrop-blur p-4 rounded-lg border border-slate-700/50 flex-1 overflow-y-auto">
            <h3 className="text-xs font-bold text-slate-300 mb-3">📐 Complete Calculation</h3>
            <div className="space-y-3 text-[10px] leading-snug">
              
              {/* Step 0 */}
              <div className="bg-slate-900/50 p-2 rounded border-l-2 border-yellow-500">
                <div className="font-bold text-yellow-300 mb-1">Step 0: Joint Location</div>
                <div className="text-slate-300">
                  Position: ({pos.x.toFixed(1)}, {pos.y.toFixed(1)}, {pos.z.toFixed(1)}) mm
                </div>
              </div>

              {/* Step 1 - Perpendicular Offset */}
              {(() => {
                const phi = (activeArm * 2 * Math.PI) / 3;
                const cos_phi = Math.cos(phi);
                const sin_phi = Math.sin(phi);
                const x_prime = pos.x * cos_phi + pos.y * sin_phi;
                const y_prime = -pos.x * sin_phi + pos.y * cos_phi;
                return (
                  <div className="bg-slate-900/50 p-2 rounded border-l-2 border-purple-500">
                    <div className="font-bold text-purple-300 mb-1">Step 1: Perpendicular Offset (y')</div>
                    <div className="text-slate-300 font-mono text-xs space-y-1">
                      <div>φ = arm {activeArm} × 120° = {(activeArm*120).toFixed(0)}°</div>
                      <div className="text-slate-400">x' = x·cos(φ) + y·sin(φ)</div>
                      <div className="text-cyan-300">x' = {x_prime.toFixed(3)} mm</div>
                      <div className="text-slate-400">y' = -x·sin(φ) + y·cos(φ)</div>
                      <div className="text-purple-300 font-bold">y' = {y_prime.toFixed(3)} mm</div>
                      <div className="text-slate-500 text-[9px]">(Shown as Purple Line in 3D)</div>
                    </div>
                  </div>
                );
              })()}

              {/* Step 2 - Effective Rod Length */}
              {(() => {
                const phi = (activeArm * 2 * Math.PI) / 3;
                const sin_phi = Math.sin(phi);
                const cos_phi = Math.cos(phi);
                const y_prime = -pos.x * sin_phi + pos.y * cos_phi;
                const val_under_sqrt = robotConstants.l**2 - y_prime**2;
                const l_eff = val_under_sqrt >= 0 ? Math.sqrt(val_under_sqrt) : 0;
                return (
                  <div className="bg-slate-900/50 p-2 rounded border-l-2 border-emerald-500">
                    <div className="font-bold text-emerald-300 mb-1">Step 2: Effective Rod Length</div>
                    <div className="text-slate-300 font-mono text-xs space-y-1">
                      <div className="text-slate-400">The perpendicular offset y' = {y_prime.toFixed(3)} mm</div>
                      <div className="text-slate-400">reduces the effective forearm length:</div>
                      <div className="text-slate-400">l_eff = √(l² - y'²)</div>
                      <div className="text-emerald-300 font-bold">l_eff = √({robotConstants.l}² - {y_prime.toFixed(3)}²)</div>
                      <div className="text-emerald-300 font-bold px-2 py-1 bg-emerald-950/50 rounded">l_eff = {l_eff.toFixed(2)} mm</div>
                    </div>
                  </div>
                );
              })()}

              {/* Step 3 - 2D Intersection */}
              {(() => {
                const phi = (activeArm * 2 * Math.PI) / 3;
                const sin_phi = Math.sin(phi);
                const cos_phi = Math.cos(phi);
                const x_prime = pos.x * cos_phi + pos.y * sin_phi;
                const y_prime = -pos.x * sin_phi + pos.y * cos_phi;
                const J = (robotConstants.R - robotConstants.r) - x_prime;
                const val_under_sqrt = robotConstants.l**2 - y_prime**2;
                const l_eff = val_under_sqrt >= 0 ? Math.sqrt(val_under_sqrt) : 0;
                const d = Math.sqrt(J*J + pos.z*pos.z);
                return (
                  <div className="bg-slate-900/50 p-2 rounded border-l-2 border-blue-500">
                    <div className="font-bold text-blue-300 mb-1">Step 3: Solve 2D Intersection</div>
                    <div className="text-slate-300 font-mono text-xs space-y-1">
                      <div className="text-slate-400">J = (R - r) - x'</div>
                      <div className="text-cyan-300">J = {J.toFixed(3)} mm</div>
                      <div className="text-slate-400">d = √(J² + Z²)</div>
                      <div className="text-cyan-300">d = {d.toFixed(3)} mm</div>
                      <div className="text-blue-300 font-bold mt-2">Circle 1 (bicep): R = {robotConstants.L} mm</div>
                      <div className="text-blue-300 font-bold">Circle 2 (forearm): R = {l_eff.toFixed(2)} mm</div>
                    </div>
                  </div>
                );
              })()}

              {/* Step 4 - Motor Angle */}
              {reachable && angles && (
                <div className="bg-slate-900/50 p-2 rounded border-l-2 border-red-500">
                  <div className="font-bold text-red-300 mb-1">Step 4: Motor Angle</div>
                  <div className="text-slate-300 font-mono text-xs space-y-1">
                    <div className="text-slate-400">θ = atan2(e2d_z, e2d_x)</div>
                    <div className="text-slate-400">(from 2D elbow coordinates)</div>
                    <div className="text-red-400 font-bold mt-2 px-2 py-1 bg-red-950/50 rounded">
                      θ = {angles[activeArm].toFixed(1)}°
                    </div>
                  </div>
                </div>
              )}
            </div>
          </div>
        </div>

      </div>
    </div>
  );
}

// ═══════════════════════════════════════════════════════════════
//  SYSTEM DATA
// ═══════════════════════════════════════════════════════════════
const systemData = {
  nodes: {
    'A':      { title: 'Operator / ROS2 CLI / GUI', type: 'Interface & Config', description: 'Entry point for simulation and hardware operation in major_project_ws2. Typical flow from README: launch simulation with delta_robot_spawn.launch.py, then run kinematics and motion_planner nodes, and optionally start the PyQt GUI and task tools.', actions_taken: ['Edits task JSON files (config/examples/)', 'Writes .gcode files for CNC-style paths', 'Runs ros2 launch delta_robot_sim delta_robot_spawn.launch.py', 'Runs ros2 run delta_robot kinematics', 'Runs ros2 run delta_robot motion_planner', 'Runs ros2 launch delta_robot_gui delta_robot_gui.launch.py'], files: ['README.md', 'delta_robot/README.md', 'delta_robot/config/examples/example_task.json', 'delta_robot/config/examples/circle_test.gcode', 'delta_robot/config/delta_config.yaml'], parameters: ['json_units (meters|mm)', 'default_duration_s', 'motion_rate_hz', 'home_x_mm / home_y_mm / home_z_mm'] },
    'B':      { title: 'Task & G-code Runner', type: 'Python ROS2 Tooling', description: 'Merged task execution path representing both json_task_sequencer.py and gcode_parser.py. Both use the same motion planner service backend for execution: MoveToPoint/MoveToPose for point-wise flow and PlayCustomTrajectory for batched flow.', services_called: ['delta_motion_planner/move_to_point', 'delta_motion_planner/move_to_pose', 'delta_motion_planner/play_custom_trajectory'], parameters: ['move_to_point_service', 'move_to_pose_service', 'play_custom_trajectory_service', 'json_units/default_units', 'default_duration_s', 'motion_rate_hz', 'home_x/y/z_mm', 'home_z_mm'], files: ['delta_robot/json_task_sequencer.py', 'delta_robot/gcode_parser.py'], actions_taken: ['JSON: move/home/wait/tilt/spin/suction action parsing', 'G-code: G0/G1/G28/G90/G91/G20/G21 parsing', 'Shared planner services: MoveToPoint + MoveToPose + PlayCustomTrajectory', 'Auto fallback from batched to point-wise mode when needed'] },
    'B2':     { title: 'Live Slider / Teach Input', type: 'Topic Input Mode', description: 'Interactive live control path (for slider/teach UI) that drives motion planner in live mode using topics instead of task services.', topics_published: ['delta_motion_planner/live_target (geometry_msgs/Point)', 'delta_motion_planner/live_orientation (std_msgs/Float64MultiArray)'], parameters: ['live_target_topic', 'live_orientation_topic', 'live_controller_ms'], files: ['delta_robot_gui (live control UI path)'], actions_taken: ['Publishes continuous live target updates', 'Publishes tilt/spin orientation updates', 'Used for real-time teach/slider control'] },
    'C':      { title: 'motion_planner (C++)', type: 'C++ ROS2 Node', description: 'The core trajectory generation engine. Receives high-level move commands, delegates IK/FK math to kinematics, and publishes motor commands on a separate execution thread. Supports 5 demo trajectories and custom point-list trajectories via PlayCustomTrajectory. Publishes to both simulation (JointTrajectory) and hardware (DeltaJoints) simultaneously. Live-teach mode is topic-driven via live_target and live_orientation topics.', topics_published: ['delta_motors/set_joints (DeltaJoints)', 'delta_motors/set_joint_vels (DeltaJointVels)', '/joint_trajectory_controller/joint_trajectory'], topics_subscribed: ['delta_motion_planner/live_target (geometry_msgs/Point)', 'delta_motion_planner/live_orientation (std_msgs/Float64MultiArray)'], services_provided: ['play_demo_trajectory', 'move_to_point', 'move_to_pose', 'move_to_configuration', 'motion_demo (start/stop auto-demo)', 'play_custom_trajectory', 'set_motion_mode'], services_called: ['delta_kinematics/delta_ik', 'delta_kinematics/delta_fk', 'delta_kinematics/convert_to_joint_trajectory', 'delta_kinematics/convert_to_joint_vel_trajectory'], parameters: ['traj_step_ms (default 10 ms)', 'live_controller_ms', 'live_target_topic', 'live_orientation_topic'], threads: ['TrajectoryExecutor — detached std::thread for non-blocking', 'Init Timer — deferred until kinematics online', 'Live Controller Timer — fixed-rate live loop', 'Demo Timer — 32s auto-demo cycle'], files: ['src/motion_planner.cpp', 'include/motion_planner.hpp'], actions_taken: ['Live mode: consumes live_target + live_orientation topics', 'Task mode: same service backend used by JSON and G-code tools', 'Demo: circle — XY circle at Z=-180mm, R=40mm', 'Demo: pringle — saddle-shaped 3D path', 'Demo: axes — sequential X/Y/Z axis translations', 'Demo: up_down — 5 cycle vertical oscillation', 'Demo: scan — snake-scan from CSV file'] },
    'D':      { title: 'delta_kinematics (C++)', type: 'C++ ROS2 Node', description: 'Provides FK, IK, Jacobian computation, trajectory conversion, and velocity mapping for the 5-DOF Delta Robot (3 base joints + tilt + spin). IK uses tangent-half-angle substitution from closure equations; FK uses 3-sphere intersection (see delta_kinematics.ipynb). Publishes RobotConfig at configured rate and applies limits from delta_config.yaml.', services_provided: ['delta_kinematics/delta_fk — Forward Kinematics (3-sphere intersection)', 'delta_kinematics/delta_ik — Inverse Kinematics (tangent-half-angle)', 'convert_to_joint_trajectory — EE path → joint path', 'convert_to_joint_vel_trajectory — EE vel → joint vel'], services_called: ['delta_motors/set_joint_limits'], topics_published: ['delta_robot/robot_config (RobotConfig @ 50Hz)'], topics_subscribed: ['delta_motors/motor_position_feedback (DeltaJoints)', 'delta_motors/motor_velocity_feedback (DeltaJointVels)'], parameters: ['base_triangle_side_length (173.205 mm)', 'end_effector_side_length (56.292 mm)', 'active_link_length (100 mm)', 'passive_link_length (200 mm)', 'passive_link_width (42.85 mm)', 'joint_min (-10° / -0.175 rad)', 'joint_max (90° / 1.571 rad)', 'max_joint_velocity (11.1 rad/s)', 'robot_config_freq (50 Hz)'], files: ['delta_robot/src/kinematics.cpp', 'delta_robot/include/kinematics.hpp', 'delta_robot/config/delta_config.yaml', 'delta_kinematics.ipynb'], actions_taken: ['FK: 3-sphere intersection with Z-plane solving', 'IK: Tangent-half-angle substitution (E·cos + F·sin + G = 0)', 'Jacobian: Jθ⁻¹ · Jp velocity mapping for 5-DOF', 'Gradient: forward/central/backward differencing', 'Aux angles: passive-link derived angles', '5-DOF support: base + tilt + spin'] },
    'M':      { title: 'motor_control_node.py', type: 'Python ROS2 Node', description: 'Hardware serial driver over ESP32 bridge for 5-DOF Waveshare servos (IDs 1-3 base, 4-5 end-effector). Supports binary (0x7E SETN + XOR CRC) and text (SET/PING/SCAN) protocols. Tolerates missing motors, runs watchdog-protected feedback loop, and auto-detects /dev/ttyUSB* or /dev/ttyACM* at 500000 baud. TOF/IMU integration is not part of this current runtime path.', topics_subscribed: ['delta_motors/set_joints (DeltaJoints)', 'delta_motors/set_joint_vels (DeltaJointVels)'], topics_published: ['delta_motors/motor_position_feedback (DeltaJoints)', 'delta_motors/motor_velocity_feedback (DeltaJointVels)', '/servo/target (Float32MultiArray)', '/servo/actual (Float32MultiArray)'], services_provided: ['delta_motors/set_joint_limits'], parameters: ['device_name (auto-detect)', 'baudrate (500000)', 'bicep moving_speed / moving_acc', 'ee_moving_speed / ee_moving_acc', 'max_write_retries (3)', 'use_binary_bridge (true)', 'stream_feedback_period_ms (20)', 'read_fail_watchdog_limit (5)'], files: ['delta_robot/motor_control_node.py', 'esp32/Servo-Driver-with-ESP32_berickson/src/servo_driver.ino', 'esp32/Servo-Driver-with-ESP32_berickson/src/serial_bridge.h'], actions_taken: ['Binary protocol: 0x7E SOF + SETN cmd + XOR CRC', 'Text protocol: SET / PING / SCAN / TORQUE / LIST', 'Motor position: 0–4095 ticks (12-bit, 2048=center)', 'Conversion: θ·(4096/2π) + 2048 = motor ticks', 'Auto-detect /dev/ttyUSB* + /dev/ttyACM* at startup', 'Streamed feedback: FBP id=N pos=P speed=S', 'ESP32 bridge path: host serial @ 500000 with queued command execution', '5-DOF: controls biceps + end-effector DOF simultaneously'] },
    'E':      { title: 'ros2_control', type: 'Controller Manager', description: 'Executes trajectories on the Gazebo simulation hardware interface via the joint_trajectory_controller plugin. Manages 7 joints at 250 Hz update rate: 3 bicep (jbf1-3) with PID gains p=100, i=1, d=10, plus 4 end-effector joints (Bevelj1, Bevelj2, Tj1, BeveljEE).', topics_subscribed: ['/joint_trajectory_controller/joint_trajectory (JointTrajectory)'], parameters: ['update_rate: 250 Hz', 'joints: jbf1, jbf2, jbf3, Bevelj1, Bevelj2, Tj1, BeveljEE', 'command_interfaces: position', 'state_interfaces: position, velocity', 'PID gains (jbf1-3): p=100, i=1, d=10'], files: ['config/ros2_controllers.yaml'] },
    'H':      { title: 'Gazebo / Sim Physics', type: 'Simulation Environment', description: 'Ignition Gazebo Harmonic (gz-sim-8) calculates the physics, collisions, and dynamics of the Delta robot SDF model. Acts as the digital twin. Provides full 3D rendering, collision detection, and joint/link state publication.', topics_published: ['/clock', '/tf', '/tf_static', '/model/delta_robot/...'], files: ['worlds/empty.sdf', 'delta_robot_description/models/model.sdf', 'delta_robot_description/meshes/'], actions_taken: ['Physics simulation at real-time factor', 'SDF model with custom meshes', 'ros2_control hardware interface integration'] },
    'G':      { title: 'joint_state_broadcaster', type: 'ROS2 Controller', description: 'Reads simulated hardware state from ros2_control registers and publishes /joint_states continuously. Broadcasts position and velocity for all 7 joints.', topics_published: ['/joint_states (sensor_msgs/JointState)'], parameters: ['Broadcasts: position + velocity for all joints'] },
    'F':      { title: 'ros_gz_bridge', type: 'Bridge Node', description: 'Translates messages between Ignition/Gazebo transport and standard ROS2 topics. Configured via YAML for selective topic bridging with QoS overrides.', topics_bridged: ['/joint_states', '/clock (rosgz_interfaces/Clock)', '/tf (tf2_msgs/TFMessage)', '/tf_static (tf2_msgs/TFMessage)'], files: ['config/ros_gz_bridge.yaml'], parameters: ['config_file', 'qos: tf_static → transient_local durability'] },
    'I':      { title: 'joint_state_bridge.py', type: 'Python ROS2 Node', description: 'Subscribes to /joint_states, extracts bicep joints (jbf1-3), applies angle correction (raw < -0.5 → raw + π), publishes DeltaJoints + DeltaJointVels. Only active when use_sim_feedback=true. Bridges the gap between ros2_control joint naming and delta_motors topic convention.', topics_subscribed: ['/joint_states (sensor_msgs/JointState)'], topics_published: ['delta_motors/motor_position_feedback (DeltaJoints)', 'delta_motors/motor_velocity_feedback (DeltaJointVels)'], parameters: ['Joint names: jbf1, jbf2, jbf3', 'Correction: raw < -0.5 → raw + π'], files: ['delta_robot/joint_state_bridge.py'] },
    'J':      { title: 'RViz2', type: 'Visualization GUI', description: 'Standard ROS2 3D visualization tool. Renders the robot URDF/SDF model in real-time using /tf and /joint_states. Pre-configured with delta_robot.rviz layout.', topics_subscribed: ['/joint_states', '/tf', '/tf_static', '/clock'], files: ['config/delta_robot.rviz'] },
    'K':      { title: 'System Launch & Config', type: 'Launch / YAML Configs', description: 'Runtime orchestration and configuration for major_project_ws2. Simulation uses delta_robot_spawn.launch.py; hardware startup is typically done with ros2 run commands for kinematics, motion_planner, and motor_control_node.py as documented in README.', files: ['delta_robot_sim/launch/delta_robot_spawn.launch.py', 'delta_robot_gui/delta_robot_gui.launch.py', 'delta_robot/launch/gcode_json_tools.launch.py', 'delta_robot/config/ros2_controllers.yaml', 'delta_robot/config/delta_config.yaml', 'delta_robot_sim/config/ros_gz_bridge.yaml'], actions_taken: ['Sim pipeline: Gazebo + RViz + controllers + bridge + 3D plotter', 'HW runtime: kinematics + motion_planner + motor_control_node.py', 'GUI runtime: PyQt control center via delta_robot_gui launch', 'Task pipeline: json_task_sequencer or gcode_parser'] },
    'P1':     { title: 'see_motors.py (Plotter)', type: 'Python Visualization Tool', description: 'Live matplotlib plotter for real servo feedback. Displays target vs actual motor positions in real-time. Computes cross-correlation lag, RMSE, peak error, and estimated max RPM for motor performance analysis.', topics_subscribed: ['/servo/target (Float32MultiArray)', '/servo/actual (Float32MultiArray)'], files: ['delta_robot/see_motors.py'], actions_taken: ['Real-time target vs actual position plots', 'Cross-correlation lag computation', 'RMSE and peak error stats', 'Estimated max RPM calculation'] },
    'P2':     { title: 'plotter3d.py', type: 'Python ROS2 Node', description: '3D end-effector position plotter. Subscribes to robot_config at 50 Hz and renders the end-effector path in 3D using matplotlib. Useful for visualising trajectory quality and workspace utilisation.', topics_subscribed: ['delta_robot/robot_config (RobotConfig)'], files: ['delta_robot_sim/scripts/plotter3d.py'], actions_taken: ['3D scatter/line plot of EE path', 'Real-time matplotlib animation', 'Workspace boundary visualization'] },
    'IFACES': { title: 'deltarobot_interfaces', type: 'ROS2 Interface Package', description: 'Custom message and service definitions shared across all packages. 3 messages for joint state communication and 10 services covering kinematics, motion planning, and hardware configuration.', files: ['msg/DeltaJoints.msg — θ1, θ2, θ3 [rad]', 'msg/DeltaJointVels.msg — θ̇1, θ̇2, θ̇3 [rad/s]', 'msg/RobotConfig.msg — angles + vels + EE position', 'srv/DeltaFK.srv — joint angles → EE xyz', 'srv/DeltaIK.srv — EE xyz → joint angles', 'srv/ConvertToJointTrajectory.srv', 'srv/ConvertToJointVelTrajectory.srv', 'srv/MoveToPoint.srv', 'srv/MoveToConfiguration.srv', 'srv/MotionDemo.srv', 'srv/PlayDemoTrajectory.srv', 'srv/PlayCustomTrajectory.srv — Point[] + step_ms', 'srv/SetJointLimits.srv — min/max rad + max vel'], actions_taken: ['3 custom message types', '10 custom service definitions', 'Shared compile-time contracts across all packages'] }
  },
  edges: {
    'e1':     { title: 'JSON Tasks & Config', type: 'Service Calls', description: 'User triggers task execution via JSON files and provides static JSON config parameters.' },
    'e1_b2':  { title: 'Live UI Input',       type: 'Topics', description: 'User drives live slider/teach control input that publishes planner live topics.' },
    'e12':    { title: 'Startup Execution',   type: 'Process', description: 'User starts simulation via ros2 launch and starts hardware nodes via ros2 run commands, per README workflow.' },
    'e3':     { title: 'Task Services',       type: 'Service Calls', description: 'Merged Task & G-code Runner calls planner services: move_to_point, move_to_pose, and play_custom_trajectory.' },
    'e3b':    { title: 'Live Topics → Planner', type: 'Topics', description: 'Live slider/teach path publishes delta_motion_planner/live_target and delta_motion_planner/live_orientation to motion_planner.' },
    'e4':     { title: 'Math & Kinematics',   type: 'Service Calls', description: 'Motion Planner delegates IK/FK computation and trajectory conversion to Kinematics.' },
    'e5b':    { title: 'Traj → Hardware',     type: 'Topic', description: 'Motor command topics from motion_planner to hardware: delta_motors/set_joints and delta_motors/set_joint_vels.' },
    'e5':     { title: 'Traj → Gazebo',       type: 'Topic', description: 'JointTrajectory messages published to the simulation joint_trajectory_controller.' },
    'e6':     { title: 'Hardware State',      type: 'Internal Control', description: 'ros2_control reads simulated joint state from Gazebo hardware interface.' },
    'e7':     { title: '/joint_states',       type: 'Topic', description: 'Joint state broadcaster publishes position and velocity for all 7 joints.' },
    'e8':     { title: 'Sim Feedback',        type: 'Topics', description: 'Joint State Bridge converts /joint_states to DeltaJoints feedback for kinematics.' },
    'e8b':    { title: 'HW Feedback',         type: 'Serial + Topics', description: 'motor_control ingests serial servo feedback and publishes DeltaJoints/DeltaJointVels to kinematics (TOF/IMU integration not represented here yet).' },
    'e9':     { title: 'Sim Transport',       type: 'Topics', description: 'Gazebo publishes physics state via Ignition transport topics.' },
    'e10':    { title: 'Sim → ROS2 Bridge',   type: 'Bridge', description: 'ros_gz_bridge translates Ignition transport to ROS2 topics for RViz and other nodes.' },
    'e11':    { title: 'Viz Data',            type: 'Topics', description: 'Bridge forwards /tf and /joint_states to the Joint State Bridge for processing.' },
    'e13':    { title: 'Config Distribution', type: 'Parameters', description: 'delta_config.yaml parameters are loaded into kinematics and used by planner/runtime nodes for limits and trajectory settings.' },
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

  'B2':     { x: 460,  y: 240,  color: '#0ea5e9', label: 'Live Slider Input', typeText: 'Topic Mode' },
  'B':      { x: 670,  y: 240,  color: '#2563eb', label: 'Task + G-code Runner', typeText: 'Python Tooling' },

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
  { id: 'e1_b2', from: 'A', fs: 'bottom', fo: 0, to: 'B2', ts: 'top', toOff: 0, label: 'Live Slider Input', t: 0.5 },
  { id: 'e1',    from: 'A', fs: 'bottom', fo: 20, to: 'B', ts: 'top', toOff: 0, label: 'JSON + G-code', t: 0.5 },
  { id: 'e12',   from: 'A', fs: 'bottom', fo: -20, to: 'K', ts: 'top', toOff: 0, label: 'Launches', t: 0.5 },
  
  { id: 'e3b', from: 'B2', fs: 'bottom', fo: 0, to: 'C', ts: 'top', toOff: 0, label: 'Live Topics', t: 0.35 },
  { id: 'e3',  from: 'B', fs: 'bottom', fo: 0, to: 'C', ts: 'top', toOff: 30, label: 'Planner Srvs', t: 0.375 },
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
  const [activeTab, setActiveTab] = useState('arch');
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
    <div className="h-screen w-full bg-slate-950 font-sans text-slate-200 overflow-hidden relative flex flex-col">
      {/* Tab Navigation */}
      <div className="absolute top-3 right-3 z-50 bg-gradient-to-r from-slate-900/95 via-slate-800/95 to-slate-900/95 border border-slate-700/60 rounded-xl p-1.5 flex items-center gap-2 backdrop-blur-xl shadow-2xl">
        <button onClick={() => { setActiveTab('arch'); setSelectedId(null); }} className={`px-4 py-2 rounded-lg font-semibold text-sm transition-all flex items-center gap-2 ${activeTab === 'arch' ? 'bg-blue-600 text-white shadow-lg shadow-blue-500/50' : 'bg-slate-700/50 text-slate-300 hover:bg-slate-600/50'}`}><Activity size={18} /> Architecture</button>
        <button onClick={() => setActiveTab('learn')} className={`px-4 py-2 rounded-lg font-semibold text-sm transition-all flex items-center gap-2 ${activeTab === 'learn' ? 'bg-emerald-600 text-white shadow-lg shadow-emerald-500/50' : 'bg-slate-700/50 text-slate-300 hover:bg-slate-600/50'}`}><BookOpen size={18} /> Kinematics</button>
      </div>
      {/* Content */}
      {activeTab === 'arch' ? (
        <>
      <div 
        ref={containerRef} 
        className="flex-1 absolute inset-0 bg-slate-900 select-none"
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
            <Activity className="text-emerald-500" size={20} /> Delta Robot 5-DOF Architecture
          </h1>
          <p className="text-[10px] text-slate-400 mt-0.5 text-center">{Object.keys(nodesLayout).length} nodes · {edgesRaw.length} edges · Interactive diagram (click to inspect)</p>
          <p className="text-[10px] text-cyan-300/90 mt-0.5 text-center">Live mode uses ROS topics: delta_motion_planner/live_target + delta_motion_planner/live_orientation. G-code and JSON share the same planner services.</p>
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
                <span className="text-amber-300 font-semibold">ST3215-HS ×3 + STS3032 ×2</span>
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
                <span className="text-blue-300 font-semibold">173.21 mm</span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-400">End-Effector ▲</span>
                <span className="text-blue-300 font-semibold">56.29 mm</span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-400">Active Link</span>
                <span className="text-blue-300 font-semibold">100 mm</span>
              </div>
              <div className="flex justify-between">
                <span className="text-slate-400">Passive Link</span>
                <span className="text-blue-300 font-semibold">200 mm</span>
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

        </>
      ) : (
        <KinematicsLearning />
      )}

      <style dangerouslySetInnerHTML={{__html: `
        @keyframes flyIn{from{opacity:0;transform:translateY(10px)}to{opacity:1;transform:translateY(0)}}
        *{scrollbar-width:thin;scrollbar-color:#475569 #0b1220}
        *::-webkit-scrollbar{width:10px;height:10px}
        *::-webkit-scrollbar-track{background:#0b1220}
        *::-webkit-scrollbar-thumb{background:linear-gradient(180deg,#334155,#475569);border:2px solid #0b1220;border-radius:999px}
        *::-webkit-scrollbar-thumb:hover{background:linear-gradient(180deg,#475569,#64748b)}
        *::-webkit-scrollbar-corner{background:#0b1220}
        .custom-scrollbar::-webkit-scrollbar{width:5px}
        .custom-scrollbar::-webkit-scrollbar-track{background:transparent}
        .custom-scrollbar::-webkit-scrollbar-thumb{background:#334155;border-radius:10px}
        .custom-scrollbar::-webkit-scrollbar-thumb:hover{background:#475569}
      `}} />
    </div>
  );
}
