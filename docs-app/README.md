# Delta Robot 5-DOF Wiki App

Interactive documentation site for your forked Delta Robot workspace with 5-DOF extension (3 bicep + 2 end-effector DOF).

## Source of truth

- **Main code workspace:** `/home/rikisu/major_project_ws2/src/`
- **Legacy reference script:** `/home/rikisu/major_project_ws/src/velocity_pub/scripts/delta_explain.py` (3-DOF baseline)
- **Advanced notebook (interactive math):** `/home/rikisu/major_project_ws2/src/delta_kinematics.ipynb` (5-DOF with IK/FK derivations)

This wiki is **not** a copy of the upstream website. It is a project-specific learning/documentation site focused on your updated 5-DOF package architecture and implementation details.

## What this app documents

**Architecture Diagram (tab):**
- System-level control flow with 14 nodes and 17 data edges
- 5 domain layers: Task, Motion, Hardware, Simulation, Visualization
- ROS 2 Jazzy middleware stack with C++ kinematics engine
- Click any node or edge to see detailed specs, ROS topics, and services

**Learn Kinematics (tab):**
- Interactive 3-DOF inverse kinematics solver with real-time visualization
- Adjust end-effector position (X, Y, Z) and view calculated joint angles
- Tangent-half-angle substitution method (constraint closure equations)
- Workspace reachability analysis
- Robot constants from the advanced `delta_kinematics.ipynb` notebook

**Packages:**
- `delta_robot` — 5-DOF kinematics (C++) + motion planning + task sequencing
- `delta_robot_gui` — PyQt5 Cartesian control center
- `delta_robot_sim` — Gazebo Harmonic simulation with ros2_control
- `delta_robot_sensors` — BNO055 IMU + VL53L1X ToF sensor nodes
- `deltarobot_interfaces` — 3 message types + 10 service types
- `delta_robot_description` — SDF model with custom meshes

**Learning Resources:**
- Interactive 3D kinematics visualization (notebook implementation)
- Workspace/singularity analysis concepts
- Jacobian velocity mapping for 5-DOF end-effector control

## Run locally

```bash
cd /home/rikisu/delta_robot_wiki/docs-app
npm install
npm run dev
```

## Build & preview

```bash
cd /home/rikisu/delta_robot_wiki/docs-app
npm run build
npm run preview
```

## Notes

- The 3D kinematics lab follows equations from `delta_kinematics.ipynb`
- 5-DOF support: 3 bicep motors for XYZ + 2 end-effector motors for tilt/spin
- Hardware specs auto-detected from ws2 with Waveshare ST3215 servos @ 500 kbaud
- The app is designed for desktop and mobile viewing
