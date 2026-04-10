# Delta Robot Wiki App

Interactive documentation site for your forked Delta Robot workspace.

## Source of truth

- Main code workspace: `/home/rikisu/major_project_ws2/src/`
- Legacy reference script: `/home/rikisu/major_project_ws/src/velocity_pub/scripts/delta_explain.py`
- Advanced notebook source: `/home/rikisu/major_project_ws2/src/delta_kinematics.ipynb`

This wiki is **not** a copy of the upstream website. It is a project-specific learning/documentation site focused on your updated package architecture and implementation details.

## What this app documents

- Package-level overview for:
  - `delta_robot`
  - `delta_robot_gui`
  - `delta_robot_sim`
  - `delta_robot_sensors`
  - `deltarobot_interfaces`
  - `delta_robot_description`
- Migration context from legacy visualization (`delta_explain.py`) to the advanced notebook workflow.
- A live 3D kinematics learning lab with:
  - IK-driven pose controls (X, Y, Z)
  - Workspace sampling cloud
  - Interactive orbit/pan/zoom rendering
  - Reachability feedback and joint angle display

## Run locally

```bash
cd /home/rikisu/delta_robot_wiki/docs-app
npm install
npm run dev
```

## Build

```bash
cd /home/rikisu/delta_robot_wiki/docs-app
npm run build
npm run preview
```

## Notes

- The 3D lab follows equations and concepts from `delta_kinematics.ipynb` (workspace/configuration-space/Jacobian context).
- The app is designed for both desktop and mobile viewing.
