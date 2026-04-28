# amr_gui — Visualization & Digital Twin

`amr_gui` is the graphical and visualization layer of the OSU-AMR architecture. While the core routing and task logic live in `amr_central` and `amr_cfm`, this package houses the tools required to actually *see* what the system is doing.

It contains the 3D visual assets for the HOOVER AMRs, a Qt-based launch dashboard, and a heavily customized RViz2 environment that serves as the **Digital Twin** of the physical course.

---

## 🏗️ System Role

```
[amr_central] & [amr_roboware]
        ↓ (TF, Odom, Battery, Diagnostics)
[ROS 2 Network]
        ↓
[amrviz / Digital Twin] ← Uses [amr_meshes] for 3D rendering
        ↑
[amr_launch_gui] (Operator Launch Commands)
```

---

## 📦 Sub-Package Reference

This repository contains three distinct ROS 2 packages:

| Package | Purpose |
|---|---|
| `amr_launch_gui` | A Qt5/C++ based graphical control panel. Provides buttons to launch central nodes, spawn specific robots, and manage the system without relying purely on command-line terminals. |
| `amr_meshes` | The visual asset library. Contains `.dae`, `.ply`, and `.fbx` 3D models for the HOOVER frames, course tiles, and AprilTag overlays. |
| `amrviz` | The Digital Twin environment. Contains custom RViz2 C++ plugins (diagnostic overlays, sensor panels) and Python scripts to spoof robot data for simulation testing. |

---

## 📁 Directory Structure

```
amr_gui/
├── amr_launch_gui/          # Custom Qt Launch Dashboard
│   ├── include/             # C++ Headers (launch_strip, central_tab, etc.)
│   ├── src/                 # C++ Source logic
│   ├── ui/                  # Qt Designer UI files
│   └── package.xml
├── amr_meshes/              # 3D Assets for RViz
│   ├── meshes_amr/          # HOOVER frames, chassis, and floor tiles
│   ├── meshes_riptide/      # Legacy/shared riptide meshes
│   └── package.xml
└── amrviz/                  # Digital Twin & RViz Plugins
    ├── amrviz/
    │   ├── config/          # Layout and marker configurations
    │   ├── icons/           # UI icons for the RViz panels
    │   ├── include/amrviz/  # C++ Headers for custom RViz panels
    │   ├── launch/          # rviz_start.launch.py
    │   ├── python/          # Spoofers (OdometrySpoofer, DummyRobotSpoofer)
    │   ├── recipes/         # XML UI layouts
    │   ├── src/             # C++ Source for plugins (Apriltag, Diagnostics)
    │   └── plugins_description.xml  # Registers custom panels with RViz
    └── package.xml
```

---

## 🚀 How to Run the Digital Twin

To launch the visualizer and load the digital twin of the course:

```bash
source /opt/ros/humble/setup.bash
source ~/AMR/install/setup.bash
ros2 launch amrviz rviz_start.launch.py
```


---

## 🧪 Simulation & Spoofing

`amrviz` includes Python spoofing tools that let you simulate the fleet without turning on physical robots:

- **`DummyRobotSpoofer.py`** — Simulates the heartbeat and basic state of an AMR.
- **`OdometrySpoofer.py`** — Generates fake odometry and TF data to make the 3D models move in RViz as if executing a tour.
- **`MarkerPublisher.py`** — Renders the physical floor tiles and routes as visual markers in RViz.

These are useful for testing software changes without needing the physical lab setup.

---

## ⚠️ Development Notes

- **Custom RViz Panels:** If you modify `.ui` files in `amrviz/src` or add new C++ logic to the panels, you must recompile the `amrviz` package for the changes to appear in the RViz GUI.
- **Asset Paths:** If a robot model fails to load in RViz (appearing as a white box or throwing a mesh error), ensure the URDFs in `amr_roboware` are correctly pointing to `package://amr_meshes/` URIs.

---

## 🔗 Related Repositories

| Repo | Role |
|---|---|
| `amr_central` | Publishes the TF, state, and diagnostic data visualized here |
| `amr_roboware` | Provides URDF robot descriptions that reference `amr_meshes` |
| `amr_apriltag` | Publishes obstacle TF transforms visible in the digital twin |
| `amr_docs` | Full system documentation and lab manual |
