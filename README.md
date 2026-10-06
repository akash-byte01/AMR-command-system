# AcuNav AMR Control System — Industrial UI/UX Redesign

Next-Generation Autonomous Mobile Robot (AMR) control console redesign inspired by modern industrial control rooms and clean SaaS design principles.

## 🎯 Redesign Objectives
- **5-Second Operator Comprehension**: Instantly reveals whether the robot is connected & healthy, where it is located, what it is doing, and what safe actions can be performed.
- **Progressive Disclosure**: Replaces the information-dense layout with a focused default screen (Top status bar, 4 core KPI cards, large interactive canvas map, and quick actions), while moving deep configurations into dedicated workspaces.
- **100% Feature Parity**: Retains all ROS 2 Nav2 navigation logic, SLAM lifecycle controls, safety interlocks, manual jog teleoperation, waypoints, and fixed routes without modification to underlying backend mechanics.

---

## 🛠️ System Architecture

### 1. Left Navigation Sidebar (Collapsible)
- **Dashboard**: Primary high-level operator monitoring view.
- **Map & SLAM**: Cartographer & `slam_toolbox` lifecycle, occupancy map import/export, map editor.
- **Navigation**: Contextual point-to-point dispatch wizard, waypoint sequencing, round trips.
- **Manual Control**: Teleoperation jogger with D-Pad, giant STOP button, and linear/angular speed limiters.
- **Routes**: Saved corridor routes, live route drawing, node reversal, and along-path execution.
- **Saved Locations**: POI station cards (Home Base, Charging Dock, custom bays) with 1-click dispatch.
- **Telemetry & Sensors**: 50Hz ODOM/EKF state, SICK picoScan 150 LiDAR health, TF tree.
- **History & Logs**: System audit trail with searchable event filters.
- **Settings**: AMR IP endpoints, WebSocket bridge configuration, and collision padding.

### 2. Persistent Top Status Bar
- **AMR Identification**: Robot ID & model label.
- **Connection & State**: ROS 2 verified connection indicator, robot lifecycle state (`IDLE`, `MOVING`, `NAVIGATING`, `PAUSED`, `MANUAL`).
- **Dynamic Safety Badges**: Flashing indicators for active robot motion or manual override.
- **Critical Telemetry**: IP address, battery gauge with voltage, and persistent **EMERGENCY SAFE STOP**.

### 3. Interactive Hero Map
- 2D Canvas rendering the warehouse blueprint, obstacles, landmark waypoints, and planned path.
- Floating quick tools: `[Fit]`, `[North Up]`, `[Set Pose]`, `[Set Goal]`, `[Map Tools]`.
- Live navigation HUD drawer displaying remaining distance, current velocity, ETA, and pause/abort controls.

---

## 🚀 Getting Started

Open `index.html` directly in any modern web browser or serve locally:

```bash
# Using PowerShell
$listener = New-Object System.Net.HttpListener; $listener.Prefixes.Add("http://localhost:8080/"); $listener.Start();
# Or open directly
Start-Process "index.html"
```
