# autonomous_transport_bot
Design an autonomous robot that follows you. People don't want to carry things in everyday settings Whether you are shopping going to the beach or are tired of carrying yourself

## 🔧 Features

- SLAM-based localization and navigation
- Dynamic path planning + person-following
- Modular hardware with omnidirectional wheels
- FastAPI-based backend logging
- Plotly dashboard for live telemetry
- ROS2 + Gazebo simulation support

## 📁 Project Structure

| Folder | Description |
|--------|-------------|
| `ros2_ws/` | ROS2 workspace with robot URDF, control nodes |
| `backend/` | FastAPI app for data logging and API endpoints |
| `dashboard/` | Dash UI for telemetry and control visualization |
| `hardware/` | Schematics, microcontroller scripts |
| `docs/` | Project roadmap, diagrams, architecture |

## 🛠️ Getting Started

### Dependencies

- Ubuntu 22.04
- ROS2 Humble
- Docker (optional)
- Python 3.10

```bash
# Clone and build ROS2 workspace
cd ros2_ws && colcon build

# Run FastAPI backend
cd ../backend && uvicorn app:app --reload

# Run dashboard
cd ../dashboard && python3 dash_app.py
