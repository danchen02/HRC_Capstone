# Natural Language Control of Cobots using LLMs

> **Author:** Daniel Chen  
> **Institution:** University of Technology Sydney  
> **Supervisor:** Dr. Graeme Best  
> **Date:** November 2025

## Overview

This project demonstrates how Large Language Models can enable intuitive, natural language control of collaborative robots in dynamic workstation environments. The system bridges the gap between human communication and robot control by allowing users to command a UR3e robot arm through conversational speech or text, without requiring any programming knowledge or rigid command syntax.

### The Problem

Traditional robot control systems require precise, structured commands and cannot handle the ambiguity and flexibility of natural human language. Existing human-robot collaboration (HRC) systems struggle with:
- Understanding vague or incomplete instructions
- Adapting to changes in the workspace
- Maintaining conversational context across multiple interactions
- Requesting clarification when commands are unclear

### The Solution

This system uses a modular architecture combining:
- **Speech-to-Text** (OpenAI Whisper) - Converts spoken commands to text
- **LLM Framework** (Kimi-K2-0905) - Interprets user intent and generates action plans
- **Motion Planning** (MoveIt + ROS2) - Executes collision-free robot movements
- **Dynamic Object Database** - Tracks workspace objects in real-time

Users can give natural commands like:
- *"Pick up the red cube and place it on the table"*
- *"Stack the yellow dice on the red dice"*
- *"What tools do I need to put in nails?"* → *"Can you pass it to me?"*
- *"Move that thing over there"* (system asks for clarification)

### Key Results

The system was validated across 5 realistic test scenarios (25 total trials):
- **88% overall task success rate** - Reliably completes manipulation tasks
- **0.63s average response time** - Fast enough for natural interaction
- **100% ambiguity handling** - Always requests clarification rather than guessing
- **100% dynamic adaptation** - Successfully responds to workspace changes
- **60% context retention** - Handles most follow-up questions (primary limitation)

### Research Contribution

This work addresses the research question: *"How can Large Language Models be leveraged to enable adaptive and intuitive human-robot collaboration with cobots in dynamic workstation environments through real-time natural language interpretation?"*

The project demonstrates that LLMs can successfully bridge natural language and robot control, though limitations remain in multi-turn context retention and spatial reasoning accuracy.

---

## Features

- 🎙️ Speech-to-text input using OpenAI Whisper
- 💬 Conversational context with 5-turn memory
- 🤖 LLM-powered intent interpretation (Kimi-K2-0905)
- 🦾 MoveIt motion planning with collision avoidance
- 🔄 Real-time object database updates
- 🏠 Safety features and error recovery

---

## System Architecture

```
User Input (Speech/Text)
    ↓
LLM Framework (Intent Interpretation)
    ↓
API Bridge (Parse Commands)
    ↓
Action Library (High-Level Actions)
    ↓
Motion Planner (MoveIt + ROS2)
    ↓
UR3e Robot + RG2 Gripper
```

**Supported Actions:** MOVE, PICK, PLACE, SCAN, HOME, GRIPPER, WAIT

---

## Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- Groq API Key (https://console.groq.com/)

**Optional Hardware:**
- Universal Robots UR3e
- OnRobot RG2 Gripper

---

## Installation

### 1. Install ROS2 Humble & MoveIt2

```bash
# Install ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions -y

# Install MoveIt2
sudo apt install ros-humble-moveit -y

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Setup Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone this repository
git clone https://github.com/danchen02/HRC_Capstone.git

# Clone dependencies
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
git clone https://github.com/tonydle/UR_OnRobot_ROS2.git

# Build
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Install Python Dependencies

```bash
cd ~/ros2_ws/src/HRC_Capstone
pip install groq pyaudio pyyaml numpy
```

### 4. Configure API Key

Create `.env` file in project root:
```
GROQ_API_KEY=your_api_key_here
```

---

## Quick Start

### Running in Simulation

```bash
# Terminal 1: Launch fake robot
ros2 launch ur_robot_driver ur3.launch.py \
  robot_ip:=xxx.xxx.xxx.xxx \
  use_fake_hardware:=true \
  launch_rviz:=false

# Terminal 2: Launch MoveIt
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3 \
  launch_rviz:=true

# Terminal 3: Run control system
cd ~/ros2_ws/src/HRC_Capstone
python3 main.py
```

### Using the System

The GUI will open with:
- **Speech Input:** Click "Start Recording" to speak commands
- **Text Input:** Type commands directly
- **Feedback:** View responses in the communication feed

**Example Commands:**
```
"Go to home position"
"Pick up the red cube"
"Place it at 0.3, 0.2, 0.1"
"What objects are in the workspace?"
```

---

## Configuration

### Object Database (`config/objects.yaml`)

```yaml
cube_001:
  name: cube
  description: red cube
  position: {x: 0.3, y: 0.0, z: 0.03}
  dimensions: {length: 0.05, width: 0.05, height: 0.05}
  properties:
    graspable: true
    color: red
```

---

## Project Structure

```
HRC_Capstone/
├── config/
│   ├── objects.yaml          # Object database
│   └── workspace_config.yaml # Workspace limits
├── src/
│   ├── main.py              # Entry point
│   ├── gui_manager.py       # User interface
│   ├── llm_manager.py       # LLM integration
│   ├── api_bridge.py        # Command parser
│   ├── action_library.py    # Action primitives
│   └── motion_planner.py    # MoveIt interface
├── docs/
│   ├── Final_Report.pdf
│   └── Capstone_Presentation.mp4
└── README.md
```

---

## Known Limitations

- Context retention: 60% success across multi-turn conversations
- Requires accurate perception data for successful manipulation
- Fixed downward-facing gripper orientation
- Cannot replan mid-execution if objects move

---

## Contact

**Daniel Chen**  
University of Technology Sydney

