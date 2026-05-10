# Theo Jansen Mechanism Simulation using MuJoCo

A Python-based simulation and visualization of the **Theo Jansen walking mechanism** using **MuJoCo**, analytical kinematics, and trajectory analysis.

This project recreates the famous Theo Jansen leg mechanism and visualizes:
- Linkage motion
- Foot trajectory
- Crank-driven walking cycle
- MuJoCo interactive rendering
- Analytical kinematic replay

---

# 📌 Project Overview

The Theo Jansen mechanism is a linkage system designed to create smooth walking motion using rotating cranks and interconnected rods.

In this project:
- The mechanism kinematics are solved analytically using **circle-circle intersection methods**
- The motion is replayed inside **MuJoCo**
- Foot trajectories and walking characteristics are visualized using **Matplotlib**
- Both static plots and animated simulations are generated

---

# 🛠️ Technologies Used

- Python
- MuJoCo
- NumPy
- Matplotlib
- ImageIO

---

# 📂 Features

✅ Analytical kinematic solution of Theo Jansen linkage  
✅ MuJoCo visualization and interactive viewer  
✅ Static trajectory plotting  
✅ Animated walking cycle generation  
✅ Foot trajectory analysis  
✅ Link orientation using quaternion rotations  
✅ GIF generation for motion visualization  

---

# 📸 Simulation Outputs

## Mechanism Configuration

### Model

![Initial Pose](./model.png)

---

## Foot Trajectory and Pose Analysis

![Trajectory Analysis](./jansen_mujoco.png)

This plot shows:
- Foot trajectory during one full crank cycle
- Stance phase
- Step height
- Stride length
- Mechanism pose at 90° crank angle

---

# 🧪 Early Simulation Attempts

These were some of the initial simulation attempts while building the mechanism geometry and validating linkage behavior.

## Prototype Attempt 1

![Prototype 1](./earlier_distorted_model.png)

---

# ⚙️ How the Simulation Works

The mechanism is generated using:
- Fixed linkage lengths (Theo Jansen “holy numbers”)
- Analytical geometry
- Circle-circle intersection for joint solving
- Quaternion-based link alignment for MuJoCo rendering

The code computes:
- Joint coordinates
- Link orientations
- Foot trajectory
- Frame-by-frame animation

---

# 📚 Reference & Acknowledgement

This project was created primarily for:
- Learning kinematics
- Exploring mechanism design
- Understanding MuJoCo visualization workflows

We would also like to mention that:
- Some portions of the implementation and debugging assistance were completed with the help of AI tools during development.
- The project includes both original work and AI-assisted coding support used for learning and experimentation purposes.

---


