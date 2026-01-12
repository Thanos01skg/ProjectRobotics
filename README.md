<div align="center">

  <h1>🤖 Inverse Kinematics Robot Simulator</h1>
  
  <p>
    <b>A 2-DOF Planar Robotic Arm Simulator with Path Planning & Workspace Visualization.</b>
  </p>

  <img src="https://img.shields.io/badge/Robotics-Simulation-blue?style=for-the-badge&logo=robot" alt="Robotics Badge" />
  <img src="https://img.shields.io/badge/Language-Python-yellow?style=for-the-badge&logo=python" alt="Python Badge" />
  <img src="https://img.shields.io/badge/Status-Educational-green?style=for-the-badge" alt="Status Badge" />

  <br />
  <br />
</div>

---

## 📝 Project Overview

A simulation software that solves the **Inverse Kinematics** problem for a 2-DOF (Degree of Freedom) planar robotic arm. This project visualizes the robot's workspace, calculates joint angles based on Cartesian coordinates, and simulates valid straight-line movements.

This application is designed to demonstrate the mathematical and logical principles of robotics. It takes user-defined parameters for a robotic arm and performs real-time calculations to determine valid configurations and paths.

### 🔑 Key Functionalities
* **Workspace Visualization:** Renders the internal and external boundaries of the robot's reach.
* **Inverse Kinematics Solver:** Calculates the necessary joint angles to reach a specific $(x, y)$ target.
* **Path Validation:** Ensures that requested movements are geometrically feasible before execution.
* **Animation:** Simulates the movement of the arm from start to target positions.

---

## 🚀 Features

### 1. Dynamic Configuration & Input
The user can define the physical properties of the robot:
* **Link Lengths ($L_1, L_2$):** Input specific lengths for the two arm segments. The system validates these values to ensure they fit within the screen dimensions.
* **Arm Configuration:** Choose between **"Lefty"** (Left-handed) or **"Righty"** (Right-handed) elbow configurations.
* **Initial Position:** Define the starting $(x, y)$ coordinates of the end-effector.

### 2. Visualization Engine
The simulation renders a graphical representation of the robot's environment:
* **Cartesian Coordinate System:** Draws $X$ and $Y$ axes centered on the screen.
* **Work Envelopes:** Visually displays the **External Job Folder** (max reach) and **Internal Job Folder** (min reach) to indicate valid workspaces.

### 3. Kinematics Logic
* **Solution Calculation:** The software computes two possible sets of joint angles (solutions) for any given coordinate—one for a "left" elbow and one for a "right" elbow.
* **Solution Selection:** It automatically filters and applies the correct solution based on the user's initial configuration choice.
* **Boundary Checks:** Prevents the user from inputting coordinates that are physically unreachable (outside the external envelope or inside the internal envelope).

### 4. Movement & Animation
* **Target Input:** Users can input a target coordinate $(x_t, y_t)$ for the robot to move toward.
* **Straight-Line Path Planning:** The system calculates a straight-line trajectory from the start point to the end point.
* **Feasibility Check:** Before moving, the algorithm validates the entire path. If the straight line crosses through a "blind spot" (e.g., the internal void) or is otherwise impossible, the system alerts the user and requests new coordinates.
* **Live Animation:** If the path is valid, the robot smoothly animates along the trajectory to the target.

---

## 🛠️ Technical Concepts

<div align="center">
  </div>

| Concept | Description |
| :--- | :--- |
| **Inverse Kinematics (IK)** | Calculating variable joint parameters (angles) needed to place the end-effector at a specific position and orientation. |
| **Singularity Handling** | Managing points where the robot loses a degree of freedom (e.g., fully extended arm). |
| **Geometric Constraints** | Validating that $\sqrt{x^2 + y^2} \le L_1 + L_2$ and $\sqrt{x^2 + y^2} \ge \lvert L_1 - L_2 \rvert$. |

<br>

<div align="center">
  <sub>📌Made for Production and Management Engineering (Robotics Studies)</sub>
</div>
