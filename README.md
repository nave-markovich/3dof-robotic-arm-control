# 3dof-robotic-arm-control
Kinematic modeling, dynamic equations of motion derivation, and artificial potential-field control simulation for a 3-DOF robotic manipulator in MATLAB &amp; Mathematica.

# 3-DOF Robotic Arm Navigation & Control Simulation

## Robots Motion Planning and Control Final Project
**Department of Mechanical Engineering, Ben-Gurion University of the Negev**[cite: 3]  
**Authors:** Nave Markovich, Sharon Guberg, and Bar Mizrachi[cite: 3]

---

## 📂 Project Resources & Quick Links
* **[📂 Full Google Drive Folder](YOUR_GOOGLE_DRIVE_LINK_HERE)** (Contains simulation output videos, legacy files, and high-res figures)[cite: 4]
* **[📄 Project Documentation (PDF)](./documents/Final_project_Control_Navigation.pdf)** (Detailed supplementary specifications)[cite: 3]

---

## 🔬 Project Overview
This project presents a comprehensive framework for the modeling, simulation, and nonlinear control of a **3-Degree-of-Freedom (3-DOF) serial robotic manipulator** (inspired by the Arctos industrial robot architecture)[cite: 3]. 

The workflow spans from full symbolic kinematic and dynamic derivation using **Wolfram Mathematica** to closed-loop numerical integration and controller validation in **MATLAB (ode45)**[cite: 3].

<p align="center">
  <img src="./assets/robot_model.png" alt="3-DOF Robot Arm Model" width="600">
</p>

---

## ⚙️ Mathematical & Dynamic Modeling (Wolfram Mathematica)

### 1. Kinematics & Jacobians[cite: 3]
The generalized coordinate state vector is defined by the joint angles $q(t) = [\theta_1(t), \theta_2(t), \theta_3(t)]^T$[cite: 3]. Linear and angular Jacobians ($J_{L_i}, J_{\omega_i}$) were derived relative to each link's center of mass (CoM) to trace linear velocities ($\dot{p}_{c_i} = J_{L_i}\dot{q}$) and body rotations[cite: 3, 5].

### 2. Equations of Motion[cite: 3]
Using the local inertia tensors and rotation matrices ($R_i$), the full rigid-body dynamic behavior was formulated via the joint-space equations of motion[cite: 3, 5]:

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

Where[cite: 3]:
* $M(q)$ is the symmetric $3\times3$ mass/inertia matrix[cite: 3, 5].
* $C(q,\dot{q})$ captures the nonlinear Coriolis and centrifugal effects[cite: 3, 5].
* $G(q)$ is the gravity vector projecting link weights into generalized forces[cite: 3, 5].
* $\tau$ is the vector of input torques applied at the revolute joints[cite: 3, 5].

---

## 🚀 Control Strategy: Artificial Potential Field
To steer the end-effector to a desired Cartesian target without persistent oscillations, a nonlinear controller was developed based on an **Artificial Potential Function ($U$)** coupled with velocity damping ($K_d$)[cite: 3]:

$$\tau(q) = G(q) - \nabla U_{\text{att}}(q) - K_d\dot{q}$$

* **Gravity Compensation:** $G(q)$ actively counteracts the manipulator's structural weight[cite: 3].
* **Attractive Potential Field:** $U_{\text{att}} = \frac{1}{2}(q-q_d)^T K_{\text{att}}(q-q_d)$ establishes a unique global minimum at the target joint configuration ($q_d$), creating restoring torques proportional to position errors[cite: 3].
* **Damping Term:** $-K_d\dot{q}$ dissipates kinetic energy to guarantee asymptotic convergence analyzed via Lyapunov stability theory[cite: 3].

---

## 📂 Repository Structure & How to Run

To ensure proper execution, MATLAB functions depend on one another. Code is organized into dedicated source directories:

* `src/zero_input_dynamics/` - MATLAB files for simulating natural system dynamics without control input[cite: 4].
* `src/potential_field_control/` - MATLAB files for the closed-loop controller driving the arm toward a Cartesian target[cite: 4].
* `src/symbolic_derivation/` - Contains the full symbolic derivation notebook (`Full_Kinematics.nb`)[cite: 4].
* `data/` - Contains raw arm parameter matrices and base data[cite: 4].

*(Note: Download all `.m` files within a specific `src` directory together before executing the main script)*[cite: 4].

---

## 📊 Simulation Output Naming Convention
External simulation videos and graphs (hosted on the connected Google Drive) follow a strict naming convention to describe the motion parameters[cite: 4]:

**Format:** `start_<theta1>_<theta2>_<theta3>_end_<x>_<y>_<z>`[cite: 4]
* **Start values** represent initial joint angles in degrees[cite: 4].
* **End values** represent the desired Cartesian target point (in meters)[cite: 4].
* The letter `m` denotes a minus sign ($-$). E.g., `m04` = $-0.4$[cite: 4].
* A leading `0` represents a decimal. E.g., `02` = $0.2$[cite: 4].
* `Deff` implies the predefined default target point was used[cite: 4].

*Example:* `start_0_90_0_end_02_m04_m001` corresponds to an initial configuration of $\theta=[0^\circ, 90^\circ, 0^\circ]$ and a target of $x=0.2, y=-0.4, z=-0.01$[cite: 4].

---

## 🛠️ Tech Stack & Tools Used
* **MATLAB / Simulink:** Numerical integration (`ode45`), closed-loop simulation, and stick-figure trajectory animation[cite: 3].
* **Wolfram Mathematica:** Symbolic derivation of center-of-mass positions, Jacobians, and matrix differentiations[cite: 3, 5].
* **SolidWorks:** 3D mechanical assembly, rigid-link parameter extractions (mass, length, inertia tensors)[cite: 3].
