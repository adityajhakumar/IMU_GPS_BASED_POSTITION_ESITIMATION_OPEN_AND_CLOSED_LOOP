# 🚀 Autonomous Navigation Simulation Using Potential Fields and Kalman Filter

## 📌 Overview

This simulation demonstrates autonomous navigation using **Artificial Potential Fields (APF)** for path planning and **obstacle avoidance**, integrated with a **Kalman Filter** for state estimation and **depth recognition** for obstacle evaluation.

The system simulates a robot navigating from a start point to a goal point while avoiding multiple dynamic/static obstacles. The forces from attractive and repulsive fields guide the agent's movement.

---

## 🎯 Objective

To simulate an agent that:
- Navigates from a **start position to a goal** using attractive forces.
- Avoids **nearby obstacles** using repulsive forces.
- Maintains robust **position estimation** using a **Kalman Filter**.
- Dynamically assesses obstacle proximity through **depth recognition**.

---

## 🧠 Core Concepts

### 1. **Artificial Potential Fields (APF)**

APF is inspired by **classical mechanics**. The agent behaves like a particle in a force field:
- **Goal** acts like an **attractive well**.
- **Obstacles** act like **repulsive barriers**.

#### 🧲 Attractive Force:

This force pulls the agent towards the goal.

```text
F_att = k_att * (p_goal - p_agent)
```

Where:
- `F_att` is the attractive force vector.
- `k_att` is the attraction gain (positive scalar).
- `p_goal` is the goal position vector.
- `p_agent` is the current agent position.

#### 🧱 Repulsive Force:

This force pushes the agent away from obstacles that are within a **threshold distance** `d0`.

```text
F_rep = k_rep * (1/d - 1/d0) * (1/d^2) * (p_agent - p_obs) / ||p_agent - p_obs||, if d < d0
F_rep = 0, otherwise
```

Where:
- `F_rep` is the repulsive force.
- `k_rep` is the repulsion gain.
- `d` is the distance between agent and obstacle.
- `p_obs` is the position of the obstacle.

This causes the agent to avoid collisions with nearby obstacles.

#### 🧮 Net Force:

```text
F_net = F_att + ∑ F_rep
```

The agent moves in the direction of `F_net`.

---

### 2. **Simulation Steps (Sample)**

Below is an excerpt from the simulation log at step `668–671`:

```
At position: [ 414272.9505 1431352.9050 ]
Attractive force: [-6.0907   6.4792 ]
Repulsive force:  [ 6.3551  -6.4722 ]
Net force:        [ 0.2644   0.0070 ] → Repulsion dominates
```

The agent is **closer to an obstacle**, so the **repulsive force is stronger**, temporarily pushing the agent off its direct path to the goal.

As the agent moves further:

```
At position: [ 414273.8885 1431352.8891 ]
Attractive force: [-7.0287   6.4951 ]
Repulsive force:  [ 5.6594  -5.5223 ]
Net force:        [-1.3693   0.9728 ] → Attraction dominates
```

Now, the attractive force starts to regain control as the obstacle influence reduces.

---

## 📍 Coordinate System Used

The simulation uses **UTM (Universal Transverse Mercator)** projected coordinates:

```text
Easting  (X): ~414272 to ~414508 meters
Northing (Y): ~1431352 to ~1431405 meters
```

UTM coordinates provide **accurate position mapping** over small regions and are ideal for GPS-based robot movement simulation.

---

## 🎛️ Kalman Filter for Position Estimation

The agent may experience **sensor noise** or **uncertain motion**. To mitigate this, a **Kalman Filter** is used.

### 🔄 Prediction Step:
```text
x̂ₖ⁻ = A * x̂ₖ₋₁ + B * uₖ
Pₖ⁻ = A * Pₖ₋₁ * Aᵀ + Q
```

### 🧮 Update Step:
```text
Kₖ = Pₖ⁻ * Hᵀ * (H * Pₖ⁻ * Hᵀ + R)⁻¹
x̂ₖ = x̂ₖ⁻ + Kₖ * (zₖ - H * x̂ₖ⁻)
Pₖ = (I - Kₖ * H) * Pₖ⁻
```

Where:
- `x̂`: estimated state
- `P`: error covariance
- `K`: Kalman Gain
- `A, B, H`: system matrices
- `Q, R`: noise covariances

📚 **Reference**: Kalman, R.E. (1960), *A New Approach to Linear Filtering and Prediction Problems*.

---

## 🧠 Depth Recognition and Obstacle Activation

Obstacles are only considered **active** if within a certain distance `d0`:

```text
if distance_to_obstacle < d0:
    activate_repulsion()
else:
    ignore_obstacle()
```

This helps in reducing unnecessary computation and mimics **depth-aware obstacle avoidance**, as seen in **stereo vision**, **LiDAR**, or **depth cameras**.

---

## 💡 Real-World Analogy

Imagine a person walking through a forest:
- The goal is a **light at the end** of the path (attractive force).
- Trees are **obstacles** — the closer they are, the stronger the need to move away (repulsive force).
- The person uses their **vision and memory** to filter out noise and maintain the correct direction (Kalman filter).
- Their **depth perception** helps them ignore trees that are far away.

---

## 🛠️ How Simulation is Run (Main Loop Pseudocode)

```python
while not at_goal(agent_position):
    F_att = compute_attractive_force(agent_position, goal_position)
    F_rep_total = Vector(0, 0)
    
    for obstacle in obstacles:
        if distance(agent_position, obstacle) < repulsion_threshold:
            F_rep_total += compute_repulsive_force(agent_position, obstacle)

    F_net = F_att + F_rep_total
    agent_position = update_position(agent_position, F_net)

    # Optional: Use Kalman filter to estimate the true state
    x̂ = kalman_filter.predict_and_update(agent_position)

    render(agent_position, F_net)
```

---

## 📘 References

1. **Artificial Potential Fields for Robot Navigation**:
   - Khatib, O. (1985). *Real-Time Obstacle Avoidance for Manipulators and Mobile Robots*.
2. **Kalman Filter**:
   - Kalman, R.E. (1960). *A New Approach to Linear Filtering and Prediction Problems*.
3. **Depth Recognition**:
   - Saxena, A., Chung, S.H., Ng, A.Y. (2008). *3D Depth Reconstruction from a Single Still Image*.
4. **Obstacle Avoidance**:
   - Borenstein, J., Koren, Y. (1991). *The Vector Field Histogram — Fast Obstacle Avoidance for Mobile Robots*.

---

## 🧪 Sample Output Analysis

Each step includes:
- Computation of distance to each obstacle
- Check whether the obstacle is active (based on threshold)
- Computation of attractive and repulsive forces
- Final decision of motion based on net force vector

This gives a **transparent view into the decision-making process** of the autonomous agent.

---

## 📌 Final Remarks

This README serves as a **complete theoretical and practical documentation** of the simulation logic. It is intended to make the methodology behind path planning using potential fields, depth estimation, and Kalman filtering understandable to students, researchers, and developers alike.

Feel free to adapt or extend the model to 3D environments, dynamic goals, or real sensor data!
