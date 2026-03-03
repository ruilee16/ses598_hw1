# LQR Control of Cart-Pole System Under Earthquake Disturbances

## 1. Introduction

This project evaluates a Linear Quadratic Regulator (LQR) controller
applied to a cart-pole system in a ROS2 + Gazebo simulation environment
under earthquake disturbances.

The system state vector is defined as:

x = \[x, x_dot, theta, theta_dot\]\^T

Where: - x: Cart position (m) - x_dot: Cart velocity (m/s) - theta: Pole
angle (deg in plots) - theta_dot: Pole angular velocity

### Control Objectives

1.  Maintain the pendulum in the upright position\
2.  Keep the cart within ±2.5 m physical limits\
3.  Achieve stable operation under earthquake disturbances

------------------------------------------------------------------------

## 2. LQR Design

The LQR cost function is defined as:

J = ∫ (x\^T Q x + u\^T R u) dt

Where:

Q = diag(q1, q2, q3, q4)\
R = scalar control penalty

Different Q weight configurations were tested to understand performance
trade-offs between stability, tracking, and control effort.

------------------------------------------------------------------------

## 3. Experimental Configurations

The following Q matrices were evaluated (R kept constant):

1.  Q = diag(\[1, 100, 1, 1\]) --- High cart velocity weight\
2.  Q = diag(\[1, 1, 1, 100\]) --- High angular velocity weight\
3.  Q = diag(\[100, 1, 1, 1\]) --- High cart position weight\
4.  Q = diag(\[1, 1, 100, 1\]) --- High pole angle weight\
5.  Default balanced configuration

Each case was tested under earthquake force disturbances.

------------------------------------------------------------------------

## 4. Results and Performance Analysis

### Case 1: High Cart Velocity Weight

Q = diag(\[1,100,1,1\])

-   Max cart displacement: 2.50 m (physical boundary reached)\
-   RMS cart position: 0.95 m\
-   Max pole angle: 34.84°\
-   RMS pole angle: 12.76°\
-   Peak control force: 111.24 N

**Observation:**\
Large pole oscillations occurred. The cart approached the ±2.5 m limit.
Emphasizing cart velocity alone does not sufficiently stabilize the
pendulum. Control effort increased significantly.

------------------------------------------------------------------------

### Case 2: High Angular Velocity Weight

Q = diag(\[1,1,1,100\])

-   Max cart displacement: 0.57 m\
-   RMS cart position: 0.20 m\
-   Max pole angle: 7.13°\
-   RMS pole angle: 3.02°\
-   Peak control force: 46.52 N

**Observation:**\
System remained stable for the full duration. Both cart displacement and
pole deviation were significantly reduced. Control effort remained
moderate. This configuration showed strong disturbance rejection and
balanced behavior.

------------------------------------------------------------------------

### Case 3: High Cart Position Weight

Q = diag(\[100,1,1,1\])

-   Max cart displacement: 2.50 m (boundary reached)\
-   RMS cart position: 1.03 m\
-   Max pole angle: 8.22°\
-   RMS pole angle: 3.67°\
-   Peak control force: 59.11 N

**Observation:**\
Pole stability improved compared to Case 1, but cart drift eventually
reached physical limits. Position weighting alone does not prevent
long-term drift under disturbances.

------------------------------------------------------------------------

### Case 4: High Pole Angle Weight

Q = diag(\[1,1,100,1\])

-   Max cart displacement: 2.50 m (boundary reached)\
-   RMS cart position: 1.06 m\
-   Max pole angle: 22.02°\
-   RMS pole angle: 7.67°\
-   Peak control force: 127.45 N

**Observation:**\
Increasing pole angle weight improved short-term upright stability but
caused aggressive control actions. Large control forces were required,
leading to boundary excursions.

------------------------------------------------------------------------

### Case 5: Alternative Configuration

-   Max cart displacement: 2.50 m\
-   RMS cart position: 1.08 m\
-   Max pole angle: 22.26°\
-   RMS pole angle: 8.70°\
-   Peak control force: 102.00 N

**Observation:**\
System stability was partially maintained, but cart displacement reached
limits and control effort remained high.

------------------------------------------------------------------------

## 5. Performance Trade-Off Analysis

### 1. Stability Duration

Only the high angular velocity weight configuration maintained
long-duration stable operation without reaching cart limits.

### 2. Maximum Cart Displacement

Several configurations reached the ±2.5 m boundary, indicating
insufficient constraint handling under strong disturbances.

### 3. Pole Angle Deviation

Increasing theta or theta_dot weights reduces angular oscillations but
may increase control aggressiveness.

### 4. Control Effort

Higher state penalties generally increase control effort. Excessive
control force can lead to actuator saturation and instability.

------------------------------------------------------------------------

## 6. Key Findings

-   Emphasizing angular velocity (theta_dot) provided the best balance
    between stability and control effort.
-   Increasing pole angle weight reduces deviation but increases peak
    control force.
-   Cart position weighting alone does not guarantee long-term bounded
    motion under persistent disturbances.
-   LQR tuning inherently involves trade-offs between disturbance
    rejection, actuator effort, and state regulation.

------------------------------------------------------------------------

## 7. Learning Outcomes

Through this project:

-   The effect of Q matrix tuning on closed-loop behavior was clearly
    observed.
-   Competing control objectives (upright stability vs. position limits
    vs. energy use) were analyzed.
-   The importance of disturbance robustness in practical control
    systems was demonstrated.
-   Practical implementation experience was gained using ROS2 and Gazebo
    simulation.

------------------------------------------------------------------------

## 8. Conclusion

The experiments demonstrate that LQR performance strongly depends on the
relative weighting of state variables in the Q matrix. Under earthquake
disturbances, prioritizing angular velocity provided the most stable and
energy-efficient behavior. However, no configuration perfectly enforced
cart position constraints, highlighting a limitation of pure LQR without
additional constraint handling or state augmentation.

Future improvements could include: - State constraint handling (e.g.,
MPC) - Actuator saturation modeling - Integral action to reduce drift -
Robust control design for disturbance rejection

------------------------------------------------------------------------

End of Report
