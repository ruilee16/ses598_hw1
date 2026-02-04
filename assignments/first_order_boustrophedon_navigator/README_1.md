# First Order Boustrophedon Navigator

## Final Parameter Values & Justification

After a two-stage tuning process involving automated optimization and real-time manual refinement, the following PID parameters were selected. These values transition the robot from the unstable spiral behavior (Pic 1) to the stable rectangular path (Pic 2).

| Parameter | Value | Justification |
| :--- | :--- | :--- |
| **`Kp_linear`** | **1.0** | Reduced from the optimizer's suggestion (5.0) to **1.0**. High linear gain caused the robot to rush into turns with too much momentum, making it uncontrollable [1]. A lower value ensures steady, manageable speed. |
| **`Kd_linear`** | **0.5** | Standard damping to prevent linear overshoot when approaching waypoints. |
| **`Kp_angular`** | **4.5** | **The Turning Fix.** The default value (1.0) resulted in a turn radius of ~1.27m, which is physically too large for the 0.5m spacing [2, 3]. Increasing this to 4.5 forces the robot to turn sharply enough to fit within the lane. |
| **`Kd_angular`** | **4.0** | **The Stabilizer.** Significantly increased to **4.0** to act as a "shock absorber." This eliminates the violent "shaking and twisting" caused by the sharp turning commands, which the optimizer failed to account for. |
| **`spacing`** | **0.5** | Kept constant to define the survey density [2]. |

## Performance Metrics & Analysis

### Trajectory Comparison
*   **Baseline (Figure 1):** With default parameters ($K_p=1.0$), the robot failed to close the loop on turns, resulting in an infinite "Spiral of Death" where the turn radius exceeded the lane spacing [4].
*   **Optimized (Figure 2):** With the final tuned parameters, the robot successfully executes 180-degree turns within the 0.5m spacing constraint, producing a clean Boustrophedon (lawnmower) pattern [5].

### Error Analysis
*   **Cross-Track Error:** The error spikes momentarily during the instantaneous 180-degree turns but rapidly converges to near-zero on straight legs due to the stable linear gain [6].
*   **Velocity Profile:** The robot maintains a consistent speed on straights and executes rapid, high-velocity rotations (spikes in angular velocity) to maintain the path [7].

## Tuning Methodology

The tuning process revealed significant discrepancies between theoretical simulation and real-time control.

### 1. Automated Optimization (boustrophedon_optimizer.py)
We developed a custom script named `boustrophedon_optimizer.py` using the **Optuna** library to search for optimal gains [8].
*   **Method:** The script mathematically simulated the controller logic over 5,000 trials [9].
*   **Result:** The optimizer suggested extremely aggressive gains (e.g., `Kp_linear` $\approx$ 5.0, `Kp_angular` $\approx$ 9.2) [10].
*   **Failure Mode:** When applied to the actual ROS 2 node, these parameters performed poorly ("trash"). The robot struggled to move in a straight line and exhibited severe "shaking and twisting." The mathematical simulation in `boustrophedon_optimizer.py` did not account for the real-world latency, message delays, and physical inertia of the Turtlesim node [11].

### 2. Real-Time Manual Refinement (ros2 param set)
To fix the instability, we switched to manual tuning using the ROS 2 CLI while the controller was running.
*   **Step 1:** We utilized the `parameter_callback` feature in the controller to adjust gains on the fly [12].
*   **Step 2:** We lowered `Kp_linear` from 5.0 to 1.0 to reduce the robot's kinetic energy entering turns.
*   **Step 3:** We drastically increased `Kd_angular` to 4.0. This provided the necessary damping to stop the oscillation (shaking) induced by the high turning gain.
*   **Step 4:** We found `Kp_angular` = 4.5 was the "sweet spot"—high enough to make the tight U-turn, but low enough to remain stable.

## Challenges and Solutions

### Challenge 1: The "Spiral of Death"
**Issue:** The robot spiraled outward instead of following the lanes [4].
**Solution:** Mathematical analysis showed the turn radius $R = v / \omega$ was too large. We increased `Kp_angular` to increase $\omega$, forcing $R < 0.25m$.

### Challenge 2: Optimizer Instability
**Issue:** `boustrophedon_optimizer.py` produced parameters that caused severe oscillation.
**Solution:** We treated the optimizer results as a "theoretical upper bound" and manually added significant derivative damping (`Kd`) to compensate for the unmodeled dynamics.

### Challenge 3: Environment Restrictions
**Issue:** Installing `optuna` failed due to "externally managed environment" errors on Linux.
**Solution:** We utilized `pip install optuna --break-system-packages` to enable the optimization script in the development environment.
