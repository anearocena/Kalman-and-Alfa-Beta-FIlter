# Kalman-and-Alfa-Beta-FIlter
MATLAB implementation and comparison of Alpha-Beta and Kalman filters for aircraft trajectory tracking and noise mitigation in Secondary Surveillance Radar (SSR) data.
# Alpha-Beta and Kalman Filtering for Aircraft Tracking

[cite_start]This project implements and compares two filtering algorithms (**Alpha-Beta** and **Kalman Filter**) to estimate aircraft position and velocity based on noisy measurements from a Secondary Surveillance Radar (SSR)[cite: 39]. [cite_start]This work was developed for the *Positioning, Guidance, and Control* course at **ETSIAE (UPM)**[cite: 1, 11].

## 📋 Problem Description

[cite_start]The objective is to reconstruct the real trajectory of an aircraft by mitigating inherent radar measurement errors[cite: 39]. [cite_start]A "racetrack" trajectory including straight segments and turns is simulated, Gaussian noise is added to represent the sensor, and filters are applied to compare performance[cite: 162].

### Simulation Scenario
The simulation parameters are defined as follows:
* [cite_start]**Aircraft Velocity:** Constant at 250 kts (~128 m/s)[cite: 19].
* [cite_start]**Radar Update Rate (SSR):** Measurements every 4 seconds ($dt=4s$)[cite: 27].
* **Trajectory:**
    1.  [cite_start]Radial approach ($90^\circ$)[cite: 20].
    2.  [cite_start]Right-hand $180^\circ$ turn (5000m radius)[cite: 21].
    3.  [cite_start]Outbound leg and second turn for return[cite: 22, 23].
* **Sensor Noise (SSR):**
    * [cite_start]Distance error ($\sigma_\rho$): 200 m[cite: 25].
    * [cite_start]Angular error ($\sigma_\alpha$): $0.3^\circ$[cite: 26].

## ⚙️ Technologies and Algorithms

The code is developed entirely in **MATLAB** and addresses two approaches:

### 1. Alpha-Beta Filter ($\alpha-\beta$)
[cite_start]A simplified filter that assumes constant velocity and treats the X and Y coordinates independently[cite: 40].
* [cite_start]**Pros:** Low computational cost, simple implementation[cite: 549].
* [cite_start]**Cons:** Exhibits "lag" (delay) during maneuvers and uses fixed gains that do not adapt to signal-to-noise ratio changes[cite: 549, 550].
* [cite_start]**Parameters:** Optimal $\alpha$ and $\beta$ were calculated based on the assumed process deviation ($\sigma_s = 250 m/s^2$) and measurement noise[cite: 95].

### 2. Kalman Filter
[cite_start]A recursive optimal estimator that models the full system state ($[x, y, v_x, v_y]$) and its uncertainties using covariance matrices[cite: 45].
* [cite_start]**Initial Covariance Matrix ($P_0$):** Initialized with high values ($50,000$) to reflect initial position uncertainty[cite: 51].
* [cite_start]**Process Noise Matrix ($Q$):** Models the system's flexibility regarding maneuvers or wind gusts (imprecision of 250m in position and 0.1 m/s in velocity)[cite: 63].
* [cite_start]**Pros:** Better response in turns, superior trajectory smoothing, and dynamic gain adaptation via covariance coupling between axes[cite: 551, 552].

## 📊 Results and Conclusions

Upon analyzing the simulation results and graphs:
1.  [cite_start]**Precision:** The Kalman filter provides a significantly smoother estimation closer to the real trajectory, especially during $180^\circ$ turns where the Alpha-Beta filter tends to react with a delay[cite: 552, 559].
2.  [cite_start]**Performance:** Although the Alpha-Beta filter reduces noise, its lack of correlation between axes makes it less suitable for sharp maneuvers[cite: 550].
3.  [cite_start]**Usage:** The Kalman filter is recommended for critical tracking tasks requiring precision and robustness, reserving Alpha-Beta for systems with resource constraints[cite: 560].

## 🚀 Installation and Usage

1.  Clone the repository:
    ```bash
    git clone [https://github.com/your-username/alpha-beta-kalman-filter.git](https://github.com/your-username/alpha-beta-kalman-filter.git)
    ```
2.  Open MATLAB.
3.  Run the main script (e.g., `main.m` or `filter_ab_kalman.m`).
4.  The script will automatically generate:
    * The simulation of the true vs. measured trajectory.
    * Comparative plots of both filters' performance.

## 👤 Author

[cite_start]**Ane Arocena Blanco** [cite: 11]
Universidad Politécnica de Madrid (UPM)  
Escuela Técnica Superior de Ingeniería Aeronáutica y del Espacio (ETSIAE)
