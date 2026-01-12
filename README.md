# Kalman-and-Alfa-Beta-FIlter
MATLAB implementation and comparison of Alpha-Beta and Kalman filters for aircraft trajectory tracking and noise mitigation in Secondary Surveillance Radar (SSR) data.
# Alpha-Beta and Kalman Filtering for Aircraft Tracking

This project implements and compares two filtering algorithms (**Alpha-Beta** and **Kalman Filter**) to estimate aircraft position and velocity based on noisy measurements from a Secondary Surveillance Radar (SSR). This work was developed for the *Positioning, Guidance, and Control* course at **ETSIAE (UPM)**.

## 📋 Problem Description

The objective is to reconstruct the real trajectory of an aircraft by mitigating inherent radar measurement errors. A "racetrack" trajectory including straight segments and turns is simulated, Gaussian noise is added to represent the sensor, and filters are applied to compare performance.

### Simulation Scenario
The simulation parameters are defined as follows:
* **Aircraft Velocity:** Constant at 250 kts (~128 m/s).
* **Radar Update Rate (SSR):** Measurements every 4 seconds ($dt=4s$).
* **Trajectory:**
    1.  Radial approach ($90^\circ$).
    2.  Right-hand $180^\circ$ turn (5000m radius).
    3.  Outbound leg and second turn for return.
* **Sensor Noise (SSR):**
    * Distance error ($\sigma_\rho$): 200 m.
    * Angular error ($\sigma_\alpha$): $0.3^\circ$.

## ⚙️ Technologies and Algorithms

The code is developed entirely in **MATLAB** and addresses two approaches:

### 1. Alpha-Beta Filter ($\alpha-\beta$)
A simplified filter that assumes constant velocity and treats the X and Y coordinates independently.
* **Pros:** Low computational cost, simple implementation.
* **Cons:** Exhibits "lag" (delay) during maneuvers and uses fixed gains that do not adapt to signal-to-noise ratio changes.
* **Parameters:** Optimal $\alpha$ and $\beta$ were calculated based on the assumed process deviation ($\sigma_s = 250 m/s^2$) and measurement noise.

### 2. Kalman Filter
A recursive optimal estimator that models the full system state ($[x, y, v_x, v_y]$) and its uncertainties using covariance matrices.
* **Initial Covariance Matrix ($P_0$):** Initialized with high values ($50,000$) to reflect initial position uncertainty.
* **Process Noise Matrix ($Q$):** Models the system's flexibility regarding maneuvers or wind gusts (imprecision of 250m in position and 0.1 m/s in velocity).
* **Pros:** Better response in turns, superior trajectory smoothing, and dynamic gain adaptation via covariance coupling between axes.

## 📊 Results and Conclusions

Upon analyzing the simulation results and graphs:
1.  **Precision:** The Kalman filter provides a significantly smoother estimation closer to the real trajectory, especially during $180^\circ$ turns where the Alpha-Beta filter tends to react with a delay.
2.  **Performance:** Although the Alpha-Beta filter reduces noise, its lack of correlation between axes makes it less suitable for sharp maneuvers.
3.  **Usage:** The Kalman filter is recommended for critical tracking tasks requiring precision and robustness, reserving Alpha-Beta for systems with resource constraints.

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

**Ane Arocena Blanco** 
Universidad Politécnica de Madrid (UPM)  
Escuela Técnica Superior de Ingeniería Aeronáutica y del Espacio (ETSIAE)
