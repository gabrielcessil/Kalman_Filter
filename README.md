# Kalman Filter for Pendulum State Estimation

This repository contains a MATLAB implementation of a **Kalman Filter** applied to the **state estimation of a simple pendulum**.  
The project demonstrates how to estimate both **angular position** and **angular velocity** of a pendulum subject to measurement and process noise.

---

## Project Overview

- **System**: Simple pendulum with length `l` and mass `m`.  
- **Objective**: Estimate pendulum states (angle and angular velocity) from noisy measurements using a Kalman Filter.  
- **Approach**:
  - Continuous-time pendulum dynamics are discretized for simulation.  
  - Gaussian noise is added to both process and measurements.  
  - The Kalman Filter recursively estimates states and residuals.  
  - Results are visualized through comparative plots.  

---

## Repository Structure

- **`main.m`** Main script that:
  - Defines pendulum dynamics.  
  - Configures noise models.  
  - Runs the simulation.  
  - Applies the Kalman Filter.  
  - Generates plots comparing real, measured, and estimated states.  

- **`KalmanFilter.m`** MATLAB class implementing a discrete-time Kalman Filter with methods for:
  - Prediction and correction steps.  
  - Updating state estimates, covariance matrices, Kalman gain, and residuals.  

---

## Implementation Details

### Pendulum Dynamics

The pendulum is modeled as:

$$
\ddot{\theta} = -\frac{g}{l} \sin(\theta) + \frac{1}{ml^2}w(t)
$$

where:
- '$\theta$': angular position  
- '$g$': gravitational acceleration  
- '$l$': pendulum length  
- '$w(t)$': process noise  

Linearized system matrices are derived and discretized for the filter.

### Noise Modeling

- **Process noise covariance**: '$Q_v = 10^{-3}$'
- **Measurement noise covariance**: '$R_v = 10^{-3}$'  

### Kalman Filter Equations

Prediction based on model

$$
\hat{x}_{k|k-1} = A \hat{x}_{k-1|k-1} + Bu
$$

Estimation of Covariance Matrix

$$
P_{k|k-1} = A P_{k-1|k-1} A^T + \Gamma Q_v \Gamma^T
$$

Correction:
Calculate the Kalman gains

$$
K_k = P_{k|k-1} C^T \left(C P_{k|k-1} C^T + R_v\right)^{-1}
$$

New estimation

$$
\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k (y_k - C \hat{x}_{k|k-1})
$$

Correction the Covariance matrix

$$
P_{k|k} = (I - K_k C) P_{k|k-1}  
$$


---

## Results

The script generates the following plots:

1.  **Angle Estimation**:  
    Comparison of real angle, noisy measurements, and Kalman estimates.

2.  **Angular Velocity Estimation**:  
    Comparison of real angular velocity, noisy derivative-based measurements, and Kalman estimates.

3.  **Kalman Gain Evolution**:  
    Log-scaled evolution of Kalman gain matrix elements.

4.  **Residuals**:  
    Filter residuals over time.

---

## Usage

1.  Clone the repository:
    ```bash
    git clone [https://github.com/yourusername/kalman-pendulum.git](https://github.com/yourusername/kalman-pendulum.git)
    ```
2.  Open MATLAB and run:
    ```matlab
    main
    ```

---

## Requirements

- MATLAB (tested on R2021a or later)  
- Control System Toolbox (for `ss` and `c2d`)  

---

## References

- R.E. Kalman, *A New Approach to Linear Filtering and Prediction Problems*, Journal of Basic Engineering, 1960.  
- S. Haykin, *Kalman Filtering and Neural Networks*, Wiley, 2001.
