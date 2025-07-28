# Fundamentals of Automatic Target Tracking: Passive Sensor Ship Tracking

## Project Description

This project, developed as part of the MTH-407 course at Istanbul Technical University, Mathematical Engineering, focuses on estimating the position of a ship moving at a constant velocity on a flat plane using three passive sensors. The simulation incorporates realistic scenarios such as a ship's onboard radar scanning at 60° per second and activating sensors that fall within its field of view. Measurements from these sensors have a 0.9 probability of being recorded and include added noise. The primary objective is to estimate the ship's position using only these angular (bearing) measurements.

## Table of Contents

1. [Problem Definition and Model Derivations](#problem-definition-and-model-derivations)
2. [Initial Estimation: Nonlinear Least Squares (NLS)](#initial-estimation-nonlinear-least-squares-nls)
3. [EKF Implementation](#ekf-implementation)
4. [Simulation Details](#simulation-details)
5. [Analysis and Visualization](#analysis-and-visualization)
6. [Conclusion](#conclusion)
7. [References](#references)
8. [Student Information](#student-information)

## 1. Problem Definition and Model Derivations

### Ship Movement (State Model)

The ship's movement is modeled with a constant velocity on a flat plane. The initial state vector `x0` for the ship's position `(x(0), y(0))` and velocity `(vx, vy)` is defined as:
x0 = [100 m; 200 m; 3 m/s; 3 m/s]

The state-transition matrix `F` for a sampling interval of Δt = 1 second is:
F = [1 0 Δt 0
0 1 0 Δt
0 0 1 0
0 0 0 1]


The process noise covariance matrix `Q` is:
Q = [Δt^4/4 0 Δt^3/2 0
0 Δt^4/4 0 Δt^3/2
Δt^3/2 0 Δt^2 0
0 Δt^3/2 0 Δt^2]


### Measurements from Sensor (Measurement Model)

The measurement model is defined as:
zi(k) = atan2(y(k) - yi, x(k) - xi) + vi(k)


Where `(x(k), y(k))` is the ship's position at time `k` and `vi(k) ~ N(0, σ²)` represents the Gaussian measurement noise.

---

## 2. Initial Estimation: Nonlinear Least Squares (NLS)

The Nonlinear Least Squares (NLS) method was used to estimate the ship’s initial position using the first three bearing measurements. The estimate is used as the starting point for EKF.

**Key Equations:**

- Residual: `ri = zi - hi(x0)`
- Jacobian `Ji`:
  - ∂hi/∂x0 = `-dy / (dx² + dy²)`
  - ∂hi/∂y0 = `dx / (dx² + dy²)`
- Update: `Δx = (JᵀJ)⁻¹ Jᵀ r`
- Estimate: `x0_new = x0_old + Δx`
- Convergence: `Δx < ε`

Estimated initial position: (100.25, 202.02), True: (100.00, 200.00)

---

## 3. EKF Implementation

The Extended Kalman Filter (EKF) is used for continuous ship tracking.

### Prediction Step

- `x̂(k|k-1) = F * x̂(k-1|k-1)`
- `P(k|k-1) = F * P(k-1|k-1) * Fᵀ + Q`

### Update Step (if sensor is active)

- Jacobian `H`:
H = [-Δy / (Δx² + Δy²), Δx / (Δx² + Δy²), 0, 0]

- Kalman Gain:
Kk = P(k|k-1) * Hᵀ * (H * P(k|k-1) * Hᵀ + R)⁻¹

- State Update:
x̂(k|k) = x̂(k|k-1) + Kk * (zk - ẑk)

- Covariance Update:
P(k|k) = (I - Kk * H) * P(k|k-1)

### Radar Field of View

The ship's radar scans at 60°/s:

- `θ_radar(t) = mod(60 * (t - 1), 360)`
- Sensor angle to ship: `θi = atan2(ys - yt, xs - xt)`
- Sensor is active if:
θi ∈ [θ_radar - 30°, θ_radar + 30°]

---

## 4. Simulation Details

- Duration: 50 time steps (1 sec each)
- Ship Initial Position: (100, 200)
- Ship Velocity: (5 m/s, 3 m/s)
- Sensor Locations: (0, 0), (500, 0), (250, 400)
- Radar Field of View: ±30°
- Noise standard deviation: σ = 0.5°

---

## 5. Analysis and Visualization

- **Trajectory Plots:**
  - True path: green
  - EKF estimate: blue

These show that the EKF accurately tracks the ship, even with noisy and intermittent measurements.






