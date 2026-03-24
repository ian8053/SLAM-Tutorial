# SLAM Tutorial: From Zero to Hero

Visual animations + study notes covering the full SLAM pipeline — from Kalman Filters to ORB-SLAM3 and semantic 3D reconstruction.

All animations built with [ManimGL](https://github.com/3b1b/manim) (3Blue1Brown's animation engine).

## Course Roadmap

```
Fundamentals → Coordinate Transforms
  → Kalman Filter (linear, Gaussian)
    → EKF (nonlinear, Jacobian linearization)
      → Particle Filter (non-Gaussian, sampling-based)
        → Visual SLAM (graph optimization, Bundle Adjustment)
          → Semantic 3D Mapping (TSDF, mesh reconstruction)
```

Each stage relaxes an assumption of the previous one.

---

## Modules

### 01 — SLAM Fundamentals
> Terminology, Full SLAM vs Online SLAM, mathematical formulation, three main approaches

- [Study Notes](01-slam-fundamentals/notes.md)

---

### 02 — Coordinate Transforms
> Homogeneous coordinates, projective geometry, rigid body transforms

- [Study Notes](02-coordinate-transforms/notes.md)

---

### 03 — Kalman Filter
> Gaussian belief, predict-update cycle, sensor fusion, the 7 core equations

| Video | Source Code |
|-------|-------------|
| Kalman Filter — Sensor Fusion Overview | [ssm_kf_ekf_pf.py](03-kalman-filter/ssm_kf_ekf_pf.py) |
| KF 7 Equations Step-by-Step | [kf_7equations.py](03-kalman-filter/kf_7equations.py) |

---

### 04 — Extended Kalman Filter & EKF-SLAM
> Jacobian linearization, nonlinear state estimation, EKF-SLAM state vector

| Video | Source Code |
|-------|-------------|
| EKF Jacobian Linearization | [ekf_jacobian.py](04-ekf-slam/ekf_jacobian.py) |

- [EKF-SLAM Math Notes](04-ekf-slam/notes.md) — Full derivation: state/observation equations, A/B/H matrices, Jacobian
- [SSM & EKF Interview Notes](04-ekf-slam/interview-notes.md) — SLAM drift debugging methodology

---

### 05 — Particle Filter
> Beyond Gaussians: Monte Carlo sampling, importance weighting, AMCL

| Video | Source Code |
|-------|-------------|
| Motion Model (Prediction Step) | [pf_motion_model.py](05-particle-filter/pf_motion_model.py) |
| Measurement Update & Weighting | [pf_update_step.py](05-particle-filter/pf_update_step.py) |
| Resampling Strategies | [pf_resampling.py](05-particle-filter/pf_resampling.py) |
| Practical Limitations | [pf_limitations.py](05-particle-filter/pf_limitations.py) |

---

### 06 — ORB-SLAM3
> Visual-Inertial SLAM: three-thread architecture, tracking, local mapping, loop closing

| Video | Source Code |
|-------|-------------|
| ORB-SLAM3 Pipeline (3D Animation) | [manimgl_orbslam3_v2_fixed.py](06-orbslam3/manimgl_orbslam3_v2_fixed.py) |

- [Tracking Code Review Part 1](06-orbslam3/tracking-part1.md) — System.cc init, Tracking thread, data structures, IMU preintegration
- [Tracking Code Review Part 2](06-orbslam3/tracking-part2.md) — Continued code walkthrough
- [Tracking Q&A](06-orbslam3/tracking-notes.md) — Common tracking issues and solutions

---

### 07 — TSDF & Semantic 3D Mapping
> From depth images to 3D understanding: TSDF fusion, voxel grids, marching cubes mesh extraction

| Video | Source Code |
|-------|-------------|
| TSDF Complete Pipeline | [tsdf_final.py](07-tsdf-semantic-mapping/tsdf_final.py) |

---

## Setup

```bash
pip install manimgl matplotlib numpy
sudo apt install ffmpeg
```

### Running Animations

```bash
# Example: render KF 7 equations
cd 03-kalman-filter/
manimgl kf_7equations.py KF7Equations -w

# Example: render PF resampling
cd 05-particle-filter/
manimgl pf_resampling.py PFResampling -w
```

## License

MIT

## Author

**Ian Ho** — Robotics Software Engineer
