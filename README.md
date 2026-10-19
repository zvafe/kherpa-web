# Khepera IV 3D WebGL Simulation & Control Systems Virtual Lab 🤖

[![Deploy Khepera IV Virtual Lab to GitHub Pages](https://github.com/zvafe/kherpa-web/actions/workflows/deploy.yml/badge.svg)](https://github.com/zvafe/kherpa-web/actions/workflows/deploy.yml)
[![Live Demo](https://img.shields.io/badge/Live%20Demo-GitHub%20Pages-blue?style=flat&logo=github)](https://zvafe.github.io/kherpa-web/)

An interactive, browser-based 3D robotics laboratory and control systems masterclass built with **Three.js**, **KaTeX**, and modern web standards. Accompanied by production-ready **ROS 2** nodes for physical Khepera IV robots and Gazebo simulations.

🌐 **Live Simulation:** [https://zvafe.github.io/kherpa-web/](https://zvafe.github.io/kherpa-web/)

---

## 🌟 Features & Highlights

### 1. Interactive 3D WebGL Laboratory
- **Accurate 3D CAD Meshes:** Real OBJ/MTL CAD models for the Khepera IV chassis, drive wheels, and passive casters with physics-aligned dimensions ($R = 21\text{ mm}$, $L = 105.4\text{ mm}$).
- **8 Bumper Proximity Lasers:** Real-time raycasting emitted at authentic angular slots ($0^\circ, \pm 45^\circ, \pm 90^\circ, \pm 135^\circ, 180^\circ$) detecting arena obstacles.
- **Picture-in-Picture (PiP) WVGA Camera:** Synchronized first-person onboard camera perspective ("Robot Eye").
- **Closed-Loop Kinematics:** Unicycle differential-drive kinematic integration and Lyapunov polar-coordinate steering.
- **Dynamic Telemetry & Step Response:** Live phase-plane trajectory, real-time error tracking ($\rho, \alpha, \beta, v, \omega$), and second-order metrics ($t_r, t_s, M_p, e_{ss}$).

### 2. Educational & Intuitive UI
- **Variable Hover Tooltips:** 3-second dwell timer with smooth fade-in animations explaining core variables ($K_\rho, K_\alpha, K_\beta, \rho, \alpha, \beta, \omega, v$).
- **Multi-Modal Controls:** Step target presets, manual floor-click beacon placement (Option B), and real-time WASD teleoperation.
- **Universally Selectable Text:** Formatted for easy note-taking, equation extraction, and academic reference.

### 3. Full ROS 2 Architecture Included
- **`src/khepera4_control`**: Closed-loop Lyapunov steering nodes, open-loop benchmarks, and real-time telemetry loggers.
- **`src/khepera4_description`**: High-detail URDF robot description, mesh models, Gazebo SDF world definitions, and launch files.

---

## 🚀 Quick Start (Local Run)

You can run the web simulation locally with any static web server:

### Option A: Using Python (Built-in)
```bash
# Navigate to the guide folder
cd khepera4_guide

# Start Python HTTP server
python -m http.server 8080
```
Open your browser at `http://localhost:8080`.

### Option B: Using Node.js / npx
```bash
npx serve khepera4_guide
```

---

## 📦 Repository Structure

```text
kherpa-web/
├── .github/
│   └── workflows/
│       └── deploy.yml              # Automated GitHub Pages deployment workflow
├── khepera4_guide/                 # WebGL 3D Simulation & Interactive Guide
│   ├── index.html                  # Main dashboard & interactive lessons
│   ├── style.css                   # Custom UI styles & animations
│   ├── simulator.js                # Core simulation runtime & controls
│   ├── js/
│   │   ├── app.js                  # UI controllers & event orchestration
│   │   ├── explanations.js         # Interactive hover tooltip dwell system
│   │   ├── robot3d.js              # Three.js robot CAD loader & laser sensors
│   │   └── telemetry.js            # Telemetry curves & metric computations
│   └── assets/
│       └── models/                 # CAD meshes (.obj, .mtl)
├── src/                            # ROS 2 Packages
│   ├── khepera4_control/           # Python control nodes & launch scripts
│   └── khepera4_description/       # URDF, meshes, SDF worlds
├── khepera_iv_control_systems_research_charter.md  # Research specifications
└── README.md                       # Project documentation
```

---

## ⚙️ GitHub Pages & Actions Setup

This repository comes pre-configured with continuous deployment using **GitHub Actions**.

Whenever changes are pushed to `main` (or `master`), the workflow in [`.github/workflows/deploy.yml`](.github/workflows/deploy.yml) automatically publishes the `khepera4_guide/` directory to GitHub Pages.

### How to configure in GitHub:
1. Navigate to your repository on GitHub: `https://github.com/<your-username>/kherpa-web`
2. Go to **Settings** $\rightarrow$ **Pages** (in the left sidebar).
3. Under **Build and deployment** $\rightarrow$ **Source**, choose **GitHub Actions**.
4. That's it! Every `git push origin main` triggers the deployment automatically.
5. Your site is live at: `https://<your-username>.github.io/kherpa-web/`
