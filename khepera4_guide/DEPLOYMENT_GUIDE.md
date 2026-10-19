# Khepera IV 3D Web Virtual Robotics Lab: GitHub Pages Deployment Guide

This guide explains how to publish the **Khepera IV 3D Virtual Robotics Lab** to **GitHub Pages** so that students, professors, and collaborators can test and simulate the robot directly from any browser on any machine with zero installation.

---

## 🚀 1. Publish to GitHub (First-Time Setup)

If this workspace is not yet connected to a GitHub repository, follow these quick steps:

```bash
# 1. Initialize git in workspace (if not already done)
git init
git add .
git commit -m "feat: complete Khepera IV 3D WebGL simulator and closed-loop control lab"

# 2. Add your remote GitHub repository
git remote add origin https://github.com/<your-github-username>/<your-repo-name>.git
git branch -M main
git push -u origin main
```

---

## 🌐 2. Enable GitHub Pages

Once your code is pushed to GitHub:

1. Open your repository on GitHub in your browser: `https://github.com/<your-username>/<your-repo-name>`
2. Click on **Settings** (top navigation bar)
3. In the left sidebar under *Code and automation*, click **Pages**
4. Under **Build and deployment**:
   - **Source**: Select **GitHub Actions**
5. That's it! GitHub will automatically trigger the workflow defined in [`.github/workflows/deploy.yml`](../.github/workflows/deploy.yml).

Within ~30 seconds, your site will be live at:
```
https://<your-github-username>.github.io/<your-repo-name>/
```

---

## 💻 3. Running Locally on Any Machine (Zero Install)

You can also run this complete 3D simulator locally on Windows, macOS, or Linux using Python's built-in HTTP server:

```bash
cd khepera4_guide
python -m http.server 8080
```
Then navigate to: **`http://localhost:8080`**

Or using Node.js:
```bash
npx serve khepera4_guide
```

---

## 🛠️ Features Available in the Browser

- **3D Robot Model:** Authentic K-Team Khepera IV CAD model with blue bumper, rotating differential drive wheels, passive chrome ball casters, and glowing LED indicators.
- **Picture-in-Picture (PiP) Camera:** Live real-time perspective from the WVGA CMOS front camera (`camera_link`).
- **Interactive Setpoints (Mode B):** Click anywhere on the 3D arena floor to spawn the 3D glowing target beacon; the robot automatically steers and settles with cascade Lyapunov feedback.
- **Trajectory Tracking (Mode C):** Autonomous square and circular orbit tracking with live waypoint transitions.
- **Manual Game Drive (Mode A):** Real-time WASD and keyboard teleoperation.
- **Academic BSA Indicators:** Live rise time ($t_r$), settling time ($t_s$), overshoot ($\sigma\%$), and steady-state error ($e_{st}$).
- **Telemetry Export:** One-click CSV download containing timestamped pose, velocity, wheel speeds, and tracking error.
