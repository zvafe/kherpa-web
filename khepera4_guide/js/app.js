/**
 * Main Application Coordinator for Khepera IV Virtual Robotics Lab
 * Course: Bazele Sistemelor Automate (BSA)
 */

import { KheperaPhysics } from './physics.js';
import { KheperaController } from './controller.js';
import { Khepera3DScene } from './robot3d.js';
import { initVariableExplainer } from './explanations.js';

class VirtualRoboticsLab {
    constructor() {
        this.physics = new KheperaPhysics();
        this.controller = new KheperaController(this.physics);
        
        const canvasMain = document.getElementById('canvas-3d');
        const canvasPip = document.getElementById('canvas-pip');
        
        if (!canvasMain || !canvasPip) {
            console.error("Canvas elements not found!");
            return;
        }
        
        // Initialize 3D Scene
        this.scene3d = new Khepera3DScene(
            canvasMain,
            canvasPip,
            (targetX, targetY) => this.onGroundTargetClicked(targetX, targetY)
        );
        
        // Error Curve Canvas
        this.errCanvas = document.getElementById('canvas-error-curve');
        this.errCtx = this.errCanvas ? this.errCanvas.getContext('2d') : null;
        
        // Mode & Key States
        this.currentMode = 'B'; // 'A' (WASD), 'B' (Step/Goal), 'C' (Trajectory)
        this.keys = {};
        
        // Cached DOM Elements
        this.initDOMElements();
        
        // Setup UI Listeners
        this.setupEventListeners();
        
        // Initial setup for Mode B with +0.6m target
        this.setupInitialState();
        
        // Start 60 FPS Animation & 100 Hz Physics Loops
        this.lastTime = performance.now();
        this.simAccumulator = 0;
        this.fixedDt = 0.01; // 100 Hz physics
        
        requestAnimationFrame((t) => this.loop(t));
    }
    
    initDOMElements() {
        // Mode buttons
        this.btnModeA = document.getElementById('btn-mode-a');
        this.btnModeB = document.getElementById('btn-mode-b');
        this.btnModeC = document.getElementById('btn-mode-c');
        
        // Panels
        this.panelSimA = document.getElementById('panel-sim-a');
        this.panelSimB = document.getElementById('panel-sim-b');
        this.panelSimC = document.getElementById('panel-sim-c');
        
        // Telemetry cards
        this.liveRho = document.getElementById('live-rho');
        this.liveAlpha = document.getElementById('live-alpha');
        this.liveBeta = document.getElementById('live-beta');
        this.liveX = document.getElementById('live-x');
        this.liveY = document.getElementById('live-y');
        this.liveYaw = document.getElementById('live-yaw');
        this.liveV = document.getElementById('live-v');
        this.liveW = document.getElementById('live-w');
        
        // BSA Metrics
        this.liveTr = document.getElementById('live-tr');
        this.liveTs = document.getElementById('live-ts');
        this.liveSigma = document.getElementById('live-sigma');
        this.liveEst = document.getElementById('live-est');
        
        // Camera controls
        this.btnCamOrbit = document.getElementById('btn-cam-orbit');
        this.btnCamFollow = document.getElementById('btn-cam-follow');
        this.btnCamTop = document.getElementById('btn-cam-top');
        this.btnCamReset = document.getElementById('btn-cam-reset');
        this.btnTogglePip = document.getElementById('btn-toggle-pip');
        this.pipContainer = document.getElementById('pip-container');
        
        // Gain Sliders
        this.sliderKrho = document.getElementById('slider-krho');
        this.sliderKalpha = document.getElementById('slider-kalpha');
        this.sliderKbeta = document.getElementById('slider-kbeta');
        this.sliderKyaw = document.getElementById('slider-kyaw');
        
        this.lblKrho = document.getElementById('lbl-krho');
        this.lblKalpha = document.getElementById('lbl-kalpha');
        this.lblKbeta = document.getElementById('lbl-kbeta');
        this.lblKyaw = document.getElementById('lbl-kyaw');
        
        // Beacon placement hint badge
        this.beaconHintBadge = document.getElementById('beacon-hint-badge');
    }
    
    setupInitialState() {
        this.setMode('B');
        this.controller.setGoal(0.6, 0.0, 0.0, 'step');
        this.scene3d.setBeaconPosition(0.6, 0.0);
    }
    
    setMode(mode) {
        this.currentMode = mode;
        this.controller.stop();
        
        // Update button styles
        if (this.btnModeA) this.btnModeA.className = mode === 'A' ? 'mode-btn-active' : 'mode-btn-idle';
        if (this.btnModeB) this.btnModeB.className = mode === 'B' ? 'mode-btn-active' : 'mode-btn-idle';
        if (this.btnModeC) this.btnModeC.className = mode === 'C' ? 'mode-btn-active' : 'mode-btn-idle';
        
        if (this.panelSimA) this.panelSimA.style.display = mode === 'A' ? 'block' : 'none';
        if (this.panelSimB) this.panelSimB.style.display = mode === 'B' ? 'flex' : 'none';
        if (this.panelSimC) this.panelSimC.style.display = mode === 'C' ? 'flex' : 'none';
        
        // Show "Click 3D floor to place beacon in Option B" badge ONLY when Option B is selected
        if (this.beaconHintBadge) {
            this.beaconHintBadge.style.display = mode === 'B' ? 'flex' : 'none';
        }
        
        if (mode === 'B') {
            this.controller.setGoal(0.6, 0.0, 0.0, 'step');
            this.scene3d.setBeaconPosition(0.6, 0.0);
            if (this.scene3d && this.scene3d.beaconGroup) this.scene3d.beaconGroup.visible = true;
        } else if (mode === 'C') {
            this.controller.setTrajectory('square');
            if (this.controller.activeGoal) {
                this.scene3d.setBeaconPosition(this.controller.activeGoal.x, this.controller.activeGoal.y);
            }
            if (this.scene3d && this.scene3d.beaconGroup) this.scene3d.beaconGroup.visible = true;
        } else {
            // Mode A: Manual WASD drive - hide goal beacon
            if (this.scene3d && this.scene3d.beaconGroup) this.scene3d.beaconGroup.visible = false;
        }
    }
    
    onGroundTargetClicked(x, y) {
        // ONLY allow clicking on the floor to place a beacon if Option B is manually selected!
        if (this.currentMode !== 'B') {
            return;
        }
        this.controller.setGoal(x, y, 0.0, 'step');
        this.scene3d.setBeaconPosition(x, y);
    }
    
    setupEventListeners() {
        // Mode buttons
        if (this.btnModeA) this.btnModeA.addEventListener('click', () => this.setMode('A'));
        if (this.btnModeB) this.btnModeB.addEventListener('click', () => this.setMode('B'));
        if (this.btnModeC) this.btnModeC.addEventListener('click', () => this.setMode('C'));
        
        // Preset step buttons
        const btnStep05 = document.getElementById('btn-step-05');
        const btnStep10 = document.getElementById('btn-step-10');
        const btnStep15 = document.getElementById('btn-step-15');
        
        if (btnStep05) btnStep05.addEventListener('click', () => {
            const targetX = this.physics.x + 0.5 * Math.cos(this.physics.yaw);
            const targetY = this.physics.y + 0.5 * Math.sin(this.physics.yaw);
            this.onGroundTargetClicked(targetX, targetY);
        });
        
        if (btnStep10) btnStep10.addEventListener('click', () => {
            const targetX = this.physics.x + 1.0 * Math.cos(this.physics.yaw);
            const targetY = this.physics.y + 1.0 * Math.sin(this.physics.yaw);
            this.onGroundTargetClicked(targetX, targetY);
        });
        
        if (btnStep15) btnStep15.addEventListener('click', () => {
            const targetX = this.physics.x + 1.5 * Math.cos(this.physics.yaw);
            const targetY = this.physics.y + 1.5 * Math.sin(this.physics.yaw);
            this.onGroundTargetClicked(targetX, targetY);
        });
        
        // Trajectory buttons
        const btnTrajSquare = document.getElementById('btn-traj-square');
        const btnTrajCircle = document.getElementById('btn-traj-circle');
        
        if (btnTrajSquare) btnTrajSquare.addEventListener('click', () => {
            this.setMode('C');
            this.controller.setTrajectory('square');
            if (this.controller.activeGoal) {
                this.scene3d.setBeaconPosition(this.controller.activeGoal.x, this.controller.activeGoal.y);
            }
        });
        
        if (btnTrajCircle) btnTrajCircle.addEventListener('click', () => {
            this.setMode('C');
            this.controller.setTrajectory('circle');
            if (this.controller.activeGoal) {
                this.scene3d.setBeaconPosition(this.controller.activeGoal.x, this.controller.activeGoal.y);
            }
        });
        
        // Reset button
        const btnReset = document.getElementById('btn-reset');
        if (btnReset) btnReset.addEventListener('click', () => {
            this.physics.reset(0, 0, 0);
            this.scene3d.clearTrail();
            if (this.currentMode === 'B') {
                this.controller.setGoal(0.6, 0.0, 0.0, 'step');
                this.scene3d.setBeaconPosition(0.6, 0.0);
            } else {
                this.controller.stop();
            }
        });
        
        // Camera View Mode Buttons
        if (this.btnCamOrbit) this.btnCamOrbit.addEventListener('click', () => {
            this.scene3d.setCameraMode('orbit');
            this.updateCamBtnState('orbit');
        });
        if (this.btnCamFollow) this.btnCamFollow.addEventListener('click', () => {
            this.scene3d.setCameraMode('follow');
            this.updateCamBtnState('follow');
        });
        if (this.btnCamTop) this.btnCamTop.addEventListener('click', () => {
            this.scene3d.setCameraMode('top');
            this.updateCamBtnState('top');
        });
        if (this.btnCamReset) this.btnCamReset.addEventListener('click', () => {
            this.scene3d.resetCamera();
            this.updateCamBtnState('orbit');
        });
        
        // PiP Camera Toggle
        if (this.btnTogglePip && this.pipContainer) {
            this.btnTogglePip.addEventListener('click', () => {
                const isHidden = this.pipContainer.style.display === 'none';
                this.pipContainer.style.display = isHidden ? 'block' : 'none';
                this.btnTogglePip.textContent = isHidden ? '📷 Hide PiP Camera' : '📷 Show PiP Camera';
            });
        }
        
        // Keyboard Drive (WASD + Spacebar)
        window.addEventListener('keydown', (e) => {
            const k = e.key.toLowerCase();
            if (['w', 'a', 's', 'd', ' '].includes(k)) {
                this.keys[k] = true;
                this.highlightKey(k, true);
            }
        });
        
        window.addEventListener('keyup', (e) => {
            const k = e.key.toLowerCase();
            if (['w', 'a', 's', 'd', ' '].includes(k)) {
                this.keys[k] = false;
                this.highlightKey(k, false);
            }
        });

        // On-screen clickable / touchable buttons for mobile and mouse drive
        ['w', 'a', 's', 'd'].forEach(k => {
            const el = document.getElementById(`sim-key-${k}`);
            if (el) {
                const press = (e) => {
                    e.preventDefault();
                    this.keys[k] = true;
                    this.highlightKey(k, true);
                };
                const release = (e) => {
                    e.preventDefault();
                    this.keys[k] = false;
                    this.highlightKey(k, false);
                };
                el.addEventListener('pointerdown', press);
                el.addEventListener('pointerup', release);
                el.addEventListener('pointerleave', release);
                el.addEventListener('pointercancel', release);
            }
        });
        
        // Parameter Sliders
        const updateGains = () => {
            if (!this.sliderKrho) return;
            const kRho = parseFloat(this.sliderKrho.value);
            const kAlpha = parseFloat(this.sliderKalpha.value);
            const kBeta = parseFloat(this.sliderKbeta.value);
            const kYaw = parseFloat(this.sliderKyaw.value);
            
            this.lblKrho.textContent = kRho.toFixed(2);
            this.lblKalpha.textContent = kAlpha.toFixed(2);
            this.lblKbeta.textContent = kBeta.toFixed(2);
            this.lblKyaw.textContent = kYaw.toFixed(2);
            
            this.controller.setGains(kRho, kAlpha, kBeta, kYaw);
        };
        
        if (this.sliderKrho) this.sliderKrho.addEventListener('input', updateGains);
        if (this.sliderKalpha) this.sliderKalpha.addEventListener('input', updateGains);
        if (this.sliderKbeta) this.sliderKbeta.addEventListener('input', updateGains);
        if (this.sliderKyaw) this.sliderKyaw.addEventListener('input', updateGains);
        
        // CSV Telemetry Export Button
        const btnExportCsv = document.getElementById('btn-export-csv');
        if (btnExportCsv) {
            btnExportCsv.addEventListener('click', () => {
                const csvData = this.controller.exportTelemetryCSV();
                if (!csvData) {
                    alert("No telemetry data recorded yet. Run a step response or trajectory first!");
                    return;
                }
                const blob = new Blob([csvData], { type: 'text/csv;charset=utf-8;' });
                const url = URL.createObjectURL(blob);
                const a = document.createElement('a');
                a.href = url;
                a.download = `khepera4_telemetry_${Date.now()}.csv`;
                document.body.appendChild(a);
                a.click();
                document.body.removeChild(a);
                URL.revokeObjectURL(url);
            });
        }
    }
    
    updateCamBtnState(activeMode) {
        const btns = [
            { el: this.btnCamOrbit, mode: 'orbit' },
            { el: this.btnCamFollow, mode: 'follow' },
            { el: this.btnCamTop, mode: 'top' }
        ];
        btns.forEach(b => {
            if (b.el) {
                b.el.className = b.mode === activeMode
                    ? 'px-2.5 py-1 text-xs rounded bg-sky-600 text-white font-semibold shadow-sm transition'
                    : 'px-2.5 py-1 text-xs rounded bg-slate-800 hover:bg-slate-700 text-slate-300 font-semibold transition';
            }
        });
    }
    
    highlightKey(k, active) {
        const el = document.getElementById(`sim-key-${k}`);
        if (el) {
            el.className = active
                ? 'w-12 h-12 flex items-center justify-center font-bold rounded-lg border-2 border-emerald-400 bg-emerald-950 text-emerald-300 shadow-md shadow-emerald-500/30'
                : 'w-12 h-12 flex items-center justify-center font-bold rounded-lg border border-slate-700 bg-slate-800 text-slate-300';
        }
    }
    
    loop(nowMs) {
        const dt = (nowMs - this.lastTime) / 1000.0;
        this.lastTime = nowMs;
        
        // 1. Update Mode A WASD Keyboard Controls
        if (this.currentMode === 'A') {
            let vCmd = 0.0;
            let wCmd = 0.0;
            if (this.keys['w']) vCmd += 0.22;
            if (this.keys['s']) vCmd -= 0.22;
            if (this.keys['a']) wCmd += 1.6;
            if (this.keys['d']) wCmd -= 1.6;
            if (this.keys[' ']) { vCmd = 0; wCmd = 0; }
            this.physics.setCommand(vCmd, wCmd);
        }
        
        // 2. Fixed-Step 100 Hz Physics & Controller Loop
        this.simAccumulator += Math.min(dt, 0.1);
        while (this.simAccumulator >= this.fixedDt) {
            if (this.currentMode !== 'A') {
                this.controller.update(nowMs);
                // Synchronize 3D beacon with active waypoint in trajectory mode
                if (this.controller.activeGoal) {
                    this.scene3d.setBeaconPosition(this.controller.activeGoal.x, this.controller.activeGoal.y);
                }
            }
            this.physics.step(this.fixedDt);
            this.simAccumulator -= this.fixedDt;
        }
        
        // 3. Render 3D Scene (Main Viewport & Front WVGA PiP)
        this.scene3d.render(this.physics, nowMs);
        
        // 4. Update UI Telemetry & Error Curves
        this.updateTelemetryUI();
        this.drawErrorCurve();
        
        requestAnimationFrame((t) => this.loop(t));
    }
    
    updateTelemetryUI() {
        if (this.liveRho) this.liveRho.textContent = `${this.controller.currentRho.toFixed(3)} m`;
        if (this.liveAlpha) this.liveAlpha.textContent = `${((this.controller.currentAlpha * 180.0) / Math.PI).toFixed(1)} °`;
        if (this.liveBeta) this.liveBeta.textContent = `${((this.controller.currentBeta * 180.0) / Math.PI).toFixed(1)} °`;
        
        if (this.liveX) this.liveX.textContent = this.physics.odomX.toFixed(3);
        if (this.liveY) this.liveY.textContent = this.physics.odomY.toFixed(3);
        if (this.liveYaw) this.liveYaw.textContent = `${((this.physics.odomYaw * 180.0) / Math.PI).toFixed(1)} °`;
        
        if (this.liveV) this.liveV.textContent = this.physics.v.toFixed(2);
        if (this.liveW) this.liveW.textContent = this.physics.w.toFixed(2);
        
        // BSA Second-Order Indicators
        const m = this.controller.metrics;
        if (this.liveTr) this.liveTr.textContent = m.tr !== null ? `${m.tr.toFixed(2)} s` : '-- s';
        if (this.liveTs) this.liveTs.textContent = m.ts !== null ? `${m.ts.toFixed(2)} s` : '-- s';
        if (this.liveSigma) this.liveSigma.textContent = m.sigma !== null ? `${m.sigma.toFixed(1)} %` : '-- %';
        if (this.liveEst) this.liveEst.textContent = m.est !== null ? `${(m.est * 100.0).toFixed(1)} cm` : '-- cm';
    }
    
    drawErrorCurve() {
        if (!this.errCtx || !this.errCanvas) return;
        const ctx = this.errCtx;
        const cvs = this.errCanvas;
        
        ctx.clearRect(0, 0, cvs.width, cvs.height);
        
        // Dark background
        ctx.fillStyle = '#0f172a';
        ctx.fillRect(0, 0, cvs.width, cvs.height);
        
        // Grid lines
        ctx.strokeStyle = '#1e293b';
        ctx.lineWidth = 1;
        for (let y = 30; y < cvs.height; y += 30) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(cvs.width, y);
            ctx.stroke();
        }
        
        // Tolerance threshold band (2 cm = 0.02 m)
        ctx.fillStyle = 'rgba(16, 185, 129, 0.15)';
        ctx.fillRect(0, cvs.height - 18, cvs.width, 18);
        ctx.strokeStyle = '#10b981';
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(0, cvs.height - 18);
        ctx.lineTo(cvs.width, cvs.height - 18);
        ctx.stroke();
        ctx.setLineDash([]);
        
        const history = this.controller.history;
        if (history.length < 2) {
            ctx.fillStyle = '#64748b';
            ctx.font = '10px monospace';
            ctx.fillText('Awaiting step / trajectory start...', 12, cvs.height / 2);
            return;
        }
        
        const maxT = Math.max(4.0, history[history.length - 1].t);
        const maxRho = Math.max(0.5, this.controller.initialDist || 0.6);
        
        // Plot error curve: rho(t) in rose/red
        ctx.strokeStyle = '#f43f5e';
        ctx.lineWidth = 2;
        ctx.beginPath();
        
        for (let i = 0; i < history.length; i++) {
            const pt = history[i];
            const px = (pt.t / maxT) * (cvs.width - 20) + 10;
            const py = (1.0 - (pt.rho / maxRho)) * (cvs.height - 30) + 10;
            if (i === 0) ctx.moveTo(px, py);
            else ctx.lineTo(px, py);
        }
        ctx.stroke();
        
        // Plot linear speed v(t) in sky blue
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        for (let i = 0; i < history.length; i++) {
            const pt = history[i];
            const px = (pt.t / maxT) * (cvs.width - 20) + 10;
            const py = cvs.height - (pt.v / 0.25) * (cvs.height - 35) - 10;
            if (i === 0) ctx.moveTo(px, py);
            else ctx.lineTo(px, py);
        }
        ctx.stroke();
        
        // Legend
        ctx.fillStyle = '#f43f5e';
        ctx.font = '9px monospace';
        ctx.fillText('● Distance Error e(t)', 10, 14);
        ctx.fillStyle = '#38bdf8';
        ctx.fillText('● Speed v(t)', 150, 14);
    }
}

// Bootstrap application on DOM load
window.addEventListener('DOMContentLoaded', () => {
    window.kheperaLab = new VirtualRoboticsLab();
    initVariableExplainer();
});
