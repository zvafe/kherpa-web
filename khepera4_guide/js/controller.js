/**
 * Khepera IV Closed-Loop Controller & BSA Performance Analyzer
 * Course: Bazele Sistemelor Automate (BSA)
 * 
 * Replicates the ROS 2 node 'closed_loop_controller.py':
 * - Polar error coordinate formulation (rho, alpha, beta)
 * - Non-linear Lyapunov cascade regulation law
 * - Second-order performance indicators (tr, ts, sigma%, est)
 * - Trajectory generators (step, waypoint, square, circular orbit)
 * - Telemetry history logger with CSV export
 */

export class KheperaController {
    constructor(physics, options = {}) {
        this.physics = physics;
        
        // Control Parameters (Lyapunov / Cascade gains)
        this.kRho = options.kRho ?? 0.85;
        this.kAlpha = options.kAlpha ?? 2.1;
        this.kBeta = options.kBeta ?? -0.4;
        this.kYaw = options.kYaw ?? 1.6;
        
        this.vMax = options.vMax ?? 0.25;
        this.wMax = options.wMax ?? 1.8;
        
        this.posTolerance = options.posTolerance ?? 0.02; // 2 cm
        this.yawTolerance = options.yawTolerance ?? 0.05; // ~2.8 deg
        
        // Navigation State
        this.mode = 'manual'; // 'manual', 'step', 'waypoint', 'square', 'circle'
        this.state = 'IDLE';   // 'IDLE', 'NAVIGATING', 'ALIGNING', 'REACHED'
        
        this.waypoints = [];
        this.waypointIndex = 0;
        this.activeGoal = null; // {x, y, yaw}
        
        // Telemetry & Metrics History
        this.history = [];
        this.stepStartTime = null;
        this.initialDist = 0.0;
        this.metrics = {
            tr: null,      // Rise time (s)
            ts: null,      // Settling time (s)
            sigma: null,   // Maximum overshoot (%)
            est: null,     // Steady-state error (m)
            duration: null // Total execution time (s)
        };
        
        // Real-time live errors
        this.currentRho = 0.0;
        this.currentAlpha = 0.0;
        this.currentBeta = 0.0;
    }
    
    setGains(kRho, kAlpha, kBeta, kYaw) {
        this.kRho = parseFloat(kRho);
        this.kAlpha = parseFloat(kAlpha);
        this.kBeta = parseFloat(kBeta);
        this.kYaw = parseFloat(kYaw);
    }
    
    setGoal(gx, gy, gyaw = 0.0, mode = 'waypoint') {
        this.mode = mode;
        this.waypoints = [{ x: gx, y: gy, yaw: gyaw }];
        this.waypointIndex = 0;
        this.activeGoal = this.waypoints[0];
        
        this.state = 'NAVIGATING';
        this.initialDist = Math.hypot(gx - this.physics.odomX, gy - this.physics.odomY);
        this.stepStartTime = performance.now();
        this.history = [];
        this.metrics = { tr: null, ts: null, sigma: null, est: null, duration: null };
    }
    
    setTrajectory(mode) {
        this.mode = mode;
        const curX = this.physics.odomX;
        const curY = this.physics.odomY;
        this.waypoints = [];
        
        if (mode === 'square') {
            // 0.8m x 0.8m square centered in the arena
            const s = 0.45;
            this.waypoints = [
                { x: s, y: -s, yaw: Math.PI / 2.0 },
                { x: s, y: s, yaw: Math.PI },
                { x: -s, y: s, yaw: -Math.PI / 2.0 },
                { x: -s, y: -s, yaw: 0.0 }
            ];
        } else if (mode === 'circle') {
            // R = 0.5m circular orbit centered at origin
            const R = 0.5;
            this.waypoints = [];
            const N = 16;
            for (let i = 0; i < N; i++) {
                const theta = (2.0 * Math.PI * (i + 1) / N);
                this.waypoints.push({
                    x: R * Math.cos(theta),
                    y: R * Math.sin(theta),
                    yaw: this.normalizeAngle(theta + Math.PI / 2.0)
                });
            }
        }
        
        this.waypointIndex = 0;
        this.activeGoal = this.waypoints[0];
        this.state = 'NAVIGATING';
        this.initialDist = Math.hypot(this.activeGoal.x - curX, this.activeGoal.y - curY);
        this.stepStartTime = performance.now();
        this.history = [];
        this.metrics = { tr: null, ts: null, sigma: null, est: null, duration: null };
    }
    
    stop() {
        this.state = 'IDLE';
        this.mode = 'manual';
        this.activeGoal = null;
        this.physics.setCommand(0.0, 0.0);
    }
    
    update(nowMs) {
        if (this.state === 'IDLE' || !this.activeGoal) {
            return;
        }
        
        const curX = this.physics.odomX;
        const curY = this.physics.odomY;
        const curYaw = this.physics.odomYaw;
        
        const gx = this.activeGoal.x;
        const gy = this.activeGoal.y;
        const gyaw = this.activeGoal.yaw;
        
        // 1. Error formulation in polar robot frame
        const dx = gx - curX;
        const dy = gy - curY;
        const rho = Math.hypot(dx, dy);
        const alpha = this.normalizeAngle(Math.atan2(dy, dx) - curYaw);
        const beta = this.normalizeAngle(-curYaw - alpha + gyaw);
        
        this.currentRho = rho;
        this.currentAlpha = alpha;
        this.currentBeta = beta;
        
        let cmdV = 0.0;
        let cmdW = 0.0;
        
        // 2. Cascade Feedback Control Law
        // Phase 1: Position Approaching
        if (rho > this.posTolerance) {
            this.state = 'NAVIGATING';
            
            // If goal is significantly behind (> 90 deg), spin towards it first
            if (Math.abs(alpha) > Math.PI / 2.0) {
                cmdV = 0.0;
                cmdW = Math.max(-this.wMax, Math.min(this.wMax, this.kAlpha * alpha));
            } else {
                const rawV = this.kRho * rho;
                const rawW = this.kAlpha * alpha + this.kBeta * beta;
                cmdV = Math.max(-this.vMax, Math.min(this.vMax, rawV));
                cmdW = Math.max(-this.wMax, Math.min(this.wMax, rawW));
            }
        } 
        // Phase 2: In-Place Orientation Alignment
        else {
            const yawErr = this.normalizeAngle(gyaw - curYaw);
            if (Math.abs(yawErr) > this.yawTolerance) {
                this.state = 'ALIGNING';
                cmdV = 0.0;
                cmdW = Math.max(-this.wMax, Math.min(this.wMax, this.kYaw * yawErr));
            } else {
                // Waypoint successfully reached!
                cmdV = 0.0;
                cmdW = 0.0;
                this.waypointIndex++;
                
                if (this.waypointIndex < this.waypoints.length) {
                    this.activeGoal = this.waypoints[this.waypointIndex];
                } else {
                    // Mission completed
                    this.state = 'REACHED';
                    this.activeGoal = null;
                    this.physics.setCommand(0.0, 0.0);
                    this.computeMetrics();
                    return;
                }
            }
        }
        
        this.physics.setCommand(cmdV, cmdW);
        
        // Log telemetry sample
        if (this.stepStartTime !== null) {
            const elapsed = (nowMs - this.stepStartTime) / 1000.0;
            this.history.push({
                t: elapsed,
                x: curX,
                y: curY,
                yaw: curYaw,
                rho: rho,
                alpha: alpha,
                beta: beta,
                v: this.physics.v,
                w: this.physics.w,
                wL: this.physics.wL,
                wR: this.physics.wR
            });
        }
    }
    
    computeMetrics() {
        if (this.history.length === 0 || this.initialDist <= 0.01) {
            return;
        }
        
        const D = this.initialDist;
        const totalTime = this.history[this.history.length - 1].t;
        this.metrics.duration = totalTime;
        
        // 1. Steady-state error est
        const finalRho = this.history[this.history.length - 1].rho;
        this.metrics.est = finalRho;
        
        // 2. Rise Time tr (Time from 10% to 90% distance covered)
        // distance covered = D - rho
        let t10 = null;
        let t90 = null;
        for (const pt of this.history) {
            const covered = D - pt.rho;
            if (t10 === null && covered >= 0.10 * D) {
                t10 = pt.t;
            }
            if (t90 === null && covered >= 0.90 * D) {
                t90 = pt.t;
            }
        }
        if (t10 !== null && t90 !== null) {
            this.metrics.tr = Math.max(0.01, t90 - t10);
        }
        
        // 3. Settling Time ts (2% band criterion: error permanent within 0.02 * D or 2cm)
        const tolBand = Math.max(0.02, 0.02 * D);
        let tsFound = null;
        for (let i = 0; i < this.history.length; i++) {
            let staysWithin = true;
            for (let j = i; j < this.history.length; j++) {
                if (this.history[j].rho > tolBand) {
                    staysWithin = false;
                    break;
                }
            }
            if (staysWithin) {
                tsFound = this.history[i].t;
                break;
            }
        }
        this.metrics.ts = tsFound ?? totalTime;
        
        // 4. Maximum Overshoot sigma%
        // In mobile robots, overshoot occurs if distance traveled exceeds target distance
        // or if robot oscillates past the goal point.
        let minRho = D;
        for (const pt of this.history) {
            if (pt.rho < minRho) minRho = pt.rho;
        }
        // If it got closer than the final settled distance, the difference is overshoot
        const maxOvershootDist = Math.max(0.0, finalRho - minRho);
        this.metrics.sigma = (maxOvershootDist / D) * 100.0;
    }
    
    exportTelemetryCSV() {
        if (this.history.length === 0) return null;
        
        const header = "timestamp_s,x_m,y_m,yaw_rad,rho_error_m,alpha_deg,beta_deg,v_mps,w_radps,omega_left_radps,omega_right_radps\n";
        const rows = this.history.map(pt => {
            return [
                pt.t.toFixed(3),
                pt.x.toFixed(4),
                pt.y.toFixed(4),
                pt.yaw.toFixed(4),
                pt.rho.toFixed(4),
                ((pt.alpha * 180.0) / Math.PI).toFixed(2),
                ((pt.beta * 180.0) / Math.PI).toFixed(2),
                pt.v.toFixed(4),
                pt.w.toFixed(4),
                pt.wL.toFixed(4),
                pt.wR.toFixed(4)
            ].join(',');
        }).join('\n');
        
        return header + rows;
    }
    
    normalizeAngle(a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }
}
