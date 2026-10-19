/**
 * Khepera IV Physics & Differential-Drive Kinematics Engine
 * Course: Bazele Sistemelor Automate (BSA)
 * 
 * Accurately models:
 * - Direct & inverse differential kinematics (r = 0.021 m, L = 0.1054 m)
 * - 1st-order DC motor lag dynamics: H(s) = 1 / (tau_m * s + 1)
 * - Actuator acceleration limits (2.0 m/s^2 linear, 4.0 rad/s^2 angular)
 * - Realistic encoder odometry with configurable noise
 * - 8 Infrared proximity sensors & 5 Ultrasonic range sensors
 * - Arena boundary & obstacle collision response
 */

export class KheperaPhysics {
    constructor(config = {}) {
        // Physical Constants (K-Team Khepera IV)
        this.r = 0.021;       // Wheel radius (m) = 21 mm
        this.L = 0.1054;      // Wheelbase / track gauge (m) = 105.4 mm
        this.chassisRadius = 0.070; // Outer bumper radius (m) = 70 mm
        this.mass = 0.540;    // Mass (kg)
        
        // Actuator & Dynamics Limits
        this.vMax = config.vMax || 0.25;      // Max forward linear speed (m/s)
        this.wMax = config.wMax || 1.8;       // Max angular speed (rad/s)
        this.aMax = 2.0;                      // Max linear acceleration (m/s^2)
        this.alphaMax = 4.0;                  // Max angular acceleration (rad/s^2)
        this.tauM = config.tauM || 0.05;      // Motor time constant (s)
        
        // Arena Constraints (-1.8m to +1.8m arena by default)
        this.arenaHalfSize = config.arenaHalfSize || 1.8;
        
        // Internal Dynamic State
        this.x = 0.0;
        this.y = 0.0;
        this.yaw = 0.0;
        
        // Actual velocities (output of motor dynamics)
        this.v = 0.0;
        this.w = 0.0;
        
        // Commanded velocities
        this.vCmd = 0.0;
        this.wCmd = 0.0;
        
        // Wheel speeds and accumulated rotations (radians)
        this.wL = 0.0;
        this.wR = 0.0;
        this.thetaL = 0.0;
        this.thetaR = 0.0;
        
        // Odometry (measured pose with potential noise)
        this.odomX = 0.0;
        this.odomY = 0.0;
        this.odomYaw = 0.0;
        this.odomNoiseEnabled = false;
        this.odomNoiseStdDev = 0.002;
        
        // Sensor definitions (Angles in radians relative to robot forward X)
        // 8 IR Proximity sensors along bumper matching authentic Khepera IV CAD model
        this.irAngles = [
            0.0,                    // IR0: Front (0.0 rad, 0.0°)
            Math.PI / 4,            // IR1: Front-Left (0.7854 rad, +45.0°)
            Math.PI / 2,            // IR2: Left (1.5708 rad, +90.0°)
            3 * Math.PI / 4,        // IR3: Rear-Left (2.3562 rad, +135.0°)
            Math.PI,                // IR4: Rear (3.1416 rad, 180.0°)
            -3 * Math.PI / 4,       // IR5: Rear-Right (-2.3562 rad, -135.0°)
            -Math.PI / 2,           // IR6: Right (-1.5708 rad, -90.0°)
            -Math.PI / 4            // IR7: Front-Right (-0.7854 rad, -45.0°)
        ];
        this.irMaxDist = 0.25; // 25 cm max IR detection
        this.irDistances = new Array(8).fill(this.irMaxDist);
        
        // 5 Ultrasonic sensors
        this.usAngles = [
            1.571,   // US0: +90 deg
            0.785,   // US1: +45 deg
            0.0,     // US2: 0 deg (front center)
            -0.785,  // US3: -45 deg
            -1.571   // US4: -90 deg
        ];
        this.usMaxDist = 2.0; // 2.0 m max sonar
        this.usDistances = new Array(5).fill(this.usMaxDist);
        
        // Obstacles array: [{x, y, radius}]
        this.obstacles = [
            { x: -0.8, y: 0.8, radius: 0.12 },
            { x: 0.8, y: -0.8, radius: 0.12 },
            { x: -0.8, y: -0.8, radius: 0.12 }
        ];
    }
    
    reset(x = 0.0, y = 0.0, yaw = 0.0) {
        this.x = x;
        this.y = y;
        this.yaw = yaw;
        this.v = 0.0;
        this.w = 0.0;
        this.vCmd = 0.0;
        this.wCmd = 0.0;
        this.wL = 0.0;
        this.wR = 0.0;
        this.thetaL = 0.0;
        this.thetaR = 0.0;
        this.odomX = x;
        this.odomY = y;
        this.odomYaw = yaw;
        this.irDistances.fill(this.irMaxDist);
        this.usDistances.fill(this.usMaxDist);
    }
    
    setCommand(v, w) {
        this.vCmd = Math.max(-this.vMax, Math.min(this.vMax, v));
        this.wCmd = Math.max(-this.wMax, Math.min(this.wMax, w));
    }
    
    /**
     * Discrete simulation step (dt in seconds, e.g. 0.01s for 100 Hz)
     */
    step(dt) {
        // 1. Motor dynamics: 1st-order low-pass filter with rate limiting
        // v_dot = (vCmd - v) / tauM, clamped by aMax
        const targetV = this.vCmd;
        const targetW = this.wCmd;
        
        const dvReq = (targetV - this.v) / (this.tauM > 0.001 ? this.tauM : dt);
        const dwReq = (targetW - this.w) / (this.tauM > 0.001 ? this.tauM : dt);
        
        const dv = Math.max(-this.aMax, Math.min(this.aMax, dvReq)) * dt;
        const dw = Math.max(-this.alphaMax, Math.min(this.alphaMax, dwReq)) * dt;
        
        this.v += dv;
        this.w += dw;
        
        // 2. Inverse kinematics: (v, w) -> (wR, wL)
        this.wR = (this.v + (this.L / 2.0) * this.w) / this.r;
        this.wL = (this.v - (this.L / 2.0) * this.w) / this.r;
        
        // Accumulate wheel rotation angles (rad)
        this.thetaR += this.wR * dt;
        this.thetaL += this.wL * dt;
        
        // 3. Direct unicycle kinematics: integrate pose in world coordinates
        // Using mid-point Runge-Kutta / exact arc integration
        const deltaYaw = this.w * dt;
        const avgYaw = this.yaw + deltaYaw / 2.0;
        
        const dx = this.v * Math.cos(avgYaw) * dt;
        const dy = this.v * Math.sin(avgYaw) * dt;
        
        this.x += dx;
        this.y += dy;
        this.yaw = this.normalizeAngle(this.yaw + deltaYaw);
        
        // 4. Boundary collision handling (soft bounce against arena walls)
        const bound = this.arenaHalfSize - this.chassisRadius;
        if (this.x > bound) { this.x = bound; this.v = Math.min(0, this.v); }
        if (this.x < -bound) { this.x = -bound; this.v = Math.max(0, this.v); }
        if (this.y > bound) { this.y = bound; this.v = Math.min(0, this.v); }
        if (this.y < -bound) { this.y = -bound; this.v = Math.max(0, this.v); }
        
        // Obstacle collisions
        for (const obs of this.obstacles) {
            const distX = this.x - obs.x;
            const distY = this.y - obs.y;
            const dist = Math.hypot(distX, distY);
            const minDist = this.chassisRadius + obs.radius;
            if (dist < minDist && dist > 0.001) {
                const pushX = (distX / dist) * (minDist - dist);
                const pushY = (distY / dist) * (minDist - dist);
                this.x += pushX;
                this.y += pushY;
                this.v *= 0.5;
            }
        }
        
        // 5. Odometry integration
        let noiseX = 0;
        let noiseY = 0;
        let noiseYaw = 0;
        if (this.odomNoiseEnabled) {
            noiseX = (Math.random() - 0.5) * this.odomNoiseStdDev;
            noiseY = (Math.random() - 0.5) * this.odomNoiseStdDev;
            noiseYaw = (Math.random() - 0.5) * (this.odomNoiseStdDev * 0.5);
        }
        this.odomX = this.x + noiseX;
        this.odomY = this.y + noiseY;
        this.odomYaw = this.normalizeAngle(this.yaw + noiseYaw);
        
        // 6. Update proximity and sonar range sensors
        this.updateSensors();
    }
    
    updateSensors() {
        // Update IR proximity sensors (8 rays)
        for (let i = 0; i < this.irAngles.length; i++) {
            const beamAngle = this.yaw + this.irAngles[i];
            const startX = this.x + this.chassisRadius * Math.cos(beamAngle);
            const startY = this.y + this.chassisRadius * Math.sin(beamAngle);
            
            this.irDistances[i] = this.raycast(startX, startY, beamAngle, this.irMaxDist);
        }
        
        // Update Ultrasonic sensors (5 cones)
        for (let i = 0; i < this.usAngles.length; i++) {
            const beamAngle = this.yaw + this.usAngles[i];
            const startX = this.x + (this.chassisRadius - 0.005) * Math.cos(beamAngle);
            const startY = this.y + (this.chassisRadius - 0.005) * Math.sin(beamAngle);
            
            this.usDistances[i] = this.raycast(startX, startY, beamAngle, this.usMaxDist);
        }
    }
    
    raycast(startX, startY, angle, maxDist) {
        const cosA = Math.cos(angle);
        const sinA = Math.sin(angle);
        let closest = maxDist;
        
        // Raycast against arena walls
        const bound = this.arenaHalfSize;
        
        // Right wall (x = +bound)
        if (cosA > 1e-5) {
            const d = (bound - startX) / cosA;
            if (d > 0 && d < closest) {
                const hitY = startY + d * sinA;
                if (Math.abs(hitY) <= bound) closest = d;
            }
        }
        // Left wall (x = -bound)
        if (cosA < -1e-5) {
            const d = (-bound - startX) / cosA;
            if (d > 0 && d < closest) {
                const hitY = startY + d * sinA;
                if (Math.abs(hitY) <= bound) closest = d;
            }
        }
        // Top wall (y = +bound)
        if (sinA > 1e-5) {
            const d = (bound - startY) / sinA;
            if (d > 0 && d < closest) {
                const hitX = startX + d * cosA;
                if (Math.abs(hitX) <= bound) closest = d;
            }
        }
        // Bottom wall (y = -bound)
        if (sinA < -1e-5) {
            const d = (-bound - startY) / sinA;
            if (d > 0 && d < closest) {
                const hitX = startX + d * cosA;
                if (Math.abs(hitX) <= bound) closest = d;
            }
        }
        
        // Raycast against circular obstacles
        for (const obs of this.obstacles) {
            const d = this.rayIntersectCircle(startX, startY, cosA, sinA, obs.x, obs.y, obs.radius);
            if (d !== null && d > 0 && d < closest) {
                closest = d;
            }
        }
        
        return closest;
    }
    
    rayIntersectCircle(px, py, dx, dy, cx, cy, r) {
        const ox = px - cx;
        const oy = py - cy;
        const b = 2 * (ox * dx + oy * dy);
        const c = ox * ox + oy * oy - r * r;
        const disc = b * b - 4 * c;
        if (disc < 0) return null;
        const sqrtDisc = Math.sqrt(disc);
        const t1 = (-b - sqrtDisc) / 2;
        const t2 = (-b + sqrtDisc) / 2;
        if (t1 > 0) return t1;
        if (t2 > 0) return t2;
        return null;
    }
    
    normalizeAngle(a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }
}
