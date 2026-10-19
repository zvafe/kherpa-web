/**
 * Interactive Variable Explanations & Floating Tooltip System
 * Course: Bazele Sistemelor Automate (BSA)
 * 
 * Provides rich interactive hover menus explaining:
 * - Lyapunov controller gains (k_rho, k_alpha, k_beta, k_yaw)
 * - Polar tracking error coordinates (rho, alpha, beta)
 * - BSA 2nd-order step response indicators (tr, ts, sigma%, est)
 * - Unicycle kinematics (v, w, wheel speeds)
 */

export const VARIABLE_EXPLANATIONS = {
    k_rho: {
        title: "Position Gain",
        symbol: "k_ρ",
        category: "Outer-Loop Lyapunov Gain",
        color: "#38bdf8", // sky blue
        summary: "Controls how aggressively the robot drives forward based on distance error.",
        math: "v^* = \\text{clamp}(k_\\rho \\cdot \\rho, -v_{\\max}, v_{\\max})",
        details: "Scales the desired forward linear velocity proportional to the Euclidean distance (\\rho) between the robot and target beacon. Higher values cause the robot to reach max cruise speed earlier.",
        tuningUp: "Faster acceleration & reduced rise time (t_r), but can cause overshoot or trigger velocity saturation limits.",
        tuningDown: "Sluggish, overly cautious approach; increases settling time (t_s).",
        defaultVal: "0.85"
    },
    k_alpha: {
        title: "Heading Steering Gain",
        symbol: "k_α",
        category: "Outer-Loop Steering Gain",
        color: "#f59e0b", // amber
        summary: "Steers the robot chassis towards the line of sight to the goal.",
        math: "\\omega^* = k_\\alpha \\cdot \\alpha + k_\\beta \\cdot \\beta",
        details: "Corrects the heading angle (\\alpha) between the robot's forward heading and the target beacon vector. If target is behind (>90°), the robot rotates in place before driving.",
        tuningUp: "Sharper, faster turns towards target; excessive values cause snaking oscillations along the path.",
        tuningDown: "Slow turning; robot may drift widely off course before facing the beacon.",
        defaultVal: "2.10"
    },
    k_beta: {
        title: "Bearing / Orientation Gain",
        symbol: "k_β",
        category: "Goal Alignment Gain (Must be < 0)",
        color: "#c084fc", // purple
        summary: "Aligns the robot's approach curve with the final target heading.",
        math: "\\beta = -\\theta - \\alpha + \\theta_{\\text{goal}}, \\quad k_\\beta < 0",
        details: "Flairs the robot's arrival trajectory so that upon reaching the (x, y) beacon, the chassis is already pointing towards the desired final angle θ_goal. Proved asymptotically stable by Lyapunov analysis only when k_β is negative.",
        tuningUp: "Near 0: Robot drives in a straight line, deferring all heading alignment until after arrival.",
        tuningDown: "More negative (e.g. -0.8): Robot carves wide, elegant spiral arcs to arrive pre-aligned.",
        defaultVal: "-0.40"
    },
    k_yaw: {
        title: "In-Place Rotation Gain",
        symbol: "k_yaw",
        category: "Terminal Orientation Gain",
        color: "#34d399", // emerald
        summary: "Pivots the robot on the spot when position tolerance is reached.",
        math: "\\omega^* = \\text{clamp}(k_{\\text{yaw}} \\cdot (\\theta_{\\text{goal}} - \\theta), -\\omega_{\\max}, \\omega_{\\max})",
        details: "Once the robot enters the 2 cm position zone (\\rho ≤ 0.02 m), linear velocity is locked to zero (v = 0) and this gain eliminates any remaining angular error.",
        tuningUp: "Rapid spin to final heading, but risks wheel slip and minor yaw overshoot.",
        tuningDown: "Gentle, creeping final rotation.",
        defaultVal: "1.60"
    },
    rho: {
        title: "Distance Error",
        symbol: "ρ (rho)",
        category: "Polar Error Coordinate",
        color: "#f43f5e", // rose
        summary: "Euclidean distance between the robot's center and the target beacon.",
        math: "\\rho = \\sqrt{(g_x - x)^2 + (g_y - y)^2}",
        details: "Monitored at 50 Hz. Navigation successfully completes Phase 1 when distance error drops below the position tolerance threshold (2.0 cm).",
        tuningUp: "Target is far away; controller commands max forward velocity.",
        tuningDown: "Target is near; controller decelerates smoothly towards zero.",
        defaultVal: "Tolerance: 0.02 m"
    },
    alpha: {
        title: "Heading Error",
        symbol: "α (alpha)",
        category: "Polar Error Coordinate",
        color: "#f59e0b", // amber
        summary: "Angle between robot forward axis and line of sight to the beacon.",
        math: "\\alpha = \\text{atan2}(g_y - y, g_x - x) - \\theta",
        details: "When α = 0°, the robot is pointing directly at the beacon. Normalized to [-π, +π] radians at every control cycle.",
        tuningUp: "Positive: Target is to the robot's left.",
        tuningDown: "Negative: Target is to the robot's right.",
        defaultVal: "0.0°"
    },
    beta: {
        title: "Target Bearing Error",
        symbol: "β (beta)",
        category: "Polar Error Coordinate",
        color: "#cbd5e1", // slate
        summary: "Angle between line of sight and the final desired target orientation.",
        math: "\\beta = -\\theta - \\alpha + \\theta_{\\text{goal}}",
        details: "Guides the curvature of the approach path so the robot does not need to turn after stopping.",
        tuningUp: "Shapes path curvature during navigation.",
        tuningDown: "Zeroes out as robot aligns with target pose.",
        defaultVal: "0.0°"
    },
    pose_xy: {
        title: "Cartesian Position",
        symbol: "(x, y)",
        category: "World Coordinates",
        color: "#38bdf8", // sky
        summary: "Position of the robot center in the laboratory floor coordinate system.",
        math: "x = \\int v \\cos(\\theta) dt, \\quad y = \\int v \\sin(\\theta) dt",
        details: "Derived from optical wheel encoders and DART ground contact physics, matching the ROS 2 `/odom` topic at 50 Hz.",
        tuningUp: "+X is Forward, +Y is Left in ROS 2 frame.",
        tuningDown: "Origin (0,0) is located at the center of the laboratory floor.",
        defaultVal: "(0.0, 0.0)"
    },
    yaw: {
        title: "Chassis Heading",
        symbol: "θ (theta / yaw)",
        category: "Robot Orientation",
        color: "#c084fc", // purple
        summary: "Counter-clockwise angle of the robot's forward axis relative to +X.",
        math: "\\theta = \\int \\omega \\, dt = \\frac{r}{L} \\int (\\omega_R - \\omega_L) dt",
        details: "Measured in degrees and radians. Calculated via optical encoder differential integration.",
        tuningUp: "0°: Pointing along +X axis (East).",
        tuningDown: "90°: Pointing along +Y axis (North).",
        defaultVal: "0.0°"
    },
    speed: {
        title: "Chassis Velocities",
        symbol: "(v, ω)",
        category: "Differential Kinematics",
        color: "#34d399", // emerald
        summary: "Linear forward speed (v in m/s) and angular yaw rate (ω in rad/s).",
        math: "v = \\frac{r}{2}(\\omega_R + \\omega_L), \\quad \\omega = \\frac{r}{L}(\\omega_R - \\omega_L)",
        details: "Output of the outer-loop cascade controller, matching the ROS 2 `/cmd_vel` Twist topic. Transferred to wheel actuators via inverse kinematics.",
        tuningUp: "Max Linear Speed: 0.25 m/s (25 cm/s).",
        tuningDown: "Max Angular Speed: 1.80 rad/s (~103 °/s).",
        defaultVal: "0.00 m/s, 0.00 rad/s"
    },
    overshoot: {
        title: "Maximum Overshoot",
        symbol: "σ% (sigma)",
        category: "BSA Academic Metric",
        color: "#f59e0b", // amber
        summary: "Peak distance the robot travels beyond the target setpoint.",
        math: "\\sigma = e^{-\\frac{\\pi\\zeta}{\\sqrt{1-\\zeta^2}}} \\times 100\\%",
        details: "Indicates whether the controller is underdamped. The optimal BSA damping ratio (ζ = 0.707) guarantees overshoot strictly ≤ 4.3%.",
        tuningUp: "> 4.3%: System is underdamped (ζ < 0.7); gain k_ρ is too high.",
        tuningDown: "0%: Well-damped / critically damped approach with zero overshoot.",
        defaultVal: "Goal: ≤ 4.3%"
    },
    est: {
        title: "Steady-State Error",
        symbol: "e_st",
        category: "BSA Academic Metric",
        color: "#34d399", // emerald
        summary: "Final residual position error after coming to a complete stop.",
        math: "e_{st} = \\lim_{t \\to \\infty} |r(t) - y(t)| \\le 2\\text{ cm}",
        details: "Measures accuracy of final positioning. In physical robots, static friction and deadband cause small residual offsets; our closed loop guarantees est ≤ 2.0 cm.",
        tuningUp: "> 2 cm: Position tolerance not yet achieved.",
        tuningDown: "≤ 2 cm: Optimal stopping within acceptable lab tolerance.",
        defaultVal: "≤ 2.0 cm"
    },
    tr: {
        title: "Rise Time",
        symbol: "t_r",
        category: "BSA Academic Metric",
        color: "#38bdf8", // sky
        summary: "Time required to cover from 10% to 90% of the target step distance.",
        math: "t_r \\approx \\frac{\\pi - \\arccos(\\zeta)}{\\omega_n \\sqrt{1-\\zeta^2}}",
        details: "Evaluates responsiveness and actuator acceleration. Lower rise time means the robot accelerates briskly towards the setpoint.",
        tuningUp: "Higher: Sluggish start or acceleration limited.",
        tuningDown: "Lower: Quick initial acceleration.",
        defaultVal: "Typical: 1.5 - 2.8 s"
    },
    ts: {
        title: "Settling Time",
        symbol: "t_s",
        category: "BSA Academic Metric",
        color: "#c084fc", // purple
        summary: "Time after which error remains permanently inside the ±2% tolerance band.",
        math: "t_s \\approx \\frac{4}{\\zeta \\omega_n} \\quad (2\\% \\text{ criterion})",
        details: "The definitive speed benchmark of automated regulation taught in BSA course. Optimal ζ = 0.707 achieves the mathematically fastest settling time.",
        tuningUp: "Longer: Oscillations or slow deceleration tail.",
        tuningDown: "Shorter: Fast, crisp convergence into the target zone.",
        defaultVal: "Typical: 2.5 - 4.5 s"
    }
};

/**
 * Initializes the floating interactive hover menu system
 */
export function initVariableExplainer() {
    // 1. Create floating tooltip container in body if not present
    let tooltip = document.getElementById('explain-tooltip');
    if (!tooltip) {
        tooltip = document.createElement('div');
        tooltip.id = 'explain-tooltip';
        tooltip.className = 'fixed z-50 pointer-events-none transition-all duration-300 ease-out opacity-0 scale-95 shadow-2xl';
        tooltip.style.display = 'none';
        tooltip.style.maxWidth = '380px';
        tooltip.style.left = '-9999px';
        tooltip.style.top = '-9999px';
        document.body.appendChild(tooltip);
    }
    
    let hideTimer = null;
    let dwellTimer = null;
    let activeTriggerEl = null;
    
    function clearDwellTimer() {
        if (dwellTimer) {
            clearTimeout(dwellTimer);
            dwellTimer = null;
        }
    }
    
    function cancelHide() {
        if (hideTimer) {
            clearTimeout(hideTimer);
            hideTimer = null;
        }
    }
    
    function hideTooltipNow() {
        cancelHide();
        clearDwellTimer();
        activeTriggerEl = null;
        
        // IMMEDIATELY disable pointer events so the invisible box never blocks re-hovering
        tooltip.style.pointerEvents = 'none';
        tooltip.classList.remove('opacity-100', 'scale-100');
        tooltip.classList.add('opacity-0', 'scale-95');
        
        hideTimer = setTimeout(() => {
            tooltip.style.display = 'none';
            tooltip.style.left = '-9999px';
            tooltip.style.top = '-9999px';
            hideTimer = null;
        }, 300);
    }
    
    function scheduleHide() {
        cancelHide();
        clearDwellTimer();
        // Give 250ms grace period for user to move mouse into tooltip
        hideTimer = setTimeout(() => {
            hideTooltipNow();
        }, 250);
    }
    
    // Only capture pointer events when mouse enters an already visible tooltip
    tooltip.addEventListener('mouseenter', () => {
        cancelHide();
        tooltip.style.pointerEvents = 'auto';
    });
    tooltip.addEventListener('mouseleave', scheduleHide);
    
    function showTooltip(el, varKey) {
        const info = VARIABLE_EXPLANATIONS[varKey];
        if (!info) return;
        
        cancelHide();
        clearDwellTimer();
        activeTriggerEl = el;
        
        // Render math via KaTeX if available
        let renderedFormula = info.math;
        if (window.katex && typeof window.katex.renderToString === 'function') {
            try {
                renderedFormula = window.katex.renderToString(info.math, {
                    displayMode: true,
                    throwOnError: false
                });
            } catch (err) {
                console.warn("KaTeX render error:", err);
            }
        }
        
        // Render rich tooltip card matching the user's reference GIF
        tooltip.innerHTML = `
            <div class="bg-[#0b101b]/95 backdrop-blur-md border border-slate-700/80 rounded-xl p-4 text-xs shadow-2xl text-slate-200 space-y-2.5">
                <!-- Header: Badge + Symbol + Title -->
                <div class="flex items-center justify-between border-b border-slate-800 pb-2">
                    <div class="flex items-center gap-2">
                        <span class="w-2.5 h-2.5 rounded-full" style="background-color: ${info.color};"></span>
                        <span class="font-bold text-white text-sm tracking-tight">${info.title}</span>
                        <code class="font-mono text-xs px-1.5 py-0.5 rounded bg-slate-800 text-sky-300 font-bold">${info.symbol}</code>
                    </div>
                    <span class="text-[10px] font-mono px-2 py-0.5 rounded bg-slate-800/80 text-slate-400 border border-slate-700/60">
                        ${info.category}
                    </span>
                </div>
                
                <!-- Summary & Details -->
                <p class="text-slate-300 text-[11.5px] leading-relaxed">
                    ${info.summary}
                </p>
                
                <!-- Formula Box with KaTeX Rendered Math -->
                <div class="bg-[#121824] p-2.5 rounded-lg border border-slate-800 text-sky-300 overflow-x-auto text-center flex items-center justify-center min-h-[44px]">
                    ${renderedFormula}
                </div>
                
                <!-- Physical Details -->
                <p class="text-slate-400 text-[11px] leading-relaxed">
                    ${info.details}
                </p>
                
                <!-- Tuning Impact Guide -->
                <div class="bg-[#151c2c] p-2.5 rounded-lg border border-slate-800 space-y-1.5 text-[10.5px]">
                    <div class="flex items-start gap-1.5">
                        <span class="text-emerald-400 font-bold shrink-0">▲ Increase:</span>
                        <span class="text-slate-300">${info.tuningUp}</span>
                    </div>
                    <div class="flex items-start gap-1.5">
                        <span class="text-amber-400 font-bold shrink-0">▼ Decrease:</span>
                        <span class="text-slate-300">${info.tuningDown}</span>
                    </div>
                </div>
                
                <!-- Footer with Recommended Setting -->
                <div class="flex items-center justify-between pt-1 text-[10.5px] text-slate-500">
                    <span>Course: Bazele Sistemelor Automate</span>
                    <span class="font-mono text-emerald-400 font-semibold">Recommended: ${info.defaultVal}</span>
                </div>
            </div>
        `;
        
        // Position tooltip relative to element with smart viewport clamping
        tooltip.style.display = 'block';
        const rect = el.getBoundingClientRect();
        const tooltipW = 380;
        const margin = 16;
        
        // Preferred left: centered under/over element
        let left = rect.left + rect.width / 2 - tooltipW / 2;
        
        // If element is on the left side, align closer to its left edge
        if (rect.left < tooltipW / 2 + margin) {
            left = Math.max(margin, rect.left - 12);
        } else if (rect.right > window.innerWidth - tooltipW / 2 - margin) {
            left = Math.min(window.innerWidth - tooltipW - margin, rect.right + 12 - tooltipW);
        }
        
        // Clamping to screen boundaries
        if (left < margin) left = margin;
        if (left + tooltipW > window.innerWidth - margin) {
            left = window.innerWidth - tooltipW - margin;
        }
        
        // Prefer placing above; if near top or clipped, place below
        let top = rect.top - 12;
        const estimatedH = 290;
        if (top - estimatedH < margin) {
            // Place below element
            top = rect.bottom + 10;
        } else {
            // Place above element
            top = rect.top - estimatedH;
        }
        
        tooltip.style.left = `${left}px`;
        tooltip.style.top = `${Math.max(margin, top)}px`;
        tooltip.style.width = `${tooltipW}px`;
        
        // Force reflow before removing opacity-0 for smooth CSS fade-in
        void tooltip.offsetHeight;
        
        // Fade in with scale
        tooltip.classList.remove('opacity-0', 'scale-95');
        tooltip.classList.add('opacity-100', 'scale-100');
        tooltip.style.pointerEvents = 'auto';
    }
    
    // Attach event listeners to all elements with data-var
    document.querySelectorAll('[data-var]').forEach(el => {
        const varKey = el.getAttribute('data-var');
        
        // Add subtle dotted underline and cursor-help styling
        el.classList.add('cursor-help', 'transition-colors', 'duration-300');
        
        el.addEventListener('mouseenter', () => {
            // Cancel any pending hide
            cancelHide();
            clearDwellTimer();
            
            // Visual feedback: highlight variable while waiting
            el.classList.add('text-sky-300', 'border-sky-400');
            
            // Start 3-second dwell timer as requested
            dwellTimer = setTimeout(() => {
                showTooltip(el, varKey);
            }, 3000); // 3 seconds hover required
        });
        
        el.addEventListener('mouseleave', () => {
            // User moved mouse out
            clearDwellTimer();
            el.classList.remove('text-sky-300', 'border-sky-400');
            
            // If tooltip is showing for this element, schedule hide
            if (activeTriggerEl === el) {
                scheduleHide();
            }
        });
        
        // Click to open immediately (convenience bypass for 3s wait)
        el.addEventListener('click', (e) => {
            e.stopPropagation();
            clearDwellTimer();
            showTooltip(el, varKey);
        });
        
        // Support touch taps on mobile / tablets
        el.addEventListener('touchstart', (e) => {
            e.stopPropagation();
            clearDwellTimer();
            showTooltip(el, varKey);
        }, { passive: true });
    });
    
    // Tap outside or click background to close
    window.addEventListener('click', (e) => {
        if (!e.target.closest('[data-var]') && !e.target.closest('#explain-tooltip')) {
            hideTooltipNow();
        }
    });
    window.addEventListener('touchstart', (e) => {
        if (!e.target.closest('[data-var]') && !e.target.closest('#explain-tooltip')) {
            hideTooltipNow();
        }
    }, { passive: true });
}
