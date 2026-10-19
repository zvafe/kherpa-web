/**
 * Khepera IV 3D WebGL Renderer (Three.js)
 * Course: Bazele Sistemelor Automate (BSA)
 * 
 * Renders:
 * - High-fidelity 3D Khepera IV robot model (Chassis, wheels, casters, sensors)
 * - Rotating wheels proportional to integrated wheel odometry
 * - 3D Target Beacon with pulsing glow ring (identical to Gazebo SDF)
 * - Click-to-Goal 3D ground plane raycasting
 * - Real-time 3D trajectory ribbon trail
 * - Proximity IR & Sonar sensor beams radiating in 3D
 * - Picture-in-Picture (PiP) WVGA CMOS front camera view
 * - Multiple camera modes (Orbit, Follow-Cam, Top-Down)
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { MTLLoader } from 'three/addons/loaders/MTLLoader.js';

export class Khepera3DScene {
    constructor(canvasMain, canvasPip, onGroundClickCallback) {
        this.canvasMain = canvasMain;
        this.canvasPip = canvasPip;
        this.onGroundClick = onGroundClickCallback;
        
        this.cameraMode = 'orbit'; // 'orbit', 'follow', 'top'
        
        // Setup Three.js Scene & Renderers
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0a0e17);
        this.scene.fog = new THREE.FogExp2(0x0a0e17, 0.08);
        
        // Main Viewport Camera
        const aspect = canvasMain.clientWidth / canvasMain.clientHeight;
        this.camera = new THREE.PerspectiveCamera(45, aspect, 0.01, 50);
        this.camera.position.set(1.4, 1.2, 1.6);
        
        this.renderer = new THREE.WebGLRenderer({
            canvas: this.canvasMain,
            antialias: true,
            powerPreference: 'high-performance'
        });
        this.renderer.setSize(canvasMain.clientWidth, canvasMain.clientHeight, false);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        
        // Orbit Controls
        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.maxPolarAngle = Math.PI / 2 - 0.02; // Don't clip through floor
        this.controls.minDistance = 0.2;
        this.controls.maxDistance = 10.0;
        this.controls.target.set(0, 0.05, 0);
        
        // Picture-in-Picture Front WVGA Camera
        this.pipCamera = new THREE.PerspectiveCamera(60, 640 / 480, 0.01, 15);
        this.pipRenderer = new THREE.WebGLRenderer({
            canvas: this.canvasPip,
            antialias: true
        });
        this.pipRenderer.setSize(canvasPip.clientWidth, canvasPip.clientHeight, false);
        this.pipRenderer.setPixelRatio(1);
        
        // Raycaster for Ground Clicking
        this.raycaster = new THREE.Raycaster();
        this.mouse = new THREE.Vector2();
        this.setupGroundRaycasting();
        
        // Build Environment (Lights, Grid, Arena, Obstacles)
        this.buildEnvironment();
        
        // Build Robot Hierarchy & 3D Assets
        this.buildRobotHierarchy();
        
        // Build 3D Beacon (Gazebo goal_marker replica)
        this.buildGoalBeacon();
        
        // Build Trajectory Trail Ribbon
        this.buildTrajectoryTrail();
        
        // Build Proximity Sensor Rays
        this.buildSensorRays();
        
        // Handle Window Resizing
        window.addEventListener('resize', () => this.onWindowResize());
    }
    
    buildEnvironment() {
        // 1. Lighting (Sun + Fill + Ambient)
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
        this.scene.add(ambientLight);
        
        const sunLight = new THREE.DirectionalLight(0xffffff, 1.4);
        sunLight.position.set(3, 6, 4);
        sunLight.castShadow = true;
        sunLight.shadow.mapSize.width = 1024;
        sunLight.shadow.mapSize.height = 1024;
        sunLight.shadow.camera.near = 0.5;
        sunLight.shadow.camera.far = 15;
        sunLight.shadow.camera.left = -2.5;
        sunLight.shadow.camera.right = 2.5;
        sunLight.shadow.camera.top = 2.5;
        sunLight.shadow.camera.bottom = -2.5;
        sunLight.shadow.bias = -0.001;
        this.scene.add(sunLight);
        
        const blueRimLight = new THREE.DirectionalLight(0x0ea5e9, 0.5);
        blueRimLight.position.set(-4, 3, -3);
        this.scene.add(blueRimLight);
        
        // 2. Ground Plane
        const groundGeo = new THREE.PlaneGeometry(12, 12);
        const groundMat = new THREE.MeshStandardMaterial({
            color: 0x111622,
            roughness: 0.8,
            metalness: 0.2
        });
        this.groundMesh = new THREE.Mesh(groundGeo, groundMat);
        this.groundMesh.rotation.x = -Math.PI / 2;
        this.groundMesh.receiveShadow = true;
        this.scene.add(this.groundMesh);
        
        // 3. High-Tech Grid Floors
        // Minor 10cm grid
        const gridMinor = new THREE.GridHelper(10, 100, 0x1e293b, 0x1e293b);
        gridMinor.position.y = 0.001;
        this.scene.add(gridMinor);
        
        // Major 1m grid
        const gridMajor = new THREE.GridHelper(10, 10, 0x38bdf8, 0x1e293b);
        gridMajor.position.y = 0.002;
        this.scene.add(gridMajor);
        
        // 4. Arena Boundary Walls (3.6m x 3.6m)
        const arenaSize = 3.6;
        const wallHeight = 0.08;
        const wallMat = new THREE.MeshStandardMaterial({
            color: 0x0284c7,
            roughness: 0.3,
            metalness: 0.8,
            transparent: true,
            opacity: 0.85
        });
        
        const wallsGroup = new THREE.Group();
        const wallThickness = 0.04;
        
        // Top and bottom walls
        const wallHGeo = new THREE.BoxGeometry(arenaSize + wallThickness, wallHeight, wallThickness);
        const wallTop = new THREE.Mesh(wallHGeo, wallMat);
        wallTop.position.set(0, wallHeight / 2, arenaSize / 2);
        wallTop.castShadow = true;
        wallTop.receiveShadow = true;
        
        const wallBottom = new THREE.Mesh(wallHGeo, wallMat);
        wallBottom.position.set(0, wallHeight / 2, -arenaSize / 2);
        wallBottom.castShadow = true;
        wallBottom.receiveShadow = true;
        
        // Left and right walls
        const wallVGeo = new THREE.BoxGeometry(wallThickness, wallHeight, arenaSize);
        const wallLeft = new THREE.Mesh(wallVGeo, wallMat);
        wallLeft.position.set(-arenaSize / 2, wallHeight / 2, 0);
        wallLeft.castShadow = true;
        wallLeft.receiveShadow = true;
        
        const wallRight = new THREE.Mesh(wallVGeo, wallMat);
        wallRight.position.set(arenaSize / 2, wallHeight / 2, 0);
        wallRight.castShadow = true;
        wallRight.receiveShadow = true;
        
        wallsGroup.add(wallTop, wallBottom, wallLeft, wallRight);
        this.scene.add(wallsGroup);
        
        // 5. Arena Obstacle Pillars (Matching physics obstacles)
        const obsPositions = [
            { x: -0.8, z: -0.8 },
            { x: 0.8, z: 0.8 },
            { x: -0.8, z: 0.8 }
        ];
        const pillarGeo = new THREE.CylinderGeometry(0.12, 0.12, 0.18, 32);
        const pillarMat = new THREE.MeshStandardMaterial({
            color: 0x334155,
            roughness: 0.4,
            metalness: 0.6
        });
        const stripeMat = new THREE.MeshBasicMaterial({ color: 0xf59e0b });
        
        obsPositions.forEach(pos => {
            const pillar = new THREE.Mesh(pillarGeo, pillarMat);
            pillar.position.set(pos.x, 0.09, pos.z);
            pillar.castShadow = true;
            pillar.receiveShadow = true;
            
            // Warning ring
            const ringGeo = new THREE.CylinderGeometry(0.122, 0.122, 0.04, 32);
            const ring = new THREE.Mesh(ringGeo, stripeMat);
            ring.position.y = 0.03;
            pillar.add(ring);
            
            this.scene.add(pillar);
        });
    }
    
    buildRobotHierarchy() {
        // Robot root group (world position and yaw)
        this.robotGroup = new THREE.Group();
        this.robotGroup.position.set(0, 0, 0);
        this.scene.add(this.robotGroup);
        
        // Left & Right Wheel groups (spin independently)
        this.leftWheelGroup = new THREE.Group();
        this.rightWheelGroup = new THREE.Group();
        
        // Wheel offsets from URDF: r = 0.021, L = 0.1054 -> lateral Y = +/- 0.0527
        // In our Three.js frame: X is forward, Y is up, Z is lateral left/right
        this.leftWheelGroup.position.set(0, 0.021, -0.0527);
        this.rightWheelGroup.position.set(0, 0.021, 0.0527);
        
        this.robotGroup.add(this.leftWheelGroup);
        this.robotGroup.add(this.rightWheelGroup);
        
        // Attach PiP camera to WVGA camera link position: (0.0682, 0.0283, 0)
        this.pipCameraGroup = new THREE.Group();
        this.pipCameraGroup.position.set(0.0682, 0.0283, 0);
        // Look along robot's forward vector (+X)
        this.pipCamera.position.set(0, 0, 0);
        this.pipCamera.rotation.set(0, -Math.PI / 2, 0); // Rotate to face forward
        this.pipCameraGroup.add(this.pipCamera);
        this.robotGroup.add(this.pipCameraGroup);
        
        // Build High-Fidelity Procedural Model (Immediate rendering with exact dimensions)
        this.proceduralChassis = this.createProceduralKhepera();
        this.robotGroup.add(this.proceduralChassis);
        
        // Try loading real OBJ/MTL assets
        this.loadMeshAssets();
    }
    
    createProceduralKhepera() {
        const root = new THREE.Group();
        
        // Dimensions from URDF:
        // Chassis Diameter = 0.140 m, Height = 0.0538 m, Clearance = 0.0045 m
        const chassisRadius = 0.070;
        const chassisHeight = 0.0538;
        const groundClearance = 0.0045;
        const centerY = groundClearance + chassisHeight / 2;
        
        // 1. Lower Blue Bumper Ring (Khepera IV iconic cyan/blue bumper)
        const bumperHeight = 0.022;
        const bumperGeo = new THREE.CylinderGeometry(chassisRadius, chassisRadius, bumperHeight, 48);
        const bumperMat = new THREE.MeshStandardMaterial({
            color: 0x0284c7, // Khepera blue
            roughness: 0.35,
            metalness: 0.5
        });
        const bumperMesh = new THREE.Mesh(bumperGeo, bumperMat);
        bumperMesh.position.y = groundClearance + bumperHeight / 2;
        bumperMesh.castShadow = true;
        root.add(bumperMesh);
        
        // 2. Upper White Shell
        const upperHeight = chassisHeight - bumperHeight;
        const upperGeo = new THREE.CylinderGeometry(chassisRadius - 0.001, chassisRadius, upperHeight, 48);
        const upperMat = new THREE.MeshStandardMaterial({
            color: 0xf1f5f9, // Matte lab white
            roughness: 0.3,
            metalness: 0.1
        });
        const upperMesh = new THREE.Mesh(upperGeo, upperMat);
        upperMesh.position.y = groundClearance + bumperHeight + upperHeight / 2;
        upperMesh.castShadow = true;
        root.add(upperMesh);
        
        // 3. Top Turret Plate & Front Notch
        const topPlateGeo = new THREE.CylinderGeometry(chassisRadius - 0.004, chassisRadius - 0.001, 0.003, 48);
        const topPlateMat = new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.5 });
        const topPlate = new THREE.Mesh(topPlateGeo, topPlateMat);
        topPlate.position.y = groundClearance + chassisHeight + 0.0015;
        root.add(topPlate);
        
        // 4. Front WVGA Camera Lens & Housing
        const camHousingGeo = new THREE.BoxGeometry(0.016, 0.014, 0.024);
        const camHousingMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.8 });
        const camHousing = new THREE.Mesh(camHousingGeo, camHousingMat);
        camHousing.position.set(chassisRadius - 0.008, centerY + 0.004, 0);
        
        const lensGeo = new THREE.CylinderGeometry(0.004, 0.004, 0.004, 24);
        const lensMat = new THREE.MeshStandardMaterial({ color: 0x38bdf8, roughness: 0.1, metalness: 0.9 });
        const lens = new THREE.Mesh(lensGeo, lensMat);
        lens.rotation.z = Math.PI / 2;
        lens.position.x = 0.009;
        camHousing.add(lens);
        root.add(camHousing);
        
        // 5. Ultrasonic Sensor Transducers (5 gold/brass cylinders on top)
        const usGeo = new THREE.CylinderGeometry(0.0045, 0.0045, 0.005, 16);
        const usMat = new THREE.MeshStandardMaterial({ color: 0xfbbf24, metalness: 0.8, roughness: 0.3 });
        const usAngles = [Math.PI / 2, Math.PI / 4, 0, -Math.PI / 4, -Math.PI / 2];
        usAngles.forEach(ang => {
            const us = new THREE.Mesh(usGeo, usMat);
            us.rotation.z = Math.PI / 2;
            us.rotation.y = ang;
            us.position.set((chassisRadius - 0.006) * Math.cos(ang), groundClearance + chassisHeight - 0.008, (chassisRadius - 0.006) * Math.sin(ang));
            root.add(us);
        });
        
        // 6. Wheels (Left & Right)
        const wheelR = 0.021;
        const wheelW = 0.008;
        const tireGeo = new THREE.CylinderGeometry(wheelR, wheelR, wheelW, 32);
        tireGeo.rotateX(Math.PI / 2); // Align axle along Z
        
        const tireMat = new THREE.MeshStandardMaterial({ color: 0x18181b, roughness: 0.9 });
        const rimMat = new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.8, roughness: 0.2 });
        
        // Left wheel mesh
        const leftTire = new THREE.Mesh(tireGeo, tireMat);
        leftTire.castShadow = true;
        const leftRim = new THREE.Mesh(new THREE.CylinderGeometry(wheelR * 0.6, wheelR * 0.6, wheelW + 0.001, 24).rotateX(Math.PI / 2), rimMat);
        leftTire.add(leftRim);
        this.leftWheelGroup.add(leftTire);
        
        // Right wheel mesh
        const rightTire = new THREE.Mesh(tireGeo, tireMat);
        rightTire.castShadow = true;
        const rightRim = new THREE.Mesh(new THREE.CylinderGeometry(wheelR * 0.6, wheelR * 0.6, wheelW + 0.001, 24).rotateX(Math.PI / 2), rimMat);
        rightTire.add(rightRim);
        this.rightWheelGroup.add(rightTire);
        
        // 7. Passive Chrome Casters (Front & Rear Spheres)
        const casterGeo = new THREE.SphereGeometry(0.005, 24, 24);
        const casterMat = new THREE.MeshStandardMaterial({ color: 0xe2e8f0, metalness: 0.95, roughness: 0.05 });
        
        const frontCaster = new THREE.Mesh(casterGeo, casterMat);
        frontCaster.position.set(0.0505, 0.005, 0);
        frontCaster.castShadow = true;
        root.add(frontCaster);
        
        const rearCaster = new THREE.Mesh(casterGeo, casterMat);
        rearCaster.position.set(-0.0505, 0.005, 0);
        rearCaster.castShadow = true;
        root.add(rearCaster);
        
        // 8. Glowing Power & Status LEDs
        const ledGeo = new THREE.SphereGeometry(0.002, 12, 12);
        const ledGreen = new THREE.Mesh(ledGeo, new THREE.MeshBasicMaterial({ color: 0x22c55e }));
        ledGreen.position.set(-0.03, groundClearance + chassisHeight + 0.003, 0.015);
        root.add(ledGreen);
        
        const ledRed = new THREE.Mesh(ledGeo, new THREE.MeshBasicMaterial({ color: 0xef4444 }));
        ledRed.position.set(-0.03, groundClearance + chassisHeight + 0.003, -0.015);
        root.add(ledRed);
        
        return root;
    }
    
    loadMeshAssets() {
        const mtlLoader = new MTLLoader();
        mtlLoader.setPath('assets/models/');
        
        mtlLoader.load('khepera4_chassis.mtl', (materials) => {
            materials.preload();
            const objLoader = new OBJLoader();
            objLoader.setMaterials(materials);
            objLoader.setPath('assets/models/');
            objLoader.load('khepera4_chassis.obj', (obj) => {
                // Orient authentic CAD mesh to match Three.js frame:
                // In OBJ file: +X is Forward, +Y is Left, +Z is Up
                // In Three.js robotGroup: +X is Forward, -Z is Left, +Y is Up
                // Rotating by -90 deg around X maps: X -> X (Forward), Z -> Y (Up), Y -> -Z (Left)
                obj.rotation.set(-Math.PI / 2, 0, 0);
                obj.position.set(0, 0, 0);
                obj.scale.set(1.0, 1.0, 1.0);
                
                obj.traverse((child) => {
                    if (child.isMesh) {
                        child.castShadow = true;
                        child.receiveShadow = true;
                    }
                });
                
                // Replace procedural chassis with authentic 3D CAD mesh
                this.robotGroup.remove(this.proceduralChassis);
                this.robotGroup.add(obj);
                console.log("Loaded authentic Khepera IV CAD 3D Mesh successfully!");
            }, undefined, (err) => {
                console.log("Using high-fidelity procedural Khepera model (OBJ fallback).", err);
            });
        }, undefined, (err) => {
            console.log("Using procedural materials.", err);
        });
    }
    
    buildGoalBeacon() {
        // Gazebo SDF Goal Marker Replica
        this.beaconGroup = new THREE.Group();
        this.beaconGroup.position.set(0.6, 0, 0);
        this.scene.add(this.beaconGroup);
        
        // Ground landing ring (emerald green)
        const ringGeo = new THREE.RingGeometry(0.06, 0.08, 32);
        const ringMat = new THREE.MeshBasicMaterial({
            color: 0x10b981,
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.8
        });
        const ring = new THREE.Mesh(ringGeo, ringMat);
        ring.rotation.x = -Math.PI / 2;
        ring.position.y = 0.003;
        this.beaconGroup.add(ring);
        
        // Center red bullseye dot
        const dotGeo = new THREE.CircleGeometry(0.02, 24);
        const dotMat = new THREE.MeshBasicMaterial({ color: 0xef4444, side: THREE.DoubleSide });
        const dot = new THREE.Mesh(dotGeo, dotMat);
        dot.rotation.x = -Math.PI / 2;
        dot.position.y = 0.004;
        this.beaconGroup.add(dot);
        
        // Slender vertical flag pin
        const pinGeo = new THREE.CylinderGeometry(0.003, 0.003, 0.16, 16);
        const pinMat = new THREE.MeshStandardMaterial({ color: 0x38bdf8, metalness: 0.8 });
        const pin = new THREE.Mesh(pinGeo, pinMat);
        pin.position.y = 0.08;
        pin.castShadow = true;
        this.beaconGroup.add(pin);
        
        // Glowing top beacon sphere
        const sphereGeo = new THREE.SphereGeometry(0.018, 24, 24);
        const sphereMat = new THREE.MeshStandardMaterial({
            color: 0xf59e0b,
            emissive: 0xf59e0b,
            emissiveIntensity: 0.8,
            roughness: 0.2
        });
        this.beaconSphere = new THREE.Mesh(sphereGeo, sphereMat);
        this.beaconSphere.position.y = 0.16;
        this.beaconGroup.add(this.beaconSphere);
        
        // Subtle point light from beacon
        this.beaconLight = new THREE.PointLight(0xf59e0b, 0.8, 1.2);
        this.beaconLight.position.y = 0.16;
        this.beaconGroup.add(this.beaconLight);
    }
    
    setBeaconPosition(x, y) {
        // Map 2D world coords (X, Y) to Three.js (X, 0, -Y)
        this.beaconGroup.position.set(x, 0, -y);
    }
    
    buildTrajectoryTrail() {
        this.maxTrailPoints = 300;
        this.trailPositions = new Float32Array(this.maxTrailPoints * 3);
        this.trailCount = 0;
        
        const trailGeo = new THREE.BufferGeometry();
        trailGeo.setAttribute('position', new THREE.BufferAttribute(this.trailPositions, 3));
        
        const trailMat = new THREE.LineBasicMaterial({
            color: 0x38bdf8,
            linewidth: 2,
            transparent: true,
            opacity: 0.75
        });
        
        this.trailLine = new THREE.Line(trailGeo, trailMat);
        this.scene.add(this.trailLine);
    }
    
    clearTrail() {
        this.trailCount = 0;
        this.trailLine.geometry.setDrawRange(0, 0);
        this.trailLine.geometry.attributes.position.needsUpdate = true;
    }
    
    addTrailPoint(x, z) {
        if (this.trailCount < this.maxTrailPoints) {
            const idx = this.trailCount * 3;
            this.trailPositions[idx] = x;
            this.trailPositions[idx + 1] = 0.005; // Slightly above ground
            this.trailPositions[idx + 2] = z;
            this.trailCount++;
        } else {
            // Shift array
            for (let i = 0; i < (this.maxTrailPoints - 1) * 3; i++) {
                this.trailPositions[i] = this.trailPositions[i + 3];
            }
            const idx = (this.maxTrailPoints - 1) * 3;
            this.trailPositions[idx] = x;
            this.trailPositions[idx + 1] = 0.005;
            this.trailPositions[idx + 2] = z;
        }
        
        this.trailLine.geometry.setDrawRange(0, this.trailCount);
        this.trailLine.geometry.attributes.position.needsUpdate = true;
    }
    
    buildSensorRays() {
        this.sensorRaysGroup = new THREE.Group();
        this.scene.add(this.sensorRaysGroup);
        
        // 8 IR proximity laser lines
        this.irRayLines = [];
        const lineMat = new THREE.LineBasicMaterial({ color: 0x10b981, transparent: true, opacity: 0.6 });
        
        for (let i = 0; i < 8; i++) {
            const geo = new THREE.BufferGeometry().setFromPoints([
                new THREE.Vector3(0, 0, 0),
                new THREE.Vector3(0.25, 0, 0)
            ]);
            const line = new THREE.Line(geo, lineMat.clone());
            this.irRayLines.push(line);
            this.sensorRaysGroup.add(line);
        }
    }
    
    updateSensorRays(physics) {
        const chassisRadius = 0.070;
        const elevation = 0.0203; // Exact height of bumper sensor slots in authentic CAD
        
        for (let i = 0; i < physics.irAngles.length; i++) {
            const ang = physics.yaw + physics.irAngles[i];
            const dist = physics.irDistances[i];
            
            // Start at bumper
            const startX = physics.x + chassisRadius * Math.cos(ang);
            const startZ = -(physics.y + chassisRadius * Math.sin(ang));
            
            // End at measured distance
            const endX = physics.x + (chassisRadius + dist) * Math.cos(ang);
            const endZ = -(physics.y + (chassisRadius + dist) * Math.sin(ang));
            
            const line = this.irRayLines[i];
            const posAttr = line.geometry.attributes.position;
            posAttr.setXYZ(0, startX, elevation, startZ);
            posAttr.setXYZ(1, endX, elevation, endZ);
            posAttr.needsUpdate = true;
            
            // Color feedback: red when close, emerald when clear
            if (dist < 0.08) {
                line.material.color.setHex(0xef4444);
            } else if (dist < 0.16) {
                line.material.color.setHex(0xf59e0b);
            } else {
                line.material.color.setHex(0x10b981);
            }
        }
    }
    
    setupGroundRaycasting() {
        this.canvasMain.addEventListener('click', (event) => {
            const rect = this.canvasMain.getBoundingClientRect();
            this.mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
            this.mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
            
            this.raycaster.setFromCamera(this.mouse, this.camera);
            const intersects = this.raycaster.intersectObject(this.groundMesh);
            
            if (intersects.length > 0) {
                const pt = intersects[0].point;
                // Convert Three.js coordinates (X, 0, Z) to 2D robotics world coordinates (X, -Z)
                const targetX = pt.x;
                const targetY = -pt.z;
                
                if (this.onGroundClick) {
                    this.onGroundClick(targetX, targetY);
                }
            }
        });
    }
    
    setCameraMode(mode) {
        this.cameraMode = mode;
        if (mode === 'top') {
            this.camera.position.set(this.robotGroup.position.x, 3.2, this.robotGroup.position.z + 0.001);
            this.controls.target.set(this.robotGroup.position.x, 0, this.robotGroup.position.z);
            this.controls.update();
        }
    }
    
    resetCamera() {
        this.cameraMode = 'orbit';
        this.camera.position.set(1.4, 1.2, 1.6);
        this.controls.target.set(0, 0.05, 0);
        this.controls.update();
    }
    
    onWindowResize() {
        const width = this.canvasMain.clientWidth;
        const height = this.canvasMain.clientHeight;
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height, false);
        
        if (this.canvasPip) {
            this.pipRenderer.setSize(this.canvasPip.clientWidth, this.canvasPip.clientHeight, false);
        }
    }
    
    render(physics, nowMs) {
        // 1. Update Robot 3D Pose
        // In Three.js: X is right/forward, Y is up, Z is depth (-y in world)
        const simX = physics.x;
        const simZ = -physics.y;
        const simYaw = physics.yaw;
        
        this.robotGroup.position.set(simX, 0, simZ);
        this.robotGroup.rotation.y = simYaw;
        
        // 2. Animate Wheel Rotations (Spin around local Z axle)
        this.leftWheelGroup.rotation.z = -physics.thetaL;
        this.rightWheelGroup.rotation.z = -physics.thetaR;
        
        // 3. Update Trajectory Ribbon Trail
        this.addTrailPoint(simX, simZ);
        
        // 4. Update Proximity Sensor Beams
        this.updateSensorRays(physics);
        
        // 5. Pulse Beacon Light & Sphere
        if (this.beaconSphere) {
            const pulse = 1.0 + 0.25 * Math.sin(nowMs * 0.005);
            this.beaconSphere.scale.set(pulse, pulse, pulse);
        }
        
        // 6. Camera Follow Logic
        if (this.cameraMode === 'follow') {
            const chaseDist = 0.7;
            const chaseHeight = 0.45;
            const targetCamX = simX - chaseDist * Math.cos(simYaw);
            const targetCamZ = simZ + chaseDist * Math.sin(simYaw);
            
            this.camera.position.lerp(new THREE.Vector3(targetCamX, chaseHeight, targetCamZ), 0.08);
            this.controls.target.lerp(new THREE.Vector3(simX, 0.05, simZ), 0.08);
            this.controls.update();
        } else if (this.cameraMode === 'top') {
            this.camera.position.set(simX, 3.2, simZ + 0.001);
            this.controls.target.set(simX, 0, simZ);
            this.controls.update();
        } else {
            this.controls.update();
        }
        
        // 7. Render Main 3D Viewport
        this.renderer.render(this.scene, this.camera);
        
        // 8. Render Front WVGA Camera Picture-in-Picture (PiP)
        if (this.canvasPip && this.canvasPip.style.display !== 'none') {
            this.pipRenderer.render(this.scene, this.pipCamera);
        }
    }
}
