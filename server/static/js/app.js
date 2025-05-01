// Panel Management System
class PanelManager {
    static init() {
        this.panels = new Map();
        this.loadLayout();
        this.setupInteract();
        this.setupResize();
    }

    static setupInteract() {
        interact('.panel').draggable({
            inertia: true,
            modifiers: [
                interact.modifiers.restrictRect({
                    restriction: 'parent',
                    endOnly: true
                })
            ],
            autoScroll: true,
            listeners: {
                move: this.dragMoveListener,
                end: (e) => this.savePanelState(e.target)
            }
        });
    }

    static setupResize() {
        interact('.panel').resizable({
            edges: { right: true, bottom: true },
            listeners: {
                move: function (e) {
                    const target = e.target;
                    target.style.width = `${e.rect.width}px`;
                    target.style.height = `${e.rect.height}px`;
                    PanelManager.savePanelState(target);
                }
            }
        });
    }

    static dragMoveListener(e) {
        const target = e.target;
        const x = (parseFloat(target.getAttribute('data-x')) || 0) + e.dx;
        const y = (parseFloat(target.getAttribute('data-y')) || 0) + e.dy;

        target.style.transform = `translate(${x}px, ${y}px)`;
        target.setAttribute('data-x', x);
        target.setAttribute('data-y', y);
    }

    static savePanelState(panel) {
        const state = {
            x: panel.dataset.x || 0,
            y: panel.dataset.y || 0,
            width: panel.style.width,
            height: panel.style.height
        };
        localStorage.setItem(`panel-${panel.dataset.panelId}`, JSON.stringify(state));
    }

    static loadLayout() {
        document.querySelectorAll('.panel').forEach(panel => {
            const savedState = localStorage.getItem(`panel-${panel.dataset.panelId}`);
            if (savedState) {
                const { x, y, width, height } = JSON.parse(savedState);
                panel.style.transform = `translate(${x}px, ${y}px)`;
                panel.style.width = width;
                panel.style.height = height;
                panel.setAttribute('data-x', x);
                panel.setAttribute('data-y', y);
            }
        });
    }
}



// Example telemetry system
class TelemetrySystem {
    static init() {
        this.socket = io();
        this.socket.on('arduino_data_update', data => this.processData(data));
    }

    static processData(data) {
        // Update orientation visualization
        //TODO
        // RoverVisualizer.updateOrientation(
        //     data.orientation.pitch,
        //     data.orientation.roll
        // );

        // Update channel displays
        this.updateChannels(data.ch_raw);
        
        // Update status indicators
        this.updateStatusIndicators(data);
        
        // Update telemetry grid
        this.updateTelemetryDisplay(data);
    }

    static updateChannels(channels) {
        const container = document.getElementById('channels-display');
        container.innerHTML = channels.map((channel, index) => {
            const chNum = index + 1;
            const value = Object.values(channel)[0];
            const isAnalog = [1,2,3,4,9,10].includes(chNum);
            
            return `
                <div class="channel-item">
                    <div class="channel-label">CH${chNum}</div>
                    ${isAnalog ? `
                        <div class="channel-progress">
                            <div class="channel-fill" 
                                 style="width: ${this.normalizeAnalogValue(value)}%">
                            </div>
                        </div>
                        <div class="channel-value">${value}</div>
                    ` : `
                        <div class="digital-led ${value > 0 ? 'active' : ''}"></div>
                    `}
                </div>
            `;
        }).join('');
    }

    static normalizeAnalogValue(value) {
        // Convert 1000-2000μs to 0-100%
        return ((value - 1000) / 10).toFixed(1);
    }

    static updateStatusIndicators(data) {
        const updateIndicator = (id, state) => {
            const element = document.getElementById(id);
            element.classList.toggle('active', state);
        };

        updateIndicator('lights-status', data.lights);
        updateIndicator('estop-status', data.estop);
        updateIndicator('brakes-status', data.brakes);
    }

    static updateTelemetryDisplay(data) {
        const grid = document.getElementById('telemetry-display');
        const telemetryData = {
            'Battery': `${data.battery}%`,
            'Status': data.status,
            'Accel X': `${data.accel.x} m/s²`,
            'Accel Y': `${data.accel.y} m/s²`,
            'Motors': `M1: ${data.motors[0]}, M2: ${data.motors[1]}`,
            'Signal': data.signal
        };

        grid.innerHTML = Object.entries(telemetryData)
            .map(([key, value]) => `
                <div class="telemetry-item">
                    <div class="label">${key}</div>
                    <div class="value">${value}</div>
                </div>
            `).join('');
    }
}


/**
     * 3D Visualization System for Rover Orientation
     * 
     * Features:
     * - Real-time 3D model of rover
     * - Dynamic orientation updates (roll/pitch)
     * - Interactive camera controls
     * - Orientation readout display
     */
class RoverVisualizer {
    static init() {
        // Scene setup
        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
        this.renderer = new THREE.WebGLRenderer({ canvas: document.getElementById('rover-canvas'), antialias: true });

        // Lighting
        const ambientLight = new THREE.AmbientLight(0x404040);
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.6);
        directionalLight.position.set(5, 5, 5);
        this.scene.add(ambientLight, directionalLight);

        // Camera positioning
        this.camera.position.set(3, 3, 3);
        this.camera.lookAt(0, 0, 0);

        // Controls
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;

        // Create rover model
        this.createRoverModel();

        // Event listeners
        window.addEventListener('resize', this.onWindowResize.bind(this));
        this.animate();
    }

    /**
     * Creates 3D model of the rover
     */
    static createRoverModel() {
        // Main body
        const bodyGeometry = new THREE.BoxGeometry(2, 0.5, 1);
        const bodyMaterial = new THREE.MeshPhongMaterial({ color: 0x555555 });
        this.roverBody = new THREE.Mesh(bodyGeometry, bodyMaterial);

        // Wheels
        const wheelGeometry = new THREE.CylinderGeometry(0.3, 0.3, 0.2, 32);
        const wheelMaterial = new THREE.MeshPhongMaterial({ color: 0x333333 });

        // Add wheels
        this.wheels = [];
        const wheelPositions = [
            { x: 1, y: -0.35, z: 0.5 },
            { x: 1, y: -0.35, z: -0.5 },
            { x: -1, y: -0.35, z: 0.5 },
            { x: -1, y: -0.35, z: -0.5 }
        ];

        wheelPositions.forEach(pos => {
            const wheel = new THREE.Mesh(wheelGeometry, wheelMaterial);
            wheel.rotation.z = Math.PI / 2;
            wheel.position.set(pos.x, pos.y, pos.z);
            this.wheels.push(wheel);
            this.roverBody.add(wheel);
        });

        this.scene.add(this.roverBody);
    }

    /**
     * Updates rover orientation based on sensor data
     * @param {number} pitch - Pitch angle in degrees
     * @param {number} roll - Roll angle in degrees
     */
    static updateOrientation(pitch, roll) {
        // Convert degrees to radians
        const pitchRad = THREE.MathUtils.degToRad(pitch);
        const rollRad = THREE.MathUtils.degToRad(roll);

        // Apply rotations (order matters: ZXY)
        this.roverBody.rotation.set(rollRad, 0, -pitchRad);

        // Update UI display
        document.getElementById('pitch-value').textContent = `${pitch.toFixed(2)}°`;
        document.getElementById('roll-value').textContent = `${roll.toFixed(2)}°`;
    }

    /**
     * Handles window resize events
     */
    static onWindowResize() {
        this.camera.aspect = this.roverBody.parent.element.clientWidth / this.roverBody.parent.element.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(this.roverBody.parent.element.clientWidth, this.roverBody.parent.element.clientHeight);
    }

    /**
     * Animation loop
     */
    static animate() {
        requestAnimationFrame(() => this.animate());
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
    }
}


/**
 * Robotic Control System Web Interface
 * Version 2.0
 * Modules:
 * - WebSocketManager: Handles real-time communication
 * - MapManager: Manages map and GPS data
 * - ControlManager: Handles user input and commands
 * - VisionManager: Controls camera feeds
 * - UIManager: Handles UI interactions
 */

class WebSocketManager {
    constructor() {
        this.socket = io();
        this.initEventHandlers();
    }

    initEventHandlers() {
        this.socket.on('connect', () => UIManager.updateConnection(true));
        this.socket.on('disconnect', () => UIManager.updateConnection(false));
        this.socket.on('telemetry', data => this.handleTelemetry(data));
        this.socket.on('gps_update', data => MapManager.updatePosition(data));
        this.socket.on('kinect_status', data => VisionManager.updateStatus(data));
    }

    handleTelemetry(data) {
        console.log("handleTelemetry data " , data)
        UIManager.updateTelemetry(data);
        UIManager.updateChannels(data.channels);
        UIManager.updateOrientation(data.orientation);
    }

    sendCommand(command) {
        this.socket.emit('control_command', command);
    }
}

class MapManager {
    static init() {
        this.map = L.map('map-panel').setView([0, 0], 15);

        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(this.map);
        this.marker = L.marker([0, 0]).addTo(this.map);
        this.accuracyCircle = L.circle([0, 0], {radius: 0}).addTo(this.map);
        
        this.map.on('contextmenu', e => this.addWaypoint(e.latlng));
    }

    static updatePosition({lat, lon, accuracy}) {
        this.marker.setLatLng([lat, lon]);
        this.map.panTo([lat, lon]);
        this.accuracyCircle.setLatLng([lat, lon]).setRadius(accuracy);
    }

    static addWaypoint(latlng) {
        L.marker(latlng, {
            icon: L.divIcon({className: 'waypoint-marker', html: '📍'})
        }).addTo(this.map);
    }

    static clearWaypoints() {
        this.map.eachLayer(layer => {
            if (layer instanceof L.Marker && layer !== this.marker) {
                this.map.removeLayer(layer);
            }
        });
    }
}

class ControlManager {
    static setMode(mode) {
        document.querySelectorAll('.mode-btn').forEach(btn => 
            btn.classList.toggle('active', btn.dataset.mode === mode)
        );
        WebSocketManager.sendCommand(`mode:${mode}`);
    }

    static emergencyStop() {
        WebSocketManager.sendCommand('emergency_stop');
        UIManager.showAlert('Emergency Stop Activated', 'danger');
    }
}

class VisionManager {
    static toggleDepth() {
        document.querySelector('.depth-stream').classList.toggle('active');
        document.querySelector('.video-stream').classList.toggle('hidden');
    }

    static updateStatus({led, tilt}) {
        document.getElementById('kinect-led').value = led;
        document.getElementById('kinect-tilt').value = tilt;
        document.getElementById('tilt-value').textContent = `${tilt}°`;
    }
}

class UIManager {
    static init() {
        this.initPanelInteractions();
        this.initKinectControls();
        MapManager.init();
        new WebSocketManager();
    }

    static initPanelInteractions() {
        interact('.panel').draggable({
            inertia: true,
            modifiers: [interact.modifiers.restrictRect({
                restriction: 'body',
                endOnly: true
            })],
            autoScroll: true
        }).resizable({
            edges: { right: true, bottom: true },
            modifiers: [
                interact.modifiers.aspectRatio({ ratio: 'preserve' })
            ]
        });
    }

    static initKinectControls() {
        document.getElementById('kinect-led').addEventListener('change', e => {
            WebSocketManager.sendCommand(`kinect:led:${e.target.value}`);
        });

        document.getElementById('kinect-tilt').addEventListener('input', e => {
            WebSocketManager.sendCommand(`kinect:tilt:${e.target.value}`);
        });
    }

    static updateConnection(connected) {
        document.getElementById('connection-dot').classList.toggle('connected', connected);
        document.getElementById('connection-status').textContent = 
            connected ? 'Connected' : 'Disconnected';
    }

    static updateTelemetry(data) {
        document.getElementById('battery-level').textContent = `🔋 ${data.battery}%`;
        document.getElementById('signal-strength').textContent = `📶 ${data.signal}`;
    }

    static updateChannels(channels) {
        channels.forEach((value, index) => {
            const element = document.getElementById(`channel-${index+1}`);
            if (element) {
                const isAnalog = [1,2,3,4,9,10].includes(index+1);
                isAnalog ? 
                    element.style.height = `${Math.abs(value)}%` :
                    element.classList.toggle('active', value > 0);
            }
        });
    }
}


// Add to your app.js or script section
class VideoSystem {
    static init() {
        this.videoTabs = document.querySelectorAll('.video-tab');
        this.videoFeeds = document.querySelectorAll('.video-panel img');
        
        this.videoTabs.forEach(tab => {
            tab.addEventListener('click', () => this.switchFeed(tab));
        });
    }

    static switchFeed(clickedTab) {
        const feedType = clickedTab.dataset.feed;
        
        // Update tabs
        this.videoTabs.forEach(tab => tab.classList.remove('active'));
        clickedTab.classList.add('active');
        
        // Update video feeds
        this.videoFeeds.forEach(feed => {
            if (feed.src.includes(feedType)) {
                feed.classList.add('active-feed');
                feed.classList.remove('hidden-feed');
            } else {
                feed.classList.remove('active-feed');
                feed.classList.add('hidden-feed');
            }
        });
    }
}



// In RoverVisualizer class
function  onWindowResize() {
    const container = document.getElementById('rover-canvas').parentElement;
    const width = container.clientWidth;
    const height = container.clientHeight;
    
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
}

// Add resize observer for panels
const resizeObserver = new ResizeObserver(entries => {
    entries.forEach(entry => {
        const panel = entry.target;
        if(panel.dataset.panelId === 'rover3d') {
            RoverVisualizer.onWindowResize();
        }
    });
});

document.querySelectorAll('.panel').forEach(panel => {
    resizeObserver.observe(panel);
});




// Initialize systems when DOM is ready
document.addEventListener('DOMContentLoaded', () => {
    PanelManager.init();
    VideoSystem.init();
    MapManager.init();
    TelemetrySystem.init();
    // RoverVisualizer.init();
    UIManager.init()

    // Simulate sensor data (replace with real data)
    setInterval(() => {
        const simulatedPitch = Math.sin(Date.now() / 1000) * 15;
        const simulatedRoll = Math.cos(Date.now() / 800) * 20;
        // RoverVisualizer.updateOrientation(simulatedPitch, simulatedRoll);
    }, 50);
});
