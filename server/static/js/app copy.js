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

// Initialize application when ready
document.addEventListener('DOMContentLoaded', () => UIManager.init());