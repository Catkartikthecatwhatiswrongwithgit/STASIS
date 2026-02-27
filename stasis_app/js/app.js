// Stasis Sentinel App - Main JavaScript Application

// ============================================
// CONFIGURATION
// ============================================
const CONFIG = {
    API_URL: 'http://localhost:5000/api',
    WS_URL: 'ws://localhost:5000/ws',
    MAP_CENTER: [11.0168, 76.9558],
    MAP_ZOOM: 15,
    DEFAULT_PATROL_RADIUS: 250,
    STREAM_URL: 'http://192.168.4.1:81/stream',
    RECONNECT_INTERVAL: 5000,
    TELEMETRY_TIMEOUT: 10000
};

// ============================================
// ROLES & PERMISSIONS
// ============================================
const ROLES = {
    ADMIN: { id: 4, name: 'Admin', color: '#ff4444', level: 4 },
    MODERATOR: { id: 3, name: 'Moderator', color: '#00d4ff', level: 3 },
    OFFICER: { id: 2, name: 'Officer', color: '#00ff88', level: 2 },
    SUPPORTER: { id: 1, name: 'Supporter', color: '#888888', level: 1 },
    GUEST: { id: 1, name: 'Guest', color: '#00d4ff', level: 1 }
};

const PERMISSIONS = {
    CONTROL_ROVER: ['Admin', 'Moderator', 'Officer', 'Supporter', 'Guest'],
    EDIT_FENCING: ['Admin', 'Moderator', 'Officer', 'Supporter', 'Guest'],
    VIEW_STREAM: ['Admin', 'Moderator', 'Officer', 'Supporter', 'Guest'],
    SEND_CHAT: ['Admin', 'Moderator', 'Officer', 'Supporter', 'Guest'],
    VIEW_TELEMETRY: ['Admin', 'Moderator', 'Officer', 'Supporter', 'Guest'],
    MANAGE_USERS: ['Admin'],
    EDIT_SETTINGS: ['Admin', 'Moderator', 'Officer', 'Supporter', 'Guest'],
    VIEW_ALERTS: ['Admin', 'Moderator', 'Officer', 'Supporter', 'Guest'],
    START_PATROL: ['Admin', 'Moderator', 'Officer', 'Supporter', 'Guest'],
    STOP_PATROL: ['Admin', 'Moderator', 'Officer', 'Supporter', 'Guest'],
    RETURN_HOME: ['Admin', 'Moderator', 'Officer', 'Supporter', 'Guest']
};

// ============================================
// TEAM MEMBERS WITH NICKNAMES
// ============================================
const TEAM_MEMBERS = {
    main: [
        { name: 'Kartik', nickname: 'Shadow', role: 'Admin', position: 'Leader and Initiative' },
        { name: 'Harish', nickname: 'Phoenix', role: 'Moderator', position: 'Team Member' },
        { name: 'Ryan Ahmed', nickname: 'Blaze', role: 'Officer', position: 'Team Member' },
        { name: 'Mohaideen Ijaz', nickname: 'Storm', role: 'Officer', position: 'Team Member' },
        { name: 'Swethen', nickname: 'Frost', role: 'Officer', position: 'Team Member' }
    ],
    supporters: [
        { name: 'Stephen', nickname: 'Echo', role: 'Supporter', position: 'Team Member' },
        { name: 'Krish', nickname: 'Nova', role: 'Supporter', position: 'Team Member' },
        { name: 'Inba', nickname: 'Spark', role: 'Supporter', position: 'Team Member' },
        { name: 'Baavasri', nickname: 'Viper', role: 'Supporter', position: 'Team Member' },
        { name: 'Janane', nickname: 'Raven', role: 'Supporter', position: 'Team Member' }
    ]
};

// ============================================
// APPLICATION STATE
// ============================================
let state = {
    currentUser: null,
    isGuest: false,
    connected: false,
    lastTelemetryTime: 0,
    rover: {
        connected: false,
        position: null,
        battery: 0,
        speed: 0,
        heading: 0,
        status: 'OFFLINE',
        temp: 0,
        distance: 999,
        hazard: false
    },
    map: null,
    miniMap: null,
    fencingMap: null,
    roverMarker: null,
    homeMarker: null,
    fencePolygon: null,
    fencePoints: [],
    fencingMode: false,
    ws: null,
    chat: {
        channels: [
            { id: 'general', name: 'General', icon: 'fa-hashtag' },
            { id: 'alerts', name: 'Alerts', icon: 'fa-exclamation-triangle' },
            { id: 'patrol', name: 'Patrol', icon: 'fa-route' }
        ],
        activeChannel: 'general',
        messages: {}
    },
    settings: {
        patrolRadius: CONFIG.DEFAULT_PATROL_RADIUS,
        streamEnabled: true,
        notificationsEnabled: true,
        autoReturnHome: true
    },
    alerts: [],
    packetsReceived: 0,
    simulationMode: false
};

// ============================================
// INITIALIZATION
// ============================================
document.addEventListener('DOMContentLoaded', function() {
    initApp();
});

function initApp() {
    // Clear any corrupted localStorage
    try {
        const savedUser = localStorage.getItem('stasis_user');
        if (savedUser) {
            const parsed = JSON.parse(savedUser);
            if (parsed && parsed.role && parsed.role.name) {
                state.currentUser = parsed;
                state.isGuest = parsed.role.name === 'Guest';
            } else {
                localStorage.removeItem('stasis_user');
            }
        }
    } catch (e) {
        localStorage.removeItem('stasis_user');
    }
    
    if (!state.currentUser) {
        showLoginModal();
    } else {
        showApp();
    }
    
    initEventListeners();
}

function initEventListeners() {
    // Navigation items
    document.querySelectorAll('.nav-item').forEach(function(item) {
        item.addEventListener('click', function(e) {
            e.preventDefault();
            const view = item.getAttribute('data-view');
            if (view) switchView(view);
        });
    });
    
    // Login form
    const loginForm = document.getElementById('loginForm');
    if (loginForm) {
        loginForm.addEventListener('submit', handleLogin);
    }
    
    // Chat input
    const chatInput = document.getElementById('chatInput');
    if (chatInput) {
        chatInput.addEventListener('keypress', function(e) {
            if (e.key === 'Enter') sendChatMessage();
        });
    }
    
    // Chat send button
    const chatSendBtn = document.querySelector('.chat-input-container button');
    if (chatSendBtn) {
        chatSendBtn.addEventListener('click', sendChatMessage);
    }
    
    // Window resize
    window.addEventListener('resize', function() {
        if (state.miniMap) state.miniMap.invalidateSize();
        if (state.map) state.map.invalidateSize();
        if (state.fencingMap) state.fencingMap.invalidateSize();
    });
}

// ============================================
// AUTHENTICATION
// ============================================
function showLoginModal() {
    const modal = document.getElementById('loginModal');
    if (modal) {
        modal.classList.remove('hidden');
        modal.style.display = 'flex';
        modal.style.zIndex = '99999';
    }
}

function hideLoginModal() {
    const modal = document.getElementById('loginModal');
    if (modal) {
        modal.classList.add('hidden');
        modal.style.display = 'none';
    }
}

function handleLogin(e) {
    e.preventDefault();
    e.stopPropagation();
    
    const emailInput = document.getElementById('loginEmail');
    const email = emailInput ? emailInput.value.trim() : '';
    
    if (!email) {
        continueAsGuest();
        return;
    }
    
    let user = null;
    let role = ROLES.SUPPORTER;
    
    for (let i = 0; i < TEAM_MEMBERS.main.length; i++) {
        const member = TEAM_MEMBERS.main[i];
        if (email.toLowerCase().includes(member.name.toLowerCase())) {
            user = member;
            role = ROLES[member.role.toUpperCase()] || ROLES.OFFICER;
            break;
        }
    }
    
    if (!user) {
        for (let i = 0; i < TEAM_MEMBERS.supporters.length; i++) {
            const member = TEAM_MEMBERS.supporters[i];
            if (email.toLowerCase().includes(member.name.toLowerCase())) {
                user = member;
                role = ROLES.SUPPORTER;
                break;
            }
        }
    }
    
    if (!user) {
        user = { name: email.split('@')[0], nickname: 'Visitor', role: 'Supporter', position: 'Visitor' };
        role = ROLES.SUPPORTER;
    }
    
    state.currentUser = { 
        name: user.name, 
        nickname: user.nickname, 
        email: email, 
        role: role, 
        isGuest: false 
    };
    state.isGuest = false;
    localStorage.setItem('stasis_user', JSON.stringify(state.currentUser));
    hideLoginModal();
    showApp();
    showNotification('Welcome', 'Logged in as ' + (state.currentUser.nickname || state.currentUser.name), 'success');
}

function continueAsGuest() {
    state.currentUser = {
        name: 'Guest',
        nickname: 'Guest',
        email: 'guest@stasis.local',
        role: ROLES.GUEST,
        position: 'Guest User',
        isGuest: true
    };
    state.isGuest = true;
    localStorage.setItem('stasis_user', JSON.stringify(state.currentUser));
    hideLoginModal();
    showApp();
}

function handleLogout() {
    state.currentUser = null;
    state.isGuest = false;
    localStorage.removeItem('stasis_user');
    disconnectWebSocket();
    showLoginModal();
}

function showApp() {
    updateUserPanel();
    initMaps();
    connectWebSocket();
    initChat();
    initSettings();
    initTeamView();
    updateUIForConnection(false);
}

// ============================================
// PERMISSIONS
// ============================================
function hasPermission(permission) {
    if (!state.currentUser) return false;
    const allowedRoles = PERMISSIONS[permission];
    if (!allowedRoles) return false;
    return allowedRoles.indexOf(state.currentUser.role.name) !== -1;
}

function checkPermissionAndAlert(permission, action) {
    return true;
}

// ============================================
// NAVIGATION
// ============================================
function switchView(viewName) {
    document.querySelectorAll('.nav-item').forEach(function(item) {
        item.classList.toggle('active', item.getAttribute('data-view') === viewName);
    });
    document.querySelectorAll('.view').forEach(function(view) {
        view.classList.toggle('active', view.id === viewName + 'View');
    });
    
    setTimeout(function() {
        if (viewName === 'map' && state.map) state.map.invalidateSize();
        if (viewName === 'fencing' && state.fencingMap) state.fencingMap.invalidateSize();
    }, 100);
}

// ============================================
// MAPS
// ============================================
function initMaps() {
    setTimeout(initMiniMap, 100);
    setTimeout(initMainMap, 200);
    setTimeout(initFencingMap, 300);
}

function initMiniMap() {
    const miniMapEl = document.getElementById('miniMap');
    if (!miniMapEl || state.miniMap) return;
    
    try {
        state.miniMap = L.map('miniMap', { zoomControl: false }).setView(CONFIG.MAP_CENTER, CONFIG.MAP_ZOOM);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: ''
        }).addTo(state.miniMap);
        
        state.homeMarker = L.marker(CONFIG.MAP_CENTER, { icon: createHomeIcon() }).addTo(state.miniMap);
        state.roverMarker = L.marker(CONFIG.MAP_CENTER, { icon: createRoverIcon() }).addTo(state.miniMap);
    } catch (e) {
        console.error('MiniMap error:', e);
    }
}

function initMainMap() {
    const mainMapEl = document.getElementById('mainMap');
    if (!mainMapEl || state.map) return;
    
    try {
        state.map = L.map('mainMap').setView(CONFIG.MAP_CENTER, CONFIG.MAP_ZOOM);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: ''
        }).addTo(state.map);
        
        state.homeMarker = L.marker(CONFIG.MAP_CENTER, { icon: createHomeIcon(), title: 'Base Station' }).addTo(state.map);
        state.roverMarker = L.marker(CONFIG.MAP_CENTER, { icon: createRoverIcon(), title: 'Rover' }).addTo(state.map);
    } catch (e) {
        console.error('MainMap error:', e);
    }
}

function initFencingMap() {
    const fencingMapEl = document.getElementById('fencingMap');
    if (!fencingMapEl || state.fencingMap) return;
    
    try {
        state.fencingMap = L.map('fencingMap').setView(CONFIG.MAP_CENTER, CONFIG.MAP_ZOOM);
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: ''
        }).addTo(state.fencingMap);
        
        state.fencingMap.on('click', handleFenceMapClick);
        L.marker(CONFIG.MAP_CENTER, { icon: createHomeIcon(), title: 'Base Station' }).addTo(state.fencingMap);
    } catch (e) {
        console.error('FencingMap error:', e);
    }
}

function createRoverIcon() {
    return L.divIcon({
        className: 'rover-marker',
        html: '<div style="background:#00d4ff;width:24px;height:24px;border-radius:50%;border:3px solid white;box-shadow:0 2px 8px rgba(0,0,0,0.5);display:flex;align-items:center;justify-content:center;"><i class="fas fa-robot" style="color:white;font-size:11px;"></i></div>',
        iconSize: [24, 24],
        iconAnchor: [12, 12]
    });
}

function createHomeIcon() {
    return L.divIcon({
        className: 'home-marker',
        html: '<div style="background:#00ff88;width:20px;height:20px;border-radius:4px;border:2px solid white;display:flex;align-items:center;justify-content:center;"><i class="fas fa-home" style="color:#1a1a2e;font-size:10px;"></i></div>',
        iconSize: [20, 20],
        iconAnchor: [10, 10]
    });
}

function createPointIcon() {
    return L.divIcon({
        className: 'fence-point',
        html: '<div style="background:#ff4444;width:14px;height:14px;border-radius:50%;border:2px solid white;"></div>',
        iconSize: [14, 14],
        iconAnchor: [7, 7]
    });
}

function updateRoverPosition(lat, lng) {
    state.rover.position = [lat, lng];
    if (state.roverMarker) {
        state.roverMarker.setLatLng([lat, lng]);
    }
    if (state.miniMap) {
        state.miniMap.panTo([lat, lng], { animate: true });
    }
    if (state.map) {
        state.map.panTo([lat, lng], { animate: true });
    }
    
    const latEl = document.getElementById('roverLat');
    const lngEl = document.getElementById('roverLng');
    if (latEl) latEl.textContent = lat.toFixed(6);
    if (lngEl) lngEl.textContent = lng.toFixed(6);
}

function centerOnRover() {
    if (state.rover.position && state.map) {
        state.map.setView(state.rover.position, state.map.getZoom());
    }
}

function centerOnBase() {
    if (state.map) {
        state.map.setView(CONFIG.MAP_CENTER, state.map.getZoom());
    }
}

// ============================================
// FENCING
// ============================================
function startFencingMode() {
    state.fencingMode = true;
    showNotification('Fencing Mode', 'Click on the map to add fence points', 'info');
    if (state.fencingMap) state.fencingMap.getContainer().style.cursor = 'crosshair';
}

function handleFenceMapClick(e) {
    if (!state.fencingMode) return;
    const point = [e.latlng.lat, e.latlng.lng];
    const marker = L.marker(point, { icon: createPointIcon() }).addTo(state.fencingMap);
    state.fencePoints.push(marker);
    updateFencePolygon();
}

function updateFencePolygon() {
    if (state.fencePolygon) state.fencingMap.removeLayer(state.fencePolygon);
    const points = state.fencePoints.map(function(m) { return m.getLatLng(); });
    if (points.length > 2) {
        state.fencePolygon = L.polygon(points, {
            color: '#00d4ff',
            fillColor: '#00d4ff',
            fillOpacity: 0.2,
            weight: 2
        }).addTo(state.fencingMap);
    }
    updateFencingInfo();
}

function updateFencingInfo() {
    const points = state.fencePoints.length;
    const area = state.fencePolygon ? calculatePolygonArea(state.fencePolygon) : 0;
    
    const fencePointsEl = document.getElementById('fencePoints');
    const fenceAreaEl = document.getElementById('fenceArea');
    
    if (fencePointsEl) fencePointsEl.textContent = points;
    if (fenceAreaEl) fenceAreaEl.textContent = area.toFixed(2) + ' m²';
}

function calculatePolygonArea(polygon) {
    const latlngs = polygon.getLatLngs()[0];
    let area = 0;
    for (let i = 0; i < latlngs.length; i++) {
        const j = (i + 1) % latlngs.length;
        area += latlngs[i].lat * latlngs[j].lng;
        area -= latlngs[j].lat * latlngs[i].lng;
    }
    return Math.abs(area / 2) * 111319.9 * 111319.9;
}

function clearFence() {
    state.fencePoints.forEach(function(m) { state.fencingMap.removeLayer(m); });
    state.fencePoints = [];
    if (state.fencePolygon) {
        state.fencingMap.removeLayer(state.fencePolygon);
        state.fencePolygon = null;
    }
    updateFencingInfo();
    showNotification('Fence Cleared', 'Patrol fence has been cleared', 'success');
}

function saveFence() {
    if (state.fencePoints.length < 3) {
        showNotification('Invalid Fence', 'At least 3 points are required', 'error');
        return;
    }
    const points = state.fencePoints.map(function(m) { return [m.getLatLng().lat, m.getLatLng().lng]; });
    sendCommand('set_fence', { points: points });
    showNotification('Fence Saved', 'Patrol fence has been saved', 'success');
}

function toggleFencingMode() {
    if (state.fencingMode) {
        state.fencingMode = false;
        if (state.fencingMap) state.fencingMap.getContainer().style.cursor = '';
    } else {
        startFencingMode();
    }
}

function updateRadiusDisplay() {
    const slider = document.getElementById('patrolRadius');
    const value = document.getElementById('radiusValue');
    if (slider && value) {
        value.textContent = slider.value + 'm';
    }
}

// ============================================
// WEBSOCKET CONNECTION
// ============================================
function connectWebSocket() {
    disconnectWebSocket();
    
    try {
        state.ws = new WebSocket(CONFIG.WS_URL);
        
        state.ws.onopen = function() {
            console.log('WebSocket connected');
            state.connected = true;
            state.simulationMode = false;
            updateUIForConnection(true);
            
            if (state.currentUser) {
                state.ws.send(JSON.stringify({ type: 'auth', user: state.currentUser }));
            }
        };
        
        state.ws.onmessage = function(event) {
            try {
                const data = JSON.parse(event.data);
                handleWebSocketMessage(data);
                state.lastTelemetryTime = Date.now();
            } catch (e) {
                console.error('WebSocket message error:', e);
            }
        };
        
        state.ws.onclose = function() {
            console.log('WebSocket disconnected');
            state.connected = false;
            state.rover.connected = false;
            updateUIForConnection(false);
            
            setTimeout(connectWebSocket, CONFIG.RECONNECT_INTERVAL);
        };
        
        state.ws.onerror = function(error) {
            console.error('WebSocket error:', error);
        };
        
    } catch (error) {
        console.error('Failed to connect WebSocket:', error);
        startSimulationMode();
    }
    
    setTimeout(function() {
        if (!state.connected && !state.simulationMode) {
            startSimulationMode();
        }
    }, 3000);
}

function disconnectWebSocket() {
    if (state.ws) {
        state.ws.close();
        state.ws = null;
    }
}

function handleWebSocketMessage(data) {
    switch (data.type) {
        case 'telemetry':
            handleTelemetry(data);
            break;
        case 'chat':
            handleChatMessage(data.payload || data);
            break;
        case 'alert':
            handleAlert(data.payload || data);
            break;
        case 'heartbeat':
            state.connected = true;
            state.rover.connected = true;
            updateUIForConnection(true);
            break;
    }
}

function sendCommand(command, params) {
    if (params === undefined) params = {};
    
    if (state.ws && state.ws.readyState === WebSocket.OPEN) {
        state.ws.send(JSON.stringify({ type: 'command', command: command, params: params, user: state.currentUser }));
    } else {
        showNotification('Offline', 'Not connected to base station', 'error');
    }
}

// ============================================
// TELEMETRY
// ============================================
function handleTelemetry(data) {
    state.connected = true;
    state.rover.connected = true;
    state.packetsReceived++;
    
    state.rover.battery = data.bat !== undefined ? data.bat : state.rover.battery;
    state.rover.speed = data.speed !== undefined ? data.speed : state.rover.speed;
    state.rover.heading = data.heading !== undefined ? data.heading : state.rover.heading;
    state.rover.status = data.status !== undefined ? data.status : state.rover.status;
    state.rover.temp = data.temp !== undefined ? data.temp : state.rover.temp;
    state.rover.distance = data.distance !== undefined ? data.distance : state.rover.distance;
    state.rover.hazard = data.hazard !== undefined ? data.hazard : false;
    
    if (data.lat && data.lng) {
        updateRoverPosition(data.lat, data.lng);
    }
    
    state.lastTelemetryTime = Date.now();
    updateUIForConnection(true);
    updateDashboard();
}

function updateDashboard() {
    const batteryEl = document.getElementById('roverBattery');
    if (batteryEl) {
        const bat = state.rover.battery || 0;
        batteryEl.textContent = bat.toFixed(1) + '%';
        batteryEl.className = 'value ' + (bat < 20 ? 'text-danger' : bat < 50 ? 'text-warning' : 'text-success');
    }
    
    const tempEl = document.getElementById('roverTemp');
    if (tempEl) {
        const temp = state.rover.temp || 0;
        tempEl.textContent = temp.toFixed(1) + '°C';
        tempEl.className = 'value ' + (temp > 50 ? 'text-danger' : temp > 35 ? 'text-warning' : '');
    }
    
    const distanceEl = document.getElementById('roverDistance');
    if (distanceEl) {
        distanceEl.textContent = (state.rover.distance || 999).toFixed(1) + ' cm';
    }
    
    const stateEl = document.getElementById('roverState');
    if (stateEl) {
        stateEl.textContent = (state.rover.status || 'OFFLINE').toUpperCase();
    }
    
    const headingEl = document.getElementById('roverHeading');
    if (headingEl) {
        headingEl.textContent = (state.rover.heading || 0).toFixed(0) + '°';
    }
    
    const speedEl = document.getElementById('roverSpeed');
    if (speedEl) {
        speedEl.textContent = ((state.rover.speed || 0) * 3.6).toFixed(1) + ' km/h';
    }
    
    const packetsEl = document.getElementById('packetsToday');
    if (packetsEl) {
        packetsEl.textContent = state.packetsReceived;
    }
}

function updateUIForConnection(connected) {
    const statusDot = document.querySelector('.status-dot');
    const statusText = document.querySelector('.connection-status span');
    
    if (statusDot) {
        statusDot.style.background = connected ? '#00ff88' : '#ff4444';
    }
    if (statusText) {
        statusText.textContent = connected ? 'Connected' : 'Disconnected';
    }
    
    const controlsCard = document.querySelector('.controls-card');
    if (controlsCard) {
        const note = controlsCard.querySelector('.control-note');
        if (connected) {
            controlsCard.style.opacity = '1';
            if (note) note.innerHTML = '<i class="fas fa-wifi"></i> Connected - Control enabled';
        } else {
            controlsCard.style.opacity = '0.6';
            if (note) note.innerHTML = '<i class="fas fa-exclamation-triangle"></i> Offline - Simulation mode';
        }
    }
}

// ============================================
// SIMULATION MODE (when offline)
// ============================================
let simulationInterval = null;

function startSimulationMode() {
    if (simulationInterval) return;
    
    state.simulationMode = true;
    state.connected = false;
    state.rover.connected = false;
    
    state.rover.position = [CONFIG.MAP_CENTER[0], CONFIG.MAP_CENTER[1]];
    state.rover.battery = 75;
    state.rover.speed = 0;
    state.rover.heading = 0;
    state.rover.status = 'IDLE';
    state.rover.temp = 25 + Math.random() * 5;
    state.rover.distance = 100;
    
    updateUIForConnection(false);
    updateDashboard();
    
    simulationInterval = setInterval(function() {
        state.rover.temp = 22 + Math.random() * 10;
        
        if (state.rover.status === 'PATROL') {
            const lat = state.rover.position[0] + (Math.random() - 0.5) * 0.0008;
            const lng = state.rover.position[1] + (Math.random() - 0.5) * 0.0008;
            updateRoverPosition(lat, lng);
            state.rover.speed = 0.3 + Math.random() * 0.4;
            state.rover.heading = (state.rover.heading + (Math.random() - 0.5) * 30 + 360) % 360;
        } else {
            state.rover.speed = 0;
        }
        
        if (state.rover.status !== 'DOCKED') {
            state.rover.battery = Math.max(0, state.rover.battery - 0.003);
        }
        
        state.rover.distance = 30 + Math.random() * 150;
        
        updateDashboard();
    }, 1500);
}

// ============================================
// CHAT
// ============================================
function initChat() {
    state.chat.channels.forEach(function(channel) {
        state.chat.messages[channel.id] = [];
    });
    
    addSystemMessage('Welcome to Stasis Sentinel!');
    renderChannels();
    renderMessages();
}

function addSystemMessage(content) {
    if (!state.chat.messages.general) state.chat.messages.general = [];
    state.chat.messages.general.push({
        id: Date.now(),
        user: 'System',
        role: 'Admin',
        content: content,
        timestamp: new Date()
    });
}

function renderChannels() {
    const channelList = document.querySelector('.channel-list');
    if (!channelList) return;
    
    var html = '';
    for (var i = 0; i < state.chat.channels.length; i++) {
        var channel = state.chat.channels[i];
        var active = channel.id === state.chat.activeChannel ? 'active' : '';
        html += '<div class="channel ' + active + '" data-channel="' + channel.id + '">';
        html += '<i class="fas ' + channel.icon + '"></i>';
        html += '<span>' + channel.name + '</span>';
        html += '</div>';
    }
    channelList.innerHTML = html;
    
    var channels = channelList.querySelectorAll('.channel');
    for (var i = 0; i < channels.length; i++) {
        channels[i].addEventListener('click', function() {
            state.chat.activeChannel = this.getAttribute('data-channel');
            renderChannels();
            renderMessages();
        });
    }
}

function renderMessages() {
    const messagesContainer = document.getElementById('messagesContainer');
    if (!messagesContainer) return;
    
    var messages = state.chat.messages[state.chat.activeChannel] || [];
    
    if (messages.length === 0) {
        messagesContainer.innerHTML = '<div class="no-messages">No messages yet</div>';
        return;
    }
    
    var html = '';
    for (var i = 0; i < messages.length; i++) {
        var msg = messages[i];
        html += '<div class="message">';
        html += '<div class="message-header">';
        html += '<span class="message-user" style="color:' + getRoleColor(msg.role) + '">' + escapeHtml(msg.user) + '</span>';
        html += '<span class="message-time">' + formatTime(msg.timestamp) + '</span>';
        html += '</div>';
        html += '<div class="message-content">' + escapeHtml(msg.content) + '</div>';
        html += '</div>';
    }
    messagesContainer.innerHTML = html;
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
}

function getRoleColor(role) {
    var r = ROLES[role ? role.toUpperCase() : 'GUEST'];
    return r ? r.color : '#888';
}

function sendChatMessage() {
    const input = document.getElementById('chatInput');
    var content = input ? input.value.trim() : '';
    if (!content || !state.currentUser) return;
    
    var message = {
        id: Date.now(),
        user: state.currentUser.nickname || state.currentUser.name,
        role: state.currentUser.role.name,
        content: content,
        timestamp: new Date()
    };
    
    if (!state.chat.messages[state.chat.activeChannel]) {
        state.chat.messages[state.chat.activeChannel] = [];
    }
    state.chat.messages[state.chat.activeChannel].push(message);
    
    if (state.ws && state.ws.readyState === WebSocket.OPEN) {
        state.ws.send(JSON.stringify({ type: 'chat', channel: state.chat.activeChannel, message: message }));
    }
    
    if (input) input.value = '';
    renderMessages();
}

function handleChatKeypress(e) {
    if (e.key === 'Enter') {
        sendChatMessage();
    }
}

function handleChatMessage(data) {
    if (!state.chat.messages[data.channel]) {
        state.chat.messages[data.channel] = [];
    }
    state.chat.messages[data.channel].push(data.message);
    
    if (data.channel === state.chat.activeChannel) {
        renderMessages();
    }
}

// ============================================
// ALERTS
// ============================================
function handleAlert(data) {
    var alert = {
        id: Date.now(),
        type: data.type || 'warning',
        title: data.title || 'Alert',
        message: data.message || '',
        timestamp: new Date(),
        status: data.status || 'UNKNOWN'
    };
    
    state.alerts.unshift(alert);
    if (state.alerts.length > 20) state.alerts.pop();
    
    showNotification(alert.title, alert.message, alert.type === 'hazard' ? 'error' : 'warning');
    updateAlertsList();
}

function updateAlertsList() {
    const alertsList = document.getElementById('alertsList');
    if (!alertsList) return;
    
    if (state.alerts.length === 0) {
        alertsList.innerHTML = '<div class="no-data">No alerts</div>';
        return;
    }
    
    var html = '';
    for (var i = 0; i < Math.min(state.alerts.length, 5); i++) {
        var alert = state.alerts[i];
        html += '<div class="alert-item ' + alert.type + '">';
        html += '<div class="alert-header">';
        html += '<span class="alert-title">' + escapeHtml(alert.title) + '</span>';
        html += '<span class="alert-time">' + formatTime(alert.timestamp) + '</span>';
        html += '</div>';
        html += '<div class="alert-message">' + escapeHtml(alert.message) + '</div>';
        html += '</div>';
    }
    alertsList.innerHTML = html;
    
    var hazardsEl = document.getElementById('hazardsDetected');
    if (hazardsEl) {
        hazardsEl.textContent = state.alerts.length;
    }
}

function showNotification(title, message, type) {
    type = type || 'info';
    var toast = document.createElement('div');
    toast.className = 'toast toast-' + type;
    toast.innerHTML = '<strong>' + escapeHtml(title) + '</strong><p>' + escapeHtml(message) + '</p>';
    toast.style.cssText = 'position:fixed;bottom:20px;right:20px;background:' + 
        (type === 'error' ? '#ff4444' : type === 'success' ? '#00ff88' : '#00d4ff') + 
        ';color:' + (type === 'success' || type === 'info' ? '#1a1a2e' : 'white') + 
        ';padding:15px 20px;border-radius:8px;z-index:10000;animation:slideIn 0.3s ease;max-width:300px;';
    document.body.appendChild(toast);
    setTimeout(function() {
        toast.style.animation = 'slideOut 0.3s ease';
        setTimeout(function() { toast.remove(); }, 300);
    }, 3000);
}

function toggleNotifications() {
    var panel = document.getElementById('notificationPanel');
    if (panel) panel.classList.toggle('show');
}

function clearNotifications() {
    state.alerts = [];
    updateAlertsList();
    
    var badge = document.getElementById('notifBadge');
    if (badge) badge.style.display = 'none';
}

// ============================================
// USER PANEL
// ============================================
function updateUserPanel() {
    var nameEl = document.querySelector('.user-name');
    var roleEl = document.querySelector('.user-role');
    var avatarEl = document.getElementById('userAvatar');
    
    if (nameEl && state.currentUser) {
        nameEl.textContent = state.currentUser.nickname || state.currentUser.name;
    }
    if (roleEl && state.currentUser) {
        roleEl.textContent = state.currentUser.role.name;
    }
    if (avatarEl && state.currentUser) {
        var color = state.currentUser.role.color || '#666';
        avatarEl.style.background = color;
    }
}

// ============================================
// TEAM VIEW
// ============================================
function initTeamView() {
    renderTeamMembers();
    renderPermissionsTable();
}

function renderTeamMembers() {
    var mainTeamGrid = document.getElementById('mainTeamGrid');
    var supportersGrid = document.getElementById('supportersGrid');
    
    if (mainTeamGrid) {
        var html = '';
        for (var i = 0; i < TEAM_MEMBERS.main.length; i++) {
            var member = TEAM_MEMBERS.main[i];
            var role = ROLES[member.role.toUpperCase()] || ROLES.SUPPORTER;
            html += '<div class="team-card">';
            html += '<div class="member-avatar"><i class="fas fa-user"></i></div>';
            html += '<div class="member-info">';
            html += '<span class="member-name">' + member.name + ' <span class="nickname">(' + member.nickname + ')</span></span>';
            html += '<span class="member-role">' + member.position + '</span>';
            html += '</div>';
            html += '<span class="rank-badge ' + member.role.toLowerCase() + '">' + role.name + '</span>';
            html += '</div>';
        }
        mainTeamGrid.innerHTML = html;
    }
    if (supportersGrid) {
        var html = '';
        for (var i = 0; i < TEAM_MEMBERS.supporters.length; i++) {
            var member = TEAM_MEMBERS.supporters[i];
            var role = ROLES[member.role.toUpperCase()] || ROLES.SUPPORTER;
            html += '<div class="team-card">';
            html += '<div class="member-avatar"><i class="fas fa-user"></i></div>';
            html += '<div class="member-info">';
            html += '<span class="member-name">' + member.name + ' <span class="nickname">(' + member.nickname + ')</span></span>';
            html += '<span class="member-role">' + member.position + '</span>';
            html += '</div>';
            html += '<span class="rank-badge ' + member.role.toLowerCase() + '">' + role.name + '</span>';
            html += '</div>';
        }
        supportersGrid.innerHTML = html;
    }
}

function renderPermissionsTable() {
    var permissionsTable = document.querySelector('.permissions-table table');
    if (!permissionsTable) return;
    
    var roles = ['Admin', 'Moderator', 'Officer', 'Supporter', 'Guest'];
    var permissions = Object.keys(PERMISSIONS);
    
    var tbody = permissionsTable.querySelector('tbody');
    if (tbody) {
        var html = '';
        for (var i = 0; i < permissions.length; i++) {
            var perm = permissions[i];
            html += '<tr><td>' + formatPermission(perm) + '</td>';
            for (var j = 0; j < roles.length; j++) {
                var role = roles[j];
                var hasPerm = PERMISSIONS[perm].indexOf(role) !== -1;
                html += '<td class="' + (hasPerm ? 'text-success' : 'text-danger') + '">';
                html += '<i class="fas ' + (hasPerm ? 'fa-check' : 'fa-times') + '"></i></td>';
            }
            html += '</tr>';
        }
        tbody.innerHTML = html;
    }
}

function formatPermission(perm) {
    return perm.replace(/_/g, ' ').replace(/\b\w/g, function(l) { return l.toUpperCase(); });
}

// ============================================
// SETTINGS
// ============================================
function initSettings() {
    var savedSettings = localStorage.getItem('stasis_settings');
    if (savedSettings) {
        try {
            state.settings = Object.assign({}, state.settings, JSON.parse(savedSettings));
        } catch (e) {}
    }
    
    var radiusSlider = document.getElementById('patrolRadius');
    if (radiusSlider) {
        radiusSlider.value = state.settings.patrolRadius;
        updateRadiusDisplay();
    }
}

function saveSettings() {
    var radiusSlider = document.getElementById('patrolRadius');
    if (radiusSlider) {
        state.settings.patrolRadius = parseInt(radiusSlider.value);
    }
    
    localStorage.setItem('stasis_settings', JSON.stringify(state.settings));
    showNotification('Settings Saved', 'Your settings have been saved', 'success');
}

function resetSettings() {
    state.settings = {
        patrolRadius: CONFIG.DEFAULT_PATROL_RADIUS,
        streamEnabled: true,
        notificationsEnabled: true,
        autoReturnHome: true
    };
    localStorage.removeItem('stasis_settings');
    initSettings();
    showNotification('Settings Reset', 'Settings have been reset to defaults', 'info');
}

// ============================================
// ROVER CONTROLS
// ============================================
function sendCommandToRover(cmd) {
    if (!state.connected && !state.simulationMode) {
        showNotification('Offline', 'Not connected to base station', 'error');
        return;
    }
    
    switch (cmd) {
        case 'STOP':
            state.rover.status = 'IDLE';
            sendCommand('STOP');
            showNotification('Stopped', 'Rover has been stopped', 'info');
            break;
        case 'START_PATROL':
            state.rover.status = 'PATROL';
            sendCommand('START_PATROL');
            showNotification('Patrol Started', 'Rover is now patrolling', 'success');
            break;
        case 'RETURN_BASE':
            state.rover.status = 'RETURNING';
            sendCommand('RETURN_BASE');
            showNotification('Returning', 'Rover is returning to base', 'info');
            break;
    }
    
    updateDashboard();
}

// ============================================
// UTILITY FUNCTIONS
// ============================================
function formatTime(date) {
    if (typeof date === 'string') date = new Date(date);
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
}

function escapeHtml(text) {
    var div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
}

function toggleSidebar() {
    var sidebar = document.querySelector('.sidebar');
    if (sidebar) {
        sidebar.classList.toggle('collapsed');
    }
}

// ============================================
// CSS ANIMATIONS
// ============================================
(function() {
    var style = document.createElement('style');
    style.textContent = 
        '@keyframes slideIn { from { transform: translateX(100%); opacity: 0; } to { transform: translateX(0); opacity: 1; } }' +
        '@keyframes slideOut { from { transform: translateX(0); opacity: 1; } to { transform: translateX(100%); opacity: 0; } }' +
        '.message { margin-bottom: 10px; padding: 8px; background: rgba(255,255,255,0.05); border-radius: 5px; }' +
        '.message-header { display: flex; justify-content: space-between; margin-bottom: 5px; }' +
        '.message-time { font-size: 11px; color: #888; }' +
        '.message-content { font-size: 14px; }' +
        '.toast p { margin: 5px 0 0 0; font-size: 13px; }' +
        '.guest-note { font-size: 12px; color: #888; margin-top: 5px; }' +
        '.login-info { background: rgba(0,212,255,0.1); padding: 10px; border-radius: 5px; margin-bottom: 15px; text-align: center; }' +
        '.login-info p { margin: 0; font-size: 13px; color: #00d4ff; }' +
        '.alert-item { padding: 10px; margin-bottom: 8px; border-radius: 5px; background: rgba(255,68,68,0.1); border-left: 3px solid #ff4444; }' +
        '.alert-item.warning { background: rgba(255,165,0,0.1); border-left-color: orange; }' +
        '.alert-header { display: flex; justify-content: space-between; margin-bottom: 5px; }' +
        '.alert-title { font-weight: bold; font-size: 12px; }' +
        '.alert-time { font-size: 10px; color: #888; }' +
        '.alert-message { font-size: 11px; color: #ccc; }' +
        '.no-messages { text-align: center; color: #666; padding: 20px; }' +
        '.text-danger { color: #ff4444 !important; }' +
        '.text-warning { color: #ffaa00 !important; }' +
        '.text-success { color: #00ff88 !important; }' +
        '.control-note { font-size: 12px; color: #888; margin-top: 10px; text-align: center; }';
    document.head.appendChild(style);
})();
