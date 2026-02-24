// Stasis Sentinel App - Main JavaScript Application

// ============================================
// CONFIGURATION
// ============================================
const CONFIG = {
    API_URL: 'http://localhost:5000/api',
    WS_URL: 'ws://localhost:5000/ws',
    MAP_CENTER: [11.0168, 76.9558], // Default center (Coimbatore, India)
    MAP_ZOOM: 15,
    DEFAULT_PATROL_RADIUS: 250, // meters
    MAX_PATROL_RADIUS: 1000, // meters
    STREAM_URL: 'http://192.168.4.1:81/stream',
    RECONNECT_INTERVAL: 5000
};

// ============================================
// ROLES & PERMISSIONS
// ============================================
const ROLES = {
    ADMIN: { id: 4, name: 'Admin', color: '#ff4444', level: 4 },
    MODERATOR: { id: 3, name: 'Moderator', color: '#00d4ff', level: 3 },
    OFFICER: { id: 2, name: 'Officer', color: '#00ff88', level: 2 },
    SUPPORTER: { id: 1, name: 'Supporter', color: '#888888', level: 1 }
};

const PERMISSIONS = {
    CONTROL_ROVER: ['Admin', 'Moderator'],
    EDIT_FENCING: ['Admin', 'Moderator'],
    VIEW_STREAM: ['Admin', 'Moderator', 'Officer'],
    SEND_CHAT: ['Admin', 'Moderator', 'Officer', 'Supporter'],
    VIEW_TELEMETRY: ['Admin', 'Moderator', 'Officer'],
    MANAGE_USERS: ['Admin'],
    EDIT_SETTINGS: ['Admin', 'Moderator'],
    VIEW_ALERTS: ['Admin', 'Moderator', 'Officer'],
    START_PATROL: ['Admin', 'Moderator'],
    STOP_PATROL: ['Admin', 'Moderator'],
    RETURN_HOME: ['Admin', 'Moderator']
};

// ============================================
// TEAM MEMBERS
// ============================================
const TEAM_MEMBERS = {
    main: [
        { name: 'Kartik', role: 'Admin', position: 'Leader and Initiative' },
        { name: 'Harish', role: 'Moderator', position: 'Team Member' },
        { name: 'Ryan Ahmed', role: 'Officer', position: 'Team Member' },
        { name: 'Mohaideen Ijaz', role: 'Officer', position: 'Team Member' },
        { name: 'Swethen', role: 'Officer', position: 'Team Member' }
    ],
    supporters: [
        { name: 'Stephen', role: 'Supporter', position: 'Team Member' },
        { name: 'Krish', role: 'Supporter', position: 'Team Member' },
        { name: 'Inba', role: 'Supporter', position: 'Team Member' },
        { name: 'Baavasri', role: 'Supporter', position: 'Team Member' },
        { name: 'Janane', role: 'Supporter', position: 'Team Member' }
    ]
};

// ============================================
// APPLICATION STATE
// ============================================
let state = {
    currentUser: null,
    rover: {
        connected: false,
        position: null,
        battery: 0,
        speed: 0,
        heading: 0,
        status: 'idle',
        patrolMode: 'auto',
        fencePoints: [],
        homePosition: null
    },
    map: null,
    fencingMap: null,
    roverMarker: null,
    fencePolygon: null,
    fencePoints: [],
    fencingMode: false,
    ws: null,
    chat: {
        channels: [
            { id: 'general', name: 'General', icon: 'fa-hashtag' },
            { id: 'alerts', name: 'Alerts', icon: 'fa-exclamation-triangle' },
            { id: 'patrol', name: 'Patrol Updates', icon: 'fa-route' }
        ],
        activeChannel: 'general',
        messages: {}
    },
    settings: {
        patrolRadius: CONFIG.DEFAULT_PATROL_RADIUS,
        streamEnabled: true,
        notificationsEnabled: true,
        chatRestricted: false,
        autoReturnHome: true,
        reportEmails: []
    },
    notifications: []
};

// ============================================
// INITIALIZATION
// ============================================
document.addEventListener('DOMContentLoaded', () => {
    initApp();
});

function initApp() {
    // Check for saved session
    const savedUser = localStorage.getItem('stasis_user');
    if (savedUser) {
        state.currentUser = JSON.parse(savedUser);
        showApp();
    } else {
        showLoginModal();
    }
    
    // Initialize event listeners
    initEventListeners();
}

function initEventListeners() {
    // Navigation
    document.querySelectorAll('.nav-item').forEach(item => {
        item.addEventListener('click', (e) => {
            e.preventDefault();
            const view = item.dataset.view;
            if (view) switchView(view);
        });
    });
    
    // Login form
    document.getElementById('loginForm')?.addEventListener('submit', handleLogin);
    
    // Chat
    document.getElementById('chatInput')?.addEventListener('keypress', (e) => {
        if (e.key === 'Enter') sendChatMessage();
    });
    
    document.getElementById('sendChatBtn')?.addEventListener('click', sendChatMessage);
    
    // Settings
    document.getElementById('saveSettingsBtn')?.addEventListener('click', saveSettings);
    
    // Notifications
    document.getElementById('notifBtn')?.addEventListener('click', toggleNotifications);
    
    // Logout
    document.getElementById('logoutBtn')?.addEventListener('click', handleLogout);
    
    // Fencing controls
    document.getElementById('startFencingBtn')?.addEventListener('click', startFencingMode);
    document.getElementById('clearFenceBtn')?.addEventListener('click', clearFence);
    document.getElementById('saveFenceBtn')?.addEventListener('click', saveFence);
    
    // Patrol radius slider
    document.getElementById('patrolRadiusSlider')?.addEventListener('input', (e) => {
        document.getElementById('radiusValue').textContent = e.target.value + 'm';
        state.settings.patrolRadius = parseInt(e.target.value);
    });
}

// ============================================
// AUTHENTICATION
// ============================================
function showLoginModal() {
    document.getElementById('loginModal')?.classList.remove('hidden');
}

function hideLoginModal() {
    document.getElementById('loginModal')?.classList.add('hidden');
}

function handleLogin(e) {
    e.preventDefault();
    
    const email = document.getElementById('loginEmail').value;
    const password = document.getElementById('loginPassword').value;
    
    // Find user in team members
    let user = null;
    let role = ROLES.SUPPORTER;
    
    // Check main team
    for (const member of TEAM_MEMBERS.main) {
        if (email.toLowerCase().includes(member.name.toLowerCase())) {
            user = member;
            role = ROLES[member.role.toUpperCase()] || ROLES.OFFICER;
            break;
        }
    }
    
    // Check supporters if not found in main
    if (!user) {
        for (const member of TEAM_MEMBERS.supporters) {
            if (email.toLowerCase().includes(member.name.toLowerCase())) {
                user = member;
                role = ROLES.SUPPORTER;
                break;
            }
        }
    }
    
    // Demo mode - allow any email
    if (!user) {
        user = { name: email.split('@')[0], role: 'Supporter', position: 'Guest' };
        role = ROLES.SUPPORTER;
    }
    
    state.currentUser = {
        ...user,
        email: email,
        role: role
    };
    
    localStorage.setItem('stasis_user', JSON.stringify(state.currentUser));
    hideLoginModal();
    showApp();
}

function handleLogout() {
    state.currentUser = null;
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
    startTelemetrySimulation();
}

// ============================================
// PERMISSIONS
// ============================================
function hasPermission(permission) {
    if (!state.currentUser) return false;
    
    const allowedRoles = PERMISSIONS[permission];
    if (!allowedRoles) return false;
    
    return allowedRoles.includes(state.currentUser.role.name);
}

function checkPermissionAndAlert(permission, action) {
    if (!hasPermission(permission)) {
        showNotification('Permission Denied', `You don't have permission to ${action}`, 'error');
        return false;
    }
    return true;
}

// ============================================
// NAVIGATION
// ============================================
function switchView(viewName) {
    // Update nav items
    document.querySelectorAll('.nav-item').forEach(item => {
        item.classList.toggle('active', item.dataset.view === viewName);
    });
    
    // Update views
    document.querySelectorAll('.view').forEach(view => {
        view.classList.toggle('active', view.id === `${viewName}View`);
    });
    
    // Initialize map if switching to map or fencing view
    if (viewName === 'map' && !state.map) {
        setTimeout(() => initMainMap(), 100);
    }
    if (viewName === 'fencing' && !state.fencingMap) {
        setTimeout(() => initFencingMap(), 100);
    }
}

// ============================================
// MAPS
// ============================================
function initMaps() {
    // Initialize mini map on dashboard
    setTimeout(() => initMiniMap(), 100);
}

function initMiniMap() {
    const miniMapEl = document.getElementById('miniMap');
    if (!miniMapEl || state.miniMap) return;
    
    state.miniMap = L.map('miniMap').setView(CONFIG.MAP_CENTER, CONFIG.MAP_ZOOM);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '© OpenStreetMap'
    }).addTo(state.miniMap);
    
    // Add rover marker
    state.roverMarker = L.marker(CONFIG.MAP_CENTER, {
        icon: createRoverIcon()
    }).addTo(state.miniMap);
}

function initMainMap() {
    const mainMapEl = document.getElementById('mainMap');
    if (!mainMapEl || state.map) return;
    
    state.map = L.map('mainMap').setView(CONFIG.MAP_CENTER, CONFIG.MAP_ZOOM);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '© OpenStreetMap'
    }).addTo(state.map);
    
    // Add rover marker
    if (!state.roverMarker) {
        state.roverMarker = L.marker(state.rover.position || CONFIG.MAP_CENTER, {
            icon: createRoverIcon()
        }).addTo(state.map);
    } else {
        state.roverMarker.addTo(state.map);
    }
    
    // Add home marker
    if (state.rover.homePosition) {
        L.marker(state.rover.homePosition, {
            icon: createHomeIcon()
        }).addTo(state.map);
    }
    
    // Add fence polygon if exists
    if (state.rover.fencePoints.length > 2) {
        state.fencePolygon = L.polygon(state.rover.fencePoints, {
            color: '#00d4ff',
            fillColor: '#00d4ff',
            fillOpacity: 0.2
        }).addTo(state.map);
    }
}

function initFencingMap() {
    const fencingMapEl = document.getElementById('fencingMap');
    if (!fencingMapEl || state.fencingMap) return;
    
    state.fencingMap = L.map('fencingMap').setView(CONFIG.MAP_CENTER, CONFIG.MAP_ZOOM);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '© OpenStreetMap'
    }).addTo(state.fencingMap);
    
    // Add click handler for fencing
    state.fencingMap.on('click', handleFenceMapClick);
    
    // Add existing fence points
    if (state.rover.fencePoints.length > 0) {
        state.fencePolygon = L.polygon(state.rover.fencePoints, {
            color: '#00d4ff',
            fillColor: '#00d4ff',
            fillOpacity: 0.2
        }).addTo(state.fencingMap);
        
        state.fencePoints = state.rover.fencePoints.map(p => 
            L.marker(p, { icon: createPointIcon() }).addTo(state.fencingMap)
        );
    }
}

function createRoverIcon() {
    return L.divIcon({
        className: 'rover-marker',
        html: '<div style="background:#00d4ff;width:20px;height:20px;border-radius:50%;border:3px solid white;box-shadow:0 2px 5px rgba(0,0,0,0.5);"></div>',
        iconSize: [20, 20],
        iconAnchor: [10, 10]
    });
}

function createHomeIcon() {
    return L.divIcon({
        className: 'home-marker',
        html: '<div style="background:#00ff88;width:16px;height:16px;border-radius:50%;border:2px solid white;"></div>',
        iconSize: [16, 16],
        iconAnchor: [8, 8]
    });
}

function createPointIcon() {
    return L.divIcon({
        className: 'fence-point',
        html: '<div style="background:#ff4444;width:12px;height:12px;border-radius:50%;border:2px solid white;"></div>',
        iconSize: [12, 12],
        iconAnchor: [6, 6]
    });
}

function updateRoverPosition(lat, lng) {
    state.rover.position = [lat, lng];
    
    if (state.roverMarker) {
        state.roverMarker.setLatLng([lat, lng]);
    }
    
    // Update mini map
    if (state.miniMap) {
        state.miniMap.panTo([lat, lng]);
    }
    
    // Update main map
    if (state.map) {
        state.map.panTo([lat, lng]);
    }
}

// ============================================
// FENCING
// ============================================
function startFencingMode() {
    if (!checkPermissionAndAlert('EDIT_FENCING', 'edit patrol fencing')) return;
    
    state.fencingMode = true;
    showNotification('Fencing Mode', 'Click on the map to add fence points', 'info');
    
    if (state.fencingMap) {
        state.fencingMap.getContainer().style.cursor = 'crosshair';
    }
}

function handleFenceMapClick(e) {
    if (!state.fencingMode) return;
    
    const point = [e.latlng.lat, e.latlng.lng];
    
    // Add marker
    const marker = L.marker(point, { icon: createPointIcon() }).addTo(state.fencingMap);
    state.fencePoints.push(marker);
    
    // Update polygon
    updateFencePolygon();
}

function updateFencePolygon() {
    // Remove existing polygon
    if (state.fencePolygon) {
        state.fencingMap.removeLayer(state.fencePolygon);
    }
    
    // Get points
    const points = state.fencePoints.map(m => m.getLatLng());
    
    if (points.length > 2) {
        state.fencePolygon = L.polygon(points, {
            color: '#00d4ff',
            fillColor: '#00d4ff',
            fillOpacity: 0.2
        }).addTo(state.fencingMap);
    }
    
    // Update info
    updateFencingInfo();
}

function updateFencingInfo() {
    const points = state.fencePoints.length;
    const area = state.fencePolygon ? calculatePolygonArea(state.fencePolygon) : 0;
    
    document.getElementById('fencePointCount').textContent = points;
    document.getElementById('fenceArea').textContent = area.toFixed(2) + ' m²';
}

function calculatePolygonArea(polygon) {
    const latlngs = polygon.getLatLngs()[0];
    let area = 0;
    
    for (let i = 0; i < latlngs.length; i++) {
        const j = (i + 1) % latlngs.length;
        area += latlngs[i].lat * latlngs[j].lng;
        area -= latlngs[j].lat * latlngs[i].lng;
    }
    
    return Math.abs(area / 2) * 111319.9 * 111319.9; // Convert to m²
}

function clearFence() {
    if (!checkPermissionAndAlert('EDIT_FENCING', 'clear patrol fencing')) return;
    
    // Remove markers
    state.fencePoints.forEach(m => state.fencingMap.removeLayer(m));
    state.fencePoints = [];
    
    // Remove polygon
    if (state.fencePolygon) {
        state.fencingMap.removeLayer(state.fencePolygon);
        state.fencePolygon = null;
    }
    
    updateFencingInfo();
    showNotification('Fence Cleared', 'Patrol fence has been cleared', 'success');
}

function saveFence() {
    if (!checkPermissionAndAlert('EDIT_FENCING', 'save patrol fencing')) return;
    
    if (state.fencePoints.length < 3) {
        showNotification('Invalid Fence', 'At least 3 points are required', 'error');
        return;
    }
    
    state.rover.fencePoints = state.fencePoints.map(m => [m.getLatLng().lat, m.getLatLng().lng]);
    
    // Send to server
    sendCommand('set_fence', { points: state.rover.fencePoints });
    
    showNotification('Fence Saved', 'Patrol fence has been saved and sent to rover', 'success');
}

// ============================================
// WEBSOCKET CONNECTION
// ============================================
function connectWebSocket() {
    if (state.ws) {
        state.ws.close();
    }
    
    try {
        state.ws = new WebSocket(CONFIG.WS_URL);
        
        state.ws.onopen = () => {
            console.log('WebSocket connected');
            updateConnectionStatus(true);
            
            // Authenticate
            state.ws.send(JSON.stringify({
                type: 'auth',
                user: state.currentUser
            }));
        };
        
        state.ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            handleWebSocketMessage(data);
        };
        
        state.ws.onclose = () => {
            console.log('WebSocket disconnected');
            updateConnectionStatus(false);
            
            // Attempt reconnect
            setTimeout(connectWebSocket, CONFIG.RECONNECT_INTERVAL);
        };
        
        state.ws.onerror = (error) => {
            console.error('WebSocket error:', error);
        };
    } catch (error) {
        console.error('Failed to connect WebSocket:', error);
        // Use simulation mode
        startTelemetrySimulation();
    }
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
            handleTelemetry(data.payload);
            break;
        case 'chat':
            handleChatMessage(data.payload);
            break;
        case 'alert':
            handleAlert(data.payload);
            break;
        case 'user_joined':
            handleUserJoined(data.payload);
            break;
        case 'user_left':
            handleUserLeft(data.payload);
            break;
    }
}

function sendCommand(command, params = {}) {
    if (state.ws && state.ws.readyState === WebSocket.OPEN) {
        state.ws.send(JSON.stringify({
            type: 'command',
            command: command,
            params: params,
            user: state.currentUser
        }));
    } else {
        // Simulation mode
        console.log('Command (simulated):', command, params);
        simulateCommand(command, params);
    }
}

// ============================================
// TELEMETRY
// ============================================
function handleTelemetry(data) {
    // Update rover state
    state.rover.battery = data.battery || state.rover.battery;
    state.rover.speed = data.speed || state.rover.speed;
    state.rover.heading = data.heading || state.rover.heading;
    state.rover.status = data.status || state.rover.status;
    
    if (data.gps) {
        updateRoverPosition(data.gps.lat, data.gps.lng);
    }
    
    // Update UI
    updateDashboard();
}

function updateDashboard() {
    // Battery
    const batteryEl = document.getElementById('batteryLevel');
    if (batteryEl) {
        batteryEl.textContent = state.rover.battery + '%';
        batteryEl.className = state.rover.battery < 20 ? 'text-danger' : 
                              state.rover.battery < 50 ? 'text-warning' : '';
    }
    
    // Speed
    const speedEl = document.getElementById('speedValue');
    if (speedEl) speedEl.textContent = state.rover.speed.toFixed(1) + ' m/s';
    
    // Status
    const statusEl = document.getElementById('roverStatus');
    if (statusEl) {
        statusEl.textContent = state.rover.status.toUpperCase();
        statusEl.className = state.rover.status === 'patrolling' ? 'text-success' :
                            state.rover.status === 'returning' ? 'text-warning' : '';
    }
    
    // Heading
    const headingEl = document.getElementById('headingValue');
    if (headingEl) headingEl.textContent = state.rover.heading + '°';
}

function updateConnectionStatus(connected) {
    state.rover.connected = connected;
    
    const statusDot = document.querySelector('.status-dot');
    const statusText = document.querySelector('.connection-status span');
    
    if (statusDot) {
        statusDot.style.background = connected ? '#00ff88' : '#ff4444';
    }
    
    if (statusText) {
        statusText.textContent = connected ? 'Connected' : 'Disconnected';
    }
}

// ============================================
// SIMULATION MODE
// ============================================
let simulationInterval = null;

function startTelemetrySimulation() {
    if (simulationInterval) return;
    
    // Set initial position
    state.rover.position = CONFIG.MAP_CENTER;
    state.rover.homePosition = CONFIG.MAP_CENTER;
    state.rover.battery = 85;
    state.rover.speed = 0;
    state.rover.heading = 0;
    state.rover.status = 'idle';
    state.rover.connected = true;
    
    updateConnectionStatus(true);
    updateDashboard();
    
    // Simulate telemetry updates
    simulationInterval = setInterval(() => {
        if (state.rover.status === 'patrolling') {
            // Simulate movement
            const lat = state.rover.position[0] + (Math.random() - 0.5) * 0.0005;
            const lng = state.rover.position[1] + (Math.random() - 0.5) * 0.0005;
            updateRoverPosition(lat, lng);
            
            state.rover.speed = 0.5 + Math.random() * 1.5;
            state.rover.heading = Math.floor(Math.random() * 360);
        }
        
        // Simulate battery drain
        if (state.rover.status !== 'idle' && state.rover.status !== 'charging') {
            state.rover.battery = Math.max(0, state.rover.battery - 0.01);
        }
        
        updateDashboard();
    }, 1000);
}

function simulateCommand(command, params) {
    switch (command) {
        case 'start_patrol':
            state.rover.status = 'patrolling';
            showNotification('Patrol Started', 'Rover is now patrolling', 'success');
            break;
        case 'stop_patrol':
            state.rover.status = 'idle';
            state.rover.speed = 0;
            showNotification('Patrol Stopped', 'Rover has stopped', 'info');
            break;
        case 'return_home':
            state.rover.status = 'returning';
            showNotification('Returning Home', 'Rover is returning to base', 'info');
            setTimeout(() => {
                state.rover.status = 'idle';
                state.rover.position = state.rover.homePosition;
                updateRoverPosition(state.rover.homePosition[0], state.rover.homePosition[1]);
                showNotification('Home Reached', 'Rover has returned to base', 'success');
            }, 5000);
            break;
        case 'set_fence':
            state.rover.fencePoints = params.points;
            showNotification('Fence Updated', 'Patrol fence has been updated', 'success');
            break;
    }
    
    updateDashboard();
}

// ============================================
// CHAT
// ============================================
function initChat() {
    // Initialize messages for each channel
    state.chat.channels.forEach(channel => {
        state.chat.messages[channel.id] = [];
    });
    
    // Add welcome message
    state.chat.messages.general.push({
        id: Date.now(),
        user: 'System',
        role: 'Admin',
        content: 'Welcome to Stasis Sentinel Chat!',
        timestamp: new Date()
    });
    
    renderChannels();
    renderMessages();
}

function renderChannels() {
    const channelsEl = document.getElementById('chatChannels');
    if (!channelsEl) return;
    
    channelsEl.innerHTML = state.chat.channels.map(channel => `
        <div class="channel ${channel.id === state.chat.activeChannel ? 'active' : ''}" 
             data-channel="${channel.id}">
            <i class="fas ${channel.icon}"></i>
            <span>${channel.name}</span>
        </div>
    `).join('');
    
    // Add click handlers
    channelsEl.querySelectorAll('.channel').forEach(el => {
        el.addEventListener('click', () => {
            state.chat.activeChannel = el.dataset.channel;
            renderChannels();
            renderMessages();
        });
    });
}

function renderMessages() {
    const messagesEl = document.getElementById('chatMessages');
    if (!messagesEl) return;
    
    const messages = state.chat.messages[state.chat.activeChannel] || [];
    
    if (messages.length === 0) {
        messagesEl.innerHTML = '<div class="no-data">No messages yet</div>';
        return;
    }
    
    messagesEl.innerHTML = messages.map(msg => `
        <div class="message">
            <div class="message-header">
                <span class="message-user" style="color: ${ROLES[msg.role]?.color || '#888'}">${msg.user}</span>
                <span class="message-time">${formatTime(msg.timestamp)}</span>
            </div>
            <div class="message-content">${msg.content}</div>
        </div>
    `).join('');
    
    // Scroll to bottom
    messagesEl.scrollTop = messagesEl.scrollHeight;
}

function sendChatMessage() {
    if (!checkPermissionAndAlert('SEND_CHAT', 'send chat messages')) return;
    if (state.settings.chatRestricted && state.currentUser.role.level < 3) {
        showNotification('Chat Restricted', 'Chat is currently restricted to moderators and above', 'error');
        return;
    }
    
    const input = document.getElementById('chatInput');
    const content = input.value.trim();
    
    if (!content) return;
    
    const message = {
        id: Date.now(),
        user: state.currentUser.name,
        role: state.currentUser.role.name,
        content: content,
        timestamp: new Date()
    };
    
    // Add to local messages
    state.chat.messages[state.chat.activeChannel].push(message);
    
    // Send via WebSocket
    if (state.ws && state.ws.readyState === WebSocket.OPEN) {
        state.ws.send(JSON.stringify({
            type: 'chat',
            channel: state.chat.activeChannel,
            message: message
        }));
    }
    
    input.value = '';
    renderMessages();
}

function handleChatMessage(data) {
    if (!state.chat.messages[data.channel]) {
        state.chat.messages[data.channel] = [];
    }
    
    state.chat.messages[data.channel].push(data.message);
    
    if (data.channel === state.chat.activeChannel) {
        renderMessages();
    }
    
    // Show notification if not in chat view
    showNotification(`New message in ${data.channel}`, `${data.message.user}: ${data.message.content}`, 'info');
}

// ============================================
// ALERTS
// ============================================
function handleAlert(data) {
    state.notifications.unshift({
        id: Date.now(),
        type: data.type || 'info',
        title: data.title,
        message: data.message,
        timestamp: new Date()
    });
    
    showNotification(data.title, data.message, data.type);
    updateNotificationBadge();
}

function showNotification(title, message, type = 'info') {
    // Create toast notification
    const toast = document.createElement('div');
    toast.className = `toast toast-${type}`;
    toast.innerHTML = `
        <strong>${title}</strong>
        <p>${message}</p>
    `;
    
    toast.style.cssText = `
        position: fixed;
        bottom: 20px;
        right: 20px;
        background: ${type === 'error' ? '#ff4444' : type === 'success' ? '#00ff88' : '#00d4ff'};
        color: ${type === 'success' || type === 'info' ? '#1a1a2e' : 'white'};
        padding: 15px 20px;
        border-radius: 8px;
        z-index: 10000;
        animation: slideIn 0.3s ease;
    `;
    
    document.body.appendChild(toast);
    
    setTimeout(() => {
        toast.style.animation = 'slideOut 0.3s ease';
        setTimeout(() => toast.remove(), 300);
    }, 3000);
}

function updateNotificationBadge() {
    const badge = document.querySelector('.notif-badge');
    if (badge) {
        badge.textContent = state.notifications.length;
        badge.style.display = state.notifications.length > 0 ? 'block' : 'none';
    }
}

function toggleNotifications() {
    const panel = document.getElementById('notificationPanel');
    if (panel) {
        panel.classList.toggle('show');
    }
}

// ============================================
// USER MANAGEMENT
// ============================================
function updateUserPanel() {
    const nameEl = document.querySelector('.user-name');
    const roleEl = document.querySelector('.user-role');
    
    if (nameEl) nameEl.textContent = state.currentUser.name;
    if (roleEl) roleEl.textContent = state.currentUser.role.name;
}

function handleUserJoined(data) {
    showNotification('User Joined', `${data.user.name} has joined`, 'info');
}

function handleUserLeft(data) {
    showNotification('User Left', `${data.user.name} has left`, 'info');
}

// ============================================
// TEAM VIEW
// ============================================
function initTeamView() {
    renderTeamMembers();
    renderPermissionsTable();
}

function renderTeamMembers() {
    const mainTeamEl = document.getElementById('mainTeamMembers');
    const supportersEl = document.getElementById('supporterMembers');
    
    if (mainTeamEl) {
        mainTeamEl.innerHTML = TEAM_MEMBERS.main.map(member => createMemberCard(member)).join('');
    }
    
    if (supportersEl) {
        supportersEl.innerHTML = TEAM_MEMBERS.supporters.map(member => createMemberCard(member)).join('');
    }
}

function createMemberCard(member) {
    const role = ROLES[member.role.toUpperCase()] || ROLES.SUPPORTER;
    return `
        <div class="team-card">
            <div class="member-avatar">
                <i class="fas fa-user"></i>
            </div>
            <div class="member-info">
                <div class="member-name">${member.name}</div>
                <div class="member-role">${member.position}</div>
            </div>
            <span class="rank-badge ${member.role.toLowerCase()}">${role.name}</span>
        </div>
    `;
}

function renderPermissionsTable() {
    const tableEl = document.getElementById('permissionsTable');
    if (!tableEl) return;
    
    const permissions = Object.keys(PERMISSIONS);
    const roles = ['Admin', 'Moderator', 'Officer', 'Supporter'];
    
    tableEl.innerHTML = `
        <table>
            <thead>
                <tr>
                    <th>Permission</th>
                    ${roles.map(r => `<th>${r}</th>`).join('')}
                </tr>
            </thead>
            <tbody>
                ${permissions.map(perm => `
                    <tr>
                        <td>${formatPermission(perm)}</td>
                        ${roles.map(role => `
                            <td class="${PERMISSIONS[perm].includes(role) ? 'text-success' : 'text-danger'}">
                                <i class="fas ${PERMISSIONS[perm].includes(role) ? 'fa-check' : 'fa-times'}"></i>
                            </td>
                        `).join('')}
                    </tr>
                `).join('')}
            </tbody>
        </table>
    `;
}

function formatPermission(perm) {
    return perm.replace(/_/g, ' ').replace(/\b\w/g, l => l.toUpperCase());
}

// ============================================
// SETTINGS
// ============================================
function initSettings() {
    // Load saved settings
    const savedSettings = localStorage.getItem('stasis_settings');
    if (savedSettings) {
        state.settings = { ...state.settings, ...JSON.parse(savedSettings) };
    }
    
    // Update UI
    const radiusSlider = document.getElementById('patrolRadiusSlider');
    if (radiusSlider) {
        radiusSlider.value = state.settings.patrolRadius;
        document.getElementById('radiusValue').textContent = state.settings.patrolRadius + 'm';
    }
    
    // Checkboxes
    document.getElementById('streamEnabled').checked = state.settings.streamEnabled;
    document.getElementById('notificationsEnabled').checked = state.settings.notificationsEnabled;
    document.getElementById('chatRestricted').checked = state.settings.chatRestricted;
    document.getElementById('autoReturnHome').checked = state.settings.autoReturnHome;
}

function saveSettings() {
    if (!checkPermissionAndAlert('EDIT_SETTINGS', 'edit settings')) return;
    
    // Get values from form
    state.settings.patrolRadius = parseInt(document.getElementById('patrolRadiusSlider').value);
    state.settings.streamEnabled = document.getElementById('streamEnabled').checked;
    state.settings.notificationsEnabled = document.getElementById('notificationsEnabled').checked;
    state.settings.chatRestricted = document.getElementById('chatRestricted').checked;
    state.settings.autoReturnHome = document.getElementById('autoReturnHome').checked;
    
    // Save to localStorage
    localStorage.setItem('stasis_settings', JSON.stringify(state.settings));
    
    showNotification('Settings Saved', 'Your settings have been saved', 'success');
}

// ============================================
// ROVER CONTROLS
// ============================================
function startPatrol() {
    if (!checkPermissionAndAlert('START_PATROL', 'start patrol')) return;
    sendCommand('start_patrol');
}

function stopPatrol() {
    if (!checkPermissionAndAlert('STOP_PATROL', 'stop patrol')) return;
    sendCommand('stop_patrol');
}

function returnHome() {
    if (!checkPermissionAndAlert('RETURN_HOME', 'return rover home')) return;
    sendCommand('return_home');
}

function emergencyStop() {
    sendCommand('emergency_stop');
    showNotification('Emergency Stop', 'Rover has been stopped!', 'error');
}

// ============================================
// UTILITY FUNCTIONS
// ============================================
function formatTime(date) {
    if (typeof date === 'string') date = new Date(date);
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
}

// ============================================
// CSS ANIMATIONS (injected)
// ============================================
const style = document.createElement('style');
style.textContent = `
    @keyframes slideIn {
        from { transform: translateX(100%); opacity: 0; }
        to { transform: translateX(0); opacity: 1; }
    }
    @keyframes slideOut {
        from { transform: translateX(0); opacity: 1; }
        to { transform: translateX(100%); opacity: 0; }
    }
    .message { margin-bottom: 10px; padding: 8px; background: rgba(255,255,255,0.05); border-radius: 5px; }
    .message-header { display: flex; justify-content: space-between; margin-bottom: 5px; }
    .message-time { font-size: 11px; color: #888; }
    .message-content { font-size: 14px; }
`;
document.head.appendChild(style);

// ============================================
// CONFIGURATION
// ============================================
const CONFIG = {
    API_URL: 'http://localhost:5000/api',
    WS_URL: 'ws://localhost:5000/ws',
    MAP_CENTER: [11.0168, 76.9558], // Default center (Coimbatore, India)
    MAP_ZOOM: 15,
    DEFAULT_PATROL_RADIUS: 250, // meters
    MAX_PATROL_RADIUS: 1000, // meters
    STREAM_URL: 'http://192.168.4.1:81/stream',
    RECONNECT_INTERVAL: 5000
};

// ============================================
// ROLES & PERMISSIONS
// ============================================
const ROLES = {
    ADMIN: { id: 4, name: 'Admin', color: '#ff4444', level: 4 },
    MODERATOR: { id: 3, name: 'Moderator', color: '#00d4ff', level: 3 },
    OFFICER: { id: 2, name: 'Officer', color: '#00ff88', level: 2 },
    SUPPORTER: { id: 1, name: 'Supporter', color: '#888888', level: 1 }
};

const PERMISSIONS = {
    CONTROL_ROVER: ['Admin', 'Moderator'],
    EDIT_FENCING: ['Admin', 'Moderator'],
    VIEW_STREAM: ['Admin', 'Moderator', 'Officer'],
    SEND_CHAT: ['Admin', 'Moderator', 'Officer', 'Supporter'],
    VIEW_TELEMETRY: ['Admin', 'Moderator', 'Officer'],
    MANAGE_USERS: ['Admin'],
    EDIT_SETTINGS: ['Admin', 'Moderator'],
    VIEW_ALERTS: ['Admin', 'Moderator', 'Officer'],
    START_PATROL: ['Admin', 'Moderator'],
    STOP_PATROL: ['Admin', 'Moderator'],
    RETURN_HOME: ['Admin', 'Moderator']
};

// ============================================
// TEAM MEMBERS
// ============================================
const TEAM_MEMBERS = {
    main: [
        { name: 'Kartik', role: 'Admin', position: 'Leader and Initiative' },
        { name: 'Harish', role: 'Moderator', position: 'Team Member' },
        { name: 'Ryan Ahmed', role: 'Officer', position: 'Team Member' },
        { name: 'Mohaideen Ijaz', role: 'Officer', position: 'Team Member' },
        { name: 'Swethen', role: 'Officer', position: 'Team Member' }
    ],
    supporters: [
        { name: 'Stephen', role: 'Supporter', position: 'Team Member' },
        { name: 'Krish', role: 'Supporter', position: 'Team Member' },
        { name: 'Inba', role: 'Supporter', position: 'Team Member' },
        { name: 'Baavasri', role: 'Supporter', position: 'Team Member' },
        { name: 'Janane', role: 'Supporter', position: 'Team Member' }
    ]
};

// ============================================
// APPLICATION STATE
// ============================================
let state = {
    currentUser: null,
    rover: {
        connected: false,
        position: null,
        battery: 0,
        speed: 0,
        heading: 0,
        status: 'idle',
        patrolMode: 'auto',
        fencePoints: [],
        homePosition: null
    },
    map: null,
    fencingMap: null,
    roverMarker: null,
    fencePolygon: null,
    fencePoints: [],
    fencingMode: false,
    ws: null,
    chat: {
        channels: [
            { id: 'general', name: 'General', icon: 'fa-hashtag' },
            { id: 'alerts', name: 'Alerts', icon: 'fa-exclamation-triangle' },
            { id: 'patrol', name: 'Patrol Updates', icon: 'fa-route' }
        ],
        activeChannel: 'general',
        messages: {}
    },
    settings: {
        patrolRadius: CONFIG.DEFAULT_PATROL_RADIUS,
        streamEnabled: true,
        notificationsEnabled: true,
        chatRestricted: false,
        autoReturnHome: true,
        reportEmails: []
    },
    notifications: []
};

// ============================================
// INITIALIZATION
// ============================================
document.addEventListener('DOMContentLoaded', () => {
    initApp();
});

function initApp() {
    // Check for saved session
    const savedUser = localStorage.getItem('stasis_user');
    if (savedUser) {
        state.currentUser = JSON.parse(savedUser);
        showApp();
    } else {
        showLoginModal();
    }
    
    // Initialize event listeners
    initEventListeners();
}

function initEventListeners() {
    // Navigation
    document.querySelectorAll('.nav-item').forEach(item => {
        item.addEventListener('click', (e) => {
            e.preventDefault();
            const view = item.dataset.view;
            if (view) switchView(view);
        });
    });
    
    // Login form
    document.getElementById('loginForm')?.addEventListener('submit', handleLogin);
    
    // Chat
    document.getElementById('chatInput')?.addEventListener('keypress', (e) => {
        if (e.key === 'Enter') sendChatMessage();
    });
    
    document.getElementById('sendChatBtn')?.addEventListener('click', sendChatMessage);
    
    // Settings
    document.getElementById('saveSettingsBtn')?.addEventListener('click', saveSettings);
    
    // Notifications
    document.getElementById('notifBtn')?.addEventListener('click', toggleNotifications);
    
    // Logout
    document.getElementById('logoutBtn')?.addEventListener('click', handleLogout);
    
    // Fencing controls
    document.getElementById('startFencingBtn')?.addEventListener('click', startFencingMode);
    document.getElementById('clearFenceBtn')?.addEventListener('click', clearFence);
    document.getElementById('saveFenceBtn')?.addEventListener('click', saveFence);
    
    // Patrol radius slider
    document.getElementById('patrolRadiusSlider')?.addEventListener('input', (e) => {
        document.getElementById('radiusValue').textContent = e.target.value + 'm';
        state.settings.patrolRadius = parseInt(e.target.value);
    });
}

// ============================================
// AUTHENTICATION
// ============================================
function showLoginModal() {
    document.getElementById('loginModal')?.classList.remove('hidden');
}

function hideLoginModal() {
    document.getElementById('loginModal')?.classList.add('hidden');
}

function handleLogin(e) {
    e.preventDefault();
    
    const email = document.getElementById('loginEmail').value;
    const password = document.getElementById('loginPassword').value;
    
    // Find user in team members
    let user = null;
    let role = ROLES.SUPPORTER;
    
    // Check main team
    for (const member of TEAM_MEMBERS.main) {
        if (email.toLowerCase().includes(member.name.toLowerCase())) {
            user = member;
            role = ROLES[member.role.toUpperCase()] || ROLES.OFFICER;
            break;
        }
    }
    
    // Check supporters if not found in main
    if (!user) {
        for (const member of TEAM_MEMBERS.supporters) {
            if (email.toLowerCase().includes(member.name.toLowerCase())) {
                user = member;
                role = ROLES.SUPPORTER;
                break;
            }
        }
    }
    
    // Demo mode - allow any email
    if (!user) {
        user = { name: email.split('@')[0], role: 'Supporter', position: 'Guest' };
        role = ROLES.SUPPORTER;
    }
    
    state.currentUser = {
        ...user,
        email: email,
        role: role
    };
    
    localStorage.setItem('stasis_user', JSON.stringify(state.currentUser));
    hideLoginModal();
    showApp();
}

function handleLogout() {
    state.currentUser = null;
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
    startTelemetrySimulation();
}

// ============================================
// PERMISSIONS
// ============================================
function hasPermission(permission) {
    if (!state.currentUser) return false;
    
    const allowedRoles = PERMISSIONS[permission];
    if (!allowedRoles) return false;
    
    return allowedRoles.includes(state.currentUser.role.name);
}

function checkPermissionAndAlert(permission, action) {
    if (!hasPermission(permission)) {
        showNotification('Permission Denied', `You don't have permission to ${action}`, 'error');
        return false;
    }
    return true;
}

// ============================================
// NAVIGATION
// ============================================
function switchView(viewName) {
    // Update nav items
    document.querySelectorAll('.nav-item').forEach(item => {
        item.classList.toggle('active', item.dataset.view === viewName);
    });
    
    // Update views
    document.querySelectorAll('.view').forEach(view => {
        view.classList.toggle('active', view.id === `${viewName}View`);
    });
    
    // Initialize map if switching to map or fencing view
    if (viewName === 'map' && !state.map) {
        setTimeout(() => initMainMap(), 100);
    }
    if (viewName === 'fencing' && !state.fencingMap) {
        setTimeout(() => initFencingMap(), 100);
    }
}

// ============================================
// MAPS
// ============================================
function initMaps() {
    // Initialize mini map on dashboard
    setTimeout(() => initMiniMap(), 100);
}

function initMiniMap() {
    const miniMapEl = document.getElementById('miniMap');
    if (!miniMapEl || state.miniMap) return;
    
    state.miniMap = L.map('miniMap').setView(CONFIG.MAP_CENTER, CONFIG.MAP_ZOOM);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '© OpenStreetMap'
    }).addTo(state.miniMap);
    
    // Add rover marker
    state.roverMarker = L.marker(CONFIG.MAP_CENTER, {
        icon: createRoverIcon()
    }).addTo(state.miniMap);
}

function initMainMap() {
    const mainMapEl = document.getElementById('mainMap');
    if (!mainMapEl || state.map) return;
    
    state.map = L.map('mainMap').setView(CONFIG.MAP_CENTER, CONFIG.MAP_ZOOM);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '© OpenStreetMap'
    }).addTo(state.map);
    
    // Add rover marker
    if (!state.roverMarker) {
        state.roverMarker = L.marker(state.rover.position || CONFIG.MAP_CENTER, {
            icon: createRoverIcon()
        }).addTo(state.map);
    } else {
        state.roverMarker.addTo(state.map);
    }
    
    // Add home marker
    if (state.rover.homePosition) {
        L.marker(state.rover.homePosition, {
            icon: createHomeIcon()
        }).addTo(state.map);
    }
    
    // Add fence polygon if exists
    if (state.rover.fencePoints.length > 2) {
        state.fencePolygon = L.polygon(state.rover.fencePoints, {
            color: '#00d4ff',
            fillColor: '#00d4ff',
            fillOpacity: 0.2
        }).addTo(state.map);
    }
}

function initFencingMap() {
    const fencingMapEl = document.getElementById('fencingMap');
    if (!fencingMapEl || state.fencingMap) return;
    
    state.fencingMap = L.map('fencingMap').setView(CONFIG.MAP_CENTER, CONFIG.MAP_ZOOM);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '© OpenStreetMap'
    }).addTo(state.fencingMap);
    
    // Add click handler for fencing
    state.fencingMap.on('click', handleFenceMapClick);
    
    // Add existing fence points
    if (state.rover.fencePoints.length > 0) {
        state.fencePolygon = L.polygon(state.rover.fencePoints, {
            color: '#00d4ff',
            fillColor: '#00d4ff',
            fillOpacity: 0.2
        }).addTo(state.fencingMap);
        
        state.fencePoints = state.rover.fencePoints.map(p => 
            L.marker(p, { icon: createPointIcon() }).addTo(state.fencingMap)
        );
    }
}

function createRoverIcon() {
    return L.divIcon({
        className: 'rover-marker',
        html: '<div style="background:#00d4ff;width:20px;height:20px;border-radius:50%;border:3px solid white;box-shadow:0 2px 5px rgba(0,0,0,0.5);"></div>',
        iconSize: [20, 20],
        iconAnchor: [10, 10]
    });
}

function createHomeIcon() {
    return L.divIcon({
        className: 'home-marker',
        html: '<div style="background:#00ff88;width:16px;height:16px;border-radius:50%;border:2px solid white;"></div>',
        iconSize: [16, 16],
        iconAnchor: [8, 8]
    });
}

function createPointIcon() {
    return L.divIcon({
        className: 'fence-point',
        html: '<div style="background:#ff4444;width:12px;height:12px;border-radius:50%;border:2px solid white;"></div>',
        iconSize: [12, 12],
        iconAnchor: [6, 6]
    });
}

function updateRoverPosition(lat, lng) {
    state.rover.position = [lat, lng];
    
    if (state.roverMarker) {
        state.roverMarker.setLatLng([lat, lng]);
    }
    
    // Update mini map
    if (state.miniMap) {
        state.miniMap.panTo([lat, lng]);
    }
    
    // Update main map
    if (state.map) {
        state.map.panTo([lat, lng]);
    }
}

// ============================================
// FENCING
// ============================================
function startFencingMode() {
    if (!checkPermissionAndAlert('EDIT_FENCING', 'edit patrol fencing')) return;
    
    state.fencingMode = true;
    showNotification('Fencing Mode', 'Click on the map to add fence points', 'info');
    
    if (state.fencingMap) {
        state.fencingMap.getContainer().style.cursor = 'crosshair';
    }
}

function handleFenceMapClick(e) {
    if (!state.fencingMode) return;
    
    const point = [e.latlng.lat, e.latlng.lng];
    
    // Add marker
    const marker = L.marker(point, { icon: createPointIcon() }).addTo(state.fencingMap);
    state.fencePoints.push(marker);
    
    // Update polygon
    updateFencePolygon();
}

function updateFencePolygon() {
    // Remove existing polygon
    if (state.fencePolygon) {
        state.fencingMap.removeLayer(state.fencePolygon);
    }
    
    // Get points
    const points = state.fencePoints.map(m => m.getLatLng());
    
    if (points.length > 2) {
        state.fencePolygon = L.polygon(points, {
            color: '#00d4ff',
            fillColor: '#00d4ff',
            fillOpacity: 0.2
        }).addTo(state.fencingMap);
    }
    
    // Update info
    updateFencingInfo();
}

function updateFencingInfo() {
    const points = state.fencePoints.length;
    const area = state.fencePolygon ? calculatePolygonArea(state.fencePolygon) : 0;
    
    document.getElementById('fencePointCount').textContent = points;
    document.getElementById('fenceArea').textContent = area.toFixed(2) + ' m²';
}

function calculatePolygonArea(polygon) {
    const latlngs = polygon.getLatLngs()[0];
    let area = 0;
    
    for (let i = 0; i < latlngs.length; i++) {
        const j = (i + 1) % latlngs.length;
        area += latlngs[i].lat * latlngs[j].lng;
        area -= latlngs[j].lat * latlngs[i].lng;
    }
    
    return Math.abs(area / 2) * 111319.9 * 111319.9; // Convert to m²
}

function clearFence() {
    if (!checkPermissionAndAlert('EDIT_FENCING', 'clear patrol fencing')) return;
    
    // Remove markers
    state.fencePoints.forEach(m => state.fencingMap.removeLayer(m));
    state.fencePoints = [];
    
    // Remove polygon
    if (state.fencePolygon) {
        state.fencingMap.removeLayer(state.fencePolygon);
        state.fencePolygon = null;
    }
    
    updateFencingInfo();
    showNotification('Fence Cleared', 'Patrol fence has been cleared', 'success');
}

function saveFence() {
    if (!checkPermissionAndAlert('EDIT_FENCING', 'save patrol fencing')) return;
    
    if (state.fencePoints.length < 3) {
        showNotification('Invalid Fence', 'At least 3 points are required', 'error');
        return;
    }
    
    state.rover.fencePoints = state.fencePoints.map(m => [m.getLatLng().lat, m.getLatLng().lng]);
    
    // Send to server
    sendCommand('set_fence', { points: state.rover.fencePoints });
    
    showNotification('Fence Saved', 'Patrol fence has been saved and sent to rover', 'success');
}

// ============================================
// WEBSOCKET CONNECTION
// ============================================
function connectWebSocket() {
    if (state.ws) {
        state.ws.close();
    }
    
    try {
        state.ws = new WebSocket(CONFIG.WS_URL);
        
        state.ws.onopen = () => {
            console.log('WebSocket connected');
            updateConnectionStatus(true);
            
            // Authenticate
            state.ws.send(JSON.stringify({
                type: 'auth',
                user: state.currentUser
            }));
        };
        
        state.ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            handleWebSocketMessage(data);
        };
        
        state.ws.onclose = () => {
            console.log('WebSocket disconnected');
            updateConnectionStatus(false);
            
            // Attempt reconnect
            setTimeout(connectWebSocket, CONFIG.RECONNECT_INTERVAL);
        };
        
        state.ws.onerror = (error) => {
            console.error('WebSocket error:', error);
        };
    } catch (error) {
        console.error('Failed to connect WebSocket:', error);
        // Use simulation mode
        startTelemetrySimulation();
    }
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
            handleTelemetry(data.payload);
            break;
        case 'chat':
            handleChatMessage(data.payload);
            break;
        case 'alert':
            handleAlert(data.payload);
            break;
        case 'user_joined':
            handleUserJoined(data.payload);
            break;
        case 'user_left':
            handleUserLeft(data.payload);
            break;
    }
}

function sendCommand(command, params = {}) {
    if (state.ws && state.ws.readyState === WebSocket.OPEN) {
        state.ws.send(JSON.stringify({
            type: 'command',
            command: command,
            params: params,
            user: state.currentUser
        }));
    } else {
        // Simulation mode
        console.log('Command (simulated):', command, params);
        simulateCommand(command, params);
    }
}

// ============================================
// TELEMETRY
// ============================================
function handleTelemetry(data) {
    // Update rover state
    state.rover.battery = data.battery || state.rover.battery;
    state.rover.speed = data.speed || state.rover.speed;
    state.rover.heading = data.heading || state.rover.heading;
    state.rover.status = data.status || state.rover.status;
    
    if (data.gps) {
        updateRoverPosition(data.gps.lat, data.gps.lng);
    }
    
    // Update UI
    updateDashboard();
}

function updateDashboard() {
    // Battery
    const batteryEl = document.getElementById('batteryLevel');
    if (batteryEl) {
        batteryEl.textContent = state.rover.battery + '%';
        batteryEl.className = state.rover.battery < 20 ? 'text-danger' : 
                              state.rover.battery < 50 ? 'text-warning' : '';
    }
    
    // Speed
    const speedEl = document.getElementById('speedValue');
    if (speedEl) speedEl.textContent = state.rover.speed.toFixed(1) + ' m/s';
    
    // Status
    const statusEl = document.getElementById('roverStatus');
    if (statusEl) {
        statusEl.textContent = state.rover.status.toUpperCase();
        statusEl.className = state.rover.status === 'patrolling' ? 'text-success' :
                            state.rover.status === 'returning' ? 'text-warning' : '';
    }
    
    // Heading
    const headingEl = document.getElementById('headingValue');
    if (headingEl) headingEl.textContent = state.rover.heading + '°';
}

function updateConnectionStatus(connected) {
    state.rover.connected = connected;
    
    const statusDot = document.querySelector('.status-dot');
    const statusText = document.querySelector('.connection-status span');
    
    if (statusDot) {
        statusDot.style.background = connected ? '#00ff88' : '#ff4444';
    }
    
    if (statusText) {
        statusText.textContent = connected ? 'Connected' : 'Disconnected';
    }
}

// ============================================
// SIMULATION MODE
// ============================================
let simulationInterval = null;

function startTelemetrySimulation() {
    if (simulationInterval) return;
    
    // Set initial position
    state.rover.position = CONFIG.MAP_CENTER;
    state.rover.homePosition = CONFIG.MAP_CENTER;
    state.rover.battery = 85;
    state.rover.speed = 0;
    state.rover.heading = 0;
    state.rover.status = 'idle';
    state.rover.connected = true;
    
    updateConnectionStatus(true);
    updateDashboard();
    
    // Simulate telemetry updates
    simulationInterval = setInterval(() => {
        if (state.rover.status === 'patrolling') {
            // Simulate movement
            const lat = state.rover.position[0] + (Math.random() - 0.5) * 0.0005;
            const lng = state.rover.position[1] + (Math.random() - 0.5) * 0.0005;
            updateRoverPosition(lat, lng);
            
            state.rover.speed = 0.5 + Math.random() * 1.5;
            state.rover.heading = Math.floor(Math.random() * 360);
        }
        
        // Simulate battery drain
        if (state.rover.status !== 'idle' && state.rover.status !== 'charging') {
            state.rover.battery = Math.max(0, state.rover.battery - 0.01);
        }
        
        updateDashboard();
    }, 1000);
}

function simulateCommand(command, params) {
    switch (command) {
        case 'start_patrol':
            state.rover.status = 'patrolling';
            showNotification('Patrol Started', 'Rover is now patrolling', 'success');
            break;
        case 'stop_patrol':
            state.rover.status = 'idle';
            state.rover.speed = 0;
            showNotification('Patrol Stopped', 'Rover has stopped', 'info');
            break;
        case 'return_home':
            state.rover.status = 'returning';
            showNotification('Returning Home', 'Rover is returning to base', 'info');
            setTimeout(() => {
                state.rover.status = 'idle';
                state.rover.position = state.rover.homePosition;
                updateRoverPosition(state.rover.homePosition[0], state.rover.homePosition[1]);
                showNotification('Home Reached', 'Rover has returned to base', 'success');
            }, 5000);
            break;
        case 'set_fence':
            state.rover.fencePoints = params.points;
            showNotification('Fence Updated', 'Patrol fence has been updated', 'success');
            break;
    }
    
    updateDashboard();
}

// ============================================
// CHAT
// ============================================
function initChat() {
    // Initialize messages for each channel
    state.chat.channels.forEach(channel => {
        state.chat.messages[channel.id] = [];
    });
    
    // Add welcome message
    state.chat.messages.general.push({
        id: Date.now(),
        user: 'System',
        role: 'Admin',
        content: 'Welcome to Stasis Sentinel Chat!',
        timestamp: new Date()
    });
    
    renderChannels();
    renderMessages();
}

function renderChannels() {
    const channelsEl = document.getElementById('chatChannels');
    if (!channelsEl) return;
    
    channelsEl.innerHTML = state.chat.channels.map(channel => `
        <div class="channel ${channel.id === state.chat.activeChannel ? 'active' : ''}" 
             data-channel="${channel.id}">
            <i class="fas ${channel.icon}"></i>
            <span>${channel.name}</span>
        </div>
    `).join('');
    
    // Add click handlers
    channelsEl.querySelectorAll('.channel').forEach(el => {
        el.addEventListener('click', () => {
            state.chat.activeChannel = el.dataset.channel;
            renderChannels();
            renderMessages();
        });
    });
}

function renderMessages() {
    const messagesEl = document.getElementById('chatMessages');
    if (!messagesEl) return;
    
    const messages = state.chat.messages[state.chat.activeChannel] || [];
    
    if (messages.length === 0) {
        messagesEl.innerHTML = '<div class="no-data">No messages yet</div>';
        return;
    }
    
    messagesEl.innerHTML = messages.map(msg => `
        <div class="message">
            <div class="message-header">
                <span class="message-user" style="color: ${ROLES[msg.role]?.color || '#888'}">${msg.user}</span>
                <span class="message-time">${formatTime(msg.timestamp)}</span>
            </div>
            <div class="message-content">${msg.content}</div>
        </div>
    `).join('');
    
    // Scroll to bottom
    messagesEl.scrollTop = messagesEl.scrollHeight;
}

function sendChatMessage() {
    if (!checkPermissionAndAlert('SEND_CHAT', 'send chat messages')) return;
    if (state.settings.chatRestricted && state.currentUser.role.level < 3) {
        showNotification('Chat Restricted', 'Chat is currently restricted to moderators and above', 'error');
        return;
    }
    
    const input = document.getElementById('chatInput');
    const content = input.value.trim();
    
    if (!content) return;
    
    const message = {
        id: Date.now(),
        user: state.currentUser.name,
        role: state.currentUser.role.name,
        content: content,
        timestamp: new Date()
    };
    
    // Add to local messages
    state.chat.messages[state.chat.activeChannel].push(message);
    
    // Send via WebSocket
    if (state.ws && state.ws.readyState === WebSocket.OPEN) {
        state.ws.send(JSON.stringify({
            type: 'chat',
            channel: state.chat.activeChannel,
            message: message
        }));
    }
    
    input.value = '';
    renderMessages();
}

function handleChatMessage(data) {
    if (!state.chat.messages[data.channel]) {
        state.chat.messages[data.channel] = [];
    }
    
    state.chat.messages[data.channel].push(data.message);
    
    if (data.channel === state.chat.activeChannel) {
        renderMessages();
    }
    
    // Show notification if not in chat view
    showNotification(`New message in ${data.channel}`, `${data.message.user}: ${data.message.content}`, 'info');
}

// ============================================
// ALERTS
// ============================================
function handleAlert(data) {
    state.notifications.unshift({
        id: Date.now(),
        type: data.type || 'info',
        title: data.title,
        message: data.message,
        timestamp: new Date()
    });
    
    showNotification(data.title, data.message, data.type);
    updateNotificationBadge();
}

function showNotification(title, message, type = 'info') {
    // Create toast notification
    const toast = document.createElement('div');
    toast.className = `toast toast-${type}`;
    toast.innerHTML = `
        <strong>${title}</strong>
        <p>${message}</p>
    `;
    
    toast.style.cssText = `
        position: fixed;
        bottom: 20px;
        right: 20px;
        background: ${type === 'error' ? '#ff4444' : type === 'success' ? '#00ff88' : '#00d4ff'};
        color: ${type === 'success' || type === 'info' ? '#1a1a2e' : 'white'};
        padding: 15px 20px;
        border-radius: 8px;
        z-index: 10000;
        animation: slideIn 0.3s ease;
    `;
    
    document.body.appendChild(toast);
    
    setTimeout(() => {
        toast.style.animation = 'slideOut 0.3s ease';
        setTimeout(() => toast.remove(), 300);
    }, 3000);
}

function updateNotificationBadge() {
    const badge = document.querySelector('.notif-badge');
    if (badge) {
        badge.textContent = state.notifications.length;
        badge.style.display = state.notifications.length > 0 ? 'block' : 'none';
    }
}

function toggleNotifications() {
    const panel = document.getElementById('notificationPanel');
    if (panel) {
        panel.classList.toggle('show');
    }
}

// ============================================
// USER MANAGEMENT
// ============================================
function updateUserPanel() {
    const nameEl = document.querySelector('.user-name');
    const roleEl = document.querySelector('.user-role');
    
    if (nameEl) nameEl.textContent = state.currentUser.name;
    if (roleEl) roleEl.textContent = state.currentUser.role.name;
}

function handleUserJoined(data) {
    showNotification('User Joined', `${data.user.name} has joined`, 'info');
}

function handleUserLeft(data) {
    showNotification('User Left', `${data.user.name} has left`, 'info');
}

// ============================================
// TEAM VIEW
// ============================================
function initTeamView() {
    renderTeamMembers();
    renderPermissionsTable();
}

function renderTeamMembers() {
    const mainTeamEl = document.getElementById('mainTeamMembers');
    const supportersEl = document.getElementById('supporterMembers');
    
    if (mainTeamEl) {
        mainTeamEl.innerHTML = TEAM_MEMBERS.main.map(member => createMemberCard(member)).join('');
    }
    
    if (supportersEl) {
        supportersEl.innerHTML = TEAM_MEMBERS.supporters.map(member => createMemberCard(member)).join('');
    }
}

function createMemberCard(member) {
    const role = ROLES[member.role.toUpperCase()] || ROLES.SUPPORTER;
    return `
        <div class="team-card">
            <div class="member-avatar">
                <i class="fas fa-user"></i>
            </div>
            <div class="member-info">
                <div class="member-name">${member.name}</div>
                <div class="member-role">${member.position}</div>
            </div>
            <span class="rank-badge ${member.role.toLowerCase()}">${role.name}</span>
        </div>
    `;
}

function renderPermissionsTable() {
    const tableEl = document.getElementById('permissionsTable');
    if (!tableEl) return;
    
    const permissions = Object.keys(PERMISSIONS);
    const roles = ['Admin', 'Moderator', 'Officer', 'Supporter'];
    
    tableEl.innerHTML = `
        <table>
            <thead>
                <tr>
                    <th>Permission</th>
                    ${roles.map(r => `<th>${r}</th>`).join('')}
                </tr>
            </thead>
            <tbody>
                ${permissions.map(perm => `
                    <tr>
                        <td>${formatPermission(perm)}</td>
                        ${roles.map(role => `
                            <td class="${PERMISSIONS[perm].includes(role) ? 'text-success' : 'text-danger'}">
                                <i class="fas ${PERMISSIONS[perm].includes(role) ? 'fa-check' : 'fa-times'}"></i>
                            </td>
                        `).join('')}
                    </tr>
                `).join('')}
            </tbody>
        </table>
    `;
}

function formatPermission(perm) {
    return perm.replace(/_/g, ' ').replace(/\b\w/g, l => l.toUpperCase());
}

// ============================================
// SETTINGS
// ============================================
function initSettings() {
    // Load saved settings
    const savedSettings = localStorage.getItem('stasis_settings');
    if (savedSettings) {
        state.settings = { ...state.settings, ...JSON.parse(savedSettings) };
    }
    
    // Update UI
    const radiusSlider = document.getElementById('patrolRadiusSlider');
    if (radiusSlider) {
        radiusSlider.value = state.settings.patrolRadius;
        document.getElementById('radiusValue').textContent = state.settings.patrolRadius + 'm';
    }
    
    // Checkboxes
    document.getElementById('streamEnabled').checked = state.settings.streamEnabled;
    document.getElementById('notificationsEnabled').checked = state.settings.notificationsEnabled;
    document.getElementById('chatRestricted').checked = state.settings.chatRestricted;
    document.getElementById('autoReturnHome').checked = state.settings.autoReturnHome;
}

function saveSettings() {
    if (!checkPermissionAndAlert('EDIT_SETTINGS', 'edit settings')) return;
    
    // Get values from form
    state.settings.patrolRadius = parseInt(document.getElementById('patrolRadiusSlider').value);
    state.settings.streamEnabled = document.getElementById('streamEnabled').checked;
    state.settings.notificationsEnabled = document.getElementById('notificationsEnabled').checked;
    state.settings.chatRestricted = document.getElementById('chatRestricted').checked;
    state.settings.autoReturnHome = document.getElementById('autoReturnHome').checked;
    
    // Save to localStorage
    localStorage.setItem('stasis_settings', JSON.stringify(state.settings));
    
    showNotification('Settings Saved', 'Your settings have been saved', 'success');
}

// ============================================
// ROVER CONTROLS
// ============================================
function startPatrol() {
    if (!checkPermissionAndAlert('START_PATROL', 'start patrol')) return;
    sendCommand('start_patrol');
}

function stopPatrol() {
    if (!checkPermissionAndAlert('STOP_PATROL', 'stop patrol')) return;
    sendCommand('stop_patrol');
}

function returnHome() {
    if (!checkPermissionAndAlert('RETURN_HOME', 'return rover home')) return;
    sendCommand('return_home');
}

function emergencyStop() {
    sendCommand('emergency_stop');
    showNotification('Emergency Stop', 'Rover has been stopped!', 'error');
}

// ============================================
// UTILITY FUNCTIONS
// ============================================
function formatTime(date) {
    if (typeof date === 'string') date = new Date(date);
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
}

// ============================================
// CSS ANIMATIONS (injected)
// ============================================
const style = document.createElement('style');
style.textContent = `
    @keyframes slideIn {
        from { transform: translateX(100%); opacity: 0; }
        to { transform: translateX(0); opacity: 1; }
    }
    @keyframes slideOut {
        from { transform: translateX(0); opacity: 1; }
        to { transform: translateX(100%); opacity: 0; }
    }
    .message { margin-bottom: 10px; padding: 8px; background: rgba(255,255,255,0.05); border-radius: 5px; }
    .message-header { display: flex; justify-content: space-between; margin-bottom: 5px; }
    .message-time { font-size: 11px; color: #888; }
    .message-content { font-size: 14px; }
`;
document.head.appendChild(style);

