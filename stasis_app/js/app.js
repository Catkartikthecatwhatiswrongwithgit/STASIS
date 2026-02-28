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
// APPLICATION STATE
// ============================================
let state = {
    currentUser: null,
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
    fenceRect: null,
    fenceStart: null,
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
        patrolRadius: CONFIG.DEFAULT_PATROL_RADIUS
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
    try {
        const savedUser = localStorage.getItem('stasis_user');
        if (savedUser) {
            const parsed = JSON.parse(savedUser);
            if (parsed && parsed.name) {
                state.currentUser = parsed;
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
    document.querySelectorAll('.nav-item').forEach(function(item) {
        item.addEventListener('click', function(e) {
            e.preventDefault();
            const view = item.getAttribute('data-view');
            if (view) switchView(view);
        });
    });
    
    const loginForm = document.getElementById('loginForm');
    if (loginForm) {
        loginForm.addEventListener('submit', handleLogin);
    }
    
    const chatInput = document.getElementById('chatInput');
    if (chatInput) {
        chatInput.addEventListener('keypress', function(e) {
            if (e.key === 'Enter') sendChatMessage();
        });
    }
    
    const chatSendBtn = document.querySelector('.chat-input-container button');
    if (chatSendBtn) {
        chatSendBtn.addEventListener('click', sendChatMessage);
    }
    
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
    
    const nameInput = document.getElementById('loginEmail');
    const name = nameInput ? nameInput.value.trim() : '';
    
    if (!name) {
        continueAsGuest();
        return;
    }
    
    state.currentUser = {
        name: name,
        role: 'Officer',
        color: '#A0522D'
    };
    
    localStorage.setItem('stasis_user', JSON.stringify(state.currentUser));
    hideLoginModal();
    showApp();
    showNotification('Welcome', 'Logged in as ' + state.currentUser.name, 'success');
}

function continueAsGuest() {
    state.currentUser = {
        name: 'Guest',
        role: 'Guest',
        color: '#556B2F'
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
    updateUIForConnection(false);
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
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { attribution: '' }).addTo(state.miniMap);
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
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { attribution: '' }).addTo(state.map);
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
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { attribution: '' }).addTo(state.fencingMap);
        state.fencingMap.on('mousedown', handleFenceMouseDown);
        state.fencingMap.on('mousemove', handleFenceMouseMove);
        state.fencingMap.on('mouseup', handleFenceMouseUp);
        L.marker(CONFIG.MAP_CENTER, { icon: createHomeIcon(), title: 'Base Station' }).addTo(state.fencingMap);
    } catch (e) {
        console.error('FencingMap error:', e);
    }
}

function createRoverIcon() {
    return L.divIcon({
        className: 'rover-marker',
        html: '<div style="background:#CD853F;width:24px;height:24px;border-radius:50%;border:3px solid white;display:flex;align-items:center;justify-content:center;"><i class="fas fa-robot" style="color:white;font-size:11px;"></i></div>',
        iconSize: [24, 24], iconAnchor: [12, 12]
    });
}

function createHomeIcon() {
    return L.divIcon({
        className: 'home-marker',
        html: '<div style="background:#228B22;width:20px;height:20px;border-radius:4px;border:2px solid white;display:flex;align-items:center;justify-content:center;"><i class="fas fa-home" style="color:white;font-size:10px;"></i></div>',
        iconSize: [20, 20], iconAnchor: [10, 10]
    });
}

function updateRoverPosition(lat, lng) {
    state.rover.position = [lat, lng];
    if (state.roverMarker) state.roverMarker.setLatLng([lat, lng]);
    if (state.miniMap) state.miniMap.panTo([lat, lng], { animate: true });
    if (state.map) state.map.panTo([lat, lng], { animate: true });
    
    const latEl = document.getElementById('roverLat');
    const lngEl = document.getElementById('roverLng');
    if (latEl) latEl.textContent = lat.toFixed(6);
    if (lngEl) lngEl.textContent = lng.toFixed(6);
}

function centerOnRover() {
    if (state.rover.position && state.map) state.map.setView(state.rover.position, state.map.getZoom());
}

function centerOnBase() {
    if (state.map) state.map.setView(CONFIG.MAP_CENTER, state.map.getZoom());
}

// ============================================
// RECTANGLE FENCING
// ============================================
function startFencingMode() {
    state.fencingMode = true;
    showNotification('Fencing Mode', 'Click and drag to draw patrol area', 'info');
    if (state.fencingMap) state.fencingMap.getContainer().style.cursor = 'crosshair';
}

function handleFenceMouseDown(e) {
    if (!state.fencingMode) return;
    state.fenceStart = e.latlng;
    if (state.fenceRect) state.fencingMap.removeLayer(state.fenceRect);
}

function handleFenceMouseMove(e) {
    if (!state.fencingMode || !state.fenceStart) return;
    const bounds = L.latLngBounds(state.fenceStart, e.latlng);
    if (state.fenceRect) state.fencingMap.removeLayer(state.fenceRect);
    state.fenceRect = L.rectangle(bounds, { color: '#CD853F', weight: 2, fillOpacity: 0.2 }).addTo(state.fencingMap);
    updateFencingInfo();
}

function handleFenceMouseUp(e) {
    if (!state.fencingMode || !state.fenceStart) return;
    state.fenceStart = null;
    updateFencingInfo();
}

function updateFencingInfo() {
    const fencePointsEl = document.getElementById('fencePoints');
    const fenceAreaEl = document.getElementById('fenceArea');
    
    if (state.fenceRect && fencePointsEl && fenceAreaEl) {
        const area = state.fenceRect.getArea();
        fencePointsEl.textContent = 'Rectangle';
        fenceAreaEl.textContent = (area / 10000).toFixed(2) + ' hectares';
    } else if (fencePointsEl && fenceAreaEl) {
        fencePointsEl.textContent = '0';
        fenceAreaEl.textContent = '0';
    }
}

function clearFence() {
    if (state.fenceRect) {
        state.fencingMap.removeLayer(state.fenceRect);
        state.fenceRect = null;
    }
    state.fenceStart = null;
    updateFencingInfo();
    showNotification('Fence Cleared', 'Patrol area has been cleared', 'success');
}

function saveFence() {
    if (!state.fenceRect) {
        showNotification('No Fence', 'Please draw a patrol area first', 'error');
        return;
    }
    const bounds = state.fenceRect.getBounds();
    const points = [[bounds.getSouthWest().lat, bounds.getSouthWest().lng], [bounds.getNorthEast().lat, bounds.getNorthEast().lng]];
    sendCommand('set_fence', { bounds: points });
    showNotification('Fence Saved', 'Patrol area has been saved', 'success');
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
    if (slider && value) value.textContent = slider.value + 'm';
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
            if (state.currentUser) state.ws.send(JSON.stringify({ type: 'auth', user: state.currentUser }));
        };
        
        state.ws.onmessage = function(event) {
            try {
                const data = JSON.parse(event.data);
                handleWebSocketMessage(data);
                state.lastTelemetryTime = Date.now();
            } catch (e) { console.error('WebSocket message error:', e); }
        };
        
        state.ws.onclose = function() {
            console.log('WebSocket disconnected');
            state.connected = false;
            state.rover.connected = false;
            updateUIForConnection(false);
            setTimeout(connectWebSocket, CONFIG.RECONNECT_INTERVAL);
        };
        
        state.ws.onerror = function(error) { console.error('WebSocket error:', error); };
        
    } catch (error) {
        console.error('Failed to connect WebSocket:', error);
        startSimulationMode();
    }
    
    setTimeout(function() {
        if (!state.connected && !state.simulationMode) startSimulationMode();
    }, 3000);
}

function disconnectWebSocket() {
    if (state.ws) { state.ws.close(); state.ws = null; }
}

function handleWebSocketMessage(data) {
    switch (data.type) {
        case 'telemetry': handleTelemetry(data); break;
        case 'chat': handleChatMessage(data.payload || data); break;
        case 'alert': handleAlert(data.payload || data); break;
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
    
    if (data.lat && data.lng) updateRoverPosition(data.lat, data.lng);
    
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
    if (distanceEl) distanceEl.textContent = (state.rover.distance || 999).toFixed(1) + ' cm';
    
    const stateEl = document.getElementById('roverState');
    if (stateEl) stateEl.textContent = (state.rover.status || 'OFFLINE').toUpperCase();
    
    const headingEl = document.getElementById('roverHeading');
    if (headingEl) headingEl.textContent = (state.rover.heading || 0).toFixed(0) + '°';
    
    const speedEl = document.getElementById('roverSpeed');
    if (speedEl) speedEl.textContent = ((state.rover.speed || 0) * 3.6).toFixed(1) + ' km/h';
    
    const packetsEl = document.getElementById('packetsToday');
    if (packetsEl) packetsEl.textContent = state.packetsReceived;
}

function updateUIForConnection(connected) {
    const statusDot = document.querySelector('.status-dot');
    const statusText = document.querySelector('.connection-status span');
    
    if (statusDot) statusDot.style.background = connected ? '#228B22' : '#B22222';
    if (statusText) statusText.textContent = connected ? 'Connected' : 'Offline';
    
    const miniMapEl = document.getElementById('miniMap');
    if (miniMapEl) miniMapEl.style.opacity = connected ? '1' : '0.5';
}

// ============================================
// SIMULATION MODE
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
        
        if (state.rover.status !== 'DOCKED') state.rover.battery = Math.max(0, state.rover.battery - 0.003);
        state.rover.distance = 30 + Math.random() * 150;
        
        updateDashboard();
    }, 1500);
}

// ============================================
// CHAT
// ============================================
function initChat() {
    state.chat.channels.forEach(function(channel) { state.chat.messages[channel.id] = []; });
    addSystemMessage('Welcome to Stasis Sentinel!');
    renderChannels();
    renderMessages();
}

function addSystemMessage(content) {
    if (!state.chat.messages.general) state.chat.messages.general = [];
    state.chat.messages.general.push({ id: Date.now(), user: 'System', role: 'Admin', content: content, timestamp: new Date() });
}

function renderChannels() {
    const channelList = document.querySelector('.channel-list');
    if (!channelList) return;
    
    var html = '';
    for (var i = 0; i < state.chat.channels.length; i++) {
        var channel = state.chat.channels[i];
        var active = channel.id === state.chat.activeChannel ? 'active' : '';
        html += '<div class="channel ' + active + '" data-channel="' + channel.id + '"><i class="fas ' + channel.icon + '"></i><span>' + channel.name + '</span></div>';
    }
    channelList.innerHTML = html;
    
    channelList.querySelectorAll('.channel').forEach(function(el) {
        el.addEventListener('click', function() {
            state.chat.activeChannel = this.getAttribute('data-channel');
            renderChannels();
            renderMessages();
        });
    });
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
        html += '<div class="message"><div class="message-header"><span class="message-user">' + escapeHtml(msg.user) + '</span><span class="message-time">' + formatTime(msg.timestamp) + '</span></div><div class="message-content">' + escapeHtml(msg.content) + '</div></div>';
    }
    messagesContainer.innerHTML = html;
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
}

function sendChatMessage() {
    const input = document.getElementById('chatInput');
    var content = input ? input.value.trim() : '';
    if (!content || !state.currentUser) return;
    
    var message = { id: Date.now(), user: state.currentUser.name, role: state.currentUser.role, content: content, timestamp: new Date() };
    
    if (!state.chat.messages[state.chat.activeChannel]) state.chat.messages[state.chat.activeChannel] = [];
    state.chat.messages[state.chat.activeChannel].push(message);
    
    if (state.ws && state.ws.readyState === WebSocket.OPEN) {
        state.ws.send(JSON.stringify({ type: 'chat', channel: state.chat.activeChannel, message: message }));
    }
    
    if (input) input.value = '';
    renderMessages();
}

function handleChatMessage(data) {
    if (!state.chat.messages[data.channel]) state.chat.messages[data.channel] = [];
    state.chat.messages[data.channel].push(data.message);
    if (data.channel === state.chat.activeChannel) renderMessages();
}

// ============================================
// ALERTS
// ============================================
function handleAlert(data) {
    var alert = { id: Date.now(), type: data.type || 'warning', title: data.title || 'Alert', message: data.message || '', timestamp: new Date(), status: data.status || 'UNKNOWN' };
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
        html += '<div class="alert-item ' + alert.type + '"><div class="alert-header"><span class="alert-title">' + escapeHtml(alert.title) + '</span><span class="alert-time">' + formatTime(alert.timestamp) + '</span></div><div class="alert-message">' + escapeHtml(alert.message) + '</div></div>';
    }
    alertsList.innerHTML = html;
    
    var hazardsEl = document.getElementById('hazardsDetected');
    if (hazardsEl) hazardsEl.textContent = state.alerts.length;
}

function showNotification(title, message, type) {
    type = type || 'info';
    var toast = document.createElement('div');
    toast.className = 'toast toast-' + type;
    toast.innerHTML = '<strong>' + escapeHtml(title) + '</strong><p>' + escapeHtml(message) + '</p>';
    toast.style.cssText = 'position:fixed;bottom:20px;right:20px;background:' + (type === 'error' ? '#B22222' : type === 'success' ? '#228B22' : '#A0522D') + ';color:white;padding:15px 20px;border-radius:8px;z-index:10000;animation:slideIn 0.3s ease;max-width:300px;';
    document.body.appendChild(toast);
    setTimeout(function() { toast.style.animation = 'slideOut 0.3s ease'; setTimeout(function() { toast.remove(); }, 300); }, 3000);
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
    
    if (nameEl && state.currentUser) nameEl.textContent = state.currentUser.name;
    if (roleEl && state.currentUser) roleEl.textContent = state.currentUser.role || 'User';
    if (avatarEl && state.currentUser) avatarEl.style.background = state.currentUser.color || '#8B4513';
}

// ============================================
// SETTINGS
// ============================================
function initSettings() {
    var savedSettings = localStorage.getItem('stasis_settings');
    if (savedSettings) {
        try { state.settings = Object.assign({}, state.settings, JSON.parse(savedSettings)); } catch (e) {}
    }
    var radiusSlider = document.getElementById('patrolRadius');
    if (radiusSlider) { radiusSlider.value = state.settings.patrolRadius; updateRadiusDisplay(); }
}

function saveSettings() {
    var radiusSlider = document.getElementById('patrolRadius');
    if (radiusSlider) state.settings.patrolRadius = parseInt(radiusSlider.value);
    localStorage.setItem('stasis_settings', JSON.stringify(state.settings));
    showNotification('Settings Saved', 'Your settings have been saved', 'success');
}

function resetSettings() {
    state.settings = { patrolRadius: CONFIG.DEFAULT_PATROL_RADIUS };
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
// UTILITY
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
    if (sidebar) sidebar.classList.toggle('open');
}

// ============================================
// CSS ANIMATIONS
// ============================================
(function() {
    var style = document.createElement('style');
    style.textContent = '@keyframes slideIn { from { transform: translateX(100%); opacity: 0; } to { transform: translateX(0); opacity: 1; } } @keyframes slideOut { from { transform: translateX(0); opacity: 1; } to { transform: translateX(100%); opacity: 0; } }';
    document.head.appendChild(style);
})();
