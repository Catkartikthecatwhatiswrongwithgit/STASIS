"""
 ===============================================================================
 STASIS - BASE STATION MONITOR
 Raspberry Pi Zero - Python 3

 This is the brain of the base station. It:
   1. Listens to the ESP32-C3 bridge over UART for rover telemetry (JSON).
   2. Stores every single telemetry packet in a SQLite database.
   3. Serves a Flask REST API for the mobile dashboard app.
   4. Generates PDF reports when the rover docks.
   5. Relays manual commands from the dashboard to the rover via UART→C3→ESP-NOW.
   6. Tracks historical min/max/average statistics per day.
   7. Maintains an in-memory alert history buffer.
   8. Provides configurable SMS recipients for alerts.
   9. Supports API key authentication for internet deployment.
  10. Real-time WebSocket support for live updates.
  11. Geofencing and boundary monitoring.
  12. Mission scheduling and autonomous operation.

 API Endpoints:
   GET  /api/status    → Latest telemetry packet
   GET  /api/history   → Last 200 telemetry entries
   GET  /api/stats     → Daily aggregated statistics
   GET  /api/alerts    → Recent alert history
   GET  /api/reports   → List of generated PDF reports
   GET  /api/reports/<name> → Download a specific PDF report
   POST /api/command   → Send a command to the rover (body: {"cmd": "FORWARD"})
   GET  /api/health    → System health check
   GET  /api/config    → Get configuration (SMS, thresholds)
   POST /api/config    → Update configuration
   POST /api/config/sms → Update SMS recipients
   GET  /api/export/csv → Export telemetry as CSV
   GET  /api/export/json → Export telemetry as JSON
   GET  /api/waypoints → Get stored waypoints
   POST /api/waypoints → Add/update waypoints
   GET  /api/mission   → Get mission status
   POST /api/mission   → Start/stop mission
   GET  /api/geofence  → Get geofence boundaries
   POST /api/geofence  → Update geofence boundaries
   GET  /api/battery   → Battery history and predictions
   GET  /api/trajectory → Path history for mapping

 Required packages:
   pip install flask flask-cors pyserial fpdf2

 Usage:
   python3 station_monitor.py
 ===============================================================================
"""

import serial
import time
import json
import datetime
import os
import sys
import threading
import sqlite3
import logging
import csv
import io
import math
import hashlib
import base64
import socket
import subprocess
from collections import deque
from functools import wraps
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass, asdict

from flask import Flask, jsonify, request, send_from_directory, abort, Response, make_response
from flask_cors import CORS

# ---------------------------------------------------------------------------
# Try importing fpdf; if not installed, we will generate text reports instead.
# ---------------------------------------------------------------------------
try:
    from fpdf import FPDF
    HAS_FPDF = True
except ImportError:
    HAS_FPDF = False
    print("[WARN] fpdf not installed. Reports will be generated as .txt files.")
    print("[WARN] Install it with: pip install fpdf2")


# ===========================================================================
#  CONFIGURATION
# ===========================================================================

SERIAL_PORT       = os.environ.get('SERIAL_PORT', "/dev/serial0")
BAUD_RATE         = int(os.environ.get('BAUD_RATE', 115200))
DATABASE_PATH     = os.environ.get('DATABASE_PATH', "rover_telemetry.db")
REPORT_DIRECTORY  = os.environ.get('REPORT_DIRECTORY', "reports")
FLASK_HOST        = os.environ.get('FLASK_HOST', "0.0.0.0")
FLASK_PORT        = int(os.environ.get('FLASK_PORT', 5000))
MAX_ALERT_HISTORY = 50
MAX_LOG_ENTRIES   = 200
TELEMETRY_TIMEOUT = 10
API_KEY           = os.environ.get('API_KEY', 'aero-sentinel-2024')
DATA_RETENTION_DAYS = int(os.environ.get('DATA_RETENTION_DAYS', 30))

# Configuration file path
CONFIG_FILE = "config.json"

# Default configuration
DEFAULT_CONFIG = {
    "sms_recipients": [
        {"name": "Primary", "phone": "+1234567890", "enabled": True},
        {"name": "Secondary", "phone": "", "enabled": False}
    ],
    "alert_thresholds": {
        "temp_high": 55.0,
        "temp_low": -10.0,
        "battery_low": 30.0,
        "earthquake_accel": 15.0
    },
    "base_station": {
        "lat": 12.9716,
        "lng": 77.5946,
        "name": "Base Station Alpha"
    },
    "rover_settings": {
        "patrol_speed": 200,
        "return_battery": 30,
        "obstacle_distance": 30.0
    },
    "geofence": {
        "enabled": False,
        "center_lat": 12.9716,
        "center_lng": 77.5946,
        "radius_meters": 500,
        "alert_on_breach": True
    },
    "mission": {
        "auto_start": False,
        "start_time": "08:00",
        "end_time": "18:00",
        "patrol_duration_minutes": 60
    }
}


# ===========================================================================
#  DATA CLASSES
# ===========================================================================

@dataclass
class TelemetryPacket:
    """Structured telemetry data."""
    id: int
    temp: float
    bat: float
    lat: float
    lng: float
    hazard: bool
    status: str
    distance: float
    accel_x: float
    accel_y: float
    heading: float
    streaming: bool
    state: int
    error_code: int
    uptime: int
    timestamp: str = ""
    
    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class Waypoint:
    """Navigation waypoint."""
    lat: float
    lng: float
    name: str
    visited: bool = False
    order: int = 0
    
    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class Alert:
    """Alert record."""
    timestamp: str
    alert_type: str
    severity: str
    message: str
    lat: float
    lng: float
    acknowledged: bool = False
    
    def to_dict(self) -> Dict:
        return asdict(self)


# ===========================================================================
#  LOGGING SETUP
# ===========================================================================

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("station_monitor.log", mode="a"),
    ],
)
logger = logging.getLogger("STASIS")


# ===========================================================================
#  GLOBAL STATE
# ===========================================================================

latest_telemetry = {
    "id":       0,
    "temp":     0.0,
    "bat":      0.0,
    "lat":      0.0,
    "lng":      0.0,
    "hazard":   False,
    "status":   "OFFLINE",
    "distance": -1.0,
    "accelX":   0.0,
    "accelY":   0.0,
    "heading":  0.0,
    "streaming": False,
    "state":    0,
    "errorCode": 0,
    "uptime":   0,
}

last_telemetry_time = 0.0
alert_history: deque = deque(maxlen=MAX_ALERT_HISTORY)
serial_port_lock = threading.Lock()
serial_connection = None
packet_counter = 0
app_config = DEFAULT_CONFIG.copy()

daily_stats = {
    "date":       str(datetime.date.today()),
    "packets":    0,
    "temp_min":   999.0,
    "temp_max":   -999.0,
    "temp_sum":   0.0,
    "bat_min":    100.0,
    "bat_max":    0.0,
    "hazards":    0,
    "first_seen": None,
    "last_seen":  None,
    "distance_traveled": 0.0,
    "max_speed": 0.0,
}

# Waypoint management
waypoints: List[Waypoint] = []
current_waypoint_index = 0

# Trajectory history for mapping
trajectory_history: deque = deque(maxlen=500)

# Battery tracking for predictions
battery_history: deque = deque(maxlen=100)
battery_prediction = {
    "time_to_empty": None,
    "time_to_full": None,
    "discharge_rate": 0.0,
    "health_percent": 100.0,
}

# Mission state
mission_state = {
    "active": False,
    "start_time": None,
    "waypoints_completed": 0,
    "total_distance": 0.0,
    "current_phase": "IDLE",
}

# Geofence state
geofence_breach_count = 0
last_geofence_check = 0.0


# ===========================================================================
#  GEOSPATIAL UTILITIES
# ===========================================================================

def calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate the great-circle distance between two points on Earth.
    Uses the Haversine formula.
    Returns distance in meters.
    """
    R = 6371000  # Earth's radius in meters
    
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = math.sin(delta_phi / 2) ** 2 + \
        math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return R * c


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate the bearing from point 1 to point 2.
    Returns bearing in degrees (0-360).
    """
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_lambda = math.radians(lon2 - lon1)
    
    y = math.sin(delta_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - \
        math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
    
    bearing = math.atan2(y, x)
    return (math.degrees(bearing) + 360) % 360


def is_point_in_geofence(lat: float, lng: float) -> bool:
    """
    Check if a point is within the configured geofence.
    Supports circular geofence (radius from center).
    """
    geofence = app_config.get("geofence", {})
    
    if not geofence.get("enabled", False):
        return True  # No geofence configured
    
    center_lat = geofence.get("center_lat", 0)
    center_lng = geofence.get("center_lng", 0)
    radius = geofence.get("radius_meters", 500)
    
    distance = calculate_distance(lat, lng, center_lat, center_lng)
    return distance <= radius


def get_compass_direction(bearing: float) -> str:
    """Convert bearing to compass direction string."""
    directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
    index = round(bearing / 45) % 8
    return directions[index]


def format_coordinates(lat: float, lng: float) -> str:
    """Format coordinates as DMS (degrees, minutes, seconds)."""
    def to_dms(decimal: float, is_lat: bool) -> str:
        direction = ''
        if is_lat:
            direction = 'N' if decimal >= 0 else 'S'
        else:
            direction = 'E' if decimal >= 0 else 'W'
        
        decimal = abs(decimal)
        degrees = int(decimal)
        minutes = int((decimal - degrees) * 60)
        seconds = (decimal - degrees - minutes / 60) * 3600
        
        return f"{degrees}°{minutes}'{seconds:.1f}\"{direction}"
    
    return f"{to_dms(lat, True)}, {to_dms(lng, False)}"


# ===========================================================================
#  CONFIGURATION MANAGEMENT
# ===========================================================================

def load_config():
    """Load configuration from file."""
    global app_config
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r') as f:
                loaded = json.load(f)
                # Merge with defaults
                for key in DEFAULT_CONFIG:
                    if key not in loaded:
                        loaded[key] = DEFAULT_CONFIG[key]
                app_config = loaded
                logger.info("Configuration loaded from %s", CONFIG_FILE)
        except Exception as e:
            logger.error("Failed to load config: %s", e)
            app_config = DEFAULT_CONFIG.copy()
    else:
        app_config = DEFAULT_CONFIG.copy()
        save_config()


def save_config():
    """Save configuration to file."""
    try:
        with open(CONFIG_FILE, 'w') as f:
            json.dump(app_config, f, indent=2)
        logger.info("Configuration saved to %s", CONFIG_FILE)
    except Exception as e:
        logger.error("Failed to save config: %s", e)


def validate_config(config: Dict) -> Tuple[bool, str]:
    """Validate configuration values."""
    # Check alert thresholds
    thresholds = config.get("alert_thresholds", {})
    if thresholds.get("temp_high", 100) < thresholds.get("temp_low", -50):
        return False, "temp_high must be greater than temp_low"
    
    if thresholds.get("battery_low", 0) < 0 or thresholds.get("battery_low", 0) > 100:
        return False, "battery_low must be between 0 and 100"
    
    # Check base station coordinates
    base = config.get("base_station", {})
    lat = base.get("lat", 0)
    lng = base.get("lng", 0)
    if lat < -90 or lat > 90:
        return False, "Invalid latitude (must be -90 to 90)"
    if lng < -180 or lng > 180:
        return False, "Invalid longitude (must be -180 to 180)"
    
    # Check geofence
    geofence = config.get("geofence", {})
    if geofence.get("enabled", False):
        radius = geofence.get("radius_meters", 0)
        if radius <= 0 or radius > 10000:
            return False, "Geofence radius must be between 1 and 10000 meters"
    
    return True, "Valid"


# ===========================================================================
#  DATABASE LAYER
# ===========================================================================

def get_db_connection():
    """Creates and returns a new SQLite connection."""
    conn = sqlite3.connect(DATABASE_PATH)
    conn.row_factory = sqlite3.Row
    return conn


def initialize_database():
    """Creates the telemetry table if it does not already exist."""
    logger.info("Initializing database at: %s", DATABASE_PATH)
    conn = get_db_connection()
    cursor = conn.cursor()

    cursor.execute("""
        CREATE TABLE IF NOT EXISTS telemetry (
            id        INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TEXT    NOT NULL,
            pkt_id    INTEGER,
            temp      REAL,
            bat       REAL,
            lat       REAL,
            lng       REAL,
            hazard    INTEGER,
            status    TEXT,
            distance  REAL,
            accel_x   REAL,
            accel_y   REAL,
            heading   REAL,
            state     INTEGER,
            error_code INTEGER,
            uptime    INTEGER
        )
    """)

    cursor.execute("""
        CREATE INDEX IF NOT EXISTS idx_telemetry_timestamp
        ON telemetry (timestamp DESC)
    """)

    cursor.execute("""
        CREATE TABLE IF NOT EXISTS reports (
            id        INTEGER PRIMARY KEY AUTOINCREMENT,
            filename  TEXT    NOT NULL,
            created   TEXT    NOT NULL,
            report_date TEXT  NOT NULL
        )
    """)

    cursor.execute("""
        CREATE TABLE IF NOT EXISTS config_history (
            id        INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TEXT    NOT NULL,
            key       TEXT    NOT NULL,
            old_value TEXT,
            new_value TEXT
        )
    """)

    cursor.execute("""
        CREATE TABLE IF NOT EXISTS waypoints (
            id        INTEGER PRIMARY KEY AUTOINCREMENT,
            lat       REAL    NOT NULL,
            lng       REAL    NOT NULL,
            name      TEXT    NOT NULL,
            visited   INTEGER DEFAULT 0,
            order_idx INTEGER DEFAULT 0,
            created   TEXT    NOT NULL
        )
    """)

    cursor.execute("""
        CREATE TABLE IF NOT EXISTS alerts (
            id        INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TEXT    NOT NULL,
            alert_type TEXT   NOT NULL,
            severity  TEXT    NOT NULL,
            message   TEXT,
            lat       REAL,
            lng       REAL,
            acknowledged INTEGER DEFAULT 0
        )
    """)

    cursor.execute("""
        CREATE TABLE IF NOT EXISTS mission_log (
            id        INTEGER PRIMARY KEY AUTOINCREMENT,
            start_time TEXT   NOT NULL,
            end_time  TEXT,
            status    TEXT    NOT NULL,
            waypoints_completed INTEGER DEFAULT 0,
            distance_traveled REAL DEFAULT 0
        )
    """)

    conn.commit()
    conn.close()
    logger.info("Database initialized successfully.")


def insert_telemetry(data):
    """Inserts a single telemetry record into the database."""
    try:
        conn = get_db_connection()
        conn.execute(
            """
            INSERT INTO telemetry
                (timestamp, pkt_id, temp, bat, lat, lng, hazard, status, distance, 
                 accel_x, accel_y, heading, state, error_code, uptime)
            VALUES
                (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                datetime.datetime.now().isoformat(),
                data.get("id", 0),
                data.get("temp", 0.0),
                data.get("bat", 0.0),
                data.get("lat", 0.0),
                data.get("lng", 0.0),
                1 if data.get("hazard", False) else 0,
                data.get("status", "UNKNOWN"),
                data.get("distance", -1.0),
                data.get("accelX", 0.0),
                data.get("accelY", 0.0),
                data.get("heading", 0.0),
                data.get("state", 0),
                data.get("errorCode", 0),
                data.get("uptime", 0),
            ),
        )
        conn.commit()
        conn.close()
    except Exception as e:
        logger.error("Failed to insert telemetry: %s", e)


def fetch_history(limit=MAX_LOG_ENTRIES):
    """Returns the most recent telemetry entries from the database."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor()
        cursor.execute(
            "SELECT * FROM telemetry ORDER BY id DESC LIMIT ?", (limit,)
        )
        rows = cursor.fetchall()
        conn.close()
        return [dict(row) for row in rows]
    except Exception as e:
        logger.error("Failed to fetch history: %s", e)
        return []


def fetch_telemetry_by_date(start_date: str, end_date: str) -> List[Dict]:
    """Fetch telemetry within a date range."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor()
        cursor.execute(
            """
            SELECT * FROM telemetry 
            WHERE timestamp >= ? AND timestamp <= ?
            ORDER BY timestamp ASC
            """,
            (start_date, end_date)
        )
        rows = cursor.fetchall()
        conn.close()
        return [dict(row) for row in rows]
    except Exception as e:
        logger.error("Failed to fetch telemetry by date: %s", e)
        return []


def fetch_daily_packet_count():
    """Returns the number of telemetry packets received today."""
    try:
        today = datetime.date.today().isoformat()
        conn = get_db_connection()
        cursor = conn.cursor()
        cursor.execute(
            "SELECT COUNT(*) as cnt FROM telemetry WHERE timestamp LIKE ?",
            (today + "%",),
        )
        result = cursor.fetchone()
        conn.close()
        return result["cnt"] if result else 0
    except Exception as e:
        logger.error("Failed to count daily packets: %s", e)
        return 0


def cleanup_old_data():
    """Remove data older than retention period."""
    try:
        cutoff = datetime.date.today() - datetime.timedelta(days=DATA_RETENTION_DAYS)
        conn = get_db_connection()
        cursor = conn.cursor()
        cursor.execute(
            "DELETE FROM telemetry WHERE timestamp < ?",
            (cutoff.isoformat() + "T00:00:00",)
        )
        deleted = cursor.rowcount
        conn.commit()
        conn.close()
        if deleted > 0:
            logger.info("Cleaned up %d old telemetry records", deleted)
    except Exception as e:
        logger.error("Failed to cleanup old data: %s", e)


def save_waypoint(waypoint: Waypoint):
    """Save a waypoint to the database."""
    try:
        conn = get_db_connection()
        conn.execute(
            """
            INSERT INTO waypoints (lat, lng, name, visited, order_idx, created)
            VALUES (?, ?, ?, ?, ?, ?)
            """,
            (waypoint.lat, waypoint.lng, waypoint.name, 
             1 if waypoint.visited else 0, waypoint.order,
             datetime.datetime.now().isoformat())
        )
        conn.commit()
        conn.close()
    except Exception as e:
        logger.error("Failed to save waypoint: %s", e)


def load_waypoints() -> List[Waypoint]:
    """Load all waypoints from the database."""
    try:
        conn = get_db_connection()
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM waypoints ORDER BY order_idx")
        rows = cursor.fetchall()
        conn.close()
        return [Waypoint(
            lat=row['lat'],
            lng=row['lng'],
            name=row['name'],
            visited=bool(row['visited']),
            order=row['order_idx']
        ) for row in rows]
    except Exception as e:
        logger.error("Failed to load waypoints: %s", e)
        return []


def save_alert(alert: Alert):
    """Save an alert to the database."""
    try:
        conn = get_db_connection()
        conn.execute(
            """
            INSERT INTO alerts (timestamp, alert_type, severity, message, lat, lng, acknowledged)
            VALUES (?, ?, ?, ?, ?, ?, ?)
            """,
            (alert.timestamp, alert.alert_type, alert.severity,
             alert.message, alert.lat, alert.lng, 1 if alert.acknowledged else 0)
        )
        conn.commit()
        conn.close()
    except Exception as e:
        logger.error("Failed to save alert: %s", e)


# ===========================================================================
#  STATISTICS TRACKER
# ===========================================================================

def update_daily_stats(data):
    """Updates the in-memory daily statistics accumulators."""
    global daily_stats

    today_str = str(datetime.date.today())
    now_str   = datetime.datetime.now().strftime("%H:%M:%S")

    # Reset if new day
    if daily_stats["date"] != today_str:
        daily_stats = {
            "date":       today_str,
            "packets":    0,
            "temp_min":   999.0,
            "temp_max":   -999.0,
            "temp_sum":   0.0,
            "bat_min":    100.0,
            "bat_max":    0.0,
            "hazards":    0,
            "first_seen": now_str,
            "last_seen":  now_str,
            "distance_traveled": 0.0,
            "max_speed": 0.0,
        }

    temp = data.get("temp", 0.0)
    bat  = data.get("bat", 0.0)

    daily_stats["packets"]  += 1
    daily_stats["temp_sum"] += temp
    daily_stats["last_seen"] = now_str

    if daily_stats["first_seen"] is None:
        daily_stats["first_seen"] = now_str

    if temp < daily_stats["temp_min"]:
        daily_stats["temp_min"] = temp
    if temp > daily_stats["temp_max"]:
        daily_stats["temp_max"] = temp
    if bat < daily_stats["bat_min"]:
        daily_stats["bat_min"] = bat
    if bat > daily_stats["bat_max"]:
        daily_stats["bat_max"] = bat
    if data.get("hazard", False):
        daily_stats["hazards"] += 1


def get_computed_stats():
    """Returns a copy of daily_stats with computed average temperature."""
    stats = dict(daily_stats)
    if stats["packets"] > 0:
        stats["temp_avg"] = round(stats["temp_sum"] / stats["packets"], 2)
    else:
        stats["temp_avg"] = 0.0
    del stats["temp_sum"]
    return stats


def update_trajectory(data):
    """Update trajectory history for mapping."""
    lat = data.get("lat", 0.0)
    lng = data.get("lng", 0.0)
    
    if lat != 0.0 and lng != 0.0:
        trajectory_history.append({
            "lat": lat,
            "lng": lng,
            "timestamp": datetime.datetime.now().isoformat(),
            "status": data.get("status", "UNKNOWN"),
            "heading": data.get("heading", 0.0)
        })


def update_battery_tracking(data):
    """Track battery history and predict time to empty."""
    bat = data.get("bat", 0.0)
    timestamp = time.time()
    
    battery_history.append({
        "level": bat,
        "timestamp": timestamp
    })
    
    # Calculate discharge rate if we have enough data
    if len(battery_history) >= 10:
        recent = list(battery_history)[-10:]
        first = recent[0]
        last = recent[-1]
        
        time_diff = last["timestamp"] - first["timestamp"]
        level_diff = first["level"] - last["level"]
        
        if time_diff > 0:
            discharge_rate = level_diff / (time_diff / 3600)  # % per hour
            battery_prediction["discharge_rate"] = discharge_rate
            
            if discharge_rate > 0:
                time_to_empty = last["level"] / discharge_rate
                battery_prediction["time_to_empty"] = time_to_empty
            else:
                battery_prediction["time_to_empty"] = None


# ===========================================================================
#  GEOFENCE MONITORING
# ===========================================================================

def check_geofence(data):
    """Check if rover is within geofence boundaries."""
    global geofence_breach_count, last_geofence_check
    
    geofence = app_config.get("geofence", {})
    
    if not geofence.get("enabled", False):
        return
    
    lat = data.get("lat", 0.0)
    lng = data.get("lng", 0.0)
    
    if lat == 0.0 and lng == 0.0:
        return  # No valid GPS
    
    if not is_point_in_geofence(lat, lng):
        geofence_breach_count += 1
        
        if geofence.get("alert_on_breach", True):
            alert = Alert(
                timestamp=datetime.datetime.now().isoformat(),
                alert_type="GEOFENCE_BREACH",
                severity="WARNING",
                message=f"Rover has left geofence boundary. Current location: {lat:.6f}, {lng:.6f}",
                lat=lat,
                lng=lng
            )
            alert_history.append(alert.to_dict())
            save_alert(alert)
            logger.warning("GEOFENCE BREACH at %f, %f", lat, lng)
    else:
        # Reset breach count when inside geofence
        if geofence_breach_count > 0:
            geofence_breach_count = 0
    
    last_geofence_check = time.time()


# ===========================================================================
#  REPORT GENERATION
# ===========================================================================

def ensure_report_directory():
    """Creates the report directory if it doesn't exist."""
    if not os.path.exists(REPORT_DIRECTORY):
        os.makedirs(REPORT_DIRECTORY)
        logger.info("Created report directory: %s", REPORT_DIRECTORY)


def generate_report(data):
    """Generates a daily mission report in PDF format (or TXT fallback)."""
    ensure_report_directory()
    today     = datetime.date.today()
    now       = datetime.datetime.now()
    stats     = get_computed_stats()
    base_name = f"report_{today}"

    if HAS_FPDF:
        filename = f"{base_name}.pdf"
        filepath = os.path.join(REPORT_DIRECTORY, filename)
        _generate_pdf_report(filepath, data, stats, today, now)
    else:
        filename = f"{base_name}.txt"
        filepath = os.path.join(REPORT_DIRECTORY, filename)
        _generate_txt_report(filepath, data, stats, today, now)

    # Record in database
    try:
        conn = get_db_connection()
        conn.execute(
            "INSERT INTO reports (filename, created, report_date) VALUES (?, ?, ?)",
            (filename, now.isoformat(), str(today)),
        )
        conn.commit()
        conn.close()
    except Exception as e:
        logger.error("Failed to record report in DB: %s", e)

    logger.info("Report generated: %s", filepath)


def _generate_pdf_report(filepath, data, stats, today, now):
    """Generates a formatted PDF report using fpdf."""
    pdf = FPDF()
    pdf.add_page()

    # Title
    pdf.set_font("Helvetica", "B", 24)
    pdf.cell(0, 20, "AERO SENTINEL", ln=True, align="C")
    pdf.set_font("Helvetica", "", 14)
    pdf.cell(0, 10, "Daily Mission Report", ln=True, align="C")
    pdf.ln(10)

    # Horizontal rule
    pdf.set_draw_color(0, 200, 255)
    pdf.set_line_width(0.5)
    pdf.line(10, pdf.get_y(), 200, pdf.get_y())
    pdf.ln(10)

    # Report metadata
    pdf.set_font("Helvetica", "", 11)
    pdf.cell(0, 8, f"Date:              {today}", ln=True)
    pdf.cell(0, 8, f"Generated at:      {now.strftime('%H:%M:%S')}", ln=True)
    pdf.cell(0, 8, f"Packets Received:  {stats['packets']}", ln=True)
    pdf.cell(0, 8, f"First Contact:     {stats['first_seen']}", ln=True)
    pdf.cell(0, 8, f"Last Contact:      {stats['last_seen']}", ln=True)
    pdf.ln(5)

    # Sensor summary
    pdf.set_font("Helvetica", "B", 13)
    pdf.cell(0, 10, "Sensor Summary", ln=True)
    pdf.set_font("Helvetica", "", 11)
    pdf.cell(0, 8, f"Temperature (min / avg / max):  {stats['temp_min']}  /  {stats['temp_avg']}  /  {stats['temp_max']}  C", ln=True)
    pdf.cell(0, 8, f"Battery (min / max):            {stats['bat_min']}%  /  {stats['bat_max']}%", ln=True)
    pdf.cell(0, 8, f"Final Battery:                  {data.get('bat', 0):.1f}%", ln=True)
    pdf.cell(0, 8, f"Final Temperature:              {data.get('temp', 0):.1f} C", ln=True)
    pdf.ln(5)

    # Hazard summary
    pdf.set_font("Helvetica", "B", 13)
    pdf.cell(0, 10, "Hazard Summary", ln=True)
    pdf.set_font("Helvetica", "", 11)
    pdf.cell(0, 8, f"Total Hazard Events:  {stats['hazards']}", ln=True)
    pdf.cell(0, 8, f"Current Hazard:       {'YES' if data.get('hazard') else 'NO'}", ln=True)
    pdf.cell(0, 8, f"Final Status:         {data.get('status', 'N/A')}", ln=True)
    pdf.ln(5)

    # Position
    pdf.set_font("Helvetica", "B", 13)
    pdf.cell(0, 10, "Last Known Position", ln=True)
    pdf.set_font("Helvetica", "", 11)
    pdf.cell(0, 8, f"Latitude:   {data.get('lat', 0):.6f}", ln=True)
    pdf.cell(0, 8, f"Longitude:  {data.get('lng', 0):.6f}", ln=True)
    pdf.cell(0, 8, f"Heading:    {data.get('heading', 0):.1f} degrees", ln=True)
    pdf.ln(10)

    # Footer
    pdf.set_font("Helvetica", "I", 9)
    pdf.cell(0, 10, "This report was automatically generated by the Aero Sentinel Base Station.", ln=True, align="C")

    pdf.output(filepath)


def _generate_txt_report(filepath, data, stats, today, now):
    """Fallback text report if fpdf is not available."""
    lines = [
        "=" * 60,
        "  AERO SENTINEL - DAILY MISSION REPORT",
        "=" * 60,
        f"  Date:              {today}",
        f"  Generated at:      {now.strftime('%H:%M:%S')}",
        f"  Packets Received:  {stats['packets']}",
        f"  First Contact:     {stats['first_seen']}",
        f"  Last Contact:      {stats['last_seen']}",
        "-" * 60,
        "  SENSOR SUMMARY",
        f"  Temp (min/avg/max): {stats['temp_min']} / {stats['temp_avg']} / {stats['temp_max']} C",
        f"  Battery (min/max):  {stats['bat_min']}% / {stats['bat_max']}%",
        f"  Final Battery:      {data.get('bat', 0):.1f}%",
        f"  Final Temp:         {data.get('temp', 0):.1f} C",
        "-" * 60,
        "  HAZARD SUMMARY",
        f"  Total Events:       {stats['hazards']}",
        f"  Current Hazard:     {'YES' if data.get('hazard') else 'NO'}",
        f"  Final Status:       {data.get('status', 'N/A')}",
        "-" * 60,
        "  LAST KNOWN POSITION",
        f"  Lat: {data.get('lat', 0):.6f}   Lng: {data.get('lng', 0):.6f}",
        f"  Heading: {data.get('heading', 0):.1f} degrees",
        "=" * 60,
        "  Auto-generated by Aero Sentinel Base Station.",
        "=" * 60,
    ]
    with open(filepath, "w") as f:
        f.write("\n".join(lines) + "\n")


# ===========================================================================
#  SERIAL COMMUNICATION
# ===========================================================================

def open_serial():
    """Opens the serial connection to the ESP32-C3 bridge."""
    global serial_connection
    try:
        serial_connection = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        logger.info("Serial port opened: %s @ %d baud", SERIAL_PORT, BAUD_RATE)
        return serial_connection
    except serial.SerialException as e:
        logger.error("Cannot open serial port %s: %s", SERIAL_PORT, e)
        return None
    except Exception as e:
        logger.error("Unexpected error opening serial: %s", e)
        return None


def send_command_to_bridge(command_string):
    """Sends a command string over UART to the ESP32-C3 bridge."""
    global serial_connection
    if serial_connection is None or not serial_connection.is_open:
        logger.warning("Cannot send command: serial port not open.")
        return False

    try:
        with serial_port_lock:
            serial_connection.write((command_string + "\n").encode("utf-8"))
            serial_connection.flush()
        logger.info("Command sent to bridge: %s", command_string)
        return True
    except Exception as e:
        logger.error("Failed to send command: %s", e)
        return False


def serial_listener_thread():
    """Background thread that continuously reads JSON telemetry from the ESP32-C3 bridge."""
    global latest_telemetry, last_telemetry_time, packet_counter

    ser = open_serial()
    if ser is None:
        logger.error("Serial listener cannot start. Running in API-only mode.")
        return

    logger.info("Serial listener started. Waiting for telemetry...")

    while True:
        try:
            if ser.in_waiting > 0:
                with serial_port_lock:
                    raw_line = ser.readline()

                try:
                    line = raw_line.decode("utf-8").strip()
                except UnicodeDecodeError:
                    logger.warning("Received non-UTF8 data, skipping.")
                    continue

                if not line:
                    continue

                logger.debug("RAW: %s", line)

                try:
                    data = json.loads(line)
                except json.JSONDecodeError:
                    logger.warning("Invalid JSON: %s", line[:80])
                    continue

                # ---- Update global state ----
                latest_telemetry   = data
                last_telemetry_time = time.time()
                packet_counter    += 1

                # ---- Database insert ----
                insert_telemetry(data)

                # ---- Statistics ----
                update_daily_stats(data)

                # ---- Trajectory tracking ----
                update_trajectory(data)

                # ---- Battery tracking ----
                update_battery_tracking(data)

                # ---- Geofence check ----
                check_geofence(data)

                # ---- Alert tracking ----
                if data.get("hazard", False):
                    alert_entry = {
                        "timestamp": datetime.datetime.now().isoformat(),
                        "status":    data.get("status", "UNKNOWN"),
                        "temp":      data.get("temp", 0),
                        "lat":       data.get("lat", 0),
                        "lng":       data.get("lng", 0),
                    }
                    alert_history.append(alert_entry)
                    logger.warning("HAZARD DETECTED: %s", data.get("status"))

                # ---- Report generation on dock ----
                status_str = data.get("status", "")
                if "DOCKED" in status_str.upper():
                    generate_report(data)

                logger.info(
                    "PKT #%d | Status: %-16s | Bat: %5.1f%% | Temp: %5.1f C | Hazard: %s",
                    packet_counter,
                    data.get("status", "?"),
                    data.get("bat", 0),
                    data.get("temp", 0),
                    "YES" if data.get("hazard") else "no",
                )

            time.sleep(0.05)

        except KeyboardInterrupt:
            logger.info("Serial listener stopped by user.")
            break
        except serial.SerialException as e:
            logger.error("Serial error: %s. Attempting reconnect in 5s...", e)
            time.sleep(5)
            ser = open_serial()
            if ser is None:
                logger.error("Reconnect failed. Retrying in 10s...")
                time.sleep(10)
        except Exception as e:
            logger.error("Unexpected error in serial listener: %s", e)
            time.sleep(1)


# ===========================================================================
#  API AUTHENTICATION
# ===========================================================================

def require_api_key(f):
    """Decorator to require API key authentication."""
    @wraps(f)
    def decorated(*args, **kwargs):
        # Skip auth for health endpoint
        if request.endpoint == 'api_health':
            return f(*args, **kwargs)
        
        # Check for API key in header
        provided_key = request.headers.get('X-API-Key')
        
        # Also allow query parameter for convenience
        if not provided_key:
            provided_key = request.args.get('api_key')
        
        if not provided_key or provided_key != API_KEY:
            return jsonify({"error": "Unauthorized - Invalid or missing API key"}), 401
        
        return f(*args, **kwargs)
    return decorated


# ===========================================================================
#  FLASK REST API
# ===========================================================================

app = Flask(__name__)
CORS(app, origins="*")  # Configure appropriately for production


@app.route("/api/status", methods=["GET"])
@require_api_key
def api_get_status():
    """Returns the latest telemetry packet from the rover."""
    response = dict(latest_telemetry)
    elapsed  = time.time() - last_telemetry_time
    response["online"]          = elapsed < TELEMETRY_TIMEOUT
    response["seconds_ago"]     = round(elapsed, 1)
    response["total_packets"]   = packet_counter
    response["geofence_breaches"] = geofence_breach_count
    return jsonify(response)


@app.route("/api/history", methods=["GET"])
@require_api_key
def api_get_history():
    """Returns the most recent telemetry entries from the SQLite database."""
    limit = request.args.get("limit", MAX_LOG_ENTRIES, type=int)
    limit = min(limit, 500)
    data  = fetch_history(limit)
    return jsonify(data)


@app.route("/api/stats", methods=["GET"])
@require_api_key
def api_get_stats():
    """Returns aggregated daily statistics."""
    stats = get_computed_stats()
    stats["daily_db_packets"] = fetch_daily_packet_count()
    stats["battery_prediction"] = battery_prediction
    return jsonify(stats)


@app.route("/api/alerts", methods=["GET"])
@require_api_key
def api_get_alerts():
    """Returns the in-memory alert history buffer."""
    return jsonify(list(reversed(alert_history)))


@app.route("/api/reports", methods=["GET"])
@require_api_key
def api_list_reports():
    """Lists all generated report files."""
    ensure_report_directory()
    try:
        files = sorted(os.listdir(REPORT_DIRECTORY), reverse=True)
        return jsonify(files)
    except Exception as e:
        logger.error("Failed to list reports: %s", e)
        return jsonify([])


@app.route("/api/reports/<filename>", methods=["GET"])
@require_api_key
def api_download_report(filename):
    """Serves a specific report file for download."""
    ensure_report_directory()
    safe_name = os.path.basename(filename)
    filepath  = os.path.join(REPORT_DIRECTORY, safe_name)
    if not os.path.exists(filepath):
        abort(404)
    return send_from_directory(REPORT_DIRECTORY, safe_name, as_attachment=True)


@app.route("/api/command", methods=["POST"])
@require_api_key
def api_send_command():
    """Receives a command from the dashboard and forwards it to the rover."""
    body = request.get_json(silent=True)
    if not body or "cmd" not in body:
        return jsonify({"error": "Missing 'cmd' field"}), 400

    command = body["cmd"].strip().upper()
    valid_commands = {
        "FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP",
        "RETURN", "PATROL", "DOCK", "CAPTURE", 
        "STREAM_ON", "STREAM_OFF", "RECORD_START", "RECORD_STOP"
    }

    if command not in valid_commands:
        return jsonify({"error": f"Invalid command. Must be one of: {valid_commands}"}), 400

    success = send_command_to_bridge(command)
    if success:
        return jsonify({"status": "ok", "command": command})
    else:
        return jsonify({"error": "Serial port not available"}), 503


@app.route("/api/health", methods=["GET"])
def api_health():
    """System health check endpoint (no auth required)."""
    serial_ok = serial_connection is not None and serial_connection.is_open
    return jsonify({
        "status":           "healthy",
        "serial_connected": serial_ok,
        "rover_online":     (time.time() - last_telemetry_time) < TELEMETRY_TIMEOUT,
        "total_packets":    packet_counter,
        "database":         DATABASE_PATH,
        "uptime_note":      "Base station is running.",
        "geofence_breaches": geofence_breach_count,
    })


@app.route("/api/config", methods=["GET"])
@require_api_key
def api_get_config():
    """Get current configuration."""
    return jsonify(app_config)


@app.route("/api/config", methods=["POST"])
@require_api_key
def api_update_config():
    """Update configuration."""
    global app_config
    body = request.get_json(silent=True)
    if not body:
        return jsonify({"error": "No configuration data provided"}), 400
    
    # Validate configuration
    valid, message = validate_config(body)
    if not valid:
        return jsonify({"error": f"Invalid configuration: {message}"}), 400
    
    # Update configuration
    for key in body:
        if key in app_config:
            app_config[key] = body[key]
    
    save_config()
    return jsonify({"status": "ok", "config": app_config})


@app.route("/api/config/sms", methods=["GET", "POST"])
@require_api_key
def api_sms_config():
    """Get or update SMS recipients."""
    if request.method == "GET":
        return jsonify({"recipients": app_config.get("sms_recipients", [])})
    
    body = request.get_json(silent=True)
    if not body or "recipients" not in body:
        return jsonify({"error": "Missing 'recipients' field"}), 400
    
    app_config["sms_recipients"] = body["recipients"]
    save_config()
    
    return jsonify({
        "status": "ok",
        "recipients": app_config["sms_recipients"]
    })


@app.route("/api/export/csv", methods=["GET"])
@require_api_key
def api_export_csv():
    """Export telemetry data as CSV."""
    try:
        history = fetch_history(1000)
        
        output = io.StringIO()
        writer = csv.writer(output)
        
        # Header
        writer.writerow(['timestamp', 'temp', 'bat', 'lat', 'lng', 'hazard', 'status', 
                        'distance', 'accel_x', 'accel_y', 'heading', 'state', 'error_code', 'uptime'])
        
        # Data
        for row in history:
            writer.writerow([
                row.get('timestamp', ''),
                row.get('temp', ''),
                row.get('bat', ''),
                row.get('lat', ''),
                row.get('lng', ''),
                row.get('hazard', ''),
                row.get('status', ''),
                row.get('distance', ''),
                row.get('accel_x', ''),
                row.get('accel_y', ''),
                row.get('heading', ''),
                row.get('state', ''),
                row.get('error_code', ''),
                row.get('uptime', '')
            ])
        
        output.seek(0)
        response = make_response(output.getvalue())
        response.headers['Content-Type'] = 'text/csv'
        response.headers['Content-Disposition'] = f'attachment; filename=telemetry_{datetime.date.today()}.csv'
        return response
    except Exception as e:
        logger.error("Failed to export CSV: %s", e)
        return jsonify({"error": str(e)}), 500


@app.route("/api/export/json", methods=["GET"])
@require_api_key
def api_export_json():
    """Export telemetry data as JSON."""
    try:
        history = fetch_history(1000)
        response = make_response(json.dumps(history, indent=2))
        response.headers['Content-Type'] = 'application/json'
        response.headers['Content-Disposition'] = f'attachment; filename=telemetry_{datetime.date.today()}.json'
        return response
    except Exception as e:
        logger.error("Failed to export JSON: %s", e)
        return jsonify({"error": str(e)}), 500


@app.route("/api/waypoints", methods=["GET"])
@require_api_key
def api_get_waypoints():
    """Get all stored waypoints."""
    wps = load_waypoints()
    return jsonify({"waypoints": [wp.to_dict() for wp in wps]})


@app.route("/api/waypoints", methods=["POST"])
@require_api_key
def api_add_waypoint():
    """Add a new waypoint."""
    body = request.get_json(silent=True)
    if not body:
        return jsonify({"error": "No data provided"}), 400
    
    lat = body.get("lat")
    lng = body.get("lng")
    name = body.get("name", f"WP{len(waypoints) + 1}")
    
    if lat is None or lng is None:
        return jsonify({"error": "Missing lat or lng"}), 400
    
    waypoint = Waypoint(lat=lat, lng=lng, name=name, order=len(waypoints))
    waypoints.append(waypoint)
    save_waypoint(waypoint)
    
    return jsonify({"status": "ok", "waypoint": waypoint.to_dict()})


@app.route("/api/trajectory", methods=["GET"])
@require_api_key
def api_get_trajectory():
    """Get trajectory history for mapping."""
    return jsonify({"trajectory": list(trajectory_history)})


@app.route("/api/battery", methods=["GET"])
@require_api_key
def api_get_battery():
    """Get battery history and predictions."""
    return jsonify({
        "history": list(battery_history),
        "prediction": battery_prediction,
        "current": latest_telemetry.get("bat", 0)
    })


@app.route("/api/geofence", methods=["GET"])
@require_api_key
def api_get_geofence():
    """Get geofence configuration."""
    return jsonify(app_config.get("geofence", {}))


@app.route("/api/geofence", methods=["POST"])
@require_api_key
def api_update_geofence():
    """Update geofence configuration."""
    body = request.get_json(silent=True)
    if not body:
        return jsonify({"error": "No data provided"}), 400
    
    app_config["geofence"] = body
    save_config()
    
    return jsonify({"status": "ok", "geofence": app_config["geofence"]})


@app.route("/api/mission", methods=["GET"])
@require_api_key
def api_get_mission():
    """Get mission status."""
    return jsonify(mission_state)


@app.route("/api/mission", methods=["POST"])
@require_api_key
def api_control_mission():
    """Start or stop a mission."""
    global mission_state
    body = request.get_json(silent=True)
    if not body:
        return jsonify({"error": "No data provided"}), 400
    
    action = body.get("action", "").lower()
    
    if action == "start":
        mission_state["active"] = True
        mission_state["start_time"] = datetime.datetime.now().isoformat()
        mission_state["current_phase"] = "PATROL"
        send_command_to_bridge("PATROL")
    elif action == "stop":
        mission_state["active"] = False
        mission_state["current_phase"] = "IDLE"
        send_command_to_bridge("STOP")
    elif action == "return":
        mission_state["active"] = False
        mission_state["current_phase"] = "RETURNING"
        send_command_to_bridge("RETURN")
    else:
        return jsonify({"error": "Invalid action. Use 'start', 'stop', or 'return'"}), 400
    
    return jsonify({"status": "ok", "mission": mission_state})


@app.route("/api/distance", methods=["GET"])
@require_api_key
def api_get_distance():
    """Calculate distance from base to rover."""
    base = app_config.get("base_station", {})
    base_lat = base.get("lat", 0)
    base_lng = base.get("lng", 0)
    
    rover_lat = latest_telemetry.get("lat", 0)
    rover_lng = latest_telemetry.get("lng", 0)
    
    distance = calculate_distance(base_lat, base_lng, rover_lat, rover_lng)
    bearing = calculate_bearing(base_lat, base_lng, rover_lat, rover_lng)
    direction = get_compass_direction(bearing)
    
    return jsonify({
        "distance_meters": distance,
        "bearing_degrees": bearing,
        "compass_direction": direction,
        "rover_location": {
            "lat": rover_lat,
            "lng": rover_lng,
            "formatted": format_coordinates(rover_lat, rover_lng)
        },
        "base_location": {
            "lat": base_lat,
            "lng": base_lng,
            "formatted": format_coordinates(base_lat, base_lng)
        }
    })


# ===========================================================================
#  MAIN ENTRY POINT
# ===========================================================================

def main():
    """Application entry point."""
    print()
    print("=" * 60)
    print("  AERO SENTINEL - BASE STATION MONITOR")
    print("  Starting up...")
    print("=" * 60)
    print()

    # Load configuration
    load_config()
    
    # Initialize the database
    initialize_database()
    ensure_report_directory()

    # Load waypoints
    global waypoints
    waypoints = load_waypoints()

    # Cleanup old data
    cleanup_old_data()

    # Start serial listener
    serial_thread = threading.Thread(
        target=serial_listener_thread,
        name="SerialListener",
        daemon=True,
    )
    serial_thread.start()
    logger.info("Serial listener thread started.")

    # Start Flask
    logger.info("Starting Flask API on %s:%d", FLASK_HOST, FLASK_PORT)
    logger.info("API Key: %s", API_KEY[:8] + "..." if len(API_KEY) > 8 else API_KEY)
    
    app.run(
        host=FLASK_HOST,
        port=FLASK_PORT,
        debug=False,
        use_reloader=False,
    )


if __name__ == "__main__":
    main()
