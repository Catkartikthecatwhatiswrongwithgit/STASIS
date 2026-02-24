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
from collections import deque
from functools import wraps

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
API_KEY           = os.environ.get('API_KEY', 'stasis-2024')
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
    }
}


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
logger = logging.getLogger("Stasis")


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
}

last_telemetry_time = 0.0
alert_history = deque(maxlen=MAX_ALERT_HISTORY)
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
}


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
            accel_y   REAL
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
                (timestamp, pkt_id, temp, bat, lat, lng, hazard, status, distance, accel_x, accel_y)
            VALUES
                (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
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
    pdf.cell(0, 20, "STASIS", ln=True, align="C")
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
    pdf.ln(10)

    # Footer
    pdf.set_font("Helvetica", "I", 9)
    pdf.cell(0, 10, "This report was automatically generated by the Stasis Base Station.", ln=True, align="C")

    pdf.output(filepath)


def _generate_txt_report(filepath, data, stats, today, now):
    """Fallback text report if fpdf is not available."""
    lines = [
        "=" * 60,
        "  STASIS - DAILY MISSION REPORT",
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
        "=" * 60,
        "  Auto-generated by Stasis Base Station.",
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
    valid_commands = {"FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP"}

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
        writer.writerow(['timestamp', 'temp', 'bat', 'lat', 'lng', 'hazard', 'status', 'distance', 'accel_x', 'accel_y'])
        
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
                row.get('accel_y', '')
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
    command = body["cmd"].strip().upper()
    valid_commands = {"FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP"}

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
        writer.writerow(['timestamp', 'temp', 'bat', 'lat', 'lng', 'hazard', 'status', 'distance', 'accel_x', 'accel_y'])
        
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
                row.get('accel_y', '')
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

    command = body["cmd"].strip().upper()
    valid_commands = {"FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP"}

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
        writer.writerow(['timestamp', 'temp', 'bat', 'lat', 'lng', 'hazard', 'status', 'distance', 'accel_x', 'accel_y'])
        
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
                row.get('accel_y', '')
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
    command = body["cmd"].strip().upper()
    valid_commands = {"FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP"}

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
        writer.writerow(['timestamp', 'temp', 'bat', 'lat', 'lng', 'hazard', 'status', 'distance', 'accel_x', 'accel_y'])
        
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
                row.get('accel_y', '')
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


