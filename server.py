#!/usr/bin/env python
import sqlite3
import json
from datetime import datetime
from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
from threading import Thread, Lock
from contextlib import contextmanager

class HealthMonitoringServer(object):  # Inherit from object for Python 2
    def __init__(self):
        # Flask initialization
        self.app = Flask(__name__, static_folder='static')
        CORS(self.app)
        self.setup_routes()
        
        # Database constants
        self.DB_PATH = 'patients.db'
        self.VITAL_RANGES = {
            'heart_rate': (60, 100),
            'spo2': (95, 100),
            'temperature': (36.5, 37.5),
            'glucose_level': (70, 140)
        }
        
        # Initialize database
        self.db_lock = Lock()
        self.init_db()

    @contextmanager
    def get_db_connection(self):
        """Context manager for database connections with thread safety"""
        with self.db_lock:
            conn = sqlite3.connect(self.DB_PATH)
            try:
                yield conn
                conn.commit()
            finally:
                conn.close()

    def init_db(self):
        """Initialize database with tables and default data"""
        with self.get_db_connection() as conn:
            c = conn.cursor()
            c.execute('''CREATE TABLE IF NOT EXISTS patients
                        (id INTEGER PRIMARY KEY,
                         heart_rate REAL,
                         spo2 REAL,
                         temperature REAL,
                         glucose_level REAL,
                         medicals TEXT,
                         medical_schedule INTEGER,
                         timestamp TEXT,
                         patient_order INTEGER,
                         is_dispensing BOOLEAN DEFAULT 0)
                     ''')
            
            default_medicals = json.dumps([0, 0, 0, 0, 0])
            for patient_id in xrange(1, 4):  # xrange for Python 2
                c.execute('''INSERT OR IGNORE INTO patients 
                           VALUES (?, ?, NULL, NULL, NULL, ?, NULL, NULL, ?, ?)''',
                        (patient_id, patient_id, default_medicals, patient_id, False))

    def check_health_state(self, vitals):
        """Determine patient health state based on vital signs"""
        if any(vitals[key] is None for key in self.VITAL_RANGES.iterkeys()):  # iterkeys for Python 2
            return "unknown"
            
        for vital, (min_val, max_val) in self.VITAL_RANGES.iteritems():  # iteritems for Python 2
            if vitals[vital] < min_val or vitals[vital] > max_val:
                return "in danger"
        return "ok"

    def setup_routes(self):
        """Setup Flask routes with proper error handling"""
        @self.app.route('/api/patients', methods=['GET'])
        def get_patients():
            try:
                with self.get_db_connection() as conn:
                    c = conn.cursor()
                    c.execute('SELECT * FROM patients ORDER BY patient_order')
                    columns = ['id', 'heart_rate', 'spo2', 'temperature', 'glucose_level',
                             'medicals', 'medical_schedule', 'timestamp', 'patient_order',
                             'is_dispensing']
                    patients = []
                    for row in c.fetchall():
                        patient_dict = dict(zip(columns, row))
                        patient_dict['medicals'] = json.loads(patient_dict['medicals'])
                        patient_dict['is_dispensing'] = bool(patient_dict['is_dispensing'])
                        vitals = {
                            'heart_rate': patient_dict['heart_rate'],
                            'spo2': patient_dict['spo2'],
                            'temperature': patient_dict['temperature'],
                            'glucose_level': patient_dict['glucose_level']
                        }
                        patient_dict['health_state'] = self.check_health_state(vitals)
                        patients.append(patient_dict)
                    return jsonify(patients)
            except Exception as e:
                return jsonify({'error': str(e)}), 500

        @self.app.route('/api/patients/<int:patient_id>', methods=['POST'])
        def update_patient(patient_id):
            try:
                data = request.json
                required_fields = ['heart_rate', 'spo2', 'temperature', 'glucose_level',
                                 'medicals', 'medical_schedule', 'patient_order', 'is_dispensing']
                
                if not all(field in data for field in required_fields):
                    return jsonify({'error': 'Missing required fields'}), 400
                
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                
                with self.get_db_connection() as conn:
                    c = conn.cursor()
                    c.execute('''UPDATE patients SET 
                                heart_rate = ?, spo2 = ?, temperature = ?, 
                                glucose_level = ?, medicals = ?, 
                                medical_schedule = ?, timestamp = ?,
                                patient_order = ?, is_dispensing = ?
                                WHERE id = ?''',
                             (float(data['heart_rate']),
                              float(data['spo2']),
                              float(data['temperature']),
                              float(data['glucose_level']),
                              json.dumps(data['medicals']),
                              int(data['medical_schedule']),
                              timestamp,
                              int(data['patient_order']),
                              bool(data['is_dispensing']),
                              patient_id))
                
                return jsonify({'status': 'success'}), 200
                
            except ValueError as e:
                return jsonify({'error': 'Invalid data format: {0}'.format(str(e))}), 400
            except Exception as e:
                return jsonify({'error': str(e)}), 500

        @self.app.route('/')
        def index():
            return send_from_directory('static', 'index.html')

    def run(self):
        """Run the server"""
        self.app.run(debug=False, host='0.0.0.0', port=5000)

if __name__ == '__main__':
    server = HealthMonitoringServer()
    server.run()