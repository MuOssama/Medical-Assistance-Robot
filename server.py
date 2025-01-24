#!/usr/bin/env python
import sqlite3
import json
import logging
from datetime import datetime
from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
from threading import Thread, Lock
from contextlib import contextmanager

class HealthMonitoringServer:
    def __init__(self):
        # Flask initialization
        self.app = Flask(__name__, static_folder='static')
        CORS(self.app)
        
        # Configure logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)
        
        # Database constants
        self.DB_PATH = 'patients.db'
        self.VITAL_RANGES = {
            'heart_rate': (60, 100),
            'spo2': (95, 100),
            'temperature': (36.5, 37.5),
            'glucose_level': (70, 140)
        }
        
        # Initialize lock and database
        self.db_lock = Lock()
        self.init_db()
        self.setup_routes()

    @contextmanager
    def get_db_connection(self):
        """Enhanced context manager for database connections with error logging"""
        conn = None
        try:
            with self.db_lock:
                conn = sqlite3.connect(self.DB_PATH)
                conn.row_factory = sqlite3.Row  # Enable row factory for better data handling
                yield conn
                conn.commit()
        except sqlite3.Error as e:
            self.logger.error(f"Database error: {e}")
            if conn:
                conn.rollback()
            raise
        finally:
            if conn:
                conn.close()

    def init_db(self):
        """Initialize database with improved error handling and logging"""
        try:
            with self.get_db_connection() as conn:
                c = conn.cursor()
                
                # Create tables with proper constraints
                c.execute('''CREATE TABLE IF NOT EXISTS patients
                            (id INTEGER PRIMARY KEY,
                             heart_rate REAL CHECK(heart_rate IS NULL OR (heart_rate >= 0 AND heart_rate <= 300)),
                             spo2 REAL CHECK(spo2 IS NULL OR (spo2 >= 0 AND spo2 <= 100)),
                             temperature REAL CHECK(temperature IS NULL OR (temperature >= 30 AND temperature <= 45)),
                             glucose_level REAL CHECK(glucose_level IS NULL OR (glucose_level >= 0 AND glucose_level <= 500)),
                             medicals TEXT NOT NULL,
                             medical_schedule INTEGER,
                             timestamp TEXT,
                             patient_order INTEGER NOT NULL,
                             is_dispensing INTEGER DEFAULT 0 CHECK(is_dispensing IN (0, 1)),
                             UNIQUE(patient_order))
                         ''')
                
                # Insert default data if table is empty
                c.execute('SELECT COUNT(*) FROM patients')
                if c.fetchone()[0] == 0:
                    default_medicals = json.dumps([0, 0, 0, 0, 0])
                    for patient_id in range(1, 4):
                        c.execute('''INSERT INTO patients 
                                   (id, heart_rate, medicals, patient_order, is_dispensing)
                                   VALUES (?, ?, ?, ?, ?)''',
                                (patient_id, patient_id, default_medicals, patient_id, 0))
                
                self.logger.info("Database initialized successfully")
        except Exception as e:
            self.logger.error(f"Failed to initialize database: {e}")
            raise

    def validate_vital_signs(self, vitals):
        """Validate vital signs data"""
        errors = []
        for key, (min_val, max_val) in self.VITAL_RANGES.items():
            if key in vitals and vitals[key] is not None:
                try:
                    value = float(vitals[key])
                    if value < min_val or value > max_val:
                        errors.append(f"{key} value {value} is outside valid range ({min_val}-{max_val})")
                except ValueError:
                    errors.append(f"Invalid {key} value: {vitals[key]}")
        return errors

    def check_health_state(self, vitals):
        """Enhanced health state checking with validation"""
        try:
            if any(vitals.get(key) is None for key in self.VITAL_RANGES.keys()):
                return "unknown"
            
            errors = self.validate_vital_signs(vitals)
            if errors:
                self.logger.warning(f"Health state check found issues: {errors}")
                return "in danger"
            
            for vital, (min_val, max_val) in self.VITAL_RANGES.items():
                value = float(vitals[vital])
                if value < min_val or value > max_val:
                    return "in danger"
            return "ok"
        except Exception as e:
            self.logger.error(f"Error checking health state: {e}")
            return "unknown"

    def setup_routes(self):
        """Setup enhanced Flask routes with comprehensive error handling"""
        
        @self.app.route('/api/patients', methods=['GET'])
        def get_patients():
            try:
                with self.get_db_connection() as conn:
                    c = conn.cursor()
                    c.execute('SELECT * FROM patients ORDER BY patient_order')
                    
                    columns = [desc[0] for desc in c.description]
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
                        
                    self.logger.info(f"Retrieved {len(patients)} patients")
                    return jsonify(patients)
            except Exception as e:
                self.logger.error(f"Error getting patients: {e}")
                return jsonify({'error': str(e)}), 500

        @self.app.route('/api/patients/<int:patient_id>', methods=['GET', 'POST'])
        def update_patient(patient_id):
            try:
                if request.method == 'GET':
                    with self.get_db_connection() as conn:
                        c = conn.cursor()
                        c.execute('SELECT * FROM patients WHERE id = ?', (patient_id,))
                        row = c.fetchone()
                        
                        if not row:
                            return jsonify({'error': 'Patient not found'}), 404
                        
                        columns = [desc[0] for desc in c.description]
                        patient_dict = dict(zip(columns, row))
                        patient_dict['medicals'] = json.loads(patient_dict['medicals'])
                        patient_dict['is_dispensing'] = bool(patient_dict['is_dispensing'])
                        
                        return jsonify(patient_dict)

                else:  # POST request
                    if not request.is_json:
                        return jsonify({'error': 'Content-Type must be application/json'}), 400
                    
                    data = request.json
                    if data is None:
                        return jsonify({'error': 'Invalid JSON data'}), 400

                    # Handle dispensing state update
                    if len(data) == 1 and 'is_dispensing' in data:
                        with self.get_db_connection() as conn:
                            c = conn.cursor()
                            
                            # Verify patient exists
                            c.execute('SELECT id FROM patients WHERE id = ?', (patient_id,))
                            if not c.fetchone():
                                return jsonify({'error': f'Patient {patient_id} not found'}), 404
                            
                            # Update dispensing state
                            is_dispensing = 1 if data['is_dispensing'] else 0
                            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                            
                            c.execute('''UPDATE patients 
                                       SET is_dispensing = ?, timestamp = ? 
                                       WHERE id = ?''',
                                    (is_dispensing, timestamp, patient_id))
                            
                            self.logger.info(f"Updated dispensing state for patient {patient_id}: {is_dispensing}")
                            return jsonify({
                                'status': 'success',
                                'message': f'Updated dispensing state for patient {patient_id}',
                                'is_dispensing': bool(is_dispensing)
                            })

                    # Handle full update
                    required_fields = {
                        'heart_rate': float,
                        'spo2': float,
                        'temperature': float,
                        'glucose_level': float,
                        'medicals': list,
                        'medical_schedule': int,
                        'patient_order': int,
                        'is_dispensing': bool
                    }
                    
                    # Validate required fields
                    for field, field_type in required_fields.items():
                        if field not in data:
                            return jsonify({'error': f'Missing required field: {field}'}), 400
                        try:
                            if field != 'medicals':
                                data[field] = field_type(data[field])
                        except (ValueError, TypeError):
                            return jsonify({'error': f'Invalid type for field {field}'}), 400
                    
                    # Validate vital signs
                    vitals = {
                        'heart_rate': data['heart_rate'],
                        'spo2': data['spo2'],
                        'temperature': data['temperature'],
                        'glucose_level': data['glucose_level']
                    }
                    validation_errors = self.validate_vital_signs(vitals)
                    if validation_errors:
                        return jsonify({'error': 'Validation errors', 'details': validation_errors}), 400

                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    
                    with self.get_db_connection() as conn:
                        c = conn.cursor()
                        c.execute('''UPDATE patients SET 
                                    heart_rate = ?, spo2 = ?, temperature = ?, 
                                    glucose_level = ?, medicals = ?, 
                                    medical_schedule = ?, timestamp = ?,
                                    patient_order = ?, is_dispensing = ?
                                    WHERE id = ?''',
                                (data['heart_rate'],
                                 data['spo2'],
                                 data['temperature'],
                                 data['glucose_level'],
                                 json.dumps(data['medicals']),
                                 data['medical_schedule'],
                                 timestamp,
                                 data['patient_order'],
                                 1 if data['is_dispensing'] else 0,
                                 patient_id))
                        
                        if conn.total_changes == 0:
                            return jsonify({'error': 'No changes made to database'}), 400
                        
                        self.logger.info(f"Updated patient {patient_id} data successfully")
                        return jsonify({'status': 'success'})

            except sqlite3.IntegrityError as e:
                self.logger.error(f"Database integrity error: {e}")
                return jsonify({'error': 'Database integrity error', 'details': str(e)}), 400
            except Exception as e:
                self.logger.error(f"Error updating patient: {e}")
                return jsonify({'error': str(e)}), 500

        @self.app.route('/')
        def index():
            return send_from_directory('static', 'index.html')
            
                
        @self.app.route('/api/vitals/<int:patient_id>', methods=['POST'])
        def update_vitals(patient_id):
            try:
                data = request.json
                if not data:
                    return jsonify({'error': 'No data provided'}), 400

                with self.get_db_connection() as conn:
                    c = conn.cursor()
                    update_fields = []
                    params = []
                    
                    for field in ['heart_rate', 'spo2', 'temperature', 'glucose_level']:
                        if field in data:
                            update_fields.append(f"{field} = ?")
                            params.append(data[field])
                    
                    if not update_fields:
                        return jsonify({'error': 'No valid vital signs provided'}), 400
                    
                    params.append(datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
                    params.append(patient_id)
                    
                    query = f'''UPDATE patients SET 
                              {', '.join(update_fields)},
                              timestamp = ?
                              WHERE id = ?'''
                    
                    c.execute(query, params)
                    return jsonify({'status': 'success'})
                    
            except Exception as e:
                self.logger.error(f"Vitals update error: {e}")
                return jsonify({'error': str(e)}), 500

    def run(self):
        """Run the server with enhanced error handling"""
        try:
            self.logger.info("Starting Health Monitoring Server")
            self.app.run(debug=False, host='0.0.0.0', port=5000)
        except Exception as e:
            self.logger.error(f"Server failed to start: {e}")
            raise

if __name__ == '__main__':
    server = HealthMonitoringServer()
    server.run()