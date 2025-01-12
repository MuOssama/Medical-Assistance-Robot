#!/usr/bin/env python
import sqlite3
import json
from datetime import datetime
import rospy
from std_msgs.msg import Int32MultiArray, Bool, MultiArrayDimension, Int32
from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
from threading import Thread, Lock
from contextlib import contextmanager

class HealthMonitoringServer(object):  # Inherit from object for Python 2
    def __init__(self):
        # ROS initialization
        rospy.init_node('flask_server', anonymous=True)
        self.medication_pub = rospy.Publisher('medication_array', Int32MultiArray, queue_size=10)
        self.danger_pub = rospy.Publisher('patient_danger', Bool, queue_size=10)
        self.current_patient = 0
        self.db_lock = Lock()
        
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
        self.init_db()
        
        # Setup ROS subscriber
        rospy.Subscriber('current_patient', Int32, self.patient_callback)

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
                         patient_order INTEGER)
                     ''')
            
            default_medicals = json.dumps([0, 0, 0, 0, 0])
            for patient_id in xrange(1, 4):  # xrange for Python 2
                c.execute('''INSERT OR IGNORE INTO patients 
                           VALUES (?, ?, NULL, NULL, NULL, ?, NULL, NULL, ?)''',
                        (patient_id, patient_id, default_medicals, patient_id))

    def patient_callback(self, msg):
        """Handle updates to current patient selection"""
        self.current_patient = msg.data
        self.publish_patient_medications(msg.data)

    def check_health_state(self, vitals):
        """Determine patient health state based on vital signs"""
        if any(vitals[key] is None for key in self.VITAL_RANGES.iterkeys()):  # iterkeys for Python 2
            return "unknown"
            
        for vital, (min_val, max_val) in self.VITAL_RANGES.iteritems():  # iteritems for Python 2
            if vitals[vital] < min_val or vitals[vital] > max_val:
                return "in danger"
        return "ok"

    def publish_patient_medications(self, patient_id):
        """Publish medication array for current patient"""
        try:
            with self.get_db_connection() as conn:
                c = conn.cursor()
                c.execute('SELECT medicals FROM patients WHERE id = ?', (patient_id,))
                result = c.fetchone()
            
            if result:
                medicals = json.loads(result[0])
                msg = Int32MultiArray()
                msg.layout.dim = [MultiArrayDimension()]
                msg.layout.dim[0].size = 5
                msg.layout.dim[0].stride = 1
                msg.layout.dim[0].label = "patient_{0}".format(patient_id)  # .format for Python 2
                msg.data = [int(bool(x)) for x in medicals]
                self.medication_pub.publish(msg)
                rospy.loginfo("Published medications for patient {0}: {1}".format(
                    patient_id, str(msg.data)))  # .format for Python 2
        
        except Exception as e:
            rospy.logerr("Error publishing medication array: {0}".format(str(e)))

    def publish_danger_state(self, is_in_danger):
        """Publish patient danger state"""
        try:
            msg = Bool()
            msg.data = is_in_danger
            self.danger_pub.publish(msg)
            if is_in_danger:
                rospy.loginfo("Published danger state")
        except Exception as e:
            rospy.logerr("Error publishing danger state: {0}".format(str(e)))

    def setup_routes(self):
        """Setup Flask routes with proper error handling"""
        @self.app.route('/api/patients', methods=['GET'])
        def get_patients():
            try:
                with self.get_db_connection() as conn:
                    c = conn.cursor()
                    c.execute('SELECT * FROM patients ORDER BY patient_order')
                    columns = ['id', 'heart_rate', 'spo2', 'temperature', 'glucose_level',
                             'medicals', 'medical_schedule', 'timestamp', 'patient_order']
                    patients = []
                    for row in c.fetchall():
                        patient_dict = dict(zip(columns, row))
                        patient_dict['medicals'] = json.loads(patient_dict['medicals'])
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
                                 'medicals', 'medical_schedule', 'patient_order']
                
                if not all(field in data for field in required_fields):
                    return jsonify({'error': 'Missing required fields'}), 400
                
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                
                with self.get_db_connection() as conn:
                    c = conn.cursor()
                    c.execute('''UPDATE patients SET 
                                heart_rate = ?, spo2 = ?, temperature = ?, 
                                glucose_level = ?, medicals = ?, 
                                medical_schedule = ?, timestamp = ?,
                                patient_order = ?
                                WHERE id = ?''',
                             (float(data['heart_rate']),
                              float(data['spo2']),
                              float(data['temperature']),
                              float(data['glucose_level']),
                              json.dumps(data['medicals']),
                              int(data['medical_schedule']),
                              timestamp,
                              int(data['patient_order']),
                              patient_id))
                
                if patient_id == self.current_patient:
                    self.publish_patient_medications(patient_id)
                
                vitals = {
                    'heart_rate': float(data['heart_rate']),
                    'spo2': float(data['spo2']),
                    'temperature': float(data['temperature']),
                    'glucose_level': float(data['glucose_level'])
                }
                health_state = self.check_health_state(vitals)
                self.publish_danger_state(health_state == "in danger")
                
                return jsonify({'status': 'success'}), 200
                
            except ValueError as e:
                return jsonify({'error': 'Invalid data format: {0}'.format(str(e))}), 400
            except Exception as e:
                return jsonify({'error': str(e)}), 500

        @self.app.route('/')
        def index():
            return send_from_directory('static', 'index.html')

    def run(self):
        """Run the server with proper threading"""
        server_thread = Thread(target=lambda: self.app.run(debug=False, host='0.0.0.0', port=5000))
        server_thread.daemon = True
        server_thread.start()
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")

if __name__ == '__main__':
    server = HealthMonitoringServer()
    server.run()