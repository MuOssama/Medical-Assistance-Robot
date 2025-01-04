#!/usr/bin/env python3
import sqlite3
import json
from datetime import datetime
import rospy
from std_msgs.msg import Int32MultiArray, Bool, MultiArrayDimension
from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS
from threading import Thread

class FlaskROSServer:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('flask_server', anonymous=True)
        
        # Initialize Flask
        self.app = Flask(__name__, static_folder='static')
        CORS(self.app)
        
        # ROS Publishers
        self.medication_pub = rospy.Publisher('medication_array', Int32MultiArray, queue_size=10)
        self.danger_pub = rospy.Publisher('patient_danger', Bool, queue_size=10)
        
        # Setup Flask routes
        self.setup_routes()
        
        # Initialize database
        self.init_db()

    def init_db(self):
        conn = sqlite3.connect('patients.db')
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
                      pass_medicals BOOLEAN DEFAULT 0)
                  ''')
        
        # Initialize with fixed orders
        c.execute('''INSERT OR IGNORE INTO patients VALUES (1, 1, NULL, NULL, NULL, '[]', NULL, NULL, 1, 0)''')
        c.execute('''INSERT OR IGNORE INTO patients VALUES (2, 2, NULL, NULL, NULL, '[]', NULL, NULL, 2, 0)''')
        c.execute('''INSERT OR IGNORE INTO patients VALUES (3, 3, NULL, NULL, NULL, '[]', NULL, NULL, 3, 0)''')
        conn.commit()
        conn.close()

    def check_health_state(self, hr, spo2, temp, glucose):
        if any(v is None for v in [hr, spo2, temp, glucose]):
            return "unknown"
        return "in danger" if (
            hr < 60 or hr > 100 or
            spo2 < 95 or
            temp < 36.5 or temp > 37.5 or
            glucose < 70 or glucose > 140
        ) else "ok"

    def publish_medication_array(self, patient_id, medicals):
        try:
            med_array = json.loads(medicals)
            if not isinstance(med_array, list) or len(med_array) != 5:
                rospy.logwarn(f"Invalid medicals format for patient {patient_id}")
                return

            msg = Int32MultiArray()
            msg.layout.dim = [MultiArrayDimension()]
            msg.layout.dim[0].size = 5
            msg.layout.dim[0].stride = 1
            msg.layout.dim[0].label = f"patient_{patient_id}"
            msg.data = [int(bool(x)) for x in med_array]
            self.medication_pub.publish(msg)
            rospy.loginfo(f"Published medications for patient {patient_id}: {msg.data}")
            
            # Update pass_medicals status
            conn = sqlite3.connect('patients.db')
            c = conn.cursor()
            c.execute('UPDATE patients SET pass_medicals = 1 WHERE id = ?', (patient_id,))
            conn.commit()
            conn.close()
            
        except Exception as e:
            rospy.logerr(f"Error publishing medication array: {str(e)}")

    def publish_danger_state(self, is_in_danger):
        try:
            msg = Bool()
            msg.data = is_in_danger
            self.danger_pub.publish(msg)
            if is_in_danger:
                rospy.loginfo("Published danger state")
        except Exception as e:
            rospy.logerr(f"Error publishing danger state: {str(e)}")

    def setup_routes(self):
        @self.app.route('/api/patients', methods=['GET'])
        def get_patients():
            conn = sqlite3.connect('patients.db')
            c = conn.cursor()
            c.execute('SELECT * FROM patients ORDER BY patient_order')
            columns = ['id', 'heart_rate', 'spo2', 'temperature', 'glucose_level', 
                      'medicals', 'medical_schedule', 'timestamp', 'patient_order', 'pass_medicals']
            patients = []
            for row in c.fetchall():
                patient_dict = dict(zip(columns, row))
                patient_dict['medicals'] = json.loads(patient_dict['medicals'])
                patient_dict['health_state'] = self.check_health_state(
                    patient_dict['heart_rate'],
                    patient_dict['spo2'],
                    patient_dict['temperature'],
                    patient_dict['glucose_level']
                )
                patients.append(patient_dict)
            conn.close()
            return jsonify(patients)

        @self.app.route('/api/patients/<int:patient_id>', methods=['POST'])
        def update_patient(patient_id):
            data = request.json
            required_fields = ['heart_rate', 'spo2', 'temperature', 'glucose_level', 
                             'medicals', 'medical_schedule', 'patient_order']
            
            if not all(field in data for field in required_fields):
                return jsonify({'error': 'Missing required fields'}), 400
            
            try:
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                conn = sqlite3.connect('patients.db')
                c = conn.cursor()
                
                # Reset pass_medicals when updating patient data
                c.execute('''UPDATE patients SET 
                            heart_rate = ?, spo2 = ?, temperature = ?, 
                            glucose_level = ?, medicals = ?, 
                            medical_schedule = ?, timestamp = ?,
                            patient_order = ?, pass_medicals = 0
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
                conn.commit()
                
                self.publish_medication_array(patient_id, json.dumps(data['medicals']))
                
                health_state = self.check_health_state(
                    float(data['heart_rate']),
                    float(data['spo2']),
                    float(data['temperature']),
                    float(data['glucose_level'])
                )
                self.publish_danger_state(health_state == "in danger")
                
                conn.close()
                return jsonify({'status': 'success'}), 200
                
            except Exception as e:
                return jsonify({'error': str(e)}), 400

        @self.app.route('/')
        def index():
            return send_from_directory('static', 'index.html')

    def run(self):
        flask_thread = Thread(target=lambda: self.app.run(debug=False, host='0.0.0.0', port=5000))
        flask_thread.daemon = True
        flask_thread.start()
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")

if __name__ == '__main__':
    server = FlaskROSServer()
    server.run()