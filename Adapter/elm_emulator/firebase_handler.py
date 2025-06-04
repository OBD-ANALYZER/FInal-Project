import firebase_admin
from firebase_admin import credentials, db
import json
import os
import logging
import time

class FirebaseHandler:
    def __init__(self):
        try:
            # Initialize Firebase with service account
            cred = credentials.Certificate("google-services.json")
            firebase_admin.initialize_app(cred, {
                'databaseURL': 'https://obd-app-default-rtdb.firebaseio.com'
            })
            self.db_ref = db.reference('vehicle_data')
            self.last_update = time.time()
            self.update_interval = 1.0  # Update every second
            logging.info("Firebase initialized successfully")
        except Exception as e:
            logging.error(f"Failed to initialize Firebase: {e}")
            self.db_ref = None

    def save_data(self, data):
        """Save vehicle data to Firebase."""
        try:
            if not self.db_ref:
                return
                
            current_time = time.time()
            if current_time - self.last_update < self.update_interval:
                return
                
            # Add timestamp
            data_with_timestamp = {
                'timestamp': int(current_time * 1000),  # milliseconds
                'data': data
            }
            
            # Save to Firebase
            self.db_ref.push().set(data_with_timestamp)
            self.last_update = current_time
            
        except Exception as e:
            logging.error(f"Failed to save data to Firebase: {e}") 