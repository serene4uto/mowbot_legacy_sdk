import rclpy
from rclpy.node import Node
import requests
from datetime import datetime

from typing import Literal

TOKEN_URL = "https://biz-dev.ktraas.kt.co.kr/keycloak/realms/openrm/protocol/openid-connect/token"
ROBOT_SERIAL = "RSN800-0000"
ROBOT_STATUS_URL = f"https://biz-dev.ktraas.kt.co.kr/kt-pa-open-api/api/robots/{ROBOT_SERIAL}/robot-status"
ERROR_REPORT_URL = f"https://biz-dev.ktraas.kt.co.kr/kt-pa-open-api/api/robots/{ROBOT_SERIAL}/error-report"
CLIENT_ID = "snsolutions"
CLIENT_SECRET = "QgvVztMwFggaC3Ds1nDCgIXec9P5Ca84"

class KtServerClient(Node):
    def __init__(self):
        super().__init__('kt_server_client')
        
        self.token_url = TOKEN_URL
        self.robot_serial = ROBOT_SERIAL
        self.robot_status_url = ROBOT_STATUS_URL
        self.error_report_url = ERROR_REPORT_URL
        self.client_id = CLIENT_ID
        self.client_secret = CLIENT_SECRET
        
        self.token = None
        self.location = {
            "x": 128.213,
            "y": 38.9438
        }
        self.heading = 273
        self.error_code = None
        
        
        self.token_timer_period = 1.0
        self.token_timer = self.create_timer(self.token_timer_period, self.token_timer_callback)
        self.report_timer_period = 1.0
        self.report_timer = self.create_timer(self.report_timer_period, self.report_timer_callback)
        
    
    def token_timer_callback(self):
        self.token_timer.cancel()
        self.token = None
        self._get_token()
        self.token_timer = self.create_timer(self.token_timer_period, self.token_timer_callback)
        
        
    def report_timer_callback(self):
        self._send_robot_status()
        # self._send_error_report("E0001", "Occurred")
        
        
        
    def _send_error_report(self, error_code: str, error_status: Literal["Occurred", "Fixed"]):
        if self.token is None:
            return
        
        try:
            response = requests.post(
                url=self.error_report_url,
                headers= {
                    "Authorization": f"Bearer {self.token}",
                    "Content-Type": "application/json"
                },
                json={
                    "robot_serial": self.robot_serial,
                    "create_time": datetime.now().strftime("%y%m%d%H%M%S%f")[:17],  # Current time in the required format
                    "error_code": error_code,
                    "data": {
                        "error_status": error_status
                    }
                }
            )
            
            if response.status_code == 200:
                self.get_logger().info("Error report sent successfully")
            else:
                self.get_logger().error(f"Error sending error report: {response.status_code}")
            
        except Exception as e:
            self.get_logger().error(f"Error sending error report: {e}")


    def _send_robot_status(self):
        if self.token is None:
            return
        try:
            response = requests.post(
                self.robot_status_url,  # Fixed URL with robot serial
                headers= {
                    "Authorization": f"Bearer {self.token}",
                    "Content-Type": "application/json"
                },
                json={
                    "robot_serial": self.robot_serial,
                    "create_time": datetime.now().strftime("%y%m%d%H%M%S%f")[:17],  # Current time in the required format
                    "x": self.location["x"],
                    "y": self.location["y"],
                    "battery": 87.43,
                    "drive_status": 1,
                    "speed": 14.8723,
                    "heading": self.heading,
                    "charge": False,
                    "charge_type": "None",
                    "is_indoor": False,
                    "coord_code": "WGS84",
                    "service_mode": "mowing",
                    "service": {
                        "mowing": {
                            "fuel": 12.1,
                            "autonomous_status": "Active",
                            "cutter": {
                                "type": "rotary",
                                "width": 15
                            },
                            "lift": {
                                "status": "down",
                                "height": 5
                            },
                            "rtk": {
                                "status": "normal",
                                "error_range": 3
                            },
                            "rollangle": 30,
                            "rollover_status": "normal"
                        }
                    },
                    "task": {
                        "task_id": f"{self.robot_serial}-{datetime.now().strftime('%y%m%d%H%M%S%f')[:17]}0101",
                        "task_code": "mowing",
                        "task_status": "OnProgress"
                    }
                }
            )
            
            if response.status_code == 200:
                self.get_logger().info("Report sent successfully")
            else:
                self.get_logger().error(f"Error sending report: {response.status_code}")
        
        except Exception as e:  
            self.get_logger().error(f"Error sending report: {e}")
    
    
    def _get_token(self):
        try:
            token_response = requests.post(
            url=self.token_url, 
                data={
                    "grant_type": "client_credentials",
                    "client_id": self.client_id,
                    "client_secret": self.client_secret
                }, 
                headers= {
                    "Content-Type": "application/x-www-form-urlencoded"
                }
            )
            token_response.raise_for_status()  # Raise an error for bad status codes
            self.token = token_response.json().get("access_token")
            token_expire = token_response.json().get("expires_in")
            self.token_timer_period = token_expire - 10
            self.get_logger().info(f"Token: {self.token}")
            self.get_logger().info(f"Token expires in: {token_expire} seconds")
        
        except Exception as e:
            self.get_logger().error(f"Error getting token: {e}")
            self.token_timer_period = 1 # Retry in 1 second
    
    
    
    

def main(args=None):
    rclpy.init(args=args)
    kt_server_client = KtServerClient()
    rclpy.spin(kt_server_client)
    rclpy.shutdown()
    
