import socket
import json
import threading
import traceback
import json
from enum import Enum


def to_json_dict(obj):
    if obj is None: return obj
    if isinstance(obj, str): return obj
    if isinstance(obj, int): return obj
    if isinstance(obj, bool): return obj
    if isinstance(obj, float): return obj
    if isinstance(obj, Enum): return obj.value
    if isinstance(obj, tuple):
        return to_json_dict(list(obj))
    if isinstance(obj, set):
        return to_json_dict(list(obj))

    if isinstance(obj, list):
        result = []
        for e in obj:
            result.append(to_json_dict(e))
        return result
    
    if isinstance(obj, dict):
        result = {}
        for key in obj:
            if isinstance(key, str) or isinstance(key, int) or isinstance(key, bool):
                result[key] = to_json_dict(obj[key])
            else:
                result[str(key)] = to_json_dict(obj[key])
        return result
    
    d = {}
    for k, v in obj.__dict__.items():
        if not k.startswith("_"):
            d[k] = v
    return to_json_dict(d)

class JSON:
    @classmethod
    def stringify(self, obj):
        return json.dumps(to_json_dict(obj))
    
class MapVisualizer:
    def __init__(self, port=4321) -> None:
        self.port = port
        self.host = "127.0.0.1"
        self.connection = None
        self.start()

        self.previousMessage = {}
        self.lastRobotUpdate = 0

    def start(self):
        self.thread = threading.Thread(target=self.accept_connections, args=(), daemon=True)
        self.thread.start()

    def accept_connections(self):
        print("Waiting for connections...")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((self.host, self.port))
            s.listen()
            conn, addr = s.accept()
            print(f"Connected by {addr}")
            self.connection = conn

    def send_map(self, map):
        if self.connection == None: return
        try:
            type = 0
            data = JSON.stringify(map).encode("utf8")
            if data == self.previousMessage.get(type): return
            self.previousMessage[type] = data
            type = type.to_bytes(1, byteorder="little")
            count = len(data).to_bytes(4, "little")
            self.connection.sendall(type + count + data)
        except Exception:
            print("Connection lost!")
            print(traceback.format_exc())
            self.connection.close()
            self.connection = None
            self.start()

    def send_robot(self, robot):        
        if self.connection == None: return
        try:
            if robot.step_counter - self.lastRobotUpdate < 4:
                return
            self.lastRobotUpdate = robot.step_counter
            type = 1
            data = {
                "position": robot.position,
                "rotation": robot.rotation,
                "current_area": robot.current_area,
                "targetPoint": robot.targetPoint,
                "initial_position": robot.posicion_inicial
            }
            data = JSON.stringify(data).encode("utf8")
            if data == self.previousMessage.get(type): return
            self.previousMessage[type] = data
            type = type.to_bytes(1,byteorder="little")
            count = len(data).to_bytes(4, "little")
            self.connection.sendall(type + count + data)
        except Exception:
            print("Connection lost!")
            print(traceback.format_exc())
            self.connection.close()
            self.connection = None
            self.start()

    
    def send_minitiles(self, robot):
        if self.connection == None: return
        try:
            type = 2
            data = robot.navigator.minitiles
            data = JSON.stringify(data).encode("utf8")
            if data == self.previousMessage.get(type): return
            self.previousMessage[type] = data
            type = type.to_bytes(1, byteorder="little")
            count = len(data).to_bytes(4, "little")
            self.connection.sendall(type + count + data)
        except Exception:
            print("Connection lost!")
            print(traceback.format_exc())
            self.connection.close()
            self.connection = None
            self.start()