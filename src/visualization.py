import socket
import json
import threading
import traceback
import json


def to_json_dict(obj):
    if obj is None: return obj
    if isinstance(obj, str): return obj
    if isinstance(obj, int): return obj
    if isinstance(obj, bool): return obj
    if isinstance(obj, float): return obj

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
    
    return to_json_dict(obj.__dict__)

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
            data = JSON.stringify(map).encode("utf8")
            self.connection.sendall(data)
        except Exception:
            print("Connection lost!")
            print(traceback.format_exc())
            self.connection.close()
            self.connection = None
            self.start()