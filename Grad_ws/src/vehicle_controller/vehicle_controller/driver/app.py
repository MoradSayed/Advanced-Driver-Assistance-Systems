from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..manager import Manager

from flask import Flask, request
from flask_socketio import SocketIO, emit
import threading, json

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

latest_data = {
    "steer": 0.0,
    "vel": 0.0,
    "brake": 0.0,
    "acc": 0,
}
methods = None
class APKController:
    def __init__(self, manager:"Manager"):
        global methods
        methods = (manager.steer_manager, manager.speed_manager, manager.brake_manager, lambda _: None)
        threading.Thread(
            target=lambda: socketio.run(app, host='0.0.0.0', port=5555), 
            daemon=True
        ).start()

    @socketio.on('connect')
    def handle_connect():
        # print(f"Client connected: {request.sid}")
        emit('connection_response', {'status': 'connected'})

    @socketio.on('disconnect')
    def handle_disconnect():
        # print(f"Client disconnected: {request.sid}")
        pass

    @socketio.on('steering_data')
    def handle_steering_data(data):
        data:dict = json.loads(data)
        
        for index, (old, new) in enumerate(zip(latest_data.values(), data.values())):
            if old != new:
                methods[index](float(new))
                break

        latest_data.update({
            "steer": data['steer'],
            "vel":   data['vel'],
            "brake": data['brake'],
            "acc":   data['acc'],
        })
