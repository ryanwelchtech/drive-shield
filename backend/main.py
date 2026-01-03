"""
DriveShield Backend API

FastAPI server providing:
- REST endpoints for sensor data and detections
- WebSocket streaming for real-time AV monitoring
- Threat detection and analysis endpoints

Run with: uvicorn main:app --reload --port 8080
"""

import asyncio
import random
import time
import math
from typing import Optional, List
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# ============================================
# MODELS
# ============================================

class Detection(BaseModel):
    id: int
    type: str
    threat: Optional[str]
    confidence: float
    distance: float
    bearing: int
    velocity: float
    timestamp: int

class ThreatAnalysisRequest(BaseModel):
    detection_id: int
    image_patch: Optional[str] = None

class SensorConfig(BaseModel):
    sensor: str
    enabled: bool

# ============================================
# CONSTANTS
# ============================================

OBJECT_TYPES = ["vehicle", "pedestrian", "cyclist", "sign", "obstacle", "animal"]
THREAT_TYPES = ["adversarial_patch", "phantom_object", "spoofing", "gps_manipulation"]
SENSOR_TYPES = ["lidar", "camera", "radar", "ultrasonic", "gps"]

# ============================================
# IN-MEMORY STATE
# ============================================

sensor_status = {
    "lidar": {"status": "active", "confidence": 98.2, "fps": 10, "points_per_scan": 64000},
    "camera": {"status": "active", "confidence": 94.5, "fps": 30, "resolution": "1920x1080"},
    "radar": {"status": "active", "confidence": 97.1, "range": 200, "update_rate": 20},
    "ultrasonic": {"status": "active", "confidence": 99.0, "range": 5, "sensors": 12},
    "gps": {"status": "active", "accuracy": 1.2, "satellites": 12, "hdop": 0.8},
}

detection_history: List[dict] = []
active_connections: list[WebSocket] = []

# ============================================
# MOCK DATA GENERATORS
# ============================================

def generate_detection() -> dict:
    """Generate a realistic object detection."""
    obj_type = random.choice(OBJECT_TYPES)
    has_threat = random.random() > 0.88
    threat = random.choice(THREAT_TYPES) if has_threat else None

    # Realistic velocity based on object type
    velocity_ranges = {
        "vehicle": (0, 120),
        "pedestrian": (0, 7),
        "cyclist": (0, 30),
        "sign": (0, 0),
        "obstacle": (0, 0),
        "animal": (0, 15),
    }
    min_v, max_v = velocity_ranges.get(obj_type, (0, 50))

    detection = {
        "id": int(time.time() * 1000) + random.randint(0, 999),
        "type": obj_type,
        "threat": threat,
        "confidence": 70 + random.random() * 30,
        "distance": round(5 + random.random() * 95, 1),
        "bearing": random.randint(0, 359),
        "velocity": round(min_v + random.random() * (max_v - min_v), 1),
        "bbox": {
            "x": random.randint(100, 1820),
            "y": random.randint(100, 980),
            "width": random.randint(50, 200),
            "height": random.randint(50, 300),
        },
        "timestamp": int(time.time() * 1000),
    }

    detection_history.append(detection)
    if len(detection_history) > 100:
        detection_history.pop(0)

    return detection

def generate_lidar_points(count: int = 500) -> List[dict]:
    """Generate LIDAR point cloud data."""
    points = []
    for _ in range(count):
        angle = random.random() * math.pi * 2
        distance = 20 + random.random() * 130
        z = (random.random() - 0.5) * 10

        # Convert to cartesian
        x = math.cos(angle) * distance
        y = math.sin(angle) * distance

        points.append({
            "x": round(x, 2),
            "y": round(y, 2),
            "z": round(z, 2),
            "intensity": round(random.random(), 2),
            "classification": "threat" if random.random() > 0.995 else "normal",
        })

    return points

def update_sensor_status():
    """Simulate sensor status fluctuations."""
    for sensor in sensor_status:
        if sensor == "gps":
            sensor_status[sensor]["accuracy"] = round(0.5 + random.random() * 2, 1)
            sensor_status[sensor]["satellites"] = 8 + random.randint(0, 6)
        else:
            base_conf = {"lidar": 95, "camera": 90, "radar": 94, "ultrasonic": 97}
            variance = {"lidar": 5, "camera": 10, "radar": 6, "ultrasonic": 3}
            sensor_status[sensor]["confidence"] = round(
                base_conf.get(sensor, 90) + random.random() * variance.get(sensor, 5), 1
            )

# ============================================
# LIFESPAN & APP SETUP
# ============================================

@asynccontextmanager
async def lifespan(app: FastAPI):
    print("DriveShield API starting...")
    yield
    print("DriveShield API shutting down...")

app = FastAPI(
    title="DriveShield API",
    description="Autonomous Vehicle Threat Detection Backend",
    version="1.0.0",
    lifespan=lifespan,
)

# CORS for frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173", "http://localhost:3000", "https://ryanwelchtech.github.io"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ============================================
# REST ENDPOINTS
# ============================================

@app.get("/")
async def root():
    return {"status": "online", "service": "DriveShield API", "version": "1.0.0"}

@app.get("/api/health")
async def health():
    return {"status": "healthy", "timestamp": int(time.time() * 1000)}

@app.get("/api/sensors/status")
async def get_sensor_status():
    """Get current status of all sensors."""
    update_sensor_status()
    return sensor_status

@app.post("/api/sensors/configure")
async def configure_sensor(config: SensorConfig):
    """Enable/disable a sensor."""
    if config.sensor in sensor_status:
        sensor_status[config.sensor]["status"] = "active" if config.enabled else "inactive"
        return {"success": True, "sensor": sensor_status[config.sensor]}
    return {"success": False, "error": "Sensor not found"}

@app.get("/api/detections")
async def get_detections():
    """Get recent object detections."""
    # Generate some new detections
    new_detections = [generate_detection() for _ in range(random.randint(0, 3))]
    return {"detections": new_detections, "total": len(detection_history)}

@app.get("/api/detections/history")
async def get_detection_history(limit: int = 20):
    """Get detection history."""
    return {"detections": detection_history[-limit:], "total": len(detection_history)}

@app.get("/api/lidar/points")
async def get_lidar_points(count: int = 500):
    """Get LIDAR point cloud data."""
    return {
        "points": generate_lidar_points(count),
        "timestamp": int(time.time() * 1000),
        "sensor": sensor_status["lidar"],
    }

@app.post("/api/threat/analyze")
async def analyze_threat(request: ThreatAnalysisRequest):
    """Analyze a detection for adversarial threats."""
    is_adversarial = random.random() > 0.85

    return {
        "detection_id": request.detection_id,
        "is_adversarial": is_adversarial,
        "threat_type": random.choice(THREAT_TYPES) if is_adversarial else None,
        "confidence": round(70 + random.random() * 30, 1),
        "explanation": "Anomalous pattern detected in perception pipeline" if is_adversarial else None,
        "recommended_action": "Reduce speed and increase sensor redundancy" if is_adversarial else "Continue monitoring",
        "timestamp": int(time.time() * 1000),
    }

@app.get("/api/threats/summary")
async def get_threat_summary():
    """Get summary of detected threats."""
    threats = [d for d in detection_history if d.get("threat")]
    threat_counts = {}
    for t in threats:
        threat_type = t["threat"]
        threat_counts[threat_type] = threat_counts.get(threat_type, 0) + 1

    return {
        "total_threats": len(threats),
        "by_type": threat_counts,
        "threat_level": "CRITICAL" if any(t["threat"] == "adversarial_patch" for t in threats[-10:]) else
                       "HIGH" if len([t for t in threats[-10:] if t["threat"]]) > 2 else "LOW",
        "timestamp": int(time.time() * 1000),
    }

# ============================================
# WEBSOCKET STREAMING
# ============================================

@app.websocket("/ws/sensors")
async def websocket_sensors(websocket: WebSocket):
    """Stream real-time sensor data."""
    await websocket.accept()
    active_connections.append(websocket)

    try:
        is_running = True

        async def send_updates():
            while is_running:
                # Send sensor status every second
                update_sensor_status()
                await websocket.send_json({
                    "type": "status",
                    "data": sensor_status,
                })

                await asyncio.sleep(1)

        async def send_detections():
            while is_running:
                # Generate detections periodically
                if random.random() > 0.6:
                    detection = generate_detection()
                    await websocket.send_json({
                        "type": "detection",
                        "data": detection,
                    })

                await asyncio.sleep(0.5)

        async def send_confidence():
            while is_running:
                # Send confidence updates frequently
                confidence = 85 + random.random() * 15
                await websocket.send_json({
                    "type": "confidence",
                    "data": {
                        "value": confidence,
                        "timestamp": int(time.time() * 1000),
                    }
                })

                await asyncio.sleep(0.1)

        # Start all update tasks
        tasks = [
            asyncio.create_task(send_updates()),
            asyncio.create_task(send_detections()),
            asyncio.create_task(send_confidence()),
        ]

        # Listen for client messages
        while True:
            data = await websocket.receive_json()

            if data.get("type") == "pause":
                is_running = False
                for task in tasks:
                    task.cancel()
            elif data.get("type") == "resume":
                is_running = True
                tasks = [
                    asyncio.create_task(send_updates()),
                    asyncio.create_task(send_detections()),
                    asyncio.create_task(send_confidence()),
                ]

    except WebSocketDisconnect:
        active_connections.remove(websocket)
    except Exception as e:
        print(f"WebSocket error: {e}")
        if websocket in active_connections:
            active_connections.remove(websocket)

@app.websocket("/ws/lidar")
async def websocket_lidar(websocket: WebSocket):
    """Stream LIDAR point cloud data in real-time."""
    await websocket.accept()

    try:
        while True:
            points = generate_lidar_points(200)  # Reduced for streaming
            await websocket.send_json({
                "type": "lidar_frame",
                "data": {
                    "points": points,
                    "timestamp": int(time.time() * 1000),
                }
            })
            await asyncio.sleep(0.1)  # 10 FPS for LIDAR

    except WebSocketDisconnect:
        pass

# ============================================
# PRODUCTION INTEGRATION EXAMPLES
# ============================================

"""
To integrate real sensor data, replace the mock functions:

1. ROS2 Integration:

   import rclpy
   from sensor_msgs.msg import PointCloud2, Image
   from std_msgs.msg import Header

   class SensorBridge:
       def __init__(self):
           rclpy.init()
           self.node = rclpy.create_node('driveshield_bridge')
           self.lidar_sub = self.node.create_subscription(
               PointCloud2, '/velodyne_points', self.lidar_callback, 10
           )

       def lidar_callback(self, msg):
           # Process point cloud
           points = pointcloud2_to_array(msg)
           # Send to WebSocket clients

2. CARLA Simulator:

   import carla

   client = carla.Client('localhost', 2000)
   world = client.get_world()

   # Attach sensors to vehicle
   lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
   lidar = world.spawn_actor(lidar_bp, transform, attach_to=vehicle)

   def lidar_callback(data):
       points = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)
       # Process and stream

3. YOLO Object Detection:

   from ultralytics import YOLO

   model = YOLO('yolov8x.pt')

   def detect_objects(frame):
       results = model(frame)
       detections = []
       for box in results[0].boxes:
           detections.append({
               'type': results[0].names[int(box.cls)],
               'confidence': float(box.conf),
               'bbox': box.xyxy.tolist()[0]
           })
       return detections

4. Adversarial Detection:

   from art.estimators.classification import PyTorchClassifier
   from art.defences.detector.evasion import BinaryInputDetector

   detector = BinaryInputDetector(classifier)

   def check_adversarial(image):
       is_adversarial, confidence = detector.detect(image)
       return is_adversarial, confidence
"""

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8080)
