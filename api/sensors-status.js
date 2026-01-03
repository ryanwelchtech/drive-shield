const SENSOR_TYPES = ["lidar", "camera", "radar", "ultrasonic", "gps"];

let sensorState = {
  lidar: { confidence: 97.2, fps: 10, pointsPerScan: 64000, range: 200, verticalFOV: 40 },
  camera: { confidence: 94.8, fps: 30, resolution: "1920x1080", hdr: true, frameDelay: 33 },
  radar: { confidence: 96.5, range: 250, updateRate: 20, doppler: true, falseAlarmRate: 0.001 },
  ultrasonic: { confidence: 98.1, range: 8, sensors: 12, refreshRate: 50 },
  gps: { accuracy: 1.2, satellites: 12, hdop: 0.8, fix: "RTK", latitude: 37.7749, longitude: -122.4194 },
  fusion: { confidence: 95.5, latency: 45, tracking: 8 },
};

function updateSensorStatus() {
  const time = Date.now() / 1000;

  sensorState.lidar.confidence = 95 + Math.sin(time * 0.5) * 3 + Math.random() * 1.5;
  sensorState.lidar.fps = 10 + Math.round(Math.sin(time * 0.3) * 2);
  sensorState.lidar.pointsPerScan = 64000 + Math.round(Math.random() * 1000);

  sensorState.camera.confidence = 92 + Math.sin(time * 0.4) * 5 + Math.random() * 2;
  sensorState.camera.fps = 28 + Math.round(Math.sin(time * 0.2) * 3);
  sensorState.camera.frameDelay = 30 + Math.round(Math.random() * 8);

  sensorState.radar.confidence = 94 + Math.sin(time * 0.6) * 4 + Math.random() * 2;
  sensorState.radar.range = 245 + Math.round(Math.sin(time * 0.1) * 10);
  sensorState.radar.falseAlarmRate = 0.0005 + Math.random() * 0.002;

  sensorState.ultrasonic.confidence = 96 + Math.sin(time * 0.8) * 2 + Math.random() * 1;
  sensorState.ultrasonic.range = 7.5 + Math.round(Math.random() * 1);

  sensorState.gps.accuracy = 0.8 + Math.random() * 1;
  sensorState.gps.satellites = 10 + Math.round(Math.random() * 4);
  sensorState.gps.hdop = 0.5 + Math.random() * 0.5;
  sensorState.gps.latitude += (Math.random() - 0.5) * 0.00001;
  sensorState.gps.longitude += (Math.random() - 0.5) * 0.00001;

  sensorState.fusion.confidence = 93 + Math.sin(time * 0.3) * 4 + Math.random() * 2;
  sensorState.fusion.latency = 40 + Math.round(Math.random() * 15);

  for (const sensor of Object.keys(sensorState)) {
    if (sensor !== "fusion") {
      sensorState[sensor].status = sensorState[sensor].confidence > 85 ? "active" : "degraded";
    }
  }

  return {
    ...sensorState,
    timestamp: Date.now(),
    egoSpeed: 65 + Math.round(Math.sin(time * 0.2) * 10),
    environment: {
      weather: "clear",
      visibility: "good",
      roadCondition: "dry",
      ambientLight: time > 6 && time < 18 ? "day" : "night",
    },
  };
}

export default function handler(req, res) {
  res.status(200).json(updateSensorStatus());
}
