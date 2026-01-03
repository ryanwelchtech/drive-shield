const SENSOR_TYPES = ["lidar", "camera", "radar", "ultrasonic", "gps"];

function updateSensorStatus() {
  const sensorStatus = {};
  
  for (const sensor of SENSOR_TYPES) {
    if (sensor === "gps") {
      sensorStatus[sensor] = {
        status: "active",
        accuracy: parseFloat((0.5 + Math.random() * 2).toFixed(1)),
        satellites: 8 + Math.floor(Math.random() * 6),
        hdop: parseFloat((0.3 + Math.random() * 1).toFixed(1)),
      };
    } else {
      const baseConf = { lidar: 95, camera: 90, radar: 94, ultrasonic: 97 };
      const variance = { lidar: 5, camera: 10, radar: 6, ultrasonic: 3 };
      sensorStatus[sensor] = {
        status: "active",
        confidence: parseFloat((baseConf[sensor] + Math.random() * variance[sensor]).toFixed(1)),
        fps: sensor === "lidar" ? 10 + Math.floor(Math.random() * 5) : sensor === "camera" ? 25 + Math.floor(Math.random() * 5) : 20,
      };
      if (sensor === "lidar") sensorStatus[sensor].pointsPerScan = 64000;
      if (sensor === "camera") sensorStatus[sensor].resolution = "1920x1080";
      if (sensor === "radar") sensorStatus[sensor].range = 200;
      if (sensor === "ultrasonic") sensorStatus[sensor].range = 5;
    }
  }
  
  return sensorStatus;
}

export default function handler(req, res) {
  res.status(200).json(updateSensorStatus());
}
