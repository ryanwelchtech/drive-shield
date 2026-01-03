let lidarState = {
  egoLane: 0,
  egoY: 0,
  detectedObjects: [
    { x: -3, y: 45, width: 2, height: 4.5, type: "vehicle" },
    { x: 3, y: 80, width: 1.8, height: 4.2, type: "vehicle" },
    { x: -8, y: 25, width: 0.6, height: 1.7, type: "pedestrian" },
  ],
  time: 0,
};

function generateLidarPoints(count = 500) {
  lidarState.time += 0.016;
  lidarState.egoSpeed = 65 + Math.sin(lidarState.time * 0.5) * 10;

  lidarState.detectedObjects = lidarState.detectedObjects.map(obj => {
    obj.y -= lidarState.egoSpeed * 0.016 * 0.28;
    if (obj.y < -20) {
      obj.y = 100 + Math.random() * 50;
      obj.x = (Math.random() - 0.5) * 12;
      obj.type = Math.random() > 0.3 ? "vehicle" : "pedestrian";
      obj.width = obj.type === "vehicle" ? 1.8 + Math.random() * 0.5 : 0.5 + Math.random() * 0.3;
      obj.height = obj.type === "vehicle" ? 4 + Math.random() * 1 : 1.5 + Math.random() * 0.4;
    }
    return obj;
  });

  if (Math.random() < 0.03 && lidarState.detectedObjects.length < 8) {
    lidarState.detectedObjects.push({
      x: (Math.random() - 0.5) * 14,
      y: 120 + Math.random() * 30,
      width: 0.5 + Math.random() * 2,
      height: 1 + Math.random() * 4,
      type: Math.random() > 0.4 ? "vehicle" : Math.random() > 0.5 ? "pedestrian" : "cyclist",
    });
  }

  const points = [];
  const egoPoints = [];

  for (let i = 0; i < 300; i++) {
    const angle = (Math.random() - 0.5) * 1.2;
    const distance = 15 + Math.random() * 135;
    const x = Math.sin(angle) * distance;
    const z = (Math.random() - 0.5) * 3;
    const intensity = 0.3 + Math.random() * 0.7;

    let classification = "ground";
    if (z > 0.5) classification = "object";
    if (z > 1.5) classification = "above";

    points.push({
      x: parseFloat(x.toFixed(2)),
      y: parseFloat(distance.toFixed(2)),
      z: parseFloat(z.toFixed(2)),
      intensity: parseFloat(intensity.toFixed(2)),
      classification,
    });
  }

  lidarState.detectedObjects.forEach(obj => {
    const numPoints = obj.type === "vehicle" ? 80 : obj.type === "pedestrian" ? 25 : 40;
    for (let i = 0; i < numPoints; i++) {
      const px = obj.x + (Math.random() - 0.5) * obj.width;
      const py = obj.y + (Math.random() - 0.5) * obj.height;
      const pz = Math.random() * obj.height * 0.6;
      const intensity = 0.6 + Math.random() * 0.4;

      points.push({
        x: parseFloat(px.toFixed(2)),
        y: parseFloat(py.toFixed(2)),
        z: parseFloat(pz.toFixed(2)),
        intensity: parseFloat(intensity.toFixed(2)),
        classification: obj.type === "pedestrian" ? "pedestrian" : "vehicle",
      });
    }
  });

  for (let i = 0; i < 20; i++) {
    const angle = Math.random() * Math.PI * 2;
    const distance = 5 + Math.random() * 15;
    points.push({
      x: parseFloat(Math.cos(angle) * distance.toFixed(2)),
      y: parseFloat(Math.sin(angle) * distance.toFixed(2)),
      z: 0,
      intensity: 0.9,
      classification: "ego",
    });
  }

  return points.sort(() => Math.random() - 0.5).slice(0, 500);
}

export default function handler(req, res) {
  const count = parseInt(req.query.count) || 500;
  const points = generateLidarPoints(Math.min(count, 1000));

  res.status(200).json({
    points,
    timestamp: Date.now(),
    sensor: {
      status: "active",
      confidence: 96 + Math.random() * 3,
      fps: 10,
      horizontalFOV: 120,
      verticalFOV: 40,
      range: 200,
    },
    egoSpeed: Math.round(lidarState.egoSpeed),
    detectedObjects: lidarState.detectedObjects.length,
  });
}
