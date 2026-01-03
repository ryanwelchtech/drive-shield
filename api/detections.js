const OBJECT_TYPES = ["vehicle", "pedestrian", "cyclist", "sign", "obstacle", "animal"];
const THREAT_TYPES = ["adversarial_patch", "phantom_object", "spoofing", "gps_manipulation"];

function generateDetection() {
  const objType = OBJECT_TYPES[Math.floor(Math.random() * OBJECT_TYPES.length)];
  const hasThreat = Math.random() > 0.88;
  const threat = hasThreat ? THREAT_TYPES[Math.floor(Math.random() * THREAT_TYPES.length)] : null;

  const velocityRanges = {
    vehicle: [0, 120],
    pedestrian: [0, 7],
    cyclist: [0, 30],
    sign: [0, 0],
    obstacle: [0, 0],
    animal: [0, 15],
  };
  const [minV, maxV] = velocityRanges[objType] || [0, 50];

  return {
    id: Date.now() + Math.floor(Math.random() * 999),
    type: objType,
    threat,
    confidence: 70 + Math.random() * 30,
    distance: parseFloat((5 + Math.random() * 95).toFixed(1)),
    bearing: Math.floor(Math.random() * 360),
    velocity: parseFloat((minV + Math.random() * (maxV - minV)).toFixed(1)),
    bbox: {
      x: Math.floor(Math.random() * 1720) + 100,
      y: Math.floor(Math.random() * 880) + 100,
      width: Math.floor(Math.random() * 150) + 50,
      height: Math.floor(Math.random() * 250) + 50,
    },
    timestamp: Date.now(),
  };
}

export default function handler(req, res) {
  const detections = Array.from({ length: Math.floor(Math.random() * 4) }, generateDetection);
  res.status(200).json({ detections, total: detections.length });
}
