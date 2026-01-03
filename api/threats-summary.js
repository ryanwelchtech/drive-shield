const OBJECT_TYPES = ["vehicle", "pedestrian", "cyclist", "sign", "obstacle"];
const THREAT_TYPES = ["adversarial_patch", "phantom_object", "spoofing", "gps_manipulation"];

function generateDetection() {
  const objType = OBJECT_TYPES[Math.floor(Math.random() * OBJECT_TYPES.length)];
  const hasThreat = Math.random() > 0.88;
  const threat = hasThreat ? THREAT_TYPES[Math.floor(Math.random() * THREAT_TYPES.length)] : null;
  
  return {
    id: Date.now() + Math.random() * 1000,
    type: objType,
    threat,
    confidence: 70 + Math.random() * 30,
    distance: parseFloat((5 + Math.random() * 95).toFixed(1)),
    bearing: Math.floor(Math.random() * 360),
    timestamp: Date.now(),
  };
}

export default function handler(req, res) {
  const threatCounts = {
    adversarial_patch: 0,
    phantom_object: 0,
    spoofing: 0,
    gps_manipulation: 0,
  };
  
  const recentDetections = Array.from({ length: 20 }, generateDetection);
  recentDetections.forEach(d => {
    if (d.threat && threatCounts[d.threat] !== undefined) {
      threatCounts[d.threat]++;
    }
  });
  
  const totalThreats = Object.values(threatCounts).reduce((a, b) => a + b, 0);
  const hasCritical = threatCounts.adversarial_patch > 0;
  const hasHighThreats = Object.values(threatCounts).filter(v => v > 0).length > 2;
  
  res.status(200).json({
    total_threats: totalThreats,
    by_type: threatCounts,
    threat_level: hasCritical ? "CRITICAL" : hasHighThreats ? "HIGH" : "LOW",
    timestamp: Date.now(),
  });
}
