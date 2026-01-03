const OBJECT_TYPES = ["vehicle", "pedestrian", "cyclist", "sign", "obstacle"];
const THREAT_TYPES = ["adversarial_patch", "phantom_object", "spoofing", "gps_manipulation"];

let threatHistory = [];
let detectionHistory = [];

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
  };
  const [minV, maxV] = velocityRanges[objType] || [0, 50];

  return {
    id: Date.now() + Math.random() * 1000,
    type: objType,
    threat,
    confidence: 70 + Math.random() * 30,
    distance: parseFloat((5 + Math.random() * 95).toFixed(1)),
    bearing: Math.floor(Math.random() * 360),
    velocity: parseFloat((minV + Math.random() * (maxV - minV)).toFixed(1)),
    timestamp: Date.now(),
  };
}

export default function handler(req, res) {
  const limit = parseInt(req.query.limit) || 20;

  if (detectionHistory.length === 0) {
    for (let i = 0; i < 50; i++) {
      detectionHistory.push(generateDetection());
    }
  }

  const newDetections = Array.from({ length: Math.floor(Math.random() * 4) }, generateDetection);
  detectionHistory = [...newDetections, ...detectionHistory].slice(0, 100);

  const threats = detectionHistory.filter(d => d.threat);
  const threatCounts = {
    adversarial_patch: 0,
    phantom_object: 0,
    spoofing: 0,
    gps_manipulation: 0,
  };

  threats.forEach(t => {
    if (t.threat && threatCounts[t.threat] !== undefined) {
      threatCounts[t.threat]++;
    }
  });

  const recentThreats = threats.filter(t => Date.now() - t.timestamp < 60000);
  const hasCritical = recentThreats.some(t => t.threat === "adversarial_patch" || t.threat === "spoofing");
  const hasHighThreats = recentThreats.filter(t => t.threat).length > 3;
  const hasMediumThreats = recentThreats.length > 0;

  const avgConfidence = detectionHistory.slice(0, 20).reduce((sum, d) => sum + d.confidence, 0) / Math.min(20, detectionHistory.length);

  res.status(200).json({
    total_threats: threats.length,
    threats_last_minute: recentThreats.length,
    by_type: threatCounts,
    threat_level: hasCritical ? "CRITICAL" : hasHighThreats ? "HIGH" : hasMediumThreats ? "MEDIUM" : "LOW",
    avg_confidence: parseFloat(avgConfidence.toFixed(1)),
    active_tracking: detectionHistory.slice(0, 10).length,
    timestamp: Date.now(),
    recommendations: hasCritical
      ? ["Reduce speed immediately", "Engage manual mode", "Alert operator"]
      : hasHighThreats
      ? ["Increase sensor redundancy", "Reduce speed", "Monitor closely"]
      : hasMediumThreats
      ? ["Continue monitoring", "Log all detections"]
      : ["Systems nominal", "Continue normal operation"],
  });
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
