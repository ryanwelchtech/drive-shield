const THREAT_TYPES = ["adversarial_patch", "phantom_object", "spoofing", "gps_manipulation"];

export default function handler(req, res) {
  const { detectionId, imagePatch } = req.body || {};
  
  const isAdversarial = Math.random() > 0.85;
  
  res.status(200).json({
    detection_id: detectionId || Date.now(),
    is_adversarial: isAdversarial,
    threat_type: isAdversarial ? THREAT_TYPES[Math.floor(Math.random() * THREAT_TYPES.length)] : null,
    confidence: parseFloat((70 + Math.random() * 30).toFixed(1)),
    explanation: isAdversarial ? "Anomalous pattern detected in perception pipeline" : null,
    recommended_action: isAdversarial ? "Reduce speed and increase sensor redundancy" : "Continue monitoring",
    timestamp: Date.now(),
  });
}
