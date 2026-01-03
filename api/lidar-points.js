function generateLidarPoints(count = 500) {
  const points = [];
  for (let i = 0; i < count; i++) {
    const angle = Math.random() * Math.PI * 2;
    const distance = 20 + Math.random() * 130;
    const z = (Math.random() - 0.5) * 10;

    points.push({
      x: parseFloat((Math.cos(angle) * distance).toFixed(2)),
      y: parseFloat((Math.sin(angle) * distance).toFixed(2)),
      z: parseFloat(z.toFixed(2)),
      intensity: parseFloat(Math.random().toFixed(2)),
      classification: Math.random() > 0.995 ? "threat" : "normal",
    });
  }
  return points;
}

export default function handler(req, res) {
  res.status(200).json({
    points: generateLidarPoints(500),
    timestamp: Date.now(),
  });
}
