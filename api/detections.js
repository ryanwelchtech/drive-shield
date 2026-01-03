const OBJECT_TYPES = ["vehicle", "pedestrian", "cyclist", "sign", "obstacle", "animal"];
const THREAT_TYPES = ["adversarial_patch", "phantom_object", "spoofing", "gps_manipulation"];
const ROAD_LANES = [-3, 0, 3];
const VEHICLE_LANES = [-2, 0, 2];

let simulationState = {
  egoVehicleSpeed: 65,
  detectedVehicles: [
    { id: 1, lane: -2, distance: 45, speed: 58, type: "vehicle", angle: -15 },
    { id: 2, lane: 2, distance: 80, speed: 72, type: "vehicle", angle: 10 },
  ],
  pedestrians: [
    { id: 101, x: -8, y: 25, vx: 0.5, vy: 0, type: "pedestrian" },
  ],
  cyclists: [],
  threats: [],
  lastUpdate: Date.now(),
};

function updateSimulation() {
  const now = Date.now();
  const dt = (now - simulationState.lastUpdate) / 1000;
  simulationState.lastUpdate = now;

  simulationState.egoVehicleSpeed = 60 + Math.sin(now / 5000) * 10;

  simulationState.detectedVehicles = simulationState.detectedVehicles.map(v => {
    const relSpeed = v.speed - simulationState.egoVehicleSpeed;
    v.distance += relSpeed * dt * 0.28;
    v.distance = Math.max(10, Math.min(150, v.distance));
    if (v.distance > 140 || v.distance < 15) {
      v.speed = simulationState.egoVehicleSpeed + (Math.random() - 0.5) * 20;
    }
    return v;
  });

  if (Math.random() < 0.02 && simulationState.detectedVehicles.length < 6) {
    const newVehicle = {
      id: Date.now(),
      lane: VEHICLE_LANES[Math.floor(Math.random() * VEHICLE_LANES.length)],
      distance: 100 + Math.random() * 40,
      speed: simulationState.egoVehicleSpeed + (Math.random() - 0.5) * 30,
      type: "vehicle",
      angle: (Math.random() - 0.5) * 30,
    };
    simulationState.detectedVehicles.push(newVehicle);
  }

  simulationState.pedestrians = simulationState.pedestrians.map(p => {
    p.x += p.vx * dt * 3;
    p.y += p.vy * dt * 3;
    if (Math.abs(p.x) > 15) p.vx *= -1;
    if (p.y < -50 || p.y > 100) p.y = -50;
    return p;
  });

  if (Math.random() < 0.01 && simulationState.pedestrians.length < 4) {
    simulationState.pedestrians.push({
      id: Date.now(),
      x: (Math.random() - 0.5) * 20,
      y: -30,
      vx: (Math.random() - 0.3) * 2,
      vy: 0.8 + Math.random() * 0.5,
      type: "pedestrian",
    });
  }

  if (Math.random() < 0.005 && simulationState.cyclists.length < 3) {
    simulationState.cyclists.push({
      id: Date.now(),
      lane: VEHICLE_LANES[Math.floor(Math.random() * VEHICLE_LANES.length)],
      distance: 60 + Math.random() * 50,
      speed: simulationState.egoVehicleSpeed - 15 + Math.random() * 10,
      type: "cyclist",
      angle: (Math.random() - 0.5) * 20,
    });
  }

  simulationState.cyclists = simulationState.cyclists.filter(c => {
    const relSpeed = c.speed - simulationState.egoVehicleSpeed;
    c.distance += relSpeed * dt * 0.28;
    return c.distance > 10 && c.distance < 150;
  });

  const hasThreat = Math.random() < 0.12;
  if (hasThreat && simulationState.threats.length < 3) {
    const threatType = THREAT_TYPES[Math.floor(Math.random() * THREAT_TYPES.length)];
    simulationState.threats.push({
      id: Date.now(),
      type: threatType,
      confidence: 75 + Math.random() * 20,
      distance: 20 + Math.random() * 60,
      bearing: Math.floor(Math.random() * 360),
      timestamp: now,
    });
  }

  simulationState.threats = simulationState.threats.filter(t => {
    t.confidence -= 0.5;
    return t.confidence > 60;
  });
}

function generateDetection() {
  updateSimulation();

  const allObjects = [
    ...simulationState.detectedVehicles.map(v => ({
      id: v.id,
      type: v.type,
      lane: v.lane,
      distance: v.distance,
      speed: v.speed,
      angle: v.angle,
      threat: null,
    })),
    ...simulationState.pedestrians.map(p => ({
      id: p.id,
      type: p.type,
      x: p.x,
      y: p.y,
      distance: Math.sqrt(p.x * p.x + p.y * p.y),
      bearing: Math.floor(Math.atan2(p.y, p.x) * 180 / Math.PI),
      velocity: Math.sqrt(p.vx * p.vx + p.vy * p.vy),
      threat: null,
    })),
    ...simulationState.cyclists.map(c => ({
      id: c.id,
      type: c.type,
      lane: c.lane,
      distance: c.distance,
      speed: c.speed,
      angle: c.angle,
      threat: null,
    })),
  ];

  if (simulationState.threats.length > 0 && Math.random() < 0.4) {
    const threat = simulationState.threats[0];
    const threatObject = allObjects[Math.floor(Math.random() * allObjects.length)];
    if (threatObject) {
      threatObject.threat = threat.type;
      threatObject.confidence = threat.confidence;
    }
  }

  const selectedObjects = allObjects
    .sort(() => Math.random() - 0.5)
    .slice(0, Math.floor(Math.random() * 3) + 1);

  return selectedObjects.map(obj => ({
    ...obj,
    confidence: obj.confidence || 82 + Math.random() * 15,
    bearing: obj.bearing || Math.floor(Math.atan2(obj.distance, (obj.lane || 0) * 3) * 180 / Math.PI),
    velocity: obj.speed ? Math.abs(obj.speed - simulationState.egoVehicleSpeed) : obj.velocity || 0,
    timestamp: Date.now(),
  }));
}

export default function handler(req, res) {
  const detections = generateDetection();
  res.status(200).json({
    detections,
    total: detections.length,
    egoSpeed: Math.round(simulationState.egoVehicleSpeed),
    activeThreats: simulationState.threats.length,
  });
}
