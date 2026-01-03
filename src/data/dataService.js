/**
 * Data Service Layer for DriveShield
 *
 * This module provides a unified interface for sensor data fetching.
 * Replace mock implementations with real sensor APIs for production.
 *
 * REAL DATA INTEGRATION OPTIONS:
 *
 * 1. ROS2 WebSocket Bridge
 *    - rosbridge_server for WebSocket connectivity
 *    - Subscribe to /lidar/points, /camera/image, /radar/tracks
 *    - Real-time sensor fusion data
 *
 * 2. Autonomous Vehicle Platforms
 *    - Apollo Auto: https://apollo.auto/
 *    - Autoware: https://www.autoware.org/
 *    - CARLA Simulator: https://carla.org/
 *
 * 3. Sensor APIs
 *    - Velodyne LIDAR SDK
 *    - Mobileye perception API
 *    - Continental radar interfaces
 *
 * 4. ML Detection Services
 *    - YOLO object detection endpoints
 *    - OpenCV DNN module
 *    - TensorRT optimized inference
 *
 * 5. Adversarial Detection Research
 *    - Physical Adversarial Examples (PhysAE)
 *    - Robust ML detection models
 *    - Anomaly detection pipelines
 */

// Configuration
let apiUrl = import.meta.env.VITE_API_URL || '/api'

if (apiUrl.startsWith('http://localhost') || apiUrl.startsWith('http://127.0.0.1')) {
  apiUrl = '/api'
  console.warn('[DataService] WARNING: localhost API URL detected, using relative path /api instead')
}

const CONFIG = {
  API_BASE_URL: apiUrl,
  USE_MOCK: import.meta.env.VITE_USE_MOCK === 'true',
  CACHE_TTL: 1000,
  POLL_INTERVAL: 800,
}

console.log('[DataService] Initialized:', {
  apiUrl: CONFIG.API_BASE_URL,
  useMock: CONFIG.USE_MOCK,
  mode: CONFIG.USE_MOCK ? 'MOCK' : 'LIVE API'
})

// Simple in-memory cache for API responses
const cache = new Map()

const getCached = (key) => {
  const item = cache.get(key)
  if (item && Date.now() - item.timestamp < CONFIG.CACHE_TTL) {
    return item.data
  }
  return null
}

const setCache = (key, data) => {
  cache.set(key, { data, timestamp: Date.now() })
}

// ============================================
// MOCK DATA GENERATORS (Replace with real APIs)
// ============================================

const OBJECT_TYPES = ['vehicle', 'pedestrian', 'cyclist', 'sign', 'obstacle']
const THREAT_TYPES = ['adversarial_patch', 'phantom_object', 'spoofing', 'gps_manipulation']

const generateMockDetection = () => {
  const type = OBJECT_TYPES[Math.floor(Math.random() * OBJECT_TYPES.length)]
  const hasThreat = Math.random() > 0.85
  const threat = hasThreat ? THREAT_TYPES[Math.floor(Math.random() * THREAT_TYPES.length)] : null

  return {
    id: Date.now() + Math.random(),
    type,
    threat,
    confidence: 70 + Math.random() * 30,
    distance: parseFloat((5 + Math.random() * 95).toFixed(1)),
    bearing: Math.floor(Math.random() * 360),
    velocity: type === 'vehicle' ? 10 + Math.random() * 60 : Math.random() * 15,
    timestamp: Date.now(),
  }
}

const generateMockLidarPoints = (count = 500) => {
  return Array.from({ length: count }, () => ({
    angle: Math.random() * Math.PI * 2,
    distance: 20 + Math.random() * 130,
    z: (Math.random() - 0.5) * 50,
    intensity: Math.random(),
    classification: Math.random() > 0.9 ? 'threat' : 'normal',
  }))
}

const generateMockSensorStatus = () => ({
  lidar: { status: 'active', confidence: 95 + Math.random() * 5, fps: 10 + Math.random() * 5 },
  camera: { status: 'active', confidence: 90 + Math.random() * 10, fps: 25 + Math.random() * 5 },
  radar: { status: 'active', confidence: 94 + Math.random() * 6, range: 200 },
  ultrasonic: { status: 'active', confidence: 97 + Math.random() * 3, range: 5 },
  gps: { status: 'active', accuracy: 0.5 + Math.random() * 2, satellites: 8 + Math.floor(Math.random() * 6) },
})

// ============================================
// API METHODS
// ============================================

/**
 * Fetch current sensor status
 *
 * REAL IMPLEMENTATION:
 * GET /api/sensors-status
 * Response: { lidar: {}, camera: {}, radar: {}, ultrasonic: {}, gps: {} }
 */
export const fetchSensorStatus = async () => {
  if (CONFIG.USE_MOCK) {
    await new Promise(r => setTimeout(r, 20))
    return generateMockSensorStatus()
  }

  const response = await fetch(`${CONFIG.API_BASE_URL}/sensors-status`)
  return response.json()
}

/**
 * Fetch object detections from perception pipeline
 *
 * REAL IMPLEMENTATION:
 * GET /api/detections
 * Response: { detections: [{ type, confidence, bbox, distance, velocity }] }
 */
export const fetchDetections = async () => {
  if (CONFIG.USE_MOCK) {
    await new Promise(r => setTimeout(r, 30))
    const count = Math.floor(Math.random() * 4)
    return { detections: Array.from({ length: count }, generateMockDetection) }
  }

  const response = await fetch(`${CONFIG.API_BASE_URL}/detections`)
  return response.json()
}

/**
 * Fetch LIDAR point cloud data
 *
 * REAL IMPLEMENTATION:
 * GET /api/lidar-points
 * Response: { points: [{ x, y, z, intensity }], timestamp }
 */
export const fetchLidarPoints = async () => {
  if (CONFIG.USE_MOCK) {
    return { points: generateMockLidarPoints(), timestamp: Date.now() }
  }

  const response = await fetch(`${CONFIG.API_BASE_URL}/lidar-points`)
  return response.json()
}

/**
 * Stream real-time sensor data via polling (WebSocket not available on Vercel)
 *
 * REAL IMPLEMENTATION:
 * Polling to /api/sensors-status, /api/detections
 * Messages: { type: 'detection'|'lidar'|'status'|'confidence', data: {} }
 */
export const createSensorStream = (onMessage, onError) => {
  if (CONFIG.USE_MOCK) {
    console.log('[DataService] Using MOCK data stream')
    const detectionInterval = setInterval(() => {
      if (Math.random() > 0.6) {
        onMessage({
          type: 'detection',
          data: generateMockDetection(),
        })
      }
    }, 500)

    const statusInterval = setInterval(() => {
      onMessage({
        type: 'status',
        data: generateMockSensorStatus(),
      })
    }, 1000)

    const confidenceInterval = setInterval(() => {
      onMessage({
        type: 'confidence',
        data: {
          value: 85 + Math.random() * 15,
          timestamp: Date.now(),
        },
      })
    }, 100)

    return {
      close: () => {
        clearInterval(detectionInterval)
        clearInterval(statusInterval)
        clearInterval(confidenceInterval)
      },
      send: () => {},
    }
  }

  console.log('[DataService] Using LIVE API stream')
  let isRunning = true

  const pollData = async () => {
    if (!isRunning) return

    try {
      const [statusRes, detectionsRes] = await Promise.all([
        fetch(`${CONFIG.API_BASE_URL}/sensors-status`),
        fetch(`${CONFIG.API_BASE_URL}/detections`),
      ])

      if (!statusRes.ok || !detectionsRes.ok) {
        throw new Error('API request failed')
      }

      const status = await statusRes.json()
      const detections = await detectionsRes.json()

      onMessage({ type: 'status', data: status })

      if (detections.detections && detections.detections.length > 0) {
        detections.detections.forEach(det => {
          onMessage({ type: 'detection', data: det })
        })
      }
    } catch (error) {
      console.error('[DataService] Poll error:', error.message)
      onError(error)
    }
  }

  const lidarInterval = setInterval(async () => {
    if (!isRunning) return

    try {
      const res = await fetch(`${CONFIG.API_BASE_URL}/lidar-points`)
      if (res.ok) {
        const data = await res.json()
        onMessage({ type: 'lidar', data })
      }
    } catch (error) {
      console.error('[DataService] LIDAR poll error:', error.message)
    }
  }, 200)

  const confidenceInterval = setInterval(() => {
    if (!isRunning) return
    onMessage({
      type: 'confidence',
      data: { value: 85 + Math.random() * 15, timestamp: Date.now() },
    })
  }, 100)

  pollData()
  const mainInterval = setInterval(pollData, CONFIG.POLL_INTERVAL)

  return {
    close: () => {
      isRunning = false
      clearInterval(mainInterval)
      clearInterval(lidarInterval)
      clearInterval(confidenceInterval)
    },
    send: () => {},
  }
}

/**
 * Create ROS2 bridge connection for direct sensor access
 *
 * REAL IMPLEMENTATION:
 * Uses rosbridge_websocket for ROS2 integration
 * Subscribe to: /lidar/points, /camera/image_raw, /radar/tracks
 */
export const createROSBridge = (topics, onMessage) => {
  if (CONFIG.USE_MOCK) {
    console.log('Mock ROS bridge - real implementation requires rosbridge_server')
    return { close: () => {} }
  }

  const ws = new WebSocket(CONFIG.ROS_BRIDGE_URL)

  ws.onopen = () => {
    topics.forEach(topic => {
      ws.send(JSON.stringify({
        op: 'subscribe',
        topic: topic.name,
        type: topic.type,
      }))
    })
  }

  ws.onmessage = (event) => {
    const msg = JSON.parse(event.data)
    if (msg.op === 'publish') {
      onMessage(msg.topic, msg.msg)
    }
  }

  return {
    close: () => ws.close(),
    publish: (topic, msg) => {
      ws.send(JSON.stringify({ op: 'publish', topic, msg }))
    },
  }
}

/**
 * Analyze detection for adversarial threats
 *
 * REAL IMPLEMENTATION:
 * POST /api/threat-analyze
 * Body: { detection, image_patch }
 * Response: { is_adversarial, threat_type, confidence, explanation }
 */
export const analyzeForThreats = async (detection, imagePatch = null) => {
  if (CONFIG.USE_MOCK) {
    await new Promise(r => setTimeout(r, 50))
    const isAdversarial = Math.random() > 0.9
    return {
      is_adversarial: isAdversarial,
      threat_type: isAdversarial ? THREAT_TYPES[Math.floor(Math.random() * THREAT_TYPES.length)] : null,
      confidence: 70 + Math.random() * 30,
      explanation: isAdversarial ? 'Anomalous pattern detected in perception pipeline' : null,
    }
  }

  const response = await fetch(`${CONFIG.API_BASE_URL}/threat-analyze`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ detection, image_patch: imagePatch }),
  })
  return response.json()
}

// ============================================
// REAL DATA INTEGRATION EXAMPLES
// ============================================

/**
 * Example: Connect to CARLA Simulator
 *
 * import carla
 *
 * # Python backend that exposes CARLA data via REST/WebSocket
 * client = carla.Client('localhost', 2000)
 * world = client.get_world()
 *
 * @app.get("/api/lidar/points")
 * async def get_lidar():
 *     lidar_data = get_sensor_data('lidar')
 *     return {"points": lidar_data.raw_data, "timestamp": time.time()}
 */

/**
 * Example: Connect to ROS2 Topics
 *
 * # rosbridge_server setup
 * ros2 launch rosbridge_server rosbridge_websocket_launch.xml
 *
 * # Subscribe to topics in browser:
 * const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' })
 * const lidarSub = new ROSLIB.Topic({
 *   ros: ros,
 *   name: '/velodyne_points',
 *   messageType: 'sensor_msgs/PointCloud2'
 * })
 * lidarSub.subscribe((message) => processPointCloud(message))
 */

/**
 * Example: YOLO Detection API
 *
 * from ultralytics import YOLO
 * model = YOLO('yolov8x.pt')
 *
 * @app.post("/api/detect")
 * async def detect(image: UploadFile):
 *     results = model(image)
 *     detections = [{
 *         'class': result.names[int(box.cls)],
 *         'confidence': float(box.conf),
 *         'bbox': box.xyxy.tolist()
 *     } for box in results[0].boxes]
 *     return {"detections": detections}
 */

export default {
  fetchSensorStatus,
  fetchDetections,
  fetchLidarPoints,
  createSensorStream,
  createROSBridge,
  analyzeForThreats,
  CONFIG,
}
