import { useState, useEffect, useRef, useMemo } from 'react'
import { motion, AnimatePresence } from 'framer-motion'
import { ResponsiveContainer, AreaChart, Area, XAxis, YAxis } from 'recharts'
import { prefersReducedMotion } from '../hooks/useOptimizedAnimation'

const reducedMotion = prefersReducedMotion()

const VEHICLE_TYPES = ['sedan', 'suv', 'truck', 'coupe', 'hatchback']
const LANE_WIDTH = 3.7 // meters
const LANES = [-1, 0, 1] // left, center, right

function generateObjectId() {
  return Date.now() + Math.random() * 1000
}

// Create a vehicle with consistent world coordinates
// x: lateral position (negative = left, positive = right)
// y: forward distance from ego vehicle (always positive, 0 = at ego)
function createVehicle(egoSpeed, lane = null) {
  const chosenLane = lane !== null ? lane : LANES[Math.floor(Math.random() * LANES.length)]
  const vehicleType = VEHICLE_TYPES[Math.floor(Math.random() * VEHICLE_TYPES.length)]

  return {
    id: generateObjectId(),
    type: 'vehicle',
    vehicleType,
    lane: chosenLane,
    x: chosenLane * LANE_WIDTH, // Lateral position in meters
    y: 30 + Math.random() * 80, // Forward distance in meters (30-110m ahead)
    width: vehicleType === 'truck' ? 2.6 : vehicleType === 'suv' ? 2.0 : 1.8,
    height: vehicleType === 'truck' ? 1.9 : vehicleType === 'suv' ? 1.7 : 1.5,
    length: vehicleType === 'truck' ? 8 : vehicleType === 'suv' ? 4.8 : 4.5,
    speed: egoSpeed + (Math.random() - 0.5) * 30, // Absolute speed
    confidence: 85 + Math.random() * 13,
    threat: null,
    timestamp: Date.now(),
  }
}

// Pedestrians spawn on sidewalk (far left) and may cross
function createPedestrian() {
  const side = Math.random() > 0.5 ? 'left' : 'right'
  return {
    id: generateObjectId(),
    type: 'pedestrian',
    x: side === 'left' ? -8 - Math.random() * 3 : 8 + Math.random() * 3,
    y: 15 + Math.random() * 40, // 15-55m ahead
    width: 0.5,
    height: 1.7,
    speed: 1.2 + Math.random() * 0.6, // Walking speed m/s
    direction: side === 'left' ? 1 : -1, // Crossing direction
    crossing: Math.random() > 0.5,
    confidence: 80 + Math.random() * 15,
    threat: null,
    timestamp: Date.now(),
  }
}

// Generate LIDAR points based on actual object positions
function generateLidarPoints(vehicles, pedestrians) {
  const points = []

  // Ground points - sparse random scatter
  for (let i = 0; i < 200; i++) {
    const angle = (Math.random() - 0.5) * Math.PI * 1.2 // Front 120 degrees
    const distance = 5 + Math.random() * 120
    points.push({
      x: Math.sin(angle) * distance,
      y: Math.cos(angle) * distance,
      z: (Math.random() - 0.5) * 0.3,
      intensity: 0.3 + Math.random() * 0.3,
      classification: 'ground',
    })
  }

  // Road lane markings
  for (let d = 5; d < 100; d += 2) {
    const spread = d * 0.08
    points.push({ x: -LANE_WIDTH * 1.5 + (Math.random() - 0.5) * 0.3, y: d, z: 0, intensity: 0.8, classification: 'lane' })
    points.push({ x: LANE_WIDTH * 1.5 + (Math.random() - 0.5) * 0.3, y: d, z: 0, intensity: 0.8, classification: 'lane' })
    if (d % 4 === 0) {
      points.push({ x: 0 + (Math.random() - 0.5) * 0.2, y: d, z: 0, intensity: 0.9, classification: 'lane' })
    }
  }

  // Vehicle points - dense clusters at object locations
  vehicles.forEach(v => {
    if (v.y > 5 && v.y < 130) {
      const numPoints = Math.max(8, Math.floor(30 - v.y / 6))
      for (let i = 0; i < numPoints; i++) {
        const px = v.x + (Math.random() - 0.5) * v.width
        const py = v.y + (Math.random() - 0.5) * v.length
        const pz = Math.random() * v.height
        points.push({
          x: px,
          y: py,
          z: pz,
          intensity: 0.7 + Math.random() * 0.3,
          classification: 'vehicle',
          parentId: v.id,
        })
      }
    }
  })

  // Pedestrian points
  pedestrians.forEach(p => {
    if (p.y > 3 && p.y < 80) {
      const numPoints = Math.max(5, Math.floor(15 - p.y / 8))
      for (let i = 0; i < numPoints; i++) {
        points.push({
          x: p.x + (Math.random() - 0.5) * 0.5,
          y: p.y + (Math.random() - 0.5) * 0.5,
          z: Math.random() * p.height,
          intensity: 0.6 + Math.random() * 0.3,
          classification: 'pedestrian',
          parentId: p.id,
        })
      }
    }
  })

  return points
}

function generateDetections(vehicles, pedestrians, egoSpeed) {
  const detections = []

  vehicles.forEach(v => {
    if (v.y > 8 && v.y < 130) {
      const bearing = Math.atan2(v.x, v.y) * 180 / Math.PI
      const relVelocity = v.speed - egoSpeed
      const isThreat = Math.random() > 0.92

      detections.push({
        id: v.id,
        type: 'vehicle',
        vehicleType: v.vehicleType,
        confidence: v.confidence,
        distance: v.y,
        bearing: Math.round(bearing),
        velocity: Math.abs(relVelocity),
        relVelocity,
        lane: v.lane,
        x: v.x,
        y: v.y,
        threat: isThreat ? ['adversarial_patch', 'phantom_object', 'spoofing'][Math.floor(Math.random() * 3)] : null,
        timestamp: Date.now(),
      })
    }
  })

  pedestrians.forEach(p => {
    if (p.y > 5 && p.y < 80) {
      const bearing = Math.atan2(p.x, p.y) * 180 / Math.PI
      const isThreat = Math.random() > 0.95

      detections.push({
        id: p.id,
        type: 'pedestrian',
        confidence: p.confidence,
        distance: p.y,
        bearing: Math.round(bearing),
        velocity: p.speed * 3.6,
        x: p.x,
        y: p.y,
        crossing: p.crossing,
        threat: isThreat ? 'phantom_object' : null,
        timestamp: Date.now(),
      })
    }
  })

  return detections.sort((a, b) => a.distance - b.distance)
}

function createSensorStatus(egoSpeed, vehicles, pedestrians, detections) {
  const time = Date.now() / 1000
  const baseConfidence = 94 + Math.sin(time * 0.3) * 3

  return {
    lidar: { status: 'active', confidence: baseConfidence + Math.random() * 2, fps: 10, range: 150 },
    camera: { status: 'active', confidence: baseConfidence - 2 + Math.random() * 2, fps: 30, resolution: '1920x1080' },
    radar: { status: 'active', confidence: baseConfidence + 1 + Math.random(), range: 200, updateRate: 20 },
    ultrasonic: { status: 'active', confidence: 97 + Math.random() * 2, range: 5, sensors: 12 },
    gps: { status: 'active', accuracy: 1.2 + Math.random() * 0.5, satellites: 11 + Math.floor(Math.random() * 3), hdop: 0.7 + Math.random() * 0.3 },
    fusion: { status: 'active', confidence: baseConfidence - 1, latency: 35 + Math.floor(Math.random() * 15), tracking: vehicles.length + pedestrians.length },
    egoSpeed,
    detectionCount: detections.length,
    environment: { weather: 'clear', visibility: 'good', roadCondition: 'dry', ambientLight: 'day' },
    timestamp: Date.now(),
  }
}

export default function Dashboard({ onBack }) {
  const [isMonitoring, setIsMonitoring] = useState(true)
  const [simulationData, setSimulationData] = useState({
    vehicles: [],
    pedestrians: [],
    detections: [],
    lidarPoints: [],
    sensorStatus: {},
    egoSpeed: 65,
  })
  const [confidenceHistory, setConfidenceHistory] = useState([])

  const lidarCanvasRef = useRef(null)
  const animationRef = useRef(null)
  const stateRef = useRef({
    vehicles: [
      createVehicle(65, 0),
      createVehicle(65, -1),
      createVehicle(65, 1),
    ],
    pedestrians: [],
    egoSpeed: 65,
    lidarPoints: [],
    detections: [],
    sensorStatus: {},
    lastUpdate: 0,
  })

  // Main simulation loop
  useEffect(() => {
    if (!isMonitoring) return

    let lastTime = Date.now()

    const animate = () => {
      const now = Date.now()
      const dt = Math.min((now - lastTime) / 1000, 0.1)
      lastTime = now
      const state = stateRef.current

      // Ego speed varies slightly
      state.egoSpeed = 60 + Math.sin(now / 3000) * 15

      // Update vehicle positions based on relative speed
      state.vehicles = state.vehicles.map(v => {
        const relSpeed = v.speed - state.egoSpeed // Positive = approaching, negative = receding
        let newY = v.y - relSpeed * dt // Subtract because positive relSpeed means closing distance

        // Respawn vehicles that go out of range
        if (newY < 8 || newY > 130) {
          const newVehicle = createVehicle(state.egoSpeed)
          newVehicle.id = v.id
          newVehicle.y = newY < 8 ? 80 + Math.random() * 40 : 25 + Math.random() * 30
          return newVehicle
        }

        return { ...v, y: newY, timestamp: now }
      })

      // Occasionally add new vehicles if under limit
      if (state.vehicles.length < 5 && Math.random() < 0.02) {
        const newVehicle = createVehicle(state.egoSpeed)
        const laneClear = !state.vehicles.some(v =>
          Math.abs(v.y - newVehicle.y) < 15 && v.lane === newVehicle.lane
        )
        if (laneClear) state.vehicles.push(newVehicle)
      }

      // Update pedestrians
      state.pedestrians = state.pedestrians.map(p => {
        let newX = p.x
        let newY = p.y - state.egoSpeed * dt * 0.05 // Move relative to ego

        if (p.crossing) {
          newX += p.direction * p.speed * dt
        }

        // Remove if out of view
        if (newY < 3 || newY > 90 || Math.abs(newX) > 15) return null

        return { ...p, x: newX, y: newY, timestamp: now }
      }).filter(Boolean)

      // Occasionally add pedestrians
      if (state.pedestrians.length < 3 && Math.random() < 0.008) {
        state.pedestrians.push(createPedestrian())
      }

      // Update derived data at different rates
      if (now - state.lastUpdate > 50) {
        state.lidarPoints = generateLidarPoints(state.vehicles, state.pedestrians)
        state.detections = generateDetections(state.vehicles, state.pedestrians, state.egoSpeed)
        state.sensorStatus = createSensorStatus(state.egoSpeed, state.vehicles, state.pedestrians, state.detections)
        state.lastUpdate = now

        // Update confidence history
        const avgConf = state.detections.length > 0
          ? state.detections.reduce((sum, d) => sum + d.confidence, 0) / state.detections.length
          : 95
        setConfidenceHistory(prev => [...prev.slice(-59), { time: prev.length, value: avgConf }])
      }

      setSimulationData({
        vehicles: [...state.vehicles],
        pedestrians: [...state.pedestrians],
        detections: [...state.detections],
        lidarPoints: [...state.lidarPoints],
        sensorStatus: { ...state.sensorStatus },
        egoSpeed: state.egoSpeed,
      })

      animationRef.current = requestAnimationFrame(animate)
    }

    animationRef.current = requestAnimationFrame(animate)
    return () => { if (animationRef.current) cancelAnimationFrame(animationRef.current) }
  }, [isMonitoring])

  // LIDAR Canvas Renderer - Top-down view with ego at center-bottom
  useEffect(() => {
    if (!lidarCanvasRef.current || !isMonitoring || reducedMotion) return

    const canvas = lidarCanvasRef.current
    const ctx = canvas.getContext('2d')
    const width = canvas.width
    const height = canvas.height
    let frameId = null

    const render = () => {
      const { lidarPoints, vehicles, pedestrians } = simulationData

      // Ego vehicle at center-bottom of canvas
      const egoX = width / 2
      const egoY = height - 40
      const scale = 2.5 // pixels per meter

      // Clear with fade effect
      ctx.fillStyle = 'rgba(0, 8, 12, 0.15)'
      ctx.fillRect(0, 0, width, height)

      // Draw range rings
      ctx.strokeStyle = 'rgba(6, 182, 212, 0.15)'
      ctx.lineWidth = 0.5
      for (let r = 25; r <= 125; r += 25) {
        ctx.beginPath()
        ctx.arc(egoX, egoY, r * scale, Math.PI, 0)
        ctx.stroke()

        // Range labels
        ctx.fillStyle = 'rgba(255, 255, 255, 0.2)'
        ctx.font = '9px monospace'
        ctx.fillText(`${r}m`, egoX + r * scale + 3, egoY - 5)
      }

      // Draw center line
      ctx.strokeStyle = 'rgba(6, 182, 212, 0.1)'
      ctx.beginPath()
      ctx.moveTo(egoX, egoY)
      ctx.lineTo(egoX, 0)
      ctx.stroke()

      // Draw lane lines
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)'
      ctx.setLineDash([8, 8])
      for (let lane = -1; lane <= 1; lane++) {
        const laneX = egoX + lane * LANE_WIDTH * scale
        ctx.beginPath()
        ctx.moveTo(laneX, egoY)
        ctx.lineTo(laneX, 20)
        ctx.stroke()
      }
      ctx.setLineDash([])

      // Draw ground points
      ctx.fillStyle = 'rgba(6, 182, 212, 0.25)'
      lidarPoints.filter(p => p.classification === 'ground').forEach(p => {
        const x = egoX + p.x * scale
        const y = egoY - p.y * scale
        if (y > 10 && y < height - 10 && x > 10 && x < width - 10) {
          ctx.beginPath()
          ctx.arc(x, y, 0.8, 0, Math.PI * 2)
          ctx.fill()
        }
      })

      // Draw lane marking points
      ctx.fillStyle = 'rgba(255, 255, 255, 0.4)'
      lidarPoints.filter(p => p.classification === 'lane').forEach(p => {
        const x = egoX + p.x * scale
        const y = egoY - p.y * scale
        if (y > 10) {
          ctx.beginPath()
          ctx.arc(x, y, 1, 0, Math.PI * 2)
          ctx.fill()
        }
      })

      // Draw vehicle points (bright green clusters)
      ctx.fillStyle = 'rgba(34, 197, 94, 0.8)'
      lidarPoints.filter(p => p.classification === 'vehicle').forEach(p => {
        const x = egoX + p.x * scale
        const y = egoY - p.y * scale
        if (y > 10 && y < height - 10) {
          ctx.beginPath()
          ctx.arc(x, y, 1.5, 0, Math.PI * 2)
          ctx.fill()
        }
      })

      // Draw vehicle bounding boxes
      ctx.strokeStyle = 'rgba(34, 197, 94, 0.6)'
      ctx.lineWidth = 1
      vehicles.forEach(v => {
        if (v.y > 5 && v.y < 130) {
          const x = egoX + v.x * scale
          const y = egoY - v.y * scale
          const w = v.width * scale
          const h = v.length * scale
          ctx.strokeRect(x - w/2, y - h/2, w, h)

          // Distance label
          ctx.fillStyle = 'rgba(34, 197, 94, 0.8)'
          ctx.font = '8px monospace'
          ctx.fillText(`${v.y.toFixed(0)}m`, x - 8, y + h/2 + 10)
        }
      })

      // Draw pedestrian points (orange)
      ctx.fillStyle = 'rgba(245, 158, 11, 0.8)'
      lidarPoints.filter(p => p.classification === 'pedestrian').forEach(p => {
        const x = egoX + p.x * scale
        const y = egoY - p.y * scale
        if (y > 10 && y < height - 10) {
          ctx.beginPath()
          ctx.arc(x, y, 1.2, 0, Math.PI * 2)
          ctx.fill()
        }
      })

      // Draw pedestrian markers
      pedestrians.forEach(p => {
        if (p.y > 3 && p.y < 80) {
          const x = egoX + p.x * scale
          const y = egoY - p.y * scale

          ctx.strokeStyle = 'rgba(245, 158, 11, 0.7)'
          ctx.beginPath()
          ctx.arc(x, y, 4, 0, Math.PI * 2)
          ctx.stroke()

          if (p.crossing) {
            ctx.strokeStyle = 'rgba(239, 68, 68, 0.6)'
            ctx.beginPath()
            ctx.moveTo(x, y)
            ctx.lineTo(x + p.direction * 15, y)
            ctx.stroke()
          }
        }
      })

      // Draw ego vehicle (cyan triangle pointing up)
      ctx.fillStyle = 'rgba(6, 182, 212, 0.9)'
      ctx.beginPath()
      ctx.moveTo(egoX, egoY - 12)
      ctx.lineTo(egoX - 6, egoY + 4)
      ctx.lineTo(egoX + 6, egoY + 4)
      ctx.closePath()
      ctx.fill()

      ctx.strokeStyle = 'rgba(6, 182, 212, 0.5)'
      ctx.lineWidth = 1
      ctx.beginPath()
      ctx.arc(egoX, egoY, 8, 0, Math.PI * 2)
      ctx.stroke()

      frameId = requestAnimationFrame(render)
    }

    render()
    return () => { if (frameId) cancelAnimationFrame(frameId) }
  }, [simulationData, isMonitoring, reducedMotion])

  const threatCounts = useMemo(() => ({
    adversarialPatch: simulationData.detections.filter(d => d.threat === 'adversarial_patch').length,
    phantomObject: simulationData.detections.filter(d => d.threat === 'phantom_object').length,
    spoofing: simulationData.detections.filter(d => d.threat === 'spoofing').length,
    total: simulationData.detections.filter(d => d.threat).length,
  }), [simulationData.detections])

  const currentConfidence = useMemo(() => {
    if (simulationData.detections.length === 0) return 95
    return simulationData.detections.reduce((sum, d) => sum + d.confidence, 0) / simulationData.detections.length
  }, [simulationData.detections])

  const threatLevel = threatCounts.total > 0 ? (threatCounts.adversarialPatch > 0 ? 'CRITICAL' : 'HIGH') : 'LOW'
  const getThreatColor = (level) => level === 'CRITICAL' ? 'text-av-danger' : level === 'HIGH' ? 'text-av-warning' : 'text-av-success'
  const getThreatBg = (level) => level === 'CRITICAL' ? 'bg-av-danger/20 border-av-danger/50' : level === 'HIGH' ? 'bg-av-warning/20 border-av-warning/50' : 'bg-av-success/20 border-av-success/50'

  // Camera view - renders same objects as LIDAR but with perspective projection
  const CameraView = () => {
    const { vehicles, pedestrians, egoSpeed } = simulationData

    // Perspective projection: map world (x, y) to screen (screenX, screenY)
    // y = forward distance, x = lateral offset
    // Horizon at top, road extends toward viewer at bottom
    const worldToScreen = (worldX, worldY) => {
      const horizonY = 35 // % from top
      const vanishingX = 50 // % from left (center)

      // Clamp distance to prevent extreme values
      const clampedY = Math.max(8, Math.min(120, worldY))

      // Perspective factor - objects farther away are smaller and closer to horizon
      const perspective = Math.pow(8 / clampedY, 0.7)

      // Screen position
      const screenY = horizonY + (100 - horizonY) * (1 - perspective) * 1.1
      const lateralScale = 8 / clampedY * 12
      const screenX = vanishingX + worldX * lateralScale

      return { screenX, screenY, scale: perspective }
    }

    const renderVehicle = (v) => {
      if (v.y < 10 || v.y > 110) return null

      const { screenX, screenY, scale } = worldToScreen(v.x, v.y)
      const size = Math.max(2, 8 * scale)

      const colors = {
        truck: '#4a5568',
        suv: '#374151',
        sedan: '#6b7280',
        coupe: '#9ca3af',
        hatchback: '#d1d5db'
      }

      return (
        <motion.div
          key={v.id}
          className="absolute transform -translate-x-1/2 -translate-y-1/2"
          style={{
            left: `${screenX}%`,
            top: `${screenY}%`,
            width: `${size * 2}%`,
            height: `${size * 1.4}%`,
            zIndex: Math.floor(100 - v.y),
          }}
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ duration: 0.15 }}
        >
          <div
            className="w-full h-full rounded-sm shadow-lg relative"
            style={{ backgroundColor: colors[v.vehicleType] || colors.sedan }}
          >
            {/* Windows */}
            <div className="absolute top-[10%] left-[15%] right-[15%] h-[25%] bg-slate-800/80 rounded-t-sm" />
            {/* Tail lights (visible when ahead of us) */}
            {v.y < 60 && (
              <>
                <div className="absolute bottom-[5%] left-[8%] w-[12%] h-[8%] bg-red-500 rounded-sm" />
                <div className="absolute bottom-[5%] right-[8%] w-[12%] h-[8%] bg-red-500 rounded-sm" />
              </>
            )}
          </div>
          <div className="absolute -bottom-3 left-1/2 -translate-x-1/2 text-[7px] text-white/50 whitespace-nowrap font-mono">
            {v.y.toFixed(0)}m
          </div>
        </motion.div>
      )
    }

    const renderPedestrian = (p) => {
      if (p.y < 8 || p.y > 70) return null

      const { screenX, screenY, scale } = worldToScreen(p.x, p.y)
      const size = Math.max(1, 4 * scale)

      return (
        <motion.div
          key={p.id}
          className="absolute transform -translate-x-1/2 -translate-y-1/2"
          style={{
            left: `${screenX}%`,
            top: `${screenY}%`,
            width: `${size}%`,
            height: `${size * 2.5}%`,
            zIndex: Math.floor(100 - p.y),
          }}
          animate={p.crossing ? { x: [0, p.direction * 2, 0] } : {}}
          transition={{ duration: 0.4, repeat: Infinity }}
        >
          {/* Simple stick figure */}
          <div className="w-full h-full relative">
            <div className="absolute top-0 left-1/2 -translate-x-1/2 w-[60%] aspect-square bg-amber-200 rounded-full" />
            <div className="absolute top-[25%] left-[25%] w-[50%] h-[40%] bg-blue-600 rounded-sm" />
            <div className="absolute bottom-0 left-[30%] w-[18%] h-[35%] bg-slate-800 rounded-sm" />
            <div className="absolute bottom-0 right-[30%] w-[18%] h-[35%] bg-slate-800 rounded-sm" />
          </div>
        </motion.div>
      )
    }

    return (
      <div className="relative w-full h-full overflow-hidden rounded-lg">
        {/* Sky gradient */}
        <div className="absolute inset-0 bg-gradient-to-b from-sky-600 via-sky-400 to-sky-300" />

        {/* Horizon line */}
        <div className="absolute top-[35%] left-0 right-0 h-px bg-white/20" />

        {/* Mountains/trees silhouette */}
        <svg className="absolute top-[28%] w-full h-[10%]" viewBox="0 0 400 40" preserveAspectRatio="none">
          <polygon
            points="0,40 0,25 30,30 60,15 100,28 140,10 180,25 220,18 260,30 300,12 340,28 380,20 400,35 400,40"
            fill="rgba(0,40,20,0.4)"
          />
        </svg>

        {/* Road with perspective */}
        <svg className="absolute inset-0 w-full h-full" viewBox="0 0 400 240" preserveAspectRatio="none">
          <defs>
            <linearGradient id="roadGrad" x1="0%" y1="0%" x2="0%" y2="100%">
              <stop offset="0%" stopColor="#333" />
              <stop offset="100%" stopColor="#1a1a1a" />
            </linearGradient>
            <linearGradient id="grassGrad" x1="0%" y1="0%" x2="0%" y2="100%">
              <stop offset="0%" stopColor="#1a3d1a" />
              <stop offset="100%" stopColor="#0d1f0d" />
            </linearGradient>
          </defs>

          {/* Grass on left */}
          <polygon points="0,84 0,240 100,240 150,100" fill="url(#grassGrad)" />
          {/* Grass on right */}
          <polygon points="400,84 400,240 300,240 250,100" fill="url(#grassGrad)" />

          {/* Road surface */}
          <polygon points="150,100 250,100 400,240 0,240" fill="url(#roadGrad)" />

          {/* Center line (dashed) */}
          <line x1="200" y1="100" x2="200" y2="240" stroke="white" strokeWidth="2" strokeDasharray="15,12" />

          {/* Lane dividers */}
          <line x1="175" y1="105" x2="100" y2="240" stroke="white" strokeWidth="1" strokeDasharray="8,8" opacity="0.5" />
          <line x1="225" y1="105" x2="300" y2="240" stroke="white" strokeWidth="1" strokeDasharray="8,8" opacity="0.5" />

          {/* Road edges */}
          <line x1="150" y1="100" x2="0" y2="240" stroke="white" strokeWidth="2" />
          <line x1="250" y1="100" x2="400" y2="240" stroke="white" strokeWidth="2" />
        </svg>

        {/* Vehicles - sorted by distance (farther first) */}
        {[...vehicles].sort((a, b) => b.y - a.y).map(v => renderVehicle(v))}

        {/* Pedestrians */}
        {pedestrians.map(p => renderPedestrian(p))}

        {/* HUD overlays */}
        <div className="absolute bottom-2 left-2 right-2 flex justify-between text-[9px] text-white/50 font-mono">
          <span>FOV: 120°</span>
          <span className="text-av-success">{simulationData.sensorStatus?.camera?.fps || 30} FPS</span>
          <span>1080p</span>
        </div>

        <div className="absolute bottom-2 left-1/2 -translate-x-1/2 text-sm text-white/60 font-mono">
          {Math.round(egoSpeed * 0.621)} MPH
        </div>

        <div className="absolute top-2 left-2 flex items-center gap-1.5 px-2 py-1 rounded bg-black/50">
          <div className={`w-1.5 h-1.5 rounded-full ${isMonitoring ? 'bg-av-success animate-pulse' : 'bg-av-danger'}`} />
          <span className="text-[9px] text-white/70">FRONT CAM</span>
        </div>

        <div className="absolute top-2 right-2 flex items-center gap-1.5 px-2 py-1 rounded bg-av-danger/20 border border-av-danger/40">
          <div className="w-1.5 h-1.5 rounded-full bg-av-danger animate-pulse" />
          <span className="text-[9px] text-av-danger">REC</span>
        </div>
      </div>
    )
  }

  return (
    <div className="min-h-screen bg-black road-grid p-6">
      <header className="flex items-center justify-between mb-6">
        <div className="flex items-center gap-4">
          <motion.button
            onClick={onBack}
            className="w-10 h-10 rounded-xl bg-white/5 border border-white/10 flex items-center justify-center hover:bg-white/10"
            whileHover={{ scale: 1.05 }}
            whileTap={{ scale: 0.95 }}
          >
            <svg className="w-5 h-5" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M19 12H5M12 19l-7-7 7-7" />
            </svg>
          </motion.button>
          <div>
            <h1 className="text-2xl font-bold">
              <span className="gradient-text">DriveShield</span> Monitoring
            </h1>
            <div className="flex items-center gap-4 text-xs text-white/50">
              <span className="flex items-center gap-1">
                <span className="w-1.5 h-1.5 rounded-full bg-av-success animate-pulse" />
                {Math.round(simulationData.egoSpeed * 0.621)} mph
              </span>
              <span>Objects: {simulationData.vehicles.length + simulationData.pedestrians.length}</span>
            </div>
          </div>
        </div>
        <div className="flex items-center gap-4">
          <div className={`px-4 py-2 rounded-xl border ${getThreatBg(threatLevel)}`}>
            <span className="text-xs text-white/50 mr-2">Threat:</span>
            <span className={`font-bold ${getThreatColor(threatLevel)}`}>{threatLevel}</span>
          </div>
          <motion.button
            onClick={() => setIsMonitoring(!isMonitoring)}
            className={`glass-button flex items-center gap-2 ${isMonitoring ? 'border-av-success/50' : 'border-av-danger/50'}`}
            whileHover={{ scale: 1.02 }}
            whileTap={{ scale: 0.98 }}
          >
            <span className={`w-2 h-2 rounded-full ${isMonitoring ? 'bg-av-success animate-pulse' : 'bg-av-danger'}`} />
            {isMonitoring ? 'Monitoring' : 'Paused'}
          </motion.button>
        </div>
      </header>

      <div className="grid grid-cols-12 gap-5">
        {/* Left column - Sensors */}
        <div className="col-span-3 space-y-5">
          <div className="glass-panel p-5">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-3">Sensor Status</h3>
            <div className="space-y-2">
              {['lidar', 'camera', 'radar', 'ultrasonic', 'gps'].map((sensor) => {
                const data = simulationData.sensorStatus?.[sensor] || { status: 'inactive', confidence: 0 }
                return (
                  <div key={sensor} className="p-2 rounded-lg bg-white/5">
                    <div className="flex items-center justify-between mb-1">
                      <div className="flex items-center gap-2">
                        <div className={`w-2 h-2 rounded-full ${data.status === 'active' ? 'bg-av-success animate-pulse' : 'bg-av-danger'}`} />
                        <span className="text-xs text-white capitalize">{sensor}</span>
                      </div>
                      <span className="text-xs font-mono text-av-primary">{data.confidence?.toFixed(1)}%</span>
                    </div>
                    <div className="h-1 bg-white/10 rounded-full overflow-hidden">
                      <div className="h-full bg-gradient-to-r from-av-primary to-av-accent rounded-full" style={{ width: `${data.confidence || 0}%` }} />
                    </div>
                  </div>
                )
              })}
            </div>
          </div>

          <div className="glass-panel p-5">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-3">Stats</h3>
            <div className="grid grid-cols-2 gap-2">
              <div className="p-3 rounded-lg bg-white/5 text-center">
                <p className="text-xl font-bold text-av-primary">{Math.round(simulationData.egoSpeed * 0.621)}</p>
                <p className="text-[10px] text-white/45">mph</p>
              </div>
              <div className="p-3 rounded-lg bg-white/5 text-center">
                <p className="text-xl font-bold text-av-danger">{threatCounts.total}</p>
                <p className="text-[10px] text-white/45">Threats</p>
              </div>
              <div className="p-3 rounded-lg bg-white/5 text-center">
                <p className="text-xl font-bold text-av-success">{simulationData.vehicles.length}</p>
                <p className="text-[10px] text-white/45">Vehicles</p>
              </div>
              <div className="p-3 rounded-lg bg-white/5 text-center">
                <p className="text-xl font-bold text-av-warning">{simulationData.pedestrians.length}</p>
                <p className="text-[10px] text-white/45">Pedestrians</p>
              </div>
            </div>
          </div>

          <div className="glass-panel p-5">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-3">Threats</h3>
            <div className="space-y-2">
              {[
                { name: 'Adversarial Patches', count: threatCounts.adversarialPatch, color: 'bg-av-danger' },
                { name: 'Phantom Objects', count: threatCounts.phantomObject, color: 'bg-av-warning' },
                { name: 'Spoofing', count: threatCounts.spoofing, color: 'bg-av-purple' },
              ].map((t) => (
                <div key={t.name} className="flex items-center justify-between p-2 rounded-lg bg-white/5">
                  <div className="flex items-center gap-2">
                    <div className={`w-1.5 h-1.5 rounded-full ${t.color}`} />
                    <span className="text-xs text-white/60">{t.name}</span>
                  </div>
                  <span className="text-sm font-bold text-white">{t.count}</span>
                </div>
              ))}
            </div>
          </div>
        </div>

        {/* Center column - Main visualizations */}
        <div className="col-span-6 space-y-5">
          <div className="glass-panel p-5">
            <div className="flex items-center justify-between mb-3">
              <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider">LIDAR Point Cloud</h3>
              <div className="flex items-center gap-2">
                <span className={`w-2 h-2 rounded-full ${isMonitoring ? 'bg-av-success animate-pulse' : 'bg-av-danger'}`} />
                <span className="text-xs text-white/45">{isMonitoring ? 'Live' : 'Paused'}</span>
              </div>
            </div>
            <div className="relative">
              <canvas ref={lidarCanvasRef} width={600} height={350} className="w-full rounded-lg bg-black/50" />
              <div className="absolute top-2 left-2 px-2 py-1 rounded bg-black/50 text-xs text-white/50">
                Range: 150m
              </div>
              <div className="absolute bottom-2 right-2 flex gap-3 text-xs">
                <div className="flex items-center gap-1"><div className="w-2 h-2 rounded-full bg-av-primary" /><span className="text-white/40">Ground</span></div>
                <div className="flex items-center gap-1"><div className="w-2 h-2 rounded-full bg-av-success" /><span className="text-white/40">Vehicle</span></div>
                <div className="flex items-center gap-1"><div className="w-2 h-2 rounded-full bg-av-warning" /><span className="text-white/40">Pedestrian</span></div>
              </div>
            </div>
          </div>

          <div className="glass-panel p-5">
            <div className="flex items-center justify-between mb-3">
              <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider">Detection Confidence</h3>
              <span className={`text-xl font-bold ${currentConfidence < 85 ? 'text-av-warning' : 'text-av-success'}`}>
                {currentConfidence.toFixed(1)}%
              </span>
            </div>
            <div className="h-[120px]">
              <ResponsiveContainer width="100%" height="100%">
                <AreaChart data={confidenceHistory}>
                  <defs>
                    <linearGradient id="confGrad" x1="0" y1="0" x2="0" y2="1">
                      <stop offset="0%" stopColor="#06b6d4" stopOpacity={0.3} />
                      <stop offset="100%" stopColor="#06b6d4" stopOpacity={0} />
                    </linearGradient>
                  </defs>
                  <XAxis dataKey="time" hide />
                  <YAxis domain={[70, 100]} hide />
                  <Area type="monotone" dataKey="value" stroke="#06b6d4" strokeWidth={1.5} fill="url(#confGrad)" isAnimationActive={false} />
                </AreaChart>
              </ResponsiveContainer>
            </div>
          </div>
        </div>

        {/* Right column - Camera & Log */}
        <div className="col-span-3 space-y-5">
          <div className="glass-panel p-5">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-3">Front Camera</h3>
            <div className="aspect-video">
              <CameraView />
            </div>
          </div>

          <div className="glass-panel p-5 flex-1">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-3">Detection Log</h3>
            <div className="space-y-1.5 max-h-[280px] overflow-y-auto pr-1">
              <AnimatePresence mode="popLayout">
                {simulationData.detections.slice(0, 10).map((det) => (
                  <motion.div
                    key={det.id}
                    initial={{ opacity: 0, x: 10 }}
                    animate={{ opacity: 1, x: 0 }}
                    exit={{ opacity: 0, x: -10 }}
                    className={`p-2 rounded-lg text-xs ${det.threat ? 'bg-av-danger/15 border border-av-danger/40' : 'bg-white/5'}`}
                  >
                    <div className="flex items-center justify-between mb-1">
                      <span className={`font-medium capitalize ${det.threat ? 'text-av-danger' : 'text-white'}`}>
                        {det.type === 'vehicle' ? det.vehicleType : det.type}
                      </span>
                      <span className="text-white/30 text-[10px]">
                        {new Date(det.timestamp).toLocaleTimeString()}
                      </span>
                    </div>
                    <div className="flex items-center gap-2 text-white/50">
                      <span>{det.distance.toFixed(0)}m</span>
                      <span>{det.bearing > 0 ? '+' : ''}{det.bearing}°</span>
                      <span className={det.confidence < 85 ? 'text-av-warning' : 'text-av-success'}>
                        {det.confidence.toFixed(0)}%
                      </span>
                    </div>
                    {det.threat && (
                      <div className="mt-1 text-av-danger text-[10px] capitalize">
                        ⚠ {det.threat.replace('_', ' ')}
                      </div>
                    )}
                  </motion.div>
                ))}
              </AnimatePresence>
              {simulationData.detections.length === 0 && (
                <div className="text-center text-white/30 py-4 text-xs">Scanning...</div>
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}
