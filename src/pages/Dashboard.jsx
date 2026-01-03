import { useState, useEffect, useRef, useCallback, useMemo } from 'react'
import { motion } from 'framer-motion'
import { ResponsiveContainer, AreaChart, Area, XAxis, YAxis } from 'recharts'
import { prefersReducedMotion } from '../hooks/useOptimizedAnimation'

const reducedMotion = prefersReducedMotion()

const VEHICLE_TYPES = ['sedan', 'suv', 'truck', 'coupe', 'hatchback']
const LANES = [-1, 0, 1]
const ROAD_WIDTH = 12

function generateObjectId() {
  return Date.now() + Math.random() * 1000
}

function createVehicle(egoSpeed, lane = null) {
  const chosenLane = lane !== null ? lane : LANES[Math.floor(Math.random() * LANES.length)]
  const vehicleType = VEHICLE_TYPES[Math.floor(Math.random() * VEHICLE_TYPES.length)]
  const width = vehicleType === 'truck' ? 2.6 : vehicleType === 'suv' ? 1.9 : 1.8
  const height = vehicleType === 'truck' ? 4.2 : vehicleType === 'suv' ? 1.8 : 1.5

  return {
    id: generateObjectId(),
    type: 'vehicle',
    vehicleType,
    lane: chosenLane,
    x: chosenLane * 3.5,
    y: 30 + Math.random() * 100,
    z: 0,
    width,
    height,
    length: vehicleType === 'truck' ? 9 : vehicleType === 'suv' ? 4.7 : 4.5,
    speed: egoSpeed + (Math.random() - 0.5) * 25,
    confidence: 0,
    threat: null,
  }
}

function createPedestrian() {
  return {
    id: generateObjectId(),
    type: 'pedestrian',
    x: -15 - Math.random() * 10,
    y: -20,
    z: 0,
    width: 0.5,
    height: 1.7,
    speed: 1.2 + Math.random() * 0.5,
    direction: Math.random() > 0.5 ? 'right' : 'crossing',
    state: 'approaching',
    confidence: 0,
    threat: null,
  }
}

function generateLidarPoints(vehicles, pedestrians, egoSpeed) {
  const points = []

  for (let i = 0; i < 400; i++) {
    const angle = (Math.random() - 0.5) * 2.5
    const distance = 5 + Math.random() * 140
    const x = Math.sin(angle) * distance
    const y = Math.cos(angle) * distance
    const z = (Math.random() - 0.5) * 3
    points.push({
      x: parseFloat(x.toFixed(2)),
      y: parseFloat(y.toFixed(2)),
      z: parseFloat(z.toFixed(2)),
      intensity: 0.3 + Math.random() * 0.5,
      classification: 'ground',
    })
  }

  vehicles.forEach(v => {
    if (v.y > 5 && v.y < 150) {
      const numPoints = Math.floor(20 + v.y / 5)
      for (let i = 0; i < numPoints; i++) {
        const px = v.x + (Math.random() - 0.5) * v.width
        const py = v.y + (Math.random() - 0.5) * v.length
        const pz = Math.random() * v.height
        const intensity = 0.7 + Math.random() * 0.3
        const dist = Math.sqrt(px * px + py * py)
        const angle = Math.atan2(px, py)

        points.push({
          x: parseFloat(px.toFixed(2)),
          y: parseFloat(py.toFixed(2)),
          z: parseFloat(pz.toFixed(2)),
          intensity: parseFloat(intensity.toFixed(2)),
          classification: 'vehicle',
        })
      }
    }
  })

  pedestrians.forEach(p => {
    if (p.y > -10 && p.y < 120) {
      const numPoints = 12
      for (let i = 0; i < numPoints; i++) {
        const px = p.x + (Math.random() - 0.5) * 0.6
        const py = p.y + Math.random() * 1.7
        const pz = Math.random() * 1.7
        const intensity = 0.75 + Math.random() * 0.25

        points.push({
          x: parseFloat(px.toFixed(2)),
          y: parseFloat(py.toFixed(2)),
          z: parseFloat(pz.toFixed(2)),
          intensity: parseFloat(intensity.toFixed(2)),
          classification: 'pedestrian',
        })
      }
    }
  })

  return points
}

function generateDetections(vehicles, pedestrians, egoSpeed) {
  const detections = []

  vehicles.forEach(v => {
    if (v.y > 10 && v.y < 150) {
      const relativeSpeed = v.speed - egoSpeed
      const distFactor = Math.max(0.5, 1 - (v.y - 10) / 140)
      const confidence = Math.min(99, 85 + distFactor * 14)

      let bearing = Math.atan2(v.x, v.y) * 180 / Math.PI
      bearing = bearing > 180 ? bearing - 360 : bearing < -180 ? bearing + 360 : bearing

      const isThreat = Math.random() > 0.92

      detections.push({
        id: v.id,
        type: 'vehicle',
        vehicleType: v.vehicleType,
        confidence: parseFloat(confidence.toFixed(1)),
        distance: parseFloat(v.y.toFixed(1)),
        bearing: Math.round(bearing),
        velocity: parseFloat(Math.abs(relativeSpeed).toFixed(1)),
        lane: v.lane,
        threat: isThreat ? ['adversarial_patch', 'phantom_object', 'spoofing'][Math.floor(Math.random() * 3)] : null,
        timestamp: Date.now(),
      })
    }
  })

  pedestrians.forEach(p => {
    if (p.y > 0 && p.y < 120) {
      const distFactor = Math.max(0.4, 1 - p.y / 120)
      const confidence = Math.min(96, 80 + distFactor * 16)

      let bearing = Math.atan2(p.x, p.y) * 180 / Math.PI
      bearing = bearing > 180 ? bearing - 360 : bearing < -180 ? bearing + 360 : bearing

      const isThreat = Math.random() > 0.95

      detections.push({
        id: p.id,
        type: 'pedestrian',
        confidence: parseFloat(confidence.toFixed(1)),
        distance: parseFloat(p.y.toFixed(1)),
        bearing: Math.round(bearing),
        velocity: parseFloat((p.speed * 3.6).toFixed(1)),
        x: parseFloat(p.x.toFixed(1)),
        y: parseFloat(p.y.toFixed(1)),
        threat: isThreat ? 'phantom_object' : null,
        timestamp: Date.now(),
      })
    }
  })

  return detections.sort((a, b) => b.confidence - a.confidence)
}

function createSensorStatus(egoSpeed, vehicles, pedestrians) {
  const time = Date.now() / 1000
  const baseConfidence = 96 + Math.sin(time * 0.3) * 2

  return {
    lidar: {
      status: 'active',
      confidence: parseFloat((baseConfidence + Math.random()).toFixed(1)),
      fps: 10,
      range: 200,
      pointsPerScan: 64000,
    },
    camera: {
      status: 'active',
      confidence: parseFloat((baseConfidence - 2 + Math.random() * 2).toFixed(1)),
      fps: 30,
      resolution: '1920x1080',
    },
    radar: {
      status: 'active',
      confidence: parseFloat((baseConfidence + 0.5 + Math.random()).toFixed(1)),
      range: 250,
      updateRate: 20,
    },
    ultrasonic: {
      status: 'active',
      confidence: parseFloat((98 + Math.random()).toFixed(1)),
      range: 8,
      sensors: 12,
    },
    gps: {
      status: 'active',
      accuracy: parseFloat((1 + Math.random()).toFixed(1)),
      satellites: 12,
      hdop: parseFloat((0.7 + Math.random() * 0.3).toFixed(1)),
    },
    fusion: {
      status: 'active',
      confidence: parseFloat((baseConfidence - 1).toFixed(1)),
      latency: 45,
      tracking: vehicles.length + pedestrians.length,
    },
    egoSpeed: parseFloat(egoSpeed.toFixed(0)),
    environment: {
      weather: 'clear',
      visibility: 'good',
      roadCondition: 'dry',
      ambientLight: 'day',
    },
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

  const canvasRef = useRef(null)
  const animationRef = useRef(null)
  const lastUpdateRef = useRef(Date.now())
  const stateRef = useRef({
    vehicles: [
      createVehicle(65, -1),
      createVehicle(65, 0),
      createVehicle(65, 1),
    ],
    pedestrians: [],
    egoSpeed: 65,
    lastDetectionTime: 0,
    lastStatusTime: 0,
  })

  useEffect(() => {
    if (!isMonitoring) return

    let lastTime = Date.now()

    const animate = () => {
      const now = Date.now()
      const dt = Math.min((now - lastTime) / 1000, 0.1)
      lastTime = now
      const state = stateRef.current

      state.egoSpeed = 60 + Math.sin(now / 3000) * 15

      state.vehicles = state.vehicles.map(v => {
        const relSpeed = (v.speed - state.egoSpeed) * dt * 0.028
        let newY = v.y + relSpeed

        if (newY > 150 || newY < 8) {
          return {
            ...v,
            y: 40 + Math.random() * 80,
            speed: state.egoSpeed + (Math.random() - 0.5) * 20,
            lane: LANES[Math.floor(Math.random() * LANES.length)],
            vehicleType: VEHICLE_TYPES[Math.floor(Math.random() * VEHICLE_TYPES.length)],
          }
        }
        return { ...v, y: newY }
      })

      if (state.vehicles.length < 4 && Math.random() < 0.02) {
        const newVehicle = createVehicle(state.egoSpeed)
        const laneClear = !state.vehicles.some(v => Math.abs(v.y - newVehicle.y) < 20 && v.lane === newVehicle.lane)
        if (laneClear) {
          state.vehicles.push(newVehicle)
        }
      }

      state.pedestrians = state.pedestrians.map(p => {
        let newX = p.x
        let newY = p.y
        let newState = p.state
        let newDirection = p.direction
        let newSpeed = p.speed

        if (p.state === 'approaching') {
          newX += p.speed * dt * 3
          if (newX > -3) {
            newState = 'crossing'
            newDirection = Math.random() > 0.4 ? 'crossing' : 'walking'
          }
        } else if (p.state === 'crossing') {
          newY += p.speed * dt * 3
          newX += (p.direction === 'right' ? 1 : -0.3) * p.speed * dt * 2

          if (newY > 100 || Math.abs(newX) > 20) {
            return null
          }
          if (Math.random() < 0.01) {
            newDirection = Math.random() > 0.5 ? 'left' : 'right'
          }
        } else if (p.state === 'walking') {
          newY += p.speed * dt * 2
          newX += (p.direction === 'right' ? 1 : -1) * p.speed * dt
          if (newY > 100 || Math.abs(newX) > 25) {
            return null
          }
        }

        return {
          ...p,
          x: newX,
          y: newY,
          state: newState,
          direction: newDirection,
          speed: newSpeed,
        }
      }).filter(Boolean)

      if (state.pedestrians.length < 5 && Math.random() < 0.008) {
        state.pedestrians.push(createPedestrian())
      }

      const lidarPoints = generateLidarPoints(state.vehicles, state.pedestrians, state.egoSpeed)

      let detections = []
      if (now - state.lastDetectionTime > 300) {
        detections = generateDetections(state.vehicles, state.pedestrians, state.egoSpeed)
        state.lastDetectionTime = now
      }

      let sensorStatus = state.sensorStatus
      if (now - state.lastStatusTime > 500) {
        sensorStatus = createSensorStatus(state.egoSpeed, state.vehicles, state.pedestrians)
        state.lastStatusTime = now
        state.sensorStatus = sensorStatus
      }

      setSimulationData({
        vehicles: [...state.vehicles],
        pedestrians: [...state.pedestrians],
        detections,
        lidarPoints,
        sensorStatus,
        egoSpeed: state.egoSpeed,
      })

      animationRef.current = requestAnimationFrame(animate)
    }

    animationRef.current = requestAnimationFrame(animate)

    return () => {
      if (animationRef.current) cancelAnimationFrame(animationRef.current)
    }
  }, [isMonitoring])

  useEffect(() => {
    if (!canvasRef.current || !isMonitoring || reducedMotion) return

    const canvas = canvasRef.current
    const ctx = canvas.getContext('2d')
    const width = canvas.width
    const height = canvas.height
    let frameId = null

    const render = () => {
      const { lidarPoints, vehicles, pedestrians } = simulationData
      const centerX = width / 2
      const centerY = height * 0.7

      ctx.fillStyle = 'rgba(0, 5, 10, 0.15)'
      ctx.fillRect(0, 0, width, height)

      ctx.strokeStyle = 'rgba(6, 182, 212, 0.12)'
      ctx.lineWidth = 0.5
      ctx.beginPath()
      for (let r = 25; r <= 150; r += 25) {
        ctx.moveTo(centerX + r, centerY)
        ctx.arc(centerX, centerY, r, 0, Math.PI * 2)
      }
      ctx.stroke()

      ctx.strokeStyle = 'rgba(6, 182, 212, 0.08)'
      ctx.beginPath()
      ctx.moveTo(centerX, 0)
      ctx.lineTo(centerX, height)
      ctx.moveTo(0, centerY)
      ctx.lineTo(width, centerY)
      ctx.stroke()

      ctx.fillStyle = 'rgba(6, 182, 212, 0.4)'
      ctx.beginPath()
      lidarPoints.forEach(p => {
        const x = centerX + p.x * 1.2
        const y = centerY - p.y * 1.2
        if (x > 0 && x < width && y > 0 && y < height) {
          const size = 0.5 + p.intensity * 1
          ctx.moveTo(x + size, y)
          ctx.arc(x, y, size, 0, Math.PI * 2)
        }
      })
      ctx.fill()

      ctx.fillStyle = 'rgba(34, 197, 94, 0.6)'
      ctx.beginPath()
      vehicles.forEach(v => {
        if (v.y > 5 && v.y < 150) {
          const x = centerX + v.x * 1.2
          const y = centerY - v.y * 1.2
          if (x > -50 && x < width + 50 && y > -50 && y < height + 50) {
            for (let i = 0; i < 8; i++) {
              const ox = (Math.random() - 0.5) * v.width
              const oy = (Math.random() - 0.5) * v.length
              const oz = Math.random() * v.height * 0.5
              const px = x + ox * 1.2
              const py = y - oy * 1.2
              ctx.moveTo(px + 0.8, py)
              ctx.arc(px, py, 0.8, 0, Math.PI * 2)
            }
          }
        }
      })
      ctx.fill()

      ctx.fillStyle = 'rgba(245, 158, 11, 0.7)'
      ctx.beginPath()
      pedestrians.forEach(p => {
        if (p.y > -5 && p.y < 120) {
          const x = centerX + p.x * 1.2
          const y = centerY - p.y * 1.2
          if (x > -30 && x < width + 30 && y > -30 && y < height + 30) {
            for (let i = 0; i < 5; i++) {
              const ox = (Math.random() - 0.5) * 0.5
              const oy = Math.random() * 1.7
              const px = x + ox * 1.2
              const py = y - oy * 1.2
              ctx.moveTo(px + 0.5, py)
              ctx.arc(px, py, 0.5, 0, Math.PI * 2)
            }
          }
        }
      })
      ctx.fill()

      ctx.fillStyle = 'rgba(6, 182, 212, 0.7)'
      ctx.beginPath()
      for (let i = 0; i < 20; i++) {
        const angle = (i / 20) * Math.PI * 2
        const dist = 2 + Math.random() * 5
        const x = centerX + Math.cos(angle) * dist
        const y = centerY + Math.sin(angle) * dist
        ctx.moveTo(x + 0.6, y)
        ctx.arc(x, y, 0.6, 0, Math.PI * 2)
      }
      ctx.fill()

      frameId = requestAnimationFrame(render)
    }

    render()

    return () => {
      if (frameId) cancelAnimationFrame(frameId)
    }
  }, [simulationData, isMonitoring, reducedMotion])

  const threatCounts = useMemo(() => ({
    adversarialPatch: simulationData.detections.filter(d => d.threat === 'adversarial_patch').length,
    phantomObject: simulationData.detections.filter(d => d.threat === 'phantom_object').length,
    spoofing: simulationData.detections.filter(d => d.threat === 'spoofing').length,
    total: simulationData.detections.filter(d => d.threat).length,
  }), [simulationData.detections])

  const detectionStats = useMemo(() => ({
    vehicles: simulationData.vehicles.length,
    pedestrians: simulationData.pedestrians.length,
  }), [simulationData.vehicles, simulationData.pedestrians])

  const currentConfidence = useMemo(() => {
    if (simulationData.detections.length === 0) return 96.8
    const avg = simulationData.detections.reduce((sum, d) => sum + d.confidence, 0) / simulationData.detections.length
    return avg
  }, [simulationData.detections])

  const getThreatColor = (level) => {
    switch (level) {
      case 'CRITICAL': return 'text-av-danger'
      case 'HIGH': return 'text-av-warning'
      default: return 'text-av-success'
    }
  }

  const getThreatBg = (level) => {
    switch (level) {
      case 'CRITICAL': return 'bg-av-danger/20 border-av-danger/50'
      case 'HIGH': return 'bg-av-warning/20 border-av-warning/50'
      default: return 'bg-av-success/20 border-av-success/50'
    }
  }

  const threatLevel = threatCounts.total > 0 ? (threatCounts.adversarialPatch > 0 ? 'CRITICAL' : 'HIGH') : 'LOW'

  const perspectiveVehicle = (v, idx) => {
    const scale = Math.max(0.08, Math.min(1.2, 80 / v.y))
    const perspectiveY = 135 + (v.y / 160) * 105
    const perspectiveX = 200 + v.x * (1 + v.y / 200) * 12
    const vehicleWidth = (v.width * 2.5) * scale
    const vehicleHeight = (v.height * 2.5) * scale
    const isAhead = v.y < 80

    const colors = {
      truck: '#4a5568',
      suv: '#2d3748',
      sedan: '#718096',
      coupe: '#a0aec0',
      hatchback: '#cbd5e0',
    }

    return (
      <motion.div
        key={v.id}
        className="absolute"
        style={{
          left: `${(perspectiveX / 400) * 100}%`,
          top: `${(perspectiveY / 240) * 100}%`,
          width: vehicleWidth,
          height: vehicleHeight,
        }}
        initial={{ opacity: 0 }}
        animate={{ opacity: 1 }}
      >
        <div
          className="w-full h-full rounded-lg shadow-lg"
          style={{ backgroundColor: colors[v.vehicleType] || colors.sedan }}
        >
          <div className="absolute top-[8%] left-[12%] right-[12%] h-[22%] bg-slate-800 rounded-t-sm" />
          <div className="absolute bottom-[5%] left-[15%] right-[15%] h-[15%] bg-slate-800 rounded-b-sm" />
          {isAhead ? (
            <div className="absolute top-[8%] left-[5%] right-[5%] flex justify-between">
              <div className="w-1.5 h-1 bg-yellow-400 rounded-full shadow-yellow-400/50 shadow-sm" />
              <div className="w-1.5 h-1 bg-yellow-400 rounded-full shadow-yellow-400/50 shadow-sm" />
            </div>
          ) : (
            <div className="absolute bottom-[5%] left-[10%] right-[10%] flex justify-between">
              <div className="w-1.5 h-1 bg-red-600 rounded-full shadow-red-600/50 shadow-sm" />
              <div className="w-1.5 h-1 bg-red-600 rounded-full shadow-red-600/50 shadow-sm" />
            </div>
          )}
        </div>
        <div className="absolute -bottom-3 left-1/2 -translate-x-1/2 text-[7px] text-white/50 whitespace-nowrap">
          {v.y.toFixed(0)}m
        </div>
      </motion.div>
    )
  }

  const perspectivePedestrian = (p, idx) => {
    const scale = Math.max(0.12, Math.min(1, 70 / Math.max(10, p.y)))
    const perspectiveY = 140 + (p.y / 120) * 90
    const perspectiveX = 200 + p.x * (1 + p.y / 150) * 12
    const personWidth = 8 * scale
    const personHeight = 18 * scale

    return (
      <motion.div
        key={p.id}
        className="absolute"
        style={{
          left: `${(perspectiveX / 400) * 100}%`,
          top: `${(perspectiveY / 240) * 100}%`,
          width: personWidth,
          height: personHeight,
        }}
        animate={p.state === 'approaching' ? { y: [0, -1.5, 0] } : {}}
        transition={p.state === 'approaching' ? { duration: 0.4, repeat: Infinity } : {}}
      >
        <div className="w-full h-full relative">
          <div className="absolute top-0 left-1/2 -translate-x-1/2 w-[50%] h-[22%] bg-amber-200 rounded-full" />
          <div className="absolute top-[20%] left-[25%] w-[50%] h-[40%] bg-blue-600 rounded-sm" />
          <div className="absolute bottom-0 left-[30%] w-[18%] h-[38%] bg-black rounded-sm" />
          <div className="absolute bottom-0 right-[30%] w-[18%] h-[38%] bg-black rounded-sm" />
        </div>
      </motion.div>
    )
  }

  return (
    <motion.div
      className="min-h-screen bg-black road-grid p-6"
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      exit={{ opacity: 0 }}
    >
      <header className="flex items-center justify-between mb-6">
        <div className="flex items-center gap-4">
          <motion.button
            onClick={onBack}
            className="w-10 h-10 rounded-xl bg-white/5 border border-white/10 flex items-center justify-center hover:bg-white/10 transition-colors"
            whileHover={{ scale: 1.05 }}
            whileTap={{ scale: 0.95 }}
          >
            <svg className="w-5 h-5" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M19 12H5M12 19l-7-7 7-7" />
            </svg>
          </motion.button>
          <div>
            <h1 className="text-2xl font-bold">
              <span className="gradient-text">DriveShield</span> Monitoring Center
            </h1>
            <div className="flex items-center gap-4 text-xs text-white/50">
              <span className="flex items-center gap-1">
                <span className="w-1.5 h-1.5 rounded-full bg-av-success animate-pulse" />
                {Math.round(simulationData.egoSpeed * 0.621)} mph
              </span>
              <span>Fusion: {simulationData.sensorStatus?.fusion?.confidence?.toFixed(0) || 96}%</span>
              <span className="capitalize">{simulationData.sensorStatus?.environment?.ambientLight || 'day'} mode</span>
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
            {isMonitoring ? (
              <>
                <span className="w-2 h-2 rounded-full bg-av-success animate-pulse" />
                <span>Monitoring Active</span>
              </>
            ) : (
              <>
                <span className="w-2 h-2 rounded-full bg-av-danger" />
                <span>Monitoring Paused</span>
              </>
            )}
          </motion.button>
        </div>
      </header>

      <div className="grid grid-cols-12 gap-6">
        <div className="col-span-3 space-y-6">
          <div className="glass-panel p-6">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-4">Sensor Status</h3>
            <div className="space-y-3">
              {['lidar', 'camera', 'radar', 'ultrasonic', 'gps'].map((sensor) => {
                const data = simulationData.sensorStatus?.[sensor] || { status: 'inactive', confidence: 0 }
                return (
                  <div key={sensor} className="p-3 rounded-xl bg-white/5">
                    <div className="flex items-center justify-between mb-2">
                      <div className="flex items-center gap-3">
                        <div className={`w-2.5 h-2.5 rounded-full ${data.status === 'active' ? 'bg-av-success animate-pulse' : 'bg-av-danger'}`} />
                        <span className="text-sm text-white capitalize">{sensor}</span>
                      </div>
                      <span className="text-xs font-mono text-av-primary">{data.confidence?.toFixed(1)}%</span>
                    </div>
                    <div className="h-1 bg-white/10 rounded-full overflow-hidden">
                      <motion.div
                        className="h-full bg-gradient-to-r from-av-primary to-av-accent rounded-full"
                        animate={{ width: `${data.confidence || 0}%` }}
                        transition={{ duration: 0.5 }}
                      />
                    </div>
                  </div>
                )
              })}
              {simulationData.sensorStatus?.fusion && (
                <div className="p-3 rounded-xl bg-av-primary/10 border border-av-primary/20">
                  <div className="flex items-center justify-between mb-1">
                    <span className="text-xs text-av-primary uppercase">Sensor Fusion</span>
                    <span className="text-xs font-mono text-av-primary">{simulationData.sensorStatus.fusion.confidence?.toFixed(1)}%</span>
                  </div>
                  <div className="flex gap-3 text-[10px] text-white/40">
                    <span>Latency: {simulationData.sensorStatus.fusion.latency}ms</span>
                    <span>Tracking: {simulationData.sensorStatus.fusion.tracking} objects</span>
                  </div>
                </div>
              )}
            </div>
          </div>

          <div className="glass-panel p-6">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-4">Detection Stats</h3>
            <div className="grid grid-cols-2 gap-4">
              <div className="p-4 rounded-xl bg-white/5 text-center">
                <p className="text-2xl font-bold text-av-primary">{Math.round(simulationData.egoSpeed * 0.621)}<span className="text-sm text-white/30 ml-1">mph</span></p>
                <p className="text-xs text-white/50">Ego Speed</p>
              </div>
              <div className="p-4 rounded-xl bg-white/5 text-center">
                <p className="text-2xl font-bold text-av-danger">{threatCounts.total}</p>
                <p className="text-xs text-white/50">Threats</p>
              </div>
              <div className="p-4 rounded-xl bg-white/5 text-center">
                <p className="text-2xl font-bold text-av-success">{detectionStats.vehicles}</p>
                <p className="text-xs text-white/50">Vehicles</p>
              </div>
              <div className="p-4 rounded-xl bg-white/5 text-center">
                <p className="text-2xl font-bold text-av-warning">{detectionStats.pedestrians}</p>
                <p className="text-xs text-white/50">Pedestrians</p>
              </div>
            </div>
          </div>

          <div className="glass-panel p-6">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-4">Active Threats</h3>
            <div className="space-y-2">
              {[
                { name: 'Adversarial Patches', count: threatCounts.adversarialPatch, color: 'bg-av-danger' },
                { name: 'Phantom Objects', count: threatCounts.phantomObject, color: 'bg-av-warning' },
                { name: 'Sensor Spoofing', count: threatCounts.spoofing, color: 'bg-av-purple' },
              ].map((threat) => (
                <div key={threat.name} className="flex items-center justify-between p-3 rounded-xl bg-white/5">
                  <div className="flex items-center gap-2">
                    <div className={`w-2 h-2 rounded-full ${threat.color}`} />
                    <span className="text-xs text-white/70">{threat.name}</span>
                  </div>
                  <span className="text-sm font-bold text-white">{threat.count}</span>
                </div>
              ))}
            </div>
          </div>
        </div>

        <div className="col-span-6 space-y-6">
          <div className="glass-panel p-6">
            <div className="flex items-center justify-between mb-4">
              <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider">LIDAR Point Cloud</h3>
              <div className="flex items-center gap-2">
                <span className={`w-2 h-2 rounded-full ${isMonitoring ? 'bg-av-success animate-pulse' : 'bg-av-danger'}`} />
                <span className="text-xs text-white/50">{isMonitoring ? 'Live Feed' : 'Paused'}</span>
              </div>
            </div>
            <div className="relative">
              <canvas ref={canvasRef} width={600} height={350} className="w-full rounded-xl bg-black/50" />
              <div className="absolute top-4 left-4 flex items-center gap-2 px-3 py-1.5 rounded-full bg-black/60 border border-white/10">
                <span className="text-xs text-white/50">Range:</span>
                <span className="text-xs text-av-primary font-mono">150m</span>
              </div>
              <div className="absolute bottom-4 right-4 flex gap-4 text-xs">
                <div className="flex items-center gap-2">
                  <div className="w-3 h-3 rounded-full bg-av-primary" />
                  <span className="text-white/50">LIDAR</span>
                </div>
                <div className="flex items-center gap-2">
                  <div className="w-3 h-3 rounded-full bg-av-success" />
                  <span className="text-white/50">Vehicle</span>
                </div>
                <div className="flex items-center gap-2">
                  <div className="w-3 h-3 rounded-full bg-av-warning" />
                  <span className="text-white/50">Pedestrian</span>
                </div>
              </div>
            </div>
          </div>

          <div className="glass-panel p-6">
            <div className="flex items-center justify-between mb-4">
              <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider">Object Detection Confidence</h3>
              <span className={`text-2xl font-bold ${currentConfidence < 85 ? 'text-av-warning' : 'text-av-success'}`}>
                {currentConfidence.toFixed(1)}%
              </span>
            </div>
            <div className="h-[150px]">
              <ResponsiveContainer width="100%" height="100%">
                <AreaChart data={[]}>
                  <defs>
                    <linearGradient id="confGradient" x1="0" y1="0" x2="0" y2="1">
                      <stop offset="0%" stopColor="#06b6d4" stopOpacity={0.3} />
                      <stop offset="100%" stopColor="#06b6d4" stopOpacity={0} />
                    </linearGradient>
                  </defs>
                  <XAxis dataKey="time" hide />
                  <YAxis domain={[70, 100]} hide />
                  <Area
                    type="monotone"
                    dataKey="value"
                    stroke="#06b6d4"
                    strokeWidth={2}
                    fill="url(#confGradient)"
                    isAnimationActive={false}
                  />
                </AreaChart>
              </ResponsiveContainer>
            </div>
          </div>
        </div>

        <div className="col-span-3 space-y-6">
          <div className="glass-panel p-6">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-4">Front Camera</h3>
            <div className="relative aspect-video bg-gradient-to-b from-sky-900 via-sky-700 to-gray-800 rounded-xl overflow-hidden">
              <div className="absolute inset-0 bg-gradient-to-b from-sky-900 via-sky-600 to-blue-800" />
              <div className="absolute top-[15%] left-0 right-0 h-px bg-white/20" />
              <svg className="absolute top-[10%] w-full h-[10%]" viewBox="0 0 400 60" preserveAspectRatio="none">
                <polygon points="0,60 0,30 20,40 40,25 60,45 80,35 100,50 120,30 150,45 180,20 220,40 260,25 300,45 350,30 400,50 400,60" fill="rgba(0,0,0,0.3)" />
              </svg>
              <svg className="absolute inset-0 w-full h-full" viewBox="0 0 400 240">
                <defs>
                  <linearGradient id="roadPerspective" x1="0%" y1="0%" x2="0%" y2="100%">
                    <stop offset="0%" stopColor="#2a2a2a" />
                    <stop offset="100%" stopColor="#1a1a1a" />
                  </linearGradient>
                  <linearGradient id="grassGrad" x1="0%" y1="0%" x2="0%" y2="100%">
                    <stop offset="0%" stopColor="#1a3d1a" />
                    <stop offset="100%" stopColor="#0a1f0a" />
                  </linearGradient>
                </defs>
                <polygon points="0,100 0,240 80,240 120,120" fill="url(#grassGrad)" />
                <polygon points="400,100 400,240 320,240 280,120" fill="url(#grassGrad)" />
                <polygon points="120,120 280,120 400,240 0,240" fill="url(#roadPerspective)" />
                <line x1="200" y1="120" x2="200" y2="240" stroke="white" strokeWidth="2" strokeDasharray="20,15" />
                <line x1="160" y1="130" x2="120" y2="240" stroke="white" strokeWidth="1.5" strokeDasharray="10,10" opacity="0.6" />
                <line x1="240" y1="130" x2="280" y2="240" stroke="white" strokeWidth="1.5" strokeDasharray="10,10" opacity="0.6" />
                <line x1="120" y1="120" x2="0" y2="240" stroke="white" strokeWidth="2" />
                <line x1="280" y1="120" x2="400" y2="240" stroke="white" strokeWidth="2" />
                <g opacity="0.4">
                  {[0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180].map((offset, i) => (
                    <rect key={i} x={offset} y="200" width="8" height="4" fill="white" />
                  ))}
                </g>
              </svg>
              {isMonitoring && simulationData.vehicles.map((v, i) => perspectiveVehicle(v, i))}
              {isMonitoring && simulationData.pedestrians.map((p, i) => perspectivePedestrian(p, i))}
              <div className="absolute bottom-2 left-2 right-2 flex justify-between text-[9px] text-white/50 font-mono">
                <span>FOV: 120°</span>
                <span className="text-av-success">{simulationData.sensorStatus?.camera?.fps || 30} FPS</span>
                <span>1080p</span>
              </div>
              <div className="absolute bottom-2 left-1/2 -translate-x-1/2 text-[10px] text-white/40">
                {Math.round(simulationData.egoSpeed * 0.621)} MPH
              </div>
              <div className="absolute top-2 left-2 flex items-center gap-1 px-2 py-1 rounded bg-black/60">
                <div className={`w-2 h-2 rounded-full ${isMonitoring ? 'bg-av-success animate-pulse' : 'bg-av-danger'}`} />
                <span className="text-[10px] text-white/70">CAM FRONT</span>
              </div>
              <div className="absolute top-2 right-2 flex items-center gap-1 px-2 py-1 rounded bg-av-danger/20 border border-av-danger/30">
                <div className="w-2 h-2 rounded-full bg-av-danger animate-pulse" />
                <span className="text-[10px] text-av-danger">REC</span>
              </div>
            </div>
          </div>

          <div className="glass-panel p-6 flex-1">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-4">Detection Log</h3>
            <div className="space-y-2 max-h-[350px] overflow-y-auto">
              {simulationData.detections.slice(0, 15).map((det) => (
                <motion.div
                  key={det.id}
                  initial={{ opacity: 0, x: 20 }}
                  animate={{ opacity: 1, x: 0 }}
                  className={`p-3 rounded-xl text-xs ${det.threat ? 'bg-av-danger/10 border border-av-danger/30' : 'bg-white/5 border border-white/10'}`}
                >
                  <div className="flex items-center justify-between mb-1">
                    <span className={`font-medium capitalize ${det.threat ? 'text-av-danger' : 'text-white'}`}>
                      {det.type === 'vehicle' ? `${det.vehicleType}` : det.type}
                    </span>
                    <span className="text-white/40">{new Date(det.timestamp).toLocaleTimeString()}</span>
                  </div>
                  <div className="flex items-center gap-3 text-white/50">
                    <span>{det.distance}m</span>
                    <span>{det.bearing}°</span>
                    <span className={det.confidence < 80 ? 'text-av-warning' : 'text-av-success'}>
                      {det.confidence}%
                    </span>
                  </div>
                  {det.threat && (
                    <div className="mt-2 flex items-center gap-1 text-av-danger">
                      <svg className="w-3 h-3" viewBox="0 0 24 24" fill="currentColor">
                        <path d="M12 2L1 21h22L12 2zm0 3.5L19.5 19h-15L12 5.5zM11 10v4h2v-4h-2zm0 6v2h2v-2h-2z" />
                      </svg>
                      <span className="capitalize">{det.threat.replace('_', ' ')}</span>
                    </div>
                  )}
                </motion.div>
              ))}
              {simulationData.detections.length === 0 && (
                <div className="text-center text-white/30 py-8">
                  Scanning environment...
                </div>
              )}
            </div>
          </div>
        </div>
      </div>
    </motion.div>
  )
}

