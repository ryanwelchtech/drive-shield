import { useState, useEffect, useRef, useMemo } from 'react'
import { motion, AnimatePresence } from 'framer-motion'
import { ResponsiveContainer, AreaChart, Area, XAxis, YAxis } from 'recharts'
import { prefersReducedMotion } from '../hooks/useOptimizedAnimation'

const reducedMotion = prefersReducedMotion()

const VEHICLE_TYPES = ['sedan', 'suv', 'truck', 'coupe', 'hatchback']
const LANES = [-1, 0, 1]

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
    timestamp: Date.now(),
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
    timestamp: Date.now(),
  }
}

function generateLidarPoints(vehicles, pedestrians) {
  const points = []

  for (let i = 0; i < 300; i++) {
    const angle = (Math.random() - 0.5) * 2.2
    const distance = 8 + Math.random() * 135
    const x = Math.sin(angle) * distance
    const y = Math.cos(angle) * distance
    const z = (Math.random() - 0.5) * 2.5
    points.push({
      x: parseFloat(x.toFixed(2)),
      y: parseFloat(y.toFixed(2)),
      z: parseFloat(z.toFixed(2)),
      intensity: 0.35 + Math.random() * 0.45,
      classification: 'ground',
    })
  }

  vehicles.forEach(v => {
    if (v.y > 8 && v.y < 145) {
      const numPoints = Math.floor(15 + v.y / 8)
      for (let i = 0; i < numPoints; i++) {
        const px = v.x + (Math.random() - 0.5) * v.width * 0.9
        const py = v.y + (Math.random() - 0.5) * v.length * 0.9
        const pz = Math.random() * v.height * 0.7
        points.push({
          x: parseFloat(px.toFixed(2)),
          y: parseFloat(py.toFixed(2)),
          z: parseFloat(pz.toFixed(2)),
          intensity: 0.7 + Math.random() * 0.3,
          classification: 'vehicle',
        })
      }
    }
  })

  pedestrians.forEach(p => {
    if (p.y > -5 && p.y < 115) {
      for (let i = 0; i < 10; i++) {
        const px = p.x + (Math.random() - 0.5) * 0.55
        const py = p.y + Math.random() * 1.65
        const pz = Math.random() * 1.65
        points.push({
          x: parseFloat(px.toFixed(2)),
          y: parseFloat(py.toFixed(2)),
          z: parseFloat(pz.toFixed(2)),
          intensity: 0.75 + Math.random() * 0.25,
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
    if (v.y > 12 && v.y < 145) {
      const distFactor = Math.max(0.55, 1 - (v.y - 12) / 133)
      const confidence = Math.min(98, 86 + distFactor * 12)
      let bearing = Math.atan2(v.x, v.y) * 180 / Math.PI
      bearing = bearing > 180 ? bearing - 360 : bearing < -180 ? bearing + 360 : bearing
      const isThreat = Math.random() > 0.91

      detections.push({
        id: v.id,
        type: 'vehicle',
        vehicleType: v.vehicleType,
        confidence: parseFloat(confidence.toFixed(1)),
        distance: parseFloat(v.y.toFixed(1)),
        bearing: Math.round(bearing),
        velocity: parseFloat(Math.abs(v.speed - egoSpeed).toFixed(1)),
        lane: v.lane,
        threat: isThreat ? ['adversarial_patch', 'phantom_object', 'spoofing'][Math.floor(Math.random() * 3)] : null,
        timestamp: Date.now(),
      })
    }
  })

  pedestrians.forEach(p => {
    if (p.y > 3 && p.y < 115) {
      const distFactor = Math.max(0.45, 1 - p.y / 112)
      const confidence = Math.min(95, 82 + distFactor * 13)
      let bearing = Math.atan2(p.x, p.y) * 180 / Math.PI
      bearing = bearing > 180 ? bearing - 360 : bearing < -180 ? bearing + 360 : bearing
      const isThreat = Math.random() > 0.94

      detections.push({
        id: p.id,
        type: 'pedestrian',
        confidence: parseFloat(confidence.toFixed(1)),
        distance: parseFloat(p.y.toFixed(1)),
        bearing: Math.round(bearing),
        velocity: parseFloat((p.speed * 3.6).toFixed(1)),
        threat: isThreat ? 'phantom_object' : null,
        timestamp: Date.now(),
      })
    }
  })

  return detections.sort((a, b) => b.confidence - a.confidence)
}

function createSensorStatus(egoSpeed, vehicles, pedestrians, detections) {
  const time = Date.now() / 1000
  const baseConfidence = 95.5 + Math.sin(time * 0.4) * 2.5

  return {
    lidar: { status: 'active', confidence: parseFloat((baseConfidence + Math.random() * 1.5).toFixed(1)), fps: 10, range: 200, pointsPerScan: 64000 },
    camera: { status: 'active', confidence: parseFloat((baseConfidence - 1.5 + Math.random() * 2).toFixed(1)), fps: 30, resolution: '1920x1080' },
    radar: { status: 'active', confidence: parseFloat((baseConfidence + 0.8 + Math.random() * 1.2).toFixed(1)), range: 250, updateRate: 20 },
    ultrasonic: { status: 'active', confidence: parseFloat((97.5 + Math.random() * 1.5).toFixed(1)), range: 8, sensors: 12 },
    gps: { status: 'active', accuracy: parseFloat((1.1 + Math.random() * 0.8).toFixed(1)), satellites: 12, hdop: parseFloat((0.65 + Math.random() * 0.35).toFixed(2)) },
    fusion: { status: 'active', confidence: parseFloat((baseConfidence - 0.5).toFixed(1)), latency: 42, tracking: vehicles.length + pedestrians.length },
    egoSpeed: parseFloat(egoSpeed.toFixed(0)),
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

  const canvasRef = useRef(null)
  const animationRef = useRef(null)
  const stateRef = useRef({
    vehicles: [createVehicle(65, -1), createVehicle(65, 0), createVehicle(65, 1)],
    pedestrians: [],
    egoSpeed: 65,
    lidarPoints: [],
    detections: [],
    sensorStatus: {},
    lastLidarTime: 0,
    lastDetectionTime: 0,
    lastStatusTime: 0,
    lastConfidenceTime: 0,
  })

  useEffect(() => {
    if (!isMonitoring) return

    let lastTime = Date.now()

    const animate = () => {
      const now = Date.now()
      const dt = Math.min((now - lastTime) / 1000, 0.08)
      lastTime = now
      const state = stateRef.current

      state.egoSpeed = 58 + Math.sin(now / 2800) * 18

      state.vehicles = state.vehicles.map(v => {
        const relSpeed = (v.speed - state.egoSpeed) * dt * 0.025
        let newY = v.y + relSpeed

        if (newY > 145 || newY < 10) {
          return {
            ...v,
            y: 45 + Math.random() * 75,
            speed: state.egoSpeed + (Math.random() - 0.5) * 22,
            lane: LANES[Math.floor(Math.random() * LANES.length)],
            vehicleType: VEHICLE_TYPES[Math.floor(Math.random() * VEHICLE_TYPES.length)],
            timestamp: now,
          }
        }
        return { ...v, y: newY, timestamp: now }
      })

      if (state.vehicles.length < 4 && Math.random() < 0.018) {
        const newVehicle = createVehicle(state.egoSpeed)
        const laneClear = !state.vehicles.some(v => Math.abs(v.y - newVehicle.y) < 22 && v.lane === newVehicle.lane)
        if (laneClear) {
          state.vehicles.push({ ...newVehicle, timestamp: now })
        }
      }

      state.pedestrians = state.pedestrians.map(p => {
        let newX = p.x, newY = p.y, newState = p.state, newDirection = p.direction, newSpeed = p.speed

        if (p.state === 'approaching') {
          newX += p.speed * dt * 2.8
          if (newX > -4) { newState = 'crossing'; newDirection = Math.random() > 0.35 ? 'crossing' : 'walking' }
        } else if (p.state === 'crossing') {
          newY += p.speed * dt * 2.8
          newX += (p.direction === 'right' ? 0.9 : -0.35) * p.speed * dt * 1.8
          if (newY > 105 || Math.abs(newX) > 22) return null
          if (Math.random() < 0.012) newDirection = Math.random() > 0.5 ? 'left' : 'right'
        } else if (p.state === 'walking') {
          newY += p.speed * dt * 1.8
          newX += (p.direction === 'right' ? 0.9 : -0.9) * p.speed * dt
          if (newY > 102 || Math.abs(newX) > 28) return null
        }

        return { ...p, x: newX, y: newY, state: newState, direction: newDirection, speed: newSpeed, timestamp: now }
      }).filter(Boolean)

      if (state.pedestrians.length < 5 && Math.random() < 0.007) {
        state.pedestrians.push({ ...createPedestrian(), timestamp: now })
      }

      if (now - state.lastLidarTime > 80) {
        state.lidarPoints = generateLidarPoints(state.vehicles, state.pedestrians)
        state.lastLidarTime = now
      }

      if (now - state.lastDetectionTime > 180) {
        state.detections = generateDetections(state.vehicles, state.pedestrians, state.egoSpeed)
        state.lastDetectionTime = now
      }

      if (now - state.lastStatusTime > 450) {
        state.sensorStatus = createSensorStatus(state.egoSpeed, state.vehicles, state.pedestrians, state.detections)
        state.lastStatusTime = now
      }

      if (now - state.lastConfidenceTime > 90) {
        const avgConf = state.detections.length > 0 ? state.detections.reduce((sum, d) => sum + d.confidence, 0) / state.detections.length : 96.2
        setConfidenceHistory(prev => {
          const newPoint = { time: prev.length, value: avgConf }
          return [...prev.slice(-59), newPoint]
        })
        state.lastConfidenceTime = now
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
      const centerY = height * 0.65

      ctx.fillStyle = 'rgba(0, 8, 12, 0.18)'
      ctx.fillRect(0, 0, width, height)

      ctx.strokeStyle = 'rgba(6, 182, 212, 0.1)'
      ctx.lineWidth = 0.5
      ctx.beginPath()
      for (let r = 25; r <= 150; r += 25) {
        ctx.moveTo(centerX + r, centerY)
        ctx.arc(centerX, centerY, r, 0, Math.PI * 2)
      }
      ctx.stroke()

      ctx.strokeStyle = 'rgba(6, 182, 212, 0.07)'
      ctx.beginPath()
      ctx.moveTo(centerX, 0)
      ctx.lineTo(centerX, height)
      ctx.moveTo(0, centerY)
      ctx.lineTo(width, centerY)
      ctx.stroke()

      ctx.fillStyle = 'rgba(6, 182, 212, 0.35)'
      ctx.beginPath()
      lidarPoints.filter(p => p.classification === 'ground').forEach(p => {
        const x = centerX + p.x * 1.1
        const y = centerY - p.y * 1.1
        if (x > 5 && x < width - 5 && y > 5 && y < height - 5) {
          const size = 0.6 + p.intensity * 0.9
          ctx.moveTo(x + size, y)
          ctx.arc(x, y, size, 0, Math.PI * 2)
        }
      })
      ctx.fill()

      ctx.fillStyle = 'rgba(34, 197, 94, 0.55)'
      ctx.beginPath()
      vehicles.forEach(v => {
        if (v.y > 8 && v.y < 145) {
          const x = centerX + v.x * 1.1
          const y = centerY - v.y * 1.1
          if (x > -45 && x < width + 45 && y > -45 && y < height + 45) {
            for (let i = 0; i < 7; i++) {
              const ox = (Math.random() - 0.5) * v.width * 0.85
              const oy = (Math.random() - 0.5) * v.length * 0.85
              const px = x + ox * 1.1
              const py = y - oy * 1.1
              ctx.moveTo(px + 0.7, py)
              ctx.arc(px, py, 0.7, 0, Math.PI * 2)
            }
          }
        }
      })
      ctx.fill()

      ctx.fillStyle = 'rgba(245, 158, 11, 0.65)'
      ctx.beginPath()
      pedestrians.forEach(p => {
        if (p.y > -3 && p.y < 112) {
          const x = centerX + p.x * 1.1
          const y = centerY - p.y * 1.1
          if (x > -25 && x < width + 25 && y > -25 && y < height + 25) {
            for (let i = 0; i < 4; i++) {
              const ox = (Math.random() - 0.5) * 0.5
              const oy = Math.random() * 1.6
              const px = x + ox * 1.1
              const py = y - oy * 1.1
              ctx.moveTo(px + 0.45, py)
              ctx.arc(px, py, 0.45, 0, Math.PI * 2)
            }
          }
        }
      })
      ctx.fill()

      ctx.fillStyle = 'rgba(6, 182, 212, 0.6)'
      ctx.beginPath()
      for (let i = 0; i < 18; i++) {
        const angle = (i / 18) * Math.PI * 2
        const dist = 2.5 + Math.random() * 4.5
        const x = centerX + Math.cos(angle) * dist
        const y = centerY + Math.sin(angle) * dist
        ctx.moveTo(x + 0.55, y)
        ctx.arc(x, y, 0.55, 0, Math.PI * 2)
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
    if (simulationData.detections.length === 0) return 96.2
    return simulationData.detections.reduce((sum, d) => sum + d.confidence, 0) / simulationData.detections.length
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

  const perspectiveVehicle = (v) => {
    const scale = Math.max(0.07, Math.min(1.3, 75 / v.y))
    const perspectiveY = 138 + (v.y / 155) * 100
    const perspectiveX = 200 + v.x * (1 + v.y / 190) * 11
    const vehicleWidth = (v.width * 2.4) * scale
    const vehicleHeight = (v.height * 2.4) * scale
    const isAhead = v.y < 75

    const colors = { truck: '#4a5568', suv: '#2d3748', sedan: '#718096', coupe: '#a0aec0', hatchback: '#cbd5e0' }

    return (
      <motion.div key={v.id} className="absolute" style={{ left: `${(perspectiveX / 400) * 100}%`, top: `${(perspectiveY / 240) * 100}%`, width: vehicleWidth, height: vehicleHeight }} initial={{ opacity: 0 }} animate={{ opacity: 1 }} transition={{ duration: 0.2 }}>
        <div className="w-full h-full rounded-lg shadow-lg" style={{ backgroundColor: colors[v.vehicleType] || colors.sedan }}>
          <div className="absolute top-[7%] left-[11%] right-[11%] h-[20%] bg-slate-800 rounded-t-sm" />
          <div className="absolute bottom-[4%] left-[13%] right-[13%] h-[14%] bg-slate-800 rounded-b-sm" />
          {isAhead ? (
            <div className="absolute top-[7%] left-[4%] right-[4%] flex justify-between">
              <div className="w-1.5 h-1 bg-yellow-400 rounded-full" />
              <div className="w-1.5 h-1 bg-yellow-400 rounded-full" />
            </div>
          ) : (
            <div className="absolute bottom-[4%] left-[9%] right-[9%] flex justify-between">
              <div className="w-1.5 h-1 bg-red-600 rounded-full" />
              <div className="w-1.5 h-1 bg-red-600 rounded-full" />
            </div>
          )}
        </div>
        <div className="absolute -bottom-2.5 left-1/2 -translate-x-1/2 text-[6px] text-white/45 whitespace-nowrap">{v.y.toFixed(0)}m</div>
      </motion.div>
    )
  }

  const perspectivePedestrian = (p) => {
    const scale = Math.max(0.1, Math.min(1.1, 65 / Math.max(8, p.y)))
    const perspectiveY = 142 + (p.y / 115) * 86
    const perspectiveX = 200 + p.x * (1 + p.y / 145) * 11
    const personWidth = 7 * scale
    const personHeight = 16 * scale

    return (
      <motion.div key={p.id} className="absolute" style={{ left: `${(perspectiveX / 400) * 100}%`, top: `${(perspectiveY / 240) * 100}%`, width: personWidth, height: personHeight }} animate={p.state === 'approaching' ? { y: [0, -1.2, 0] } : {}} transition={p.state === 'approaching' ? { duration: 0.35, repeat: Infinity } : {}}>
        <div className="w-full h-full relative">
          <div className="absolute top-0 left-1/2 -translate-x-1/2 w-[48%] h-[20%] bg-amber-200 rounded-full" />
          <div className="absolute top-[18%] left-[24%] w-[48%] h-[38%] bg-blue-600 rounded-sm" />
          <div className="absolute bottom-0 left-[28%] w-[16%] h-[36%] bg-black rounded-sm" />
          <div className="absolute bottom-0 right-[28%] w-[16%] h-[36%] bg-black rounded-sm" />
        </div>
      </motion.div>
    )
  }

  return (
    <div className="min-h-screen bg-black road-grid p-6">
      <header className="flex items-center justify-between mb-6">
        <div className="flex items-center gap-4">
          <motion.button onClick={onBack} className="w-10 h-10 rounded-xl bg-white/5 border border-white/10 flex items-center justify-center hover:bg-white/10 transition-colors" whileHover={{ scale: 1.05 }} whileTap={{ scale: 0.95 }}>
            <svg className="w-5 h-5" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2"><path d="M19 12H5M12 19l-7-7 7-7" /></svg>
          </motion.button>
          <div>
            <h1 className="text-2xl font-bold"><span className="gradient-text">DriveShield</span> Monitoring Center</h1>
            <div className="flex items-center gap-4 text-xs text-white/50">
              <span className="flex items-center gap-1"><span className="w-1.5 h-1.5 rounded-full bg-av-success animate-pulse" />{Math.round(simulationData.egoSpeed * 0.621)} mph</span>
              <span>Fusion: {simulationData.sensorStatus?.fusion?.confidence?.toFixed(0) || 95}%</span>
              <span className="capitalize">{simulationData.sensorStatus?.environment?.ambientLight || 'day'} mode</span>
            </div>
          </div>
        </div>
        <div className="flex items-center gap-4">
          <div className={`px-4 py-2 rounded-xl border ${getThreatBg(threatLevel)}`}>
            <span className="text-xs text-white/50 mr-2">Threat:</span>
            <span className={`font-bold ${getThreatColor(threatLevel)}`}>{threatLevel}</span>
          </div>
          <motion.button onClick={() => setIsMonitoring(!isMonitoring)} className={`glass-button flex items-center gap-2 ${isMonitoring ? 'border-av-success/50' : 'border-av-danger/50'}`} whileHover={{ scale: 1.02 }} whileTap={{ scale: 0.98 }}>
            {isMonitoring ? (<><span className="w-2 h-2 rounded-full bg-av-success animate-pulse" /><span>Monitoring Active</span></>) : (<><span className="w-2 h-2 rounded-full bg-av-danger" /><span>Monitoring Paused</span></>)}
          </motion.button>
        </div>
      </header>

      <div className="grid grid-cols-12 gap-6">
        <div className="col-span-3 space-y-5">
          <div className="glass-panel p-5">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-3">Sensor Status</h3>
            <div className="space-y-2.5">
              {['lidar', 'camera', 'radar', 'ultrasonic', 'gps'].map((sensor) => {
                const data = simulationData.sensorStatus?.[sensor] || { status: 'inactive', confidence: 0 }
                return (
                  <div key={sensor} className="p-2.5 rounded-lg bg-white/4">
                    <div className="flex items-center justify-between mb-1.5">
                      <div className="flex items-center gap-2.5"><div className={`w-2 h-2 rounded-full ${data.status === 'active' ? 'bg-av-success' : 'bg-av-danger'} ${data.status === 'active' && !reducedMotion ? 'animate-pulse' : ''}`} /><span className="text-xs text-white capitalize">{sensor}</span></div>
                      <span className="text-xs font-mono text-av-primary">{data.confidence?.toFixed(1)}%</span>
                    </div>
                    <div className="h-1 bg-white/8 rounded-full overflow-hidden"><div className="h-full bg-gradient-to-r from-av-primary to-av-accent rounded-full transition-all duration-300" style={{ width: `${data.confidence || 0}%` }} /></div>
                  </div>
                )
              })}
              {simulationData.sensorStatus?.fusion && (
                <div className="p-2.5 rounded-lg bg-av-primary/8 border border-av-primary/25">
                  <div className="flex items-center justify-between mb-1"><span className="text-xs text-av-primary uppercase">Sensor Fusion</span><span className="text-xs font-mono text-av-primary">{simulationData.sensorStatus.fusion.confidence?.toFixed(1)}%</span></div>
                  <div className="flex gap-3 text-[9px] text-white/35"><span>Latency: {simulationData.sensorStatus.fusion.latency}ms</span><span>Tracking: {simulationData.sensorStatus.fusion.tracking} objects</span></div>
                </div>
              )}
            </div>
          </div>

          <div className="glass-panel p-5">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-3">Detection Stats</h3>
            <div className="grid grid-cols-2 gap-3">
              <div className="p-3 rounded-lg bg-white/4 text-center"><p className="text-xl font-bold text-av-primary">{Math.round(simulationData.egoSpeed * 0.621)}<span className="text-xs text-white/25 ml-1">mph</span></p><p className="text-[10px] text-white/45">Ego Speed</p></div>
              <div className="p-3 rounded-lg bg-white/4 text-center"><p className="text-xl font-bold text-av-danger">{threatCounts.total}</p><p className="text-[10px] text-white/45">Threats</p></div>
              <div className="p-3 rounded-lg bg-white/4 text-center"><p className="text-xl font-bold text-av-success">{detectionStats.vehicles}</p><p className="text-[10px] text-white/45">Vehicles</p></div>
              <div className="p-3 rounded-lg bg-white/4 text-center"><p className="text-xl font-bold text-av-warning">{detectionStats.pedestrians}</p><p className="text-[10px] text-white/45">Pedestrians</p></div>
            </div>
          </div>

          <div className="glass-panel p-5">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-3">Active Threats</h3>
            <div className="space-y-2">
              {[
                { name: 'Adversarial Patches', count: threatCounts.adversarialPatch, color: 'bg-av-danger' },
                { name: 'Phantom Objects', count: threatCounts.phantomObject, color: 'bg-av-warning' },
                { name: 'Sensor Spoofing', count: threatCounts.spoofing, color: 'bg-av-purple' },
              ].map((threat) => (
                <div key={threat.name} className="flex items-center justify-between p-2.5 rounded-lg bg-white/4">
                  <div className="flex items-center gap-2"><div className={`w-1.5 h-1.5 rounded-full ${threat.color}`} /><span className="text-xs text-white/60">{threat.name}</span></div>
                  <span className="text-sm font-bold text-white">{threat.count}</span>
                </div>
              ))}
            </div>
          </div>
        </div>

        <div className="col-span-6 space-y-5">
          <div className="glass-panel p-5">
            <div className="flex items-center justify-between mb-3">
              <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider">LIDAR Point Cloud</h3>
              <div className="flex items-center gap-2"><span className={`w-2 h-2 rounded-full ${isMonitoring ? 'bg-av-success' : 'bg-av-danger'} ${isMonitoring && !reducedMotion ? 'animate-pulse' : ''}`} /><span className="text-xs text-white/45">{isMonitoring ? 'Live Feed' : 'Paused'}</span></div>
            </div>
            <div className="relative">
              <canvas ref={canvasRef} width={600} height={340} className="w-full rounded-lg bg-black/45" />
              <div className="absolute top-3 left-3 flex items-center gap-2 px-2.5 py-1 rounded-full bg-black/55 border border-white/8">
                <span className="text-xs text-white/40">Range:</span><span className="text-xs text-av-primary font-mono">150m</span>
              </div>
              <div className="absolute bottom-3 right-3 flex gap-4 text-xs">
                <div className="flex items-center gap-1.5"><div className="w-2.5 h-2.5 rounded-full bg-av-primary" /><span className="text-white/40">Ground</span></div>
                <div className="flex items-center gap-1.5"><div className="w-2.5 h-2.5 rounded-full bg-av-success" /><span className="text-white/40">Vehicle</span></div>
                <div className="flex items-center gap-1.5"><div className="w-2.5 h-2.5 rounded-full bg-av-warning" /><span className="text-white/40">Pedestrian</span></div>
              </div>
            </div>
          </div>

          <div className="glass-panel p-5">
            <div className="flex items-center justify-between mb-3">
              <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider">Object Detection Confidence</h3>
              <span className={`text-xl font-bold ${currentConfidence < 85 ? 'text-av-warning' : 'text-av-success'}`}>{currentConfidence.toFixed(1)}%</span>
            </div>
            <div className="h-[140px]">
              <ResponsiveContainer width="100%" height="100%">
                <AreaChart data={confidenceHistory}>
                  <defs><linearGradient id="confGradient" x1="0" y1="0" x2="0" y2="1"><stop offset="0%" stopColor="#06b6d4" stopOpacity={0.35} /><stop offset="100%" stopColor="#06b6d4" stopOpacity={0} /></linearGradient></defs>
                  <XAxis dataKey="time" hide />
                  <YAxis domain={[70, 100]} hide />
                  <Area type="monotone" dataKey="value" stroke="#06b6d4" strokeWidth={1.5} fill="url(#confGradient)" isAnimationActive={false} />
                </AreaChart>
              </ResponsiveContainer>
            </div>
          </div>
        </div>

        <div className="col-span-3 space-y-5">
          <div className="glass-panel p-5">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-3">Front Camera</h3>
            <div className="relative aspect-video bg-gradient-to-b from-sky-925 via-sky-700 to-gray-825 rounded-lg overflow-hidden">
              <div className="absolute inset-0 bg-gradient-to-b from-sky-925 via-sky-575 to-blue-825" />
              <div className="absolute top-[14%] left-0 right-0 h-px bg-white/18" />
              <svg className="absolute top-[9%] w-full h-[9%]" viewBox="0 0 400 55" preserveAspectRatio="none"><polygon points="0,55 0,28 18,38 38,22 55,42 75,32 92,48 110,28 138,42 168,18 205,38 242,22 278,42 325,28 372,48 400,38 400,55" fill="rgba(0,0,0,0.28)" /></svg>
              <svg className="absolute inset-0 w-full h-full" viewBox="0 0 400 235">
                <defs>
                  <linearGradient id="roadGrad" x1="0%" y1="0%" x2="0%" y2="100%"><stop offset="0%" stopColor="#252525" /><stop offset="100%" stopColor="#181818" /></linearGradient>
                  <linearGradient id="grassGrad" x1="0%" y1="0%" x2="0%" y2="100%"><stop offset="0%" stopColor="#162618" /><stop offset="100%" stopColor="#0a180a" /></linearGradient>
                </defs>
                <polygon points="0,95 0,235 75,235 118,118" fill="url(#grassGrad)" />
                <polygon points="400,95 400,235 325,235 282,118" fill="url(#grassGrad)" />
                <polygon points="118,118 282,118 400,235 0,235" fill="url(#roadGrad)" />
                <line x1="200" y1="118" x2="200" y2="235" stroke="white" strokeWidth="1.8" strokeDasharray="18,13" />
                <line x1="158" y1="128" x2="118" y2="235" stroke="white" strokeWidth="1.3" strokeDasharray="9,8" opacity="0.55" />
                <line x1="242" y1="128" x2="282" y2="235" stroke="white" strokeWidth="1.3" strokeDasharray="9,8" opacity="0.55" />
                <line x1="118" y1="118" x2="0" y2="235" stroke="white" strokeWidth="1.8" />
                <line x1="282" y1="118" x2="400" y2="235" stroke="white" strokeWidth="1.8" />
              </svg>
              {isMonitoring && simulationData.vehicles.map(v => perspectiveVehicle(v))}
              {isMonitoring && simulationData.pedestrians.map(p => perspectivePedestrian(p))}
              <div className="absolute bottom-1.5 left-2 right-2 flex justify-between text-[8px] text-white/40 font-mono">
                <span>FOV: 120°</span><span className="text-av-success">{simulationData.sensorStatus?.camera?.fps || 30} FPS</span><span>1080p</span>
              </div>
              <div className="absolute bottom-1.5 left-1/2 -translate-x-1/2 text-[9px] text-white/35">{Math.round(simulationData.egoSpeed * 0.621)} MPH</div>
              <div className="absolute top-2 left-2 flex items-center gap-1.5 px-2 py-1 rounded bg-black/55">
                <div className={`w-1.5 h-1.5 rounded-full ${isMonitoring ? 'bg-av-success' : 'bg-av-danger'} ${isMonitoring && !reducedMotion ? 'animate-pulse' : ''}`} /><span className="text-[9px] text-white/65">CAM FRONT</span>
              </div>
              <div className="absolute top-2 right-2 flex items-center gap-1.5 px-2 py-1 rounded bg-av-danger/18 border border-av-danger/35">
                <div className="w-1.5 h-1.5 rounded-full bg-av-danger animate-pulse" /><span className="text-[9px] text-av-danger">REC</span>
              </div>
            </div>
          </div>

          <div className="glass-panel p-5 flex-1">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-3">Detection Log</h3>
            <div className="space-y-1.5 max-h-[320px] overflow-y-auto pr-1">
              <AnimatePresence mode="popLayout">
                {simulationData.detections.slice(0, 12).map((det) => (
                  <motion.div key={det.id} initial={{ opacity: 0, x: 15 }} animate={{ opacity: 1, x: 0 }} exit={{ opacity: 0, x: -15 }} transition={{ duration: 0.15 }} className={`p-2.5 rounded-lg text-xs ${det.threat ? 'bg-av-danger/12 border border-av-danger/35' : 'bg-white/5 border border-white/10'}`}>
                    <div className="flex items-center justify-between mb-1">
                      <span className={`font-medium capitalize ${det.threat ? 'text-av-danger' : 'text-white'}`}>{det.type === 'vehicle' ? det.vehicleType : det.type}</span>
                      <span className="text-white/35">{new Date(det.timestamp).toLocaleTimeString()}</span>
                    </div>
                    <div className="flex items-center gap-3 text-white/45">
                      <span>{det.distance}m</span><span>{det.bearing}°</span><span className={det.confidence < 80 ? 'text-av-warning' : 'text-av-success'}>{det.confidence}%</span>
                    </div>
                    {det.threat && (
                      <div className="mt-1.5 flex items-center gap-1 text-av-danger">
                        <svg className="w-2.5 h-2.5" viewBox="0 0 24 24" fill="currentColor"><path d="M12 2L1 21h22L12 2zm0 3.5L19.5 19h-15L12 5.5zM11 8v5h2V8h-2zm0 7v3h2v-3h-2z" /></svg>
                        <span className="capitalize text-[10px]">{det.threat.replace('_', ' ')}</span>
                      </div>
                    )}
                  </motion.div>
                ))}
              </AnimatePresence>
              {simulationData.detections.length === 0 && (
                <div className="text-center text-white/25 py-6 text-xs">Waiting for detections...</div>
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}
