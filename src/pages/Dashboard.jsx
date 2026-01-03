import { useState, useEffect, useRef, useCallback, useMemo } from 'react'
import { motion } from 'framer-motion'
import { ResponsiveContainer, AreaChart, Area, XAxis, YAxis } from 'recharts'
import dataService from '../data/dataService'
import { prefersReducedMotion } from '../hooks/useOptimizedAnimation'

// Apple UX: Check reduced motion preference once
const reducedMotion = prefersReducedMotion()

const Dashboard = ({ onBack }) => {
  const [isMonitoring, setIsMonitoring] = useState(true)
  const [detections, setDetections] = useState([])
  const [confidenceHistory, setConfidenceHistory] = useState([])
  const [currentConfidence, setCurrentConfidence] = useState(96.8)
  const [threatLevel, setThreatLevel] = useState('LOW')
  const [lidarData, setLidarData] = useState(null)
  const [sensorStatus, setSensorStatus] = useState({
    lidar: { status: 'active', confidence: 98.2 },
    camera: { status: 'active', confidence: 94.5 },
    radar: { status: 'active', confidence: 97.1 },
    ultrasonic: { status: 'active', confidence: 99.0 },
  })

  const [cameraVehicles, setCameraVehicles] = useState([
    { id: 1, lane: -1, distance: 60, speed: 55, type: 'sedan', width: 45, height: 25 },
    { id: 2, lane: 0, distance: 100, speed: 65, type: 'suv', width: 50, height: 28 },
    { id: 3, lane: 1, distance: 40, speed: 58, type: 'truck', width: 55, height: 35 },
  ])
  const [cameraPedestrians, setCameraPedestrians] = useState([
    { id: 101, x: -120, y: 50, speedX: 0.8, speedY: 0, state: 'walking' },
  ])

  const canvasRef = useRef(null)
  const streamRef = useRef(null)
  const lastUpdateRef = useRef(Date.now())
  const pointsRef = useRef([])
  const animationRef = useRef(null)

  // Memoized threat calculations to prevent unnecessary re-renders
  const threatCounts = useMemo(() => ({
    adversarialPatch: detections.filter(d => d.threat === 'adversarial_patch').length,
    phantomObject: detections.filter(d => d.threat === 'phantom_object').length,
    spoofing: detections.filter(d => d.threat === 'spoofing').length,
    total: detections.filter(d => d.threat).length,
  }), [detections])

  const detectionStats = useMemo(() => ({
    total: detections.length,
    vehicles: detections.filter(d => d.type === 'vehicle').length,
    pedestrians: detections.filter(d => d.type === 'pedestrian').length,
  }), [detections])

  // Apple UX: Throttled confidence update (16ms = 60fps)
  const updateConfidence = useCallback((newValue) => {
    const now = Date.now()
    if (now - lastUpdateRef.current < 16) return
    lastUpdateRef.current = now

    setCurrentConfidence(newValue)
    setConfidenceHistory(prev => {
      const newPoint = { time: prev.length, value: newValue }
      return [...prev, newPoint].slice(-60)
    })
  }, [])

  // Use data service stream for real-time updates
  useEffect(() => {
    if (!isMonitoring) {
      if (streamRef.current) streamRef.current.close()
      return
    }

    streamRef.current = dataService.createSensorStream(
      (message) => {
        switch (message.type) {
          case 'detection':
            setDetections(prev => [message.data, ...prev.slice(0, 19)])
            if (message.data.threat) {
              setThreatLevel(message.data.threat === 'adversarial_patch' ? 'CRITICAL' : 'HIGH')
              setTimeout(() => setThreatLevel('LOW'), 5000)
            }
            break
          case 'status':
            setSensorStatus(message.data)
            break
          case 'confidence':
            updateConfidence(message.data.value)
            break
          case 'lidar':
            if (message.data.points) {
              setLidarData(message.data)
              pointsRef.current = message.data.points.map(p => ({
                angle: Math.atan2(p.y, p.x),
                distance: Math.sqrt(p.x * p.x + p.y * p.y),
                z: p.z,
                intensity: p.intensity,
              }))
            }
            break
        }
      },
      (error) => console.error('Stream error:', error)
    )

    return () => {
      if (streamRef.current) streamRef.current.close()
    }
  }, [isMonitoring, updateConfidence])

  // Camera scene animation - vehicles in lanes, pedestrians crossing
  useEffect(() => {
    if (!isMonitoring) return

    let lastTime = Date.now()

    const animate = () => {
      const now = Date.now()
      const dt = (now - lastTime) / 1000
      lastTime = now

      setCameraVehicles(prev => prev.map(v => {
        const newDistance = v.distance + (v.speed - (sensorStatus?.egoSpeed || 65)) * dt * 0.05
        if (newDistance > 150 || newDistance < 15) {
          return {
            ...v,
            distance: 100 + Math.random() * 50,
            speed: (sensorStatus?.egoSpeed || 65) + (Math.random() - 0.5) * 20,
            type: ['sedan', 'suv', 'truck', 'coupe'][Math.floor(Math.random() * 4)]
          }
        }
        return { ...v, distance: newDistance }
      }))

      setCameraPedestrians(prev => prev.map(p => {
        let newX = p.x + p.speedX * dt * 40
        let newY = p.y + p.speedY * dt * 40

        if (p.state === 'walking') {
          if (newX > -20) {
            return { ...p, x: newX, state: 'crossing', speedX: 0.5, speedY: 0.3 }
          }
        } else if (p.state === 'crossing') {
          if (newY > 80 || newX > 80) {
            return null
          }
          if (Math.abs(newX) < 30 && Math.random() < 0.02) {
            return { ...p, x: newX, y: newY, speedX: 0.3, speedY: -0.2 }
          }
        }
        return { ...p, x: newX, y: newY }
      }).filter(Boolean))

      if (Math.random() < 0.015 && prev.length < 5) {
        setCameraPedestrians(prev => [...prev, {
          id: Date.now(),
          x: -150 - Math.random() * 50,
          y: -20,
          speedX: 0.8 + Math.random() * 0.3,
          speedY: 0,
          state: 'walking'
        }])
      }

      animationRef.current = requestAnimationFrame(animate)
    }

    animationRef.current = requestAnimationFrame(animate)

    return () => {
      if (animationRef.current) cancelAnimationFrame(animationRef.current)
    }
  }, [isMonitoring, sensorStatus?.egoSpeed])

  // LIDAR Point Cloud Visualization - GPU-accelerated Canvas
  useEffect(() => {
    if (!canvasRef.current) return

    const canvas = canvasRef.current
    const ctx = canvas.getContext('2d')
    const width = canvas.width
    const height = canvas.height

    let animationId

    const centerX = width / 2
    const centerY = height / 2

    const render = () => {
      const points = pointsRef.current
      const lidarPoints = lidarData?.points || []

      ctx.fillStyle = 'rgba(0, 0, 0, 0.12)'
      ctx.fillRect(0, 0, width, height)

      ctx.strokeStyle = 'rgba(6, 182, 212, 0.15)'
      ctx.lineWidth = 1
      ctx.beginPath()
      for (let r = 25; r <= 150; r += 25) {
        ctx.moveTo(centerX + r, centerY)
        ctx.arc(centerX, centerY, r, 0, Math.PI * 2)
      }
      ctx.stroke()

      ctx.strokeStyle = 'rgba(6, 182, 212, 0.12)'
      ctx.beginPath()
      ctx.moveTo(centerX, 0)
      ctx.lineTo(centerX, height)
      ctx.moveTo(0, centerY)
      ctx.lineTo(width, centerY)
      ctx.stroke()

      ctx.fillStyle = 'rgba(6, 182, 212, 0.5)'
      ctx.beginPath()
      lidarPoints.forEach((point) => {
        const x = centerX + point.x * 1.5
        const y = centerY - point.y * 1.5
        const size = 0.5 + point.intensity * 1.2
        ctx.moveTo(x + size, y)
        ctx.arc(x, y, size, 0, Math.PI * 2)
      })
      ctx.fill()

      ctx.fillStyle = 'rgba(34, 197, 94, 0.7)'
      ctx.beginPath()
      cameraVehicles.forEach(v => {
        if (v.distance < 150 && v.distance > 5) {
          const scale = 0.8 + v.distance / 200
          for (let i = 0; i < 15; i++) {
            const ox = v.lane * 2 + (Math.random() - 0.5) * v.width * 0.3
            const oy = v.distance + (Math.random() - 0.5) * v.height * 0.3
            const oz = Math.random() * v.height * 0.8
            const x = centerX + ox * 1.5
            const y = centerY - oy * 1.5
            ctx.moveTo(x + 1, y)
            ctx.arc(x, y, 1.2, 0, Math.PI * 2)
          }
        }
      })
      ctx.fill()

      ctx.fillStyle = 'rgba(245, 158, 11, 0.8)'
      ctx.beginPath()
      cameraPedestrians.forEach(p => {
        if (p.y > -30 && p.y < 120) {
          for (let i = 0; i < 6; i++) {
            const ox = p.x * 0.1 + (Math.random() - 0.5) * 0.8
            const oy = p.y + Math.random() * 1.8
            const oz = Math.random() * 1.7
            const x = centerX + ox * 1.5
            const y = centerY - oy * 1.5
            ctx.moveTo(x + 0.8, y)
            ctx.arc(x, y, 0.8, 0, Math.PI * 2)
          }
        }
      })
      ctx.fill()

      ctx.fillStyle = 'rgba(6, 182, 212, 0.8)'
      ctx.beginPath()
      for (let i = 0; i < 30; i++) {
        const angle = (i / 30) * Math.PI * 2
        const dist = 3 + Math.random() * 8
        const x = centerX + Math.cos(angle) * dist
        const y = centerY + Math.sin(angle) * dist
        ctx.moveTo(x + 1, y)
        ctx.arc(x, y, 1, 0, Math.PI * 2)
      }
      ctx.fill()

      if (isMonitoring) {
        ctx.fillStyle = 'rgba(239, 68, 68, 0.9)'
        ctx.beginPath()
        points.forEach((point) => {
          if (point.intensity > 0.8 && Math.random() > 0.98) {
            const x = centerX + Math.cos(point.angle) * point.distance
            const y = centerY + Math.sin(point.angle) * point.distance
            ctx.moveTo(x + 3, y)
            ctx.arc(x, y, 3, 0, Math.PI * 2)
          }
        })
        ctx.fill()
      }

      if (isMonitoring && !reducedMotion) {
        animationId = requestAnimationFrame(render)
      }
    }

    render()

    return () => {
      if (animationId) cancelAnimationFrame(animationId)
    }
  }, [isMonitoring, reducedMotion])

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

  return (
    <motion.div
      className="min-h-screen bg-black road-grid p-6"
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      exit={{ opacity: 0 }}
    >
      {/* Header */}
      <header className="flex items-center justify-between mb-6">
        <div className="flex items-center gap-4">
          <motion.button
            onClick={onBack}
            className="w-10 h-10 rounded-xl bg-white/5 border border-white/10 flex items-center justify-center hover:bg-white/10 transition-colors"
            whileHover={{ scale: 1.05 }}
            whileTap={{ scale: 0.95 }}
          >
            <svg className="w-5 h-5" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
              <path d="M19 12H5M12 19l-7-7 7-7"/>
            </svg>
          </motion.button>
          <div>
            <h1 className="text-2xl font-bold">
              <span className="gradient-text">DriveShield</span> Monitoring Center
            </h1>
            <div className="flex items-center gap-4 text-xs text-white/50">
              <span className="flex items-center gap-1">
                <span className="w-1.5 h-1.5 rounded-full bg-av-success animate-pulse"></span>
                {sensorStatus?.egoSpeed ? Math.round(sensorStatus.egoSpeed * 0.621) : 40} mph
              </span>
              <span>Fusion: {sensorStatus?.fusion?.confidence?.toFixed(0) || 96}%</span>
              <span className="capitalize">{sensorStatus?.environment?.ambientLight || 'day'} mode</span>
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
                <span className="w-2 h-2 rounded-full bg-av-success animate-pulse"></span>
                Monitoring Active
              </>
            ) : (
              <>
                <span className="w-2 h-2 rounded-full bg-av-danger"></span>
                Monitoring Paused
              </>
            )}
          </motion.button>
        </div>
      </header>

      {/* Main Grid */}
      <div className="grid grid-cols-12 gap-6">
        {/* Left Column - Sensor Status */}
        <div className="col-span-3 space-y-6">
          {/* Sensor Health */}
          <div className="glass-panel p-6">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-4">Sensor Status</h3>
            <div className="space-y-3">
              {['lidar', 'camera', 'radar', 'ultrasonic', 'gps'].map((sensor) => {
                const data = sensorStatus?.[sensor] || { status: 'inactive', confidence: 0 }
                return (
                  <div key={sensor} className="p-3 rounded-xl bg-white/5">
                    <div className="flex items-center justify-between mb-2">
                      <div className="flex items-center gap-3">
                        <div className={`w-2.5 h-2.5 rounded-full ${data.status === 'active' ? 'bg-av-success animate-pulse' : 'bg-av-danger'}`}></div>
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
                    {sensor === 'gps' && data.satellites && (
                      <div className="flex gap-3 mt-1 text-[10px] text-white/30">
                        <span>Sat: {data.satellites}</span>
                        <span>HDOP: {data.hdop?.toFixed(1)}</span>
                      </div>
                    )}
                    {sensor === 'camera' && data.fps && (
                      <div className="flex gap-3 mt-1 text-[10px] text-white/30">
                        <span>FPS: {data.fps}</span>
                        <span>{data.resolution}</span>
                      </div>
                    )}
                    {sensor === 'lidar' && data.fps && (
                      <div className="flex gap-3 mt-1 text-[10px] text-white/30">
                        <span>FPS: {data.fps}</span>
                        <span>Range: {data.range}m</span>
                      </div>
                    )}
                  </div>
                )
              })}
              {sensorStatus?.fusion && (
                <div className="p-3 rounded-xl bg-av-primary/10 border border-av-primary/20">
                  <div className="flex items-center justify-between mb-1">
                    <span className="text-xs text-av-primary uppercase">Sensor Fusion</span>
                    <span className="text-xs font-mono text-av-primary">{sensorStatus.fusion.confidence?.toFixed(1)}%</span>
                  </div>
                  <div className="flex gap-3 text-[10px] text-white/40">
                    <span>Latency: {sensorStatus.fusion.latency}ms</span>
                    <span>Tracking: {sensorStatus.fusion.tracking} objects</span>
                  </div>
                </div>
              )}
            </div>
          </div>

          {/* Detection Stats - Memoized values */}
          <div className="glass-panel p-6">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-4">Detection Stats</h3>
            <div className="grid grid-cols-2 gap-4">
              <div className="p-4 rounded-xl bg-white/5 text-center">
                <p className="text-2xl font-bold text-av-primary">{sensorStatus?.egoSpeed ? Math.round(sensorStatus.egoSpeed * 0.621) : 40}<span className="text-sm text-white/30 ml-1">mph</span></p>
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

          {/* Threat Types - Memoized values */}
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
                    <div className={`w-2 h-2 rounded-full ${threat.color}`}></div>
                    <span className="text-xs text-white/70">{threat.name}</span>
                  </div>
                  <span className="text-sm font-bold text-white">{threat.count}</span>
                </div>
              ))}
            </div>
          </div>
        </div>

        {/* Center Column - Visualizations */}
        <div className="col-span-6 space-y-6">
          {/* LIDAR Point Cloud */}
          <div className="glass-panel p-6">
            <div className="flex items-center justify-between mb-4">
              <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider">LIDAR Point Cloud</h3>
              <div className="flex items-center gap-2">
                <span className={`w-2 h-2 rounded-full ${isMonitoring ? 'bg-av-success animate-pulse' : 'bg-av-danger'}`}></span>
                <span className="text-xs text-white/50">{isMonitoring ? 'Live Feed' : 'Paused'}</span>
              </div>
            </div>
            <div className="relative">
              <canvas
                ref={canvasRef}
                width={600}
                height={350}
                className="w-full rounded-xl bg-black/50"
              />
              <div className="absolute top-4 left-4 flex items-center gap-2 px-3 py-1.5 rounded-full bg-black/60 border border-white/10">
                <span className="text-xs text-white/50">Range:</span>
                <span className="text-xs text-av-primary font-mono">150m</span>
              </div>
              <div className="absolute bottom-4 right-4 flex gap-4 text-xs">
                <div className="flex items-center gap-2">
                  <div className="w-3 h-3 rounded-full bg-av-primary"></div>
                  <span className="text-white/50">LIDAR</span>
                </div>
                <div className="flex items-center gap-2">
                  <div className="w-3 h-3 rounded-full bg-av-success"></div>
                  <span className="text-white/50">Vehicle</span>
                </div>
                <div className="flex items-center gap-2">
                  <div className="w-3 h-3 rounded-full bg-av-warning"></div>
                  <span className="text-white/50">Pedestrian</span>
                </div>
              </div>
            </div>
          </div>

          {/* Confidence Chart */}
          <div className="glass-panel p-6">
            <div className="flex items-center justify-between mb-4">
              <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider">Object Detection Confidence</h3>
              <span className={`text-2xl font-bold ${currentConfidence < 85 ? 'text-av-warning' : 'text-av-success'}`}>
                {currentConfidence.toFixed(1)}%
              </span>
            </div>
            <div className="h-[150px]">
              <ResponsiveContainer width="100%" height="100%">
                <AreaChart data={confidenceHistory}>
                  <defs>
                    <linearGradient id="confGradient" x1="0" y1="0" x2="0" y2="1">
                      <stop offset="0%" stopColor="#06b6d4" stopOpacity={0.3}/>
                      <stop offset="100%" stopColor="#06b6d4" stopOpacity={0}/>
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
                    isAnimationActive={!reducedMotion}
                  />
                </AreaChart>
              </ResponsiveContainer>
            </div>
          </div>
        </div>

        {/* Right Column - Detection Log */}
        <div className="col-span-3 space-y-6">
          {/* Camera Feed Simulation */}
          <div className="glass-panel p-6">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-4">Front Camera</h3>
            <div className="relative aspect-video bg-gradient-to-b from-sky-900 via-sky-700 to-gray-800 rounded-xl overflow-hidden">
              {/* Sky gradient */}
              <div className="absolute inset-0 bg-gradient-to-b from-sky-900 via-sky-600 to-blue-800"/>

              {/* Horizon line */}
              <div className="absolute top-[15%] left-0 right-0 h-px bg-white/20"/>

              {/* Mountains/skyline silhouette */}
              <svg className="absolute top-[10%] w-full h-[10%]" viewBox="0 0 400 60" preserveAspectRatio="none">
                <polygon points="0,60 0,30 20,40 40,25 60,45 80,35 100,50 120,30 150,45 180,20 220,40 260,25 300,45 350,30 400,50 400,60" fill="rgba(0,0,0,0.3)"/>
              </svg>

              {/* Road with perspective */}
              <svg className="absolute inset-0 w-full h-full" viewBox="0 0 400 240">
                <defs>
                  <linearGradient id="roadPerspective" x1="0%" y1="0%" x2="0%" y2="100%">
                    <stop offset="0%" stopColor="#2a2a2a"/>
                    <stop offset="100%" stopColor="#1a1a1a"/>
                  </linearGradient>
                  <linearGradient id="grassGrad" x1="0%" y1="0%" x2="0%" y2="100%">
                    <stop offset="0%" stopColor="#1a3d1a"/>
                    <stop offset="100%" stopColor="#0a1f0a"/>
                  </linearGradient>
                </defs>

                {/* Grass/shoulder */}
                <polygon points="0,100 0,240 80,240 120,120" fill="url(#grassGrad)"/>
                <polygon points="400,100 400,240 320,240 280,120" fill="url(#grassGrad)"/>

                {/* Main road surface */}
                <polygon points="120,120 280,120 400,240 0,240" fill="url(#roadPerspective)"/>

                {/* Lane markings */}
                <line x1="200" y1="120" x2="200" y2="240" stroke="white" strokeWidth="2" strokeDasharray="20,15"/>

                {/* Left lane divider */}
                <line x1="160" y1="130" x2="120" y2="240" stroke="white" strokeWidth="1.5" strokeDasharray="10,10" opacity="0.6"/>

                {/* Right lane divider */}
                <line x1="240" y1="130" x2="280" y2="240" stroke="white" strokeWidth="1.5" strokeDasharray="10,10" opacity="0.6"/>

                {/* Road edge lines */}
                <line x1="120" y1="120" x2="0" y2="240" stroke="white" strokeWidth="2"/>
                <line x1="280" y1="120" x2="400" y2="240" stroke="white" strokeWidth="2"/>

                {/* Crosswalk stripes */}
                <g opacity="0.4">
                  {[0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165, 180].map((offset, i) => (
                    <rect key={i} x={offset} y="200" width="8" height="4" fill="white"/>
                  ))}
                </g>
              </svg>

              {/* Animated vehicles in lanes */}
              {isMonitoring && cameraVehicles.map((v) => {
                const scale = Math.max(0.1, 1 - v.distance / 200)
                const laneOffset = v.lane * 45
                const perspectiveY = 130 + (v.distance / 150) * 110
                const perspectiveX = 200 + laneOffset * (1 - v.distance / 200)
                const vehicleWidth = v.width * scale * 0.8
                const vehicleHeight = v.height * scale * 0.8
                const isAhead = v.distance < 80

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
                    animate={{
                      opacity: [0.8, 1, 0.8],
                    }}
                    transition={{ duration: reducedMotion ? 0 : 2, repeat: Infinity }}
                  >
                    {/* Vehicle body */}
                    <div
                      className={`w-full h-full rounded-lg ${
                        v.type === 'truck' ? 'bg-gray-700' :
                        v.type === 'suv' ? 'bg-gray-600' : 'bg-gray-500'
                      }`}
                      style={{ boxShadow: '0 4px 15px rgba(0,0,0,0.5)' }}
                    >
                      <div className="absolute top-[10%] left-[15%] right-[15%] h-[25%] bg-gray-800 rounded-t-sm"/>
                      <div className="absolute bottom-[5%] left-[20%] right-[20%] h-[15%] bg-gray-800 rounded-b-sm"/>
                      {isAhead ? (
                        <div className="absolute top-[10%] left-[5%] right-[5%] flex justify-between">
                          <div className="w-2 h-1 bg-yellow-400 rounded-full"/>
                          <div className="w-2 h-1 bg-yellow-400 rounded-full"/>
                        </div>
                      ) : (
                        <div className="absolute bottom-[5%] left-[10%] right-[10%] flex justify-between">
                          <div className="w-2 h-1 bg-red-600 rounded-full"/>
                          <div className="w-2 h-1 bg-red-600 rounded-full"/>
                        </div>
                      )}
                    </div>
                    {/* Distance label */}
                    <div className="absolute -bottom-4 left-1/2 -translate-x-1/2 text-[8px] text-white/60 whitespace-nowrap">
                      {v.distance.toFixed(0)}m
                    </div>
                  </motion.div>
                )
              })}

              {/* Animated pedestrians crossing */}
              {isMonitoring && cameraPedestrians.map((p) => {
                const scale = Math.max(0.15, 1 - (p.y + 50) / 200)
                const personX = 200 + p.x * (1 + (p.y + 50) / 150)
                const personY = 140 + p.y

                return (
                  <motion.div
                    key={p.id}
                    className="absolute"
                    style={{
                      left: `${(personX / 400) * 100}%`,
                      top: `${(personY / 240) * 100}%`,
                      width: 8 * scale,
                      height: 18 * scale,
                    }}
                    animate={p.state === 'walking' ? {
                      y: [0, -2, 0],
                    } : {}}
                    transition={p.state === 'walking' ? {
                      duration: reducedMotion ? 0 : 0.4,
                      repeat: Infinity,
                    } : {}}
                  >
                    {/* Head */}
                    <div className="absolute top-0 left-1/2 -translate-x-1/2 w-[60%] h-[25%] bg-amber-200 rounded-full"/>
                    {/* Body */}
                    <div className="absolute top-[25%] left-[20%] w-[60%] h-[45%] bg-blue-600 rounded-sm"/>
                    {/* Legs */}
                    <div className="absolute bottom-0 left-[25%] w-[20%] h-[30%] bg-black rounded-sm"/>
                    <div className="absolute bottom-0 right-[25%] w-[20%] h-[30%] bg-black rounded-sm"/>
                  </motion.div>
                )
              })}

              {/* HUD overlay */}
              <div className="absolute bottom-2 left-2 right-2 flex justify-between text-[9px] text-white/50 font-mono">
                <span>FOV: 120°</span>
                <span className="text-av-success">{sensorStatus?.camera?.fps || 30} FPS</span>
                <span>1080p</span>
              </div>

              {/* Speed indicator */}
              <div className="absolute bottom-2 left-1/2 -translate-x-1/2 text-[10px] text-white/40">
                {sensorStatus?.egoSpeed ? Math.round(sensorStatus.egoSpeed * 0.621) : 40} MPH
              </div>

              {/* Status */}
              <div className="absolute top-2 left-2 flex items-center gap-1 px-2 py-1 rounded bg-black/60">
                <div className={`w-2 h-2 rounded-full ${isMonitoring ? 'bg-av-success animate-pulse' : 'bg-av-danger'}`}></div>
                <span className="text-[10px] text-white/70">CAM FRONT</span>
              </div>

              {/* Recording indicator */}
              <div className="absolute top-2 right-2 flex items-center gap-1 px-2 py-1 rounded bg-av-danger/20 border border-av-danger/30">
                <div className="w-2 h-2 rounded-full bg-av-danger animate-pulse"></div>
                <span className="text-[10px] text-av-danger">REC</span>
              </div>
            </div>
          </div>

          {/* Detection Log */}
          <div className="glass-panel p-6 flex-1">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-4">Detection Log</h3>
            <div className="space-y-2 max-h-[350px] overflow-y-auto">
              {detections.slice(0, 15).map((det) => (
                <motion.div
                  key={det.id}
                  initial={{ opacity: 0, x: 20 }}
                  animate={{ opacity: 1, x: 0 }}
                  className={`p-3 rounded-xl text-xs ${
                    det.threat
                      ? 'bg-av-danger/10 border border-av-danger/30'
                      : 'bg-white/5 border border-white/10'
                  }`}
                >
                  <div className="flex items-center justify-between mb-1">
                    <span className={`font-medium capitalize ${det.threat ? 'text-av-danger' : 'text-white'}`}>
                      {det.type}
                    </span>
                    <span className="text-white/40">
                      {new Date(det.timestamp).toLocaleTimeString()}
                    </span>
                  </div>
                  <div className="flex items-center gap-3 text-white/50">
                    <span>{det.distance}m</span>
                    <span>{det.bearing}°</span>
                    <span className={det.confidence < 80 ? 'text-av-warning' : 'text-av-success'}>
                      {det.confidence.toFixed(0)}%
                    </span>
                  </div>
                  {det.threat && (
                    <div className="mt-2 flex items-center gap-1 text-av-danger">
                      <svg className="w-3 h-3" viewBox="0 0 24 24" fill="currentColor">
                        <path d="M12 2L1 21h22L12 2zm0 3.5L19.5 19h-15L12 5.5zM11 10v4h2v-4h-2zm0 6v2h2v-2h-2z"/>
                      </svg>
                      <span className="capitalize">{det.threat.replace('_', ' ')}</span>
                    </div>
                  )}
                </motion.div>
              ))}
              {detections.length === 0 && (
                <div className="text-center text-white/30 py-8">
                  No detections yet
                </div>
              )}
            </div>
          </div>
        </div>
      </div>
    </motion.div>
  )
}

export default Dashboard
