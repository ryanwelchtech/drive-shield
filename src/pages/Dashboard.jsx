import { useState, useEffect, useRef } from 'react'
import { motion } from 'framer-motion'
import { LineChart, Line, XAxis, YAxis, ResponsiveContainer, AreaChart, Area } from 'recharts'

const Dashboard = ({ onBack }) => {
  const [isMonitoring, setIsMonitoring] = useState(true)
  const [detections, setDetections] = useState([])
  const [confidenceHistory, setConfidenceHistory] = useState([])
  const [currentConfidence, setCurrentConfidence] = useState(96.8)
  const [threatLevel, setThreatLevel] = useState('LOW')
  const [sensorStatus, setSensorStatus] = useState({
    lidar: { status: 'active', confidence: 98.2 },
    camera: { status: 'active', confidence: 94.5 },
    radar: { status: 'active', confidence: 97.1 },
    ultrasonic: { status: 'active', confidence: 99.0 },
  })
  const canvasRef = useRef(null)

  // Simulate real-time detections
  useEffect(() => {
    if (!isMonitoring) return

    const interval = setInterval(() => {
      // Random detection events
      if (Math.random() > 0.7) {
        const types = ['vehicle', 'pedestrian', 'cyclist', 'sign', 'obstacle']
        const threats = ['none', 'none', 'none', 'adversarial_patch', 'phantom_object', 'spoofing']
        const type = types[Math.floor(Math.random() * types.length)]
        const threat = threats[Math.floor(Math.random() * threats.length)]

        const newDetection = {
          id: Date.now(),
          type,
          threat: threat !== 'none' ? threat : null,
          confidence: 70 + Math.random() * 30,
          distance: (5 + Math.random() * 95).toFixed(1),
          bearing: Math.floor(Math.random() * 360),
          timestamp: new Date().toLocaleTimeString(),
        }

        setDetections(prev => [newDetection, ...prev.slice(0, 19)])

        if (threat !== 'none') {
          setThreatLevel(threat === 'adversarial_patch' ? 'CRITICAL' : 'HIGH')
          setTimeout(() => setThreatLevel('LOW'), 5000)
        }
      }

      // Update confidence
      setConfidenceHistory(prev => {
        const noise = (Math.random() - 0.5) * 4
        const newConf = Math.max(75, Math.min(100, currentConfidence + noise))
        setCurrentConfidence(newConf)

        return [...prev, { time: prev.length, value: newConf }].slice(-60)
      })

      // Update sensor status
      setSensorStatus(prev => ({
        lidar: { ...prev.lidar, confidence: 95 + Math.random() * 5 },
        camera: { ...prev.camera, confidence: 90 + Math.random() * 10 },
        radar: { ...prev.radar, confidence: 94 + Math.random() * 6 },
        ultrasonic: { ...prev.ultrasonic, confidence: 97 + Math.random() * 3 },
      }))
    }, 500)

    return () => clearInterval(interval)
  }, [isMonitoring, currentConfidence])

  // LIDAR Point Cloud Visualization
  useEffect(() => {
    if (!canvasRef.current) return

    const canvas = canvasRef.current
    const ctx = canvas.getContext('2d')
    const width = canvas.width
    const height = canvas.height

    let animationId
    let points = []

    // Generate initial points
    for (let i = 0; i < 500; i++) {
      points.push({
        angle: Math.random() * Math.PI * 2,
        distance: 20 + Math.random() * 130,
        z: (Math.random() - 0.5) * 50,
        intensity: Math.random(),
      })
    }

    const render = () => {
      ctx.fillStyle = 'rgba(0, 0, 0, 0.1)'
      ctx.fillRect(0, 0, width, height)

      const centerX = width / 2
      const centerY = height / 2

      // Draw grid
      ctx.strokeStyle = 'rgba(6, 182, 212, 0.1)'
      ctx.lineWidth = 1
      for (let r = 30; r <= 150; r += 30) {
        ctx.beginPath()
        ctx.arc(centerX, centerY, r, 0, Math.PI * 2)
        ctx.stroke()
      }

      // Draw points
      points.forEach((point, i) => {
        const x = centerX + Math.cos(point.angle) * point.distance
        const y = centerY + Math.sin(point.angle) * point.distance

        // Determine if point is a threat
        const isThreat = isMonitoring && Math.random() > 0.995

        ctx.beginPath()
        ctx.arc(x, y, isThreat ? 4 : 2, 0, Math.PI * 2)
        ctx.fillStyle = isThreat
          ? 'rgba(239, 68, 68, 0.9)'
          : `rgba(6, 182, 212, ${0.3 + point.intensity * 0.7})`
        ctx.fill()

        // Animate points
        point.angle += 0.002
        point.distance += (Math.random() - 0.5) * 0.5
        if (point.distance < 20) point.distance = 20
        if (point.distance > 150) point.distance = 150
      })

      // Draw vehicle in center
      ctx.fillStyle = 'rgba(6, 182, 212, 0.8)'
      ctx.fillRect(centerX - 8, centerY - 15, 16, 30)
      ctx.fillStyle = 'rgba(6, 182, 212, 0.5)'
      ctx.fillRect(centerX - 5, centerY - 20, 10, 8)

      animationId = requestAnimationFrame(render)
    }

    render()

    return () => cancelAnimationFrame(animationId)
  }, [isMonitoring])

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
            <p className="text-sm text-white/50">Real-time AV threat detection and sensor analysis</p>
          </div>
        </div>

        <div className="flex items-center gap-4">
          <div className={`px-4 py-2 rounded-xl border ${getThreatBg(threatLevel)}`}>
            <span className="text-xs text-white/50 mr-2">Threat Level:</span>
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
            <div className="space-y-4">
              {Object.entries(sensorStatus).map(([sensor, data]) => (
                <div key={sensor} className="p-4 rounded-xl bg-white/5">
                  <div className="flex items-center justify-between mb-2">
                    <div className="flex items-center gap-3">
                      <div className={`w-3 h-3 rounded-full ${data.status === 'active' ? 'bg-av-success animate-pulse' : 'bg-av-danger'}`}></div>
                      <span className="text-sm text-white capitalize">{sensor}</span>
                    </div>
                    <span className="text-sm font-mono text-av-primary">{data.confidence.toFixed(1)}%</span>
                  </div>
                  <div className="h-1.5 bg-white/10 rounded-full overflow-hidden">
                    <motion.div
                      className="h-full bg-gradient-to-r from-av-primary to-av-accent rounded-full"
                      animate={{ width: `${data.confidence}%` }}
                      transition={{ duration: 0.3 }}
                    />
                  </div>
                </div>
              ))}
            </div>
          </div>

          {/* Detection Stats */}
          <div className="glass-panel p-6">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-4">Detection Stats</h3>
            <div className="grid grid-cols-2 gap-4">
              <div className="p-4 rounded-xl bg-white/5 text-center">
                <p className="text-2xl font-bold text-av-primary">{detections.length}</p>
                <p className="text-xs text-white/50">Total Objects</p>
              </div>
              <div className="p-4 rounded-xl bg-white/5 text-center">
                <p className="text-2xl font-bold text-av-danger">
                  {detections.filter(d => d.threat).length}
                </p>
                <p className="text-xs text-white/50">Threats</p>
              </div>
              <div className="p-4 rounded-xl bg-white/5 text-center">
                <p className="text-2xl font-bold text-av-success">
                  {detections.filter(d => d.type === 'vehicle').length}
                </p>
                <p className="text-xs text-white/50">Vehicles</p>
              </div>
              <div className="p-4 rounded-xl bg-white/5 text-center">
                <p className="text-2xl font-bold text-av-warning">
                  {detections.filter(d => d.type === 'pedestrian').length}
                </p>
                <p className="text-xs text-white/50">Pedestrians</p>
              </div>
            </div>
          </div>

          {/* Threat Types */}
          <div className="glass-panel p-6">
            <h3 className="text-sm font-semibold text-white/70 uppercase tracking-wider mb-4">Active Threats</h3>
            <div className="space-y-2">
              {[
                { name: 'Adversarial Patches', count: detections.filter(d => d.threat === 'adversarial_patch').length, color: 'bg-av-danger' },
                { name: 'Phantom Objects', count: detections.filter(d => d.threat === 'phantom_object').length, color: 'bg-av-warning' },
                { name: 'Sensor Spoofing', count: detections.filter(d => d.threat === 'spoofing').length, color: 'bg-av-purple' },
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
              {/* Overlay UI */}
              <div className="absolute top-4 left-4 flex items-center gap-2 px-3 py-1.5 rounded-full bg-black/60 border border-white/10">
                <span className="text-xs text-white/50">Range:</span>
                <span className="text-xs text-av-primary font-mono">150m</span>
              </div>
              <div className="absolute bottom-4 right-4 flex gap-4 text-xs">
                <div className="flex items-center gap-2">
                  <div className="w-3 h-3 rounded-full bg-av-primary"></div>
                  <span className="text-white/50">Object</span>
                </div>
                <div className="flex items-center gap-2">
                  <div className="w-3 h-3 rounded-full bg-av-danger"></div>
                  <span className="text-white/50">Threat</span>
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
            <div className="relative aspect-video bg-gradient-to-b from-gray-800 to-gray-900 rounded-xl overflow-hidden">
              {/* Simulated camera view */}
              <div className="absolute inset-0 flex items-end justify-center pb-8">
                <div className="w-32 h-1 bg-white/20 rounded"></div>
              </div>
              {/* Road lines */}
              <svg className="absolute inset-0 w-full h-full" viewBox="0 0 200 120">
                <line x1="60" y1="120" x2="90" y2="60" stroke="rgba(255,255,255,0.3)" strokeWidth="2" strokeDasharray="8,8"/>
                <line x1="140" y1="120" x2="110" y2="60" stroke="rgba(255,255,255,0.3)" strokeWidth="2" strokeDasharray="8,8"/>
              </svg>
              {/* Detection boxes */}
              {isMonitoring && (
                <>
                  <motion.div
                    className="absolute border-2 border-av-success rounded"
                    style={{ left: '30%', top: '40%', width: '40px', height: '30px' }}
                    animate={{ opacity: [0.5, 1, 0.5] }}
                    transition={{ duration: 1, repeat: Infinity }}
                  >
                    <span className="absolute -top-5 left-0 text-[10px] text-av-success bg-black/50 px-1 rounded">CAR 94%</span>
                  </motion.div>
                  <motion.div
                    className="absolute border-2 border-av-warning rounded"
                    style={{ right: '25%', top: '45%', width: '20px', height: '35px' }}
                    animate={{ opacity: [0.5, 1, 0.5] }}
                    transition={{ duration: 1.2, repeat: Infinity }}
                  >
                    <span className="absolute -top-5 left-0 text-[10px] text-av-warning bg-black/50 px-1 rounded">PED 87%</span>
                  </motion.div>
                </>
              )}
              {/* Status */}
              <div className="absolute top-2 left-2 flex items-center gap-1 px-2 py-1 rounded bg-black/60">
                <div className={`w-2 h-2 rounded-full ${isMonitoring ? 'bg-av-success animate-pulse' : 'bg-av-danger'}`}></div>
                <span className="text-[10px] text-white/70">LIVE</span>
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
                    <span className="text-white/40">{det.timestamp}</span>
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
