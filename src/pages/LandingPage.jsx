import { useEffect, useRef, useState } from 'react'
import { motion, useScroll, useTransform } from 'framer-motion'

const LandingPage = ({ onEnterDashboard }) => {
  const containerRef = useRef(null)
  const { scrollYProgress } = useScroll({ target: containerRef })
  const [mousePosition, setMousePosition] = useState({ x: 0, y: 0 })

  const heroOpacity = useTransform(scrollYProgress, [0, 0.3], [1, 0])
  const heroScale = useTransform(scrollYProgress, [0, 0.3], [1, 0.95])

  useEffect(() => {
    const handleMouseMove = (e) => {
      setMousePosition({
        x: (e.clientX / window.innerWidth - 0.5) * 30,
        y: (e.clientY / window.innerHeight - 0.5) * 30,
      })
    }
    window.addEventListener('mousemove', handleMouseMove)
    return () => window.removeEventListener('mousemove', handleMouseMove)
  }, [])

  const features = [
    {
      icon: (
        <svg className="w-8 h-8" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
          <circle cx="12" cy="12" r="10"/>
          <path d="M12 2v4M12 18v4M2 12h4M18 12h4"/>
          <circle cx="12" cy="12" r="3"/>
        </svg>
      ),
      title: 'LIDAR Monitoring',
      description: 'Real-time point cloud analysis and object detection with adversarial anomaly detection.'
    },
    {
      icon: (
        <svg className="w-8 h-8" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
          <rect x="2" y="6" width="20" height="12" rx="2"/>
          <circle cx="12" cy="12" r="3"/>
          <path d="M2 12h4M18 12h4"/>
        </svg>
      ),
      title: 'Camera Feed Analysis',
      description: 'Multi-camera fusion with adversarial patch detection and classification confidence monitoring.'
    },
    {
      icon: (
        <svg className="w-8 h-8" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
          <path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"/>
          <path d="M12 8v4M12 16h.01"/>
        </svg>
      ),
      title: 'Threat Detection',
      description: 'Identify adversarial stop sign attacks, phantom objects, and sensor spoofing attempts.'
    },
    {
      icon: (
        <svg className="w-8 h-8" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5">
          <path d="M22 12h-4l-3 9L9 3l-3 9H2"/>
        </svg>
      ),
      title: 'Confidence Tracking',
      description: 'Monitor object detection confidence scores and alert on anomalous degradation patterns.'
    },
  ]

  const threatTypes = [
    { name: 'Adversarial Patches', severity: 'Critical', detections: 23 },
    { name: 'Phantom Objects', severity: 'High', detections: 12 },
    { name: 'Sensor Spoofing', severity: 'Critical', detections: 8 },
    { name: 'GPS Manipulation', severity: 'Medium', detections: 15 },
  ]

  return (
    <motion.div
      ref={containerRef}
      className="min-h-screen bg-black road-grid"
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
      exit={{ opacity: 0 }}
      transition={{ duration: 0.5 }}
    >
      {/* Floating Orbs */}
      <div className="fixed inset-0 overflow-hidden pointer-events-none">
        <motion.div
          className="hero-glow bg-av-primary"
          style={{
            left: '15%',
            top: '25%',
            x: mousePosition.x * 0.5,
            y: mousePosition.y * 0.5,
          }}
          animate={{
            scale: [1, 1.2, 1],
            opacity: [0.3, 0.5, 0.3],
          }}
          transition={{ duration: 8, repeat: Infinity }}
        />
        <motion.div
          className="hero-glow bg-av-secondary"
          style={{
            right: '15%',
            top: '35%',
            x: mousePosition.x * -0.3,
            y: mousePosition.y * -0.3,
          }}
          animate={{
            scale: [1.2, 1, 1.2],
            opacity: [0.4, 0.2, 0.4],
          }}
          transition={{ duration: 10, repeat: Infinity }}
        />
        <motion.div
          className="hero-glow bg-av-accent"
          style={{
            left: '50%',
            bottom: '15%',
            x: mousePosition.x * 0.2,
            y: mousePosition.y * 0.2,
          }}
          animate={{
            scale: [1, 1.3, 1],
            opacity: [0.2, 0.4, 0.2],
          }}
          transition={{ duration: 12, repeat: Infinity }}
        />
      </div>

      {/* Navigation */}
      <motion.nav
        className="fixed top-0 left-0 right-0 z-50"
        initial={{ y: -100 }}
        animate={{ y: 0 }}
        transition={{ duration: 0.8, delay: 0.2 }}
      >
        <div className="max-w-7xl mx-auto px-6 py-4">
          <div className="glass-panel px-6 py-4 flex items-center justify-between">
            <div className="flex items-center gap-3">
              <div className="w-10 h-10 rounded-xl bg-gradient-to-br from-av-primary to-av-secondary flex items-center justify-center">
                <svg className="w-6 h-6 text-white" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M12 2L3 7v6c0 5.25 3.75 10.125 9 11.25 5.25-1.125 9-6 9-11.25V7l-9-5z"/>
                  <path d="M7 13l3 3 7-7"/>
                </svg>
              </div>
              <span className="text-xl font-semibold tracking-tight">DriveShield</span>
            </div>
            <div className="hidden md:flex items-center gap-8">
              <a href="#features" className="text-sm text-white/60 hover:text-white transition-colors">Features</a>
              <a href="#threats" className="text-sm text-white/60 hover:text-white transition-colors">Threat Types</a>
              <a href="#demo" className="text-sm text-white/60 hover:text-white transition-colors">Live Demo</a>
            </div>
            <motion.button
              onClick={onEnterDashboard}
              className="glass-button text-sm"
              whileHover={{ scale: 1.02 }}
              whileTap={{ scale: 0.98 }}
            >
              Launch Platform
            </motion.button>
          </div>
        </div>
      </motion.nav>

      {/* Hero Section */}
      <motion.section
        className="min-h-screen flex items-center justify-center px-6 pt-24"
        style={{ opacity: heroOpacity, scale: heroScale }}
      >
        <div className="max-w-6xl mx-auto text-center">
          <motion.div
            initial={{ opacity: 0, y: 30 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.8, delay: 0.4 }}
          >
            <span className="inline-flex items-center gap-2 px-4 py-2 rounded-full bg-white/5 border border-white/10 text-sm text-white/70 mb-8">
              <span className="w-2 h-2 rounded-full bg-av-primary animate-pulse"></span>
              Autonomous Vehicle Security Platform
            </span>
          </motion.div>

          <motion.h1
            className="text-6xl md:text-8xl font-bold tracking-tight mb-6"
            initial={{ opacity: 0, y: 30 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.8, delay: 0.5 }}
          >
            <span className="gradient-text">Drive</span>
            <span className="text-white">Shield</span>
          </motion.h1>

          <motion.p
            className="text-xl md:text-2xl text-white/50 max-w-3xl mx-auto mb-12 leading-relaxed"
            initial={{ opacity: 0, y: 30 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.8, delay: 0.6 }}
          >
            Protect autonomous vehicles from adversarial attacks.
            <span className="text-white/70"> Monitor sensor feeds, detect threats, and visualize perception system vulnerabilities in real-time.</span>
          </motion.p>

          <motion.div
            className="flex flex-col sm:flex-row items-center justify-center gap-4"
            initial={{ opacity: 0, y: 30 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.8, delay: 0.7 }}
          >
            <motion.button
              onClick={onEnterDashboard}
              className="glass-button flex items-center gap-3"
              whileHover={{ scale: 1.02 }}
              whileTap={{ scale: 0.98 }}
            >
              <span>Enter Monitoring Center</span>
              <svg className="w-5 h-5" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M5 12h14M12 5l7 7-7 7"/>
              </svg>
            </motion.button>
            <motion.button
              className="glass-button glass-button-secondary flex items-center gap-3"
              whileHover={{ scale: 1.02 }}
              whileTap={{ scale: 0.98 }}
            >
              <svg className="w-5 h-5" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <circle cx="12" cy="12" r="10"/>
                <polygon points="10,8 16,12 10,16" fill="currentColor"/>
              </svg>
              <span>Watch Demo</span>
            </motion.button>
          </motion.div>

          {/* Vehicle Visualization */}
          <motion.div
            className="mt-20 relative"
            initial={{ opacity: 0, scale: 0.9 }}
            animate={{ opacity: 1, scale: 1 }}
            transition={{ duration: 1, delay: 0.8 }}
          >
            <div className="glass-panel p-8 max-w-4xl mx-auto">
              <VehicleSensorViz />
            </div>
          </motion.div>
        </div>
      </motion.section>

      {/* Features Section */}
      <section id="features" className="py-32 px-6">
        <div className="max-w-7xl mx-auto">
          <motion.div
            className="text-center mb-20"
            initial={{ opacity: 0, y: 30 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.8 }}
          >
            <h2 className="text-4xl md:text-5xl font-bold mb-6">
              <span className="gradient-text">Comprehensive</span> Protection
            </h2>
            <p className="text-lg text-white/50 max-w-2xl mx-auto">
              Multi-sensor monitoring and adversarial threat detection for autonomous driving systems.
            </p>
          </motion.div>

          <div className="grid md:grid-cols-2 lg:grid-cols-4 gap-6">
            {features.map((feature, index) => (
              <motion.div
                key={feature.title}
                className="glass-card p-6"
                initial={{ opacity: 0, y: 30 }}
                whileInView={{ opacity: 1, y: 0 }}
                viewport={{ once: true }}
                transition={{ duration: 0.6, delay: index * 0.1 }}
              >
                <div className="w-14 h-14 rounded-2xl bg-gradient-to-br from-av-primary/20 to-av-secondary/20 flex items-center justify-center text-av-primary mb-5">
                  {feature.icon}
                </div>
                <h3 className="text-lg font-semibold text-white mb-3">{feature.title}</h3>
                <p className="text-sm text-white/50 leading-relaxed">{feature.description}</p>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* Threat Types Section */}
      <section id="threats" className="py-32 px-6">
        <div className="max-w-7xl mx-auto">
          <div className="grid lg:grid-cols-2 gap-16 items-center">
            <motion.div
              initial={{ opacity: 0, x: -30 }}
              whileInView={{ opacity: 1, x: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.8 }}
            >
              <h2 className="text-4xl md:text-5xl font-bold mb-6">
                <span className="gradient-text-warm">Threat</span> Detection
              </h2>
              <p className="text-lg text-white/50 mb-8">
                Identify and respond to adversarial attacks targeting autonomous vehicle perception systems.
              </p>

              <div className="space-y-4">
                {threatTypes.map((threat, index) => (
                  <motion.div
                    key={threat.name}
                    className="glass-card p-5"
                    initial={{ opacity: 0, x: -20 }}
                    whileInView={{ opacity: 1, x: 0 }}
                    viewport={{ once: true }}
                    transition={{ duration: 0.5, delay: index * 0.1 }}
                  >
                    <div className="flex items-center justify-between mb-3">
                      <div className="flex items-center gap-4">
                        <div className={`w-12 h-12 rounded-xl flex items-center justify-center ${
                          threat.severity === 'Critical' ? 'bg-av-danger/20' :
                          threat.severity === 'High' ? 'bg-av-warning/20' : 'bg-av-primary/20'
                        }`}>
                          <svg className={`w-6 h-6 ${
                            threat.severity === 'Critical' ? 'text-av-danger' :
                            threat.severity === 'High' ? 'text-av-warning' : 'text-av-primary'
                          }`} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                            <path d="M12 9v4M12 17h.01"/>
                            <path d="M10.29 3.86L1.82 18a2 2 0 001.71 3h16.94a2 2 0 001.71-3L13.71 3.86a2 2 0 00-3.42 0z"/>
                          </svg>
                        </div>
                        <div>
                          <h4 className="font-semibold text-white">{threat.name}</h4>
                          <p className={`text-xs ${
                            threat.severity === 'Critical' ? 'text-av-danger' :
                            threat.severity === 'High' ? 'text-av-warning' : 'text-av-primary'
                          }`}>{threat.severity} Severity</p>
                        </div>
                      </div>
                      <div className="text-right">
                        <span className="text-2xl font-bold text-white">{threat.detections}</span>
                        <p className="text-xs text-white/40">Detections</p>
                      </div>
                    </div>
                  </motion.div>
                ))}
              </div>
            </motion.div>

            <motion.div
              className="glass-panel p-8"
              initial={{ opacity: 0, x: 30 }}
              whileInView={{ opacity: 1, x: 0 }}
              viewport={{ once: true }}
              transition={{ duration: 0.8 }}
            >
              <RadarViz />
            </motion.div>
          </div>
        </div>
      </section>

      {/* CTA Section */}
      <section id="demo" className="py-32 px-6">
        <div className="max-w-4xl mx-auto text-center">
          <motion.div
            className="glass-panel p-12"
            initial={{ opacity: 0, y: 30 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.8 }}
          >
            <h2 className="text-4xl md:text-5xl font-bold mb-6">
              Ready to <span className="gradient-text">Secure</span> Your Fleet?
            </h2>
            <p className="text-lg text-white/50 mb-10 max-w-xl mx-auto">
              Launch the monitoring platform and start detecting adversarial threats to autonomous vehicles.
            </p>
            <motion.button
              onClick={onEnterDashboard}
              className="glass-button text-lg px-10 py-5"
              whileHover={{ scale: 1.02 }}
              whileTap={{ scale: 0.98 }}
            >
              Launch DriveShield Platform
            </motion.button>
          </motion.div>
        </div>
      </section>

      {/* Footer */}
      <footer className="py-12 px-6 border-t border-white/5">
        <div className="max-w-7xl mx-auto flex flex-col md:flex-row items-center justify-between gap-4">
          <div className="flex items-center gap-3">
            <div className="w-8 h-8 rounded-lg bg-gradient-to-br from-av-primary to-av-secondary flex items-center justify-center">
              <svg className="w-4 h-4 text-white" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M12 2L3 7v6c0 5.25 3.75 10.125 9 11.25 5.25-1.125 9-6 9-11.25V7l-9-5z"/>
              </svg>
            </div>
            <span className="text-sm text-white/50">DriveShield</span>
          </div>
          <p className="text-sm text-white/30">
            Built by <a href="https://ryanwelchtech.com" className="text-white/50 hover:text-white transition-colors">Ryan Welch</a>
          </p>
        </div>
      </footer>
    </motion.div>
  )
}

// Vehicle Sensor Visualization Component
const VehicleSensorViz = () => {
  const [activeSensor, setActiveSensor] = useState(0)

  useEffect(() => {
    const interval = setInterval(() => {
      setActiveSensor((prev) => (prev + 1) % 4)
    }, 2000)
    return () => clearInterval(interval)
  }, [])

  const sensors = [
    { name: 'Front LIDAR', angle: 0 },
    { name: 'Right Camera', angle: 90 },
    { name: 'Rear Radar', angle: 180 },
    { name: 'Left Camera', angle: 270 },
  ]

  return (
    <div className="relative h-80">
      <svg className="w-full h-full" viewBox="0 0 400 280">
        {/* Vehicle Body */}
        <defs>
          <linearGradient id="vehicleGrad" x1="0%" y1="0%" x2="100%" y2="100%">
            <stop offset="0%" stopColor="rgba(6, 182, 212, 0.3)" />
            <stop offset="100%" stopColor="rgba(14, 165, 233, 0.1)" />
          </linearGradient>
        </defs>

        {/* Sensor Rings */}
        {sensors.map((sensor, index) => (
          <g key={sensor.name}>
            <motion.circle
              cx="200"
              cy="140"
              r={60 + index * 25}
              fill="none"
              stroke={activeSensor === index ? '#06b6d4' : 'rgba(255,255,255,0.1)'}
              strokeWidth={activeSensor === index ? 2 : 1}
              strokeDasharray="5,5"
              initial={{ opacity: 0 }}
              animate={{
                opacity: activeSensor === index ? 0.8 : 0.2,
                scale: activeSensor === index ? [1, 1.05, 1] : 1,
              }}
              transition={{ duration: 1, repeat: activeSensor === index ? Infinity : 0 }}
            />
          </g>
        ))}

        {/* Vehicle Representation */}
        <rect
          x="160"
          y="110"
          width="80"
          height="60"
          rx="10"
          fill="url(#vehicleGrad)"
          stroke="#06b6d4"
          strokeWidth="2"
        />
        <rect x="170" y="100" width="60" height="25" rx="5" fill="rgba(6, 182, 212, 0.2)" stroke="#06b6d4" strokeWidth="1"/>

        {/* Sensor Points */}
        <circle cx="200" cy="100" r="5" fill="#06b6d4">
          <animate attributeName="opacity" values="1;0.5;1" dur="1s" repeatCount="indefinite"/>
        </circle>
        <circle cx="245" cy="140" r="5" fill="#22c55e">
          <animate attributeName="opacity" values="1;0.5;1" dur="1.2s" repeatCount="indefinite"/>
        </circle>
        <circle cx="200" cy="175" r="5" fill="#f59e0b">
          <animate attributeName="opacity" values="1;0.5;1" dur="0.8s" repeatCount="indefinite"/>
        </circle>
        <circle cx="155" cy="140" r="5" fill="#22c55e">
          <animate attributeName="opacity" values="1;0.5;1" dur="1.1s" repeatCount="indefinite"/>
        </circle>

        {/* Detection Lines */}
        <motion.line
          x1="200" y1="100" x2="200" y2="30"
          stroke="#06b6d4"
          strokeWidth="1"
          strokeDasharray="4,4"
          initial={{ pathLength: 0 }}
          animate={{ pathLength: 1 }}
          transition={{ duration: 2, repeat: Infinity }}
        />
        <motion.line
          x1="245" y1="140" x2="350" y2="140"
          stroke="#22c55e"
          strokeWidth="1"
          strokeDasharray="4,4"
          initial={{ pathLength: 0 }}
          animate={{ pathLength: 1 }}
          transition={{ duration: 2, repeat: Infinity, delay: 0.5 }}
        />
      </svg>

      {/* Sensor Labels */}
      <div className="absolute bottom-0 left-0 right-0 flex justify-around">
        {sensors.map((sensor, index) => (
          <div
            key={sensor.name}
            className={`text-xs px-3 py-1 rounded-full transition-all ${
              activeSensor === index
                ? 'bg-av-primary/20 text-av-primary border border-av-primary/30'
                : 'text-white/40'
            }`}
          >
            {sensor.name}
          </div>
        ))}
      </div>

      {/* Status Indicator */}
      <motion.div
        className="absolute top-4 right-4 flex items-center gap-2 px-3 py-1.5 rounded-full bg-av-success/20 border border-av-success/30"
        animate={{ opacity: [0.5, 1, 0.5] }}
        transition={{ duration: 2, repeat: Infinity }}
      >
        <span className="w-2 h-2 rounded-full bg-av-success animate-pulse"></span>
        <span className="text-xs text-av-success font-medium">All Sensors Active</span>
      </motion.div>
    </div>
  )
}

// Radar Visualization Component
const RadarViz = () => {
  const [blips, setBlips] = useState([])
  const [sweepAngle, setSweepAngle] = useState(0)

  useEffect(() => {
    const sweepInterval = setInterval(() => {
      setSweepAngle(angle => (angle + 1) % 360)
    }, 30)

    const blipInterval = setInterval(() => {
      if (Math.random() > 0.6) {
        const isThreat = Math.random() > 0.75
        const angle = Math.random() * 360
        const distance = 15 + Math.random() * 80
        const newBlip = {
          id: Date.now() + Math.random(),
          angle,
          distance,
          type: isThreat ? 'threat' : 'object',
          intensity: 1,
          bearing: Math.floor(angle),
        }
        setBlips(prev => [...prev.slice(-25), newBlip])
      }
    }, 400)

    return () => {
      clearInterval(sweepInterval)
      clearInterval(blipInterval)
    }
  }, [])

  useEffect(() => {
    const fadeInterval = setInterval(() => {
      setBlips(prev => prev.filter(b => b.intensity > 0.1).map(b => ({
        ...b,
        intensity: b.intensity - 0.02
      })))
    }, 100)
    return () => clearInterval(fadeInterval)
  }, [])

  const toRad = (deg) => (deg * Math.PI) / 180

  return (
    <div className="relative">
      <div className="flex items-center justify-between mb-4">
        <h3 className="text-lg font-semibold">Perception Radar</h3>
        <div className="flex items-center gap-2">
          <span className="w-2 h-2 rounded-full bg-av-success animate-pulse"></span>
          <span className="text-xs text-white/50">LIVE</span>
        </div>
      </div>

      <div className="relative w-full aspect-square max-w-[320px] mx-auto">
        <svg className="w-full h-full" viewBox="0 0 240 240">
          <defs>
            <radialGradient id="radarFade" cx="50%" cy="50%" r="50%">
              <stop offset="0%" stopColor="rgba(6, 182, 212, 0.05)"/>
              <stop offset="100%" stopColor="rgba(6, 182, 212, 0)"/>
            </radialGradient>
            <linearGradient id="sweepGrad" x1="0%" y1="0%" x2="100%" y2="0%">
              <stop offset="0%" stopColor="rgba(6, 182, 212, 0.4)"/>
              <stop offset="100%" stopColor="rgba(6, 182, 212, 0)"/>
            </linearGradient>
            <filter id="glow">
              <feGaussianBlur stdDeviation="2" result="coloredBlur"/>
              <feMerge>
                <feMergeNode in="coloredBlur"/>
                <feMergeNode in="SourceGraphic"/>
              </feMerge>
            </filter>
          </defs>

          <circle cx="120" cy="120" r="110" fill="url(#radarFade)" stroke="rgba(6, 182, 212, 0.2)" strokeWidth="1"/>

          {[25, 50, 75, 100].map((dist) => (
            <g key={dist}>
              <circle cx="120" cy="120" r={dist} fill="none" stroke="rgba(6, 182, 212, 0.15)" strokeWidth="0.5"/>
            </g>
          ))}

          {[0, 45, 90, 135, 180, 225, 270, 315].map((angle) => {
            const rad = toRad(angle)
            const x1 = 120 + Math.cos(rad) * 10
            const y1 = 120 + Math.sin(rad) * 10
            const x2 = 120 + Math.cos(rad) * 110
            const y2 = 120 + Math.sin(rad) * 110
            return (
              <line key={angle} x1={x1} y1={y1} x2={x2} y2={y2} stroke="rgba(6, 182, 212, 0.2)" strokeWidth="0.5"/>
            )
          })}

          <text x="120" y="8" textAnchor="middle" fill="rgba(255,255,255,0.4)" fontSize="8">N</text>
          <text x="120" y="235" textAnchor="middle" fill="rgba(255,255,255,0.4)" fontSize="8">S</text>
          <text x="232" y="123" textAnchor="middle" fill="rgba(255,255,255,0.4)" fontSize="8">E</text>
          <text x="8" y="123" textAnchor="middle" fill="rgba(255,255,255,0.4)" fontSize="8">W</text>

          {[25, 50, 75, 100].map((dist) => (
            <text
              key={`label-${dist}`}
              x={120 + dist + 3}
              y={118}
              fill="rgba(255,255,255,0.25)"
              fontSize="6"
            >{dist}m</text>
          ))}

          {blips.map((blip) => {
            const rad = toRad(blip.angle)
            const x = 120 + Math.cos(rad) * blip.distance
            const y = 120 + Math.sin(rad) * blip.distance
            const color = blip.type === 'threat' ? '#ef4444' : '#22c55e'
            return (
              <g key={blip.id}>
                <circle
                  cx={x}
                  cy={y}
                  r={blip.type === 'threat' ? 5 : 3}
                  fill={color}
                  opacity={blip.intensity}
                  filter="url(#glow)"
                />
                <circle
                  cx={x}
                  cy={y}
                  r={blip.type === 'threat' ? 8 : 5}
                  fill="none"
                  stroke={color}
                  strokeWidth="0.5"
                  opacity={blip.intensity * 0.5}
                />
              </g>
            )
          })}

          <g style={{ transformOrigin: '120px 120px', transform: `rotate(${sweepAngle}deg)` }}>
            <path
              d="M120,120 L120,10 A110,110 0 0,1 230,120 Z"
              fill="url(#sweepGrad)"
            />
            <line x1="120" y1="120" x2="120" y2="10" stroke="rgba(6, 182, 212, 0.8)" strokeWidth="1.5"/>
          </g>

          <circle cx="120" cy="120" r="8" fill="#0a0a0a" stroke="#06b6d4" strokeWidth="2"/>
          <circle cx="120" cy="120" r="4" fill="#06b6d4"/>

          {blips.filter(b => b.type === 'threat').slice(-3).map((blip, i) => {
            const rad = toRad(blip.angle)
            const x = 120 + Math.cos(rad) * blip.distance
            const y = 120 + Math.sin(rad) * blip.distance
            return (
              <motion.circle
                key={`alert-${blip.id}`}
                cx={x}
                cy={y}
                r="12"
                fill="none"
                stroke="#ef4444"
                strokeWidth="1"
                initial={{ opacity: 1, scale: 1 }}
                animate={{ opacity: 0, scale: 1.5 }}
                transition={{ duration: 1, repeat: Infinity }}
              />
            )
          })}
        </svg>

        <div className="absolute top-4 left-4 space-y-1">
          <div className="flex items-center gap-2">
            <div className="w-2 h-2 rounded-full bg-av-success"></div>
            <span className="text-[10px] text-white/50">Object</span>
          </div>
          <div className="flex items-center gap-2">
            <div className="w-2 h-2 rounded-full bg-av-danger"></div>
            <span className="text-[10px] text-white/50">Threat</span>
          </div>
        </div>

        <div className="absolute bottom-4 right-4 text-[10px] text-white/30">
          Range: 100m | Azimuth: 360°
        </div>
      </div>

      <div className="flex justify-center gap-6 mt-4">
        <div className="flex items-center gap-2">
          <div className="w-3 h-3 rounded-full bg-av-success"></div>
          <span className="text-xs text-white/50">Object</span>
        </div>
        <div className="flex items-center gap-2">
          <div className="w-3 h-3 rounded-full bg-av-danger"></div>
          <span className="text-xs text-white/50">Threat</span>
        </div>
        <div className="flex items-center gap-2">
          <div className="w-3 h-3 rounded-full border border-av-primary"></div>
          <span className="text-xs text-white/50">Scan</span>
        </div>
      </div>
    </div>
  )
}

export default LandingPage
