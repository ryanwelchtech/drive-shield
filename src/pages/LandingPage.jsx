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
  const [detections, setDetections] = useState([])

  useEffect(() => {
    const interval = setInterval(() => {
      const newDetection = {
        id: Date.now(),
        angle: Math.random() * 360,
        distance: 30 + Math.random() * 70,
        type: Math.random() > 0.7 ? 'threat' : 'object',
      }
      setDetections(prev => [...prev.slice(-10), newDetection])
    }, 1500)
    return () => clearInterval(interval)
  }, [])

  return (
    <div className="relative">
      <div className="flex items-center justify-between mb-4">
        <h3 className="text-lg font-semibold">Perception Radar</h3>
        <span className="text-xs text-white/40">360° Coverage</span>
      </div>

      <div className="relative w-full aspect-square max-w-[300px] mx-auto">
        <svg className="w-full h-full" viewBox="0 0 200 200">
          {/* Radar Rings */}
          {[1, 2, 3, 4].map((ring) => (
            <circle
              key={ring}
              cx="100"
              cy="100"
              r={ring * 22}
              fill="none"
              stroke="rgba(6, 182, 212, 0.1)"
              strokeWidth="1"
            />
          ))}

          {/* Cross Lines */}
          <line x1="100" y1="10" x2="100" y2="190" stroke="rgba(6, 182, 212, 0.1)" strokeWidth="1"/>
          <line x1="10" y1="100" x2="190" y2="100" stroke="rgba(6, 182, 212, 0.1)" strokeWidth="1"/>

          {/* Sweep Line */}
          <motion.line
            x1="100"
            y1="100"
            x2="100"
            y2="10"
            stroke="rgba(6, 182, 212, 0.6)"
            strokeWidth="2"
            style={{ transformOrigin: '100px 100px' }}
            animate={{ rotate: 360 }}
            transition={{ duration: 4, repeat: Infinity, ease: 'linear' }}
          />

          {/* Sweep Gradient */}
          <motion.path
            d="M100,100 L100,10 A90,90 0 0,1 190,100 Z"
            fill="url(#sweepGradient)"
            style={{ transformOrigin: '100px 100px' }}
            animate={{ rotate: 360 }}
            transition={{ duration: 4, repeat: Infinity, ease: 'linear' }}
          />

          {/* Detections */}
          {detections.map((det) => {
            const x = 100 + det.distance * Math.cos((det.angle * Math.PI) / 180)
            const y = 100 + det.distance * Math.sin((det.angle * Math.PI) / 180)
            return (
              <motion.circle
                key={det.id}
                cx={x}
                cy={y}
                r="4"
                fill={det.type === 'threat' ? '#ef4444' : '#22c55e'}
                initial={{ opacity: 0, scale: 0 }}
                animate={{ opacity: [0, 1, 0.5], scale: [0, 1.5, 1] }}
                transition={{ duration: 2 }}
              />
            )
          })}

          {/* Center Point */}
          <circle cx="100" cy="100" r="6" fill="#06b6d4"/>
          <circle cx="100" cy="100" r="3" fill="#ffffff"/>

          {/* Gradient Definition */}
          <defs>
            <linearGradient id="sweepGradient" x1="0%" y1="0%" x2="100%" y2="0%">
              <stop offset="0%" stopColor="rgba(6, 182, 212, 0.3)"/>
              <stop offset="100%" stopColor="rgba(6, 182, 212, 0)"/>
            </linearGradient>
          </defs>
        </svg>

        {/* Range Labels */}
        <div className="absolute top-1/2 right-2 -translate-y-1/2 text-[10px] text-white/30">100m</div>
        <div className="absolute top-2 left-1/2 -translate-x-1/2 text-[10px] text-white/30">N</div>
      </div>

      {/* Legend */}
      <div className="flex justify-center gap-6 mt-4">
        <div className="flex items-center gap-2">
          <div className="w-3 h-3 rounded-full bg-av-success"></div>
          <span className="text-xs text-white/50">Object</span>
        </div>
        <div className="flex items-center gap-2">
          <div className="w-3 h-3 rounded-full bg-av-danger"></div>
          <span className="text-xs text-white/50">Threat</span>
        </div>
      </div>
    </div>
  )
}

export default LandingPage
