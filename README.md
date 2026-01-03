# DriveShield

Autonomous vehicle threat detection platform. Real-time sensor monitoring, adversarial attack detection, and perception system security visualization.

![React](https://img.shields.io/badge/React-18.2-61DAFB?style=flat-square&logo=react)
![Vite](https://img.shields.io/badge/Vite-5.0-646CFF?style=flat-square&logo=vite)
![TailwindCSS](https://img.shields.io/badge/Tailwind-3.4-06B6D4?style=flat-square&logo=tailwindcss)
![Vercel](https://img.shields.io/badge/Deployment-Vercel-black?style=flat-square&logo=vercel)
![License](https://img.shields.io/badge/License-MIT-green?style=flat-square)

## Live Demo

:link: **[View Live Demo on Vercel](https://drive-shield.vercel.app)**

## Features

### LIDAR Monitoring
- Real-time point cloud visualization
- Object detection and tracking
- Adversarial anomaly detection

### Camera Feed Analysis
- Multi-camera fusion display
- Bounding box detection visualization
- Classification confidence monitoring

### Threat Detection
- Adversarial patch identification
- Phantom object detection
- Sensor spoofing alerts
- GPS manipulation warnings

### Sensor Dashboard
- Multi-sensor health monitoring
- Confidence score tracking
- Real-time detection logging

## Architecture

```mermaid
graph TB
    subgraph Frontend [Vercel]
        Landing[Landing Page]
        Dashboard[Monitoring Dashboard]

        subgraph Visualizations
            LIDAR[LIDAR Point Cloud]
            Radar[Radar Sweep]
            Camera[Camera Feed]
            Charts[Confidence Charts]
        end
    end

    subgraph API [Vercel Serverless]
        SensorsAPI[/api/sensors-status]
        DetectionsAPI[/api/detections]
        LidarAPI[/api/lidar-points]
        ThreatAPI[/api/threat-analyze]
        SummaryAPI[/api/threats-summary]
    end

    Landing --> Dashboard
    Dashboard --> Visualizations
    Dashboard --> API
```

## Tech Stack

| Category | Technology |
|----------|------------|
| Framework | React 18 |
| Build Tool | Vite 5 |
| Styling | Tailwind CSS |
| Animations | Framer Motion |
| Charts | Recharts |
| Visualization | Canvas API |
| Deployment | Vercel (Frontend + Serverless Functions) |

## Quick Start

```bash
# Clone repository
git clone https://github.com/ryanwelchtech/drive-shield.git
cd drive-shield

# Install dependencies
npm install

# Start development server
npm run dev

# Build for production
npm run build

# Preview production build
npm run preview
```

## Deployment to Vercel

### Option 1: Vercel CLI

```bash
# Install Vercel CLI
npm i -g vercel

# Deploy
vercel
```

### Option 2: Git Integration

1. Push to GitHub
2. Import project in Vercel
3. Vercel auto-detects Vite configuration
4. Deploy

### Environment Variables

Create `.env.local`:

```env
VITE_USE_MOCK=true
VITE_API_URL=/api
```

## Threat Types Detected

| Threat | Description | Severity |
|--------|-------------|----------|
| Adversarial Patches | Physical patches that fool object detection | Critical |
| Phantom Objects | Non-existent objects injected into perception | High |
| Sensor Spoofing | Fake sensor data injection attacks | Critical |
| GPS Manipulation | Location data tampering | Medium |

## Design

Built with Apple 2026-inspired liquid glass UI:
- Glassmorphism with backdrop blur
- Cyan/teal color palette
- Smooth animations and transitions
- Dark mode optimized for monitoring
- Reduced motion support for accessibility

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/sensors-status` | GET | Get all sensor statuses |
| `/api/detections` | GET | Get object detections |
| `/api/lidar-points` | GET | Get LIDAR point cloud |
| `/api/threat-analyze` | POST | Analyze detection for threats |
| `/api/threats-summary` | GET | Get threat summary |
| `/api/detections-history` | GET | Get detection history |

## Performance Optimizations

- Code splitting with Vite
- Lazy-loaded components
- GPU-accelerated canvas rendering
- Reduced motion support
- Optimized re-renders with React.memo
- Throttled animation updates

## Author

**Ryan Welch** - Cloud & Systems Security Engineer

- Portfolio: [ryanwelchtech.com](https://ryanwelchtech.com)
- GitHub: [@ryanwelchtech](https://github.com/ryanwelchtech)
- LinkedIn: [Ryan Welch](https://linkedin.com/in/ryanwelchtech)

## License

MIT License
