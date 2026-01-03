import { useCallback, useRef, useEffect, useState } from 'react'

// Apple-style 60fps animation hook using RAF
export const useAnimationFrame = (callback, deps = []) => {
  const requestRef = useRef()
  const previousTimeRef = useRef()

  const animate = useCallback((time) => {
    if (previousTimeRef.current !== undefined) {
      const deltaTime = time - previousTimeRef.current
      callback(deltaTime, time)
    }
    previousTimeRef.current = time
    requestRef.current = requestAnimationFrame(animate)
  }, [callback])

  useEffect(() => {
    requestRef.current = requestAnimationFrame(animate)
    return () => cancelAnimationFrame(requestRef.current)
  }, deps)
}

// Throttle updates for performance (Apple uses 16ms minimum = 60fps)
export const useThrottledValue = (value, delay = 16) => {
  const lastUpdate = useRef(Date.now())
  const lastValue = useRef(value)

  if (Date.now() - lastUpdate.current >= delay) {
    lastValue.current = value
    lastUpdate.current = Date.now()
  }

  return lastValue.current
}

// Debounced callback for expensive operations
export const useDebouncedCallback = (callback, delay = 100) => {
  const timeoutRef = useRef()

  return useCallback((...args) => {
    if (timeoutRef.current) clearTimeout(timeoutRef.current)
    timeoutRef.current = setTimeout(() => callback(...args), delay)
  }, [callback, delay])
}

// Intersection observer for lazy loading (Apple-style progressive loading)
export const useLazyLoad = (ref, options = {}) => {
  const [isVisible, setIsVisible] = useState(false)

  useEffect(() => {
    if (!ref.current) return

    const observer = new IntersectionObserver(([entry]) => {
      if (entry.isIntersecting) {
        setIsVisible(true)
        observer.disconnect()
      }
    }, { threshold: 0.1, ...options })

    observer.observe(ref.current)
    return () => observer.disconnect()
  }, [ref])

  return isVisible
}

// Double buffering for Canvas rendering (prevents flicker)
export const useDoubleBuffer = (width, height) => {
  const offscreenRef = useRef(null)

  useEffect(() => {
    offscreenRef.current = document.createElement('canvas')
    offscreenRef.current.width = width
    offscreenRef.current.height = height
  }, [width, height])

  const getOffscreenContext = useCallback(() => {
    return offscreenRef.current?.getContext('2d')
  }, [])

  const copyToMain = useCallback((mainCtx) => {
    if (offscreenRef.current) {
      mainCtx.drawImage(offscreenRef.current, 0, 0)
    }
  }, [])

  return { getOffscreenContext, copyToMain }
}

// Object pool for reducing garbage collection (critical for 60fps)
export const useObjectPool = (factory, initialSize = 100) => {
  const poolRef = useRef([])

  useEffect(() => {
    poolRef.current = Array.from({ length: initialSize }, factory)
  }, [factory, initialSize])

  const acquire = useCallback(() => {
    return poolRef.current.pop() || factory()
  }, [factory])

  const release = useCallback((obj) => {
    poolRef.current.push(obj)
  }, [])

  return { acquire, release }
}

// Preload critical assets
export const preloadAssets = (urls) => {
  urls.forEach(url => {
    const link = document.createElement('link')
    link.rel = 'preload'
    link.href = url
    link.as = url.endsWith('.js') ? 'script' : 'fetch'
    document.head.appendChild(link)
  })
}

// Check for reduced motion preference (accessibility)
export const prefersReducedMotion = () => {
  return window.matchMedia('(prefers-reduced-motion: reduce)').matches
}

export default {
  useAnimationFrame,
  useThrottledValue,
  useDebouncedCallback,
  useLazyLoad,
  useDoubleBuffer,
  useObjectPool,
  preloadAssets,
  prefersReducedMotion,
}
