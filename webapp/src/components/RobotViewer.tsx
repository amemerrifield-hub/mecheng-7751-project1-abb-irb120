import { useCallback, useEffect, useMemo, useRef } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { ABB_IRB120_FK_All, type JointVectorDeg } from '../kinematics/abb120'

interface RobotViewerProps {
  trajectory: JointVectorDeg[]
  playing: boolean
  viewPreset?: 'iso' | 'front' | 'side'
  onFrameChange?: (frameIndex: number, frameCount: number) => void
  onPlaybackComplete?: () => void
}

export function RobotViewer({ trajectory, playing, viewPreset = 'iso', onFrameChange, onPlaybackComplete }: RobotViewerProps) {
  const mountRef = useRef<HTMLDivElement | null>(null)
  const trailLineRef = useRef<THREE.Line | null>(null)
  const eeMarkerRef = useRef<THREE.Mesh | null>(null)
  const linkMeshesRef = useRef<THREE.Mesh[]>([])
  const jointHousingRef = useRef<THREE.Mesh[]>([])
  const toolAxesRef = useRef<THREE.Group | null>(null)
  const toolFlangeRef = useRef<THREE.Mesh | null>(null)
  const sceneRef = useRef<THREE.Scene | null>(null)
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null)
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null)
  const controlsRef = useRef<OrbitControls | null>(null)
  const playbackIndexRef = useRef(0)
  const completedRef = useRef(false)
  const onFrameChangeRef = useRef<RobotViewerProps['onFrameChange']>(undefined)
  const onPlaybackCompleteRef = useRef<RobotViewerProps['onPlaybackComplete']>(undefined)
  const lastGoodVectorsRef = useRef<THREE.Vector3[] | null>(null)

  const cameraTarget = useRef(new THREE.Vector3(0, 0.26, 0))

  const isFiniteNumber = useCallback((value: number) => Number.isFinite(value), [])

  const mapPositionKinToWorld = useCallback((x: number, y: number, z: number) => {
    return new THREE.Vector3(x / 1000, z / 1000, -y / 1000)
  }, [])

  const mapRotationKinToWorld = useCallback((rotation: number[][]) => {
    const basis = new THREE.Matrix3().set(
      1, 0, 0,
      0, 0, 1,
      0, -1, 0,
    )

    const kin = new THREE.Matrix3().set(
      rotation[0][0], rotation[0][1], rotation[0][2],
      rotation[1][0], rotation[1][1], rotation[1][2],
      rotation[2][0], rotation[2][1], rotation[2][2],
    )

    const basisT = basis.clone().transpose()
    const world = basis.clone().multiply(kin).multiply(basisT)
    const elements = world.elements
    const matrix4 = new THREE.Matrix4().set(
      elements[0], elements[3], elements[6], 0,
      elements[1], elements[4], elements[7], 0,
      elements[2], elements[5], elements[8], 0,
      0, 0, 0, 1,
    )
    const quaternion = new THREE.Quaternion()
    quaternion.setFromRotationMatrix(matrix4)
    return quaternion
  }, [])

  useEffect(() => {
    onFrameChangeRef.current = onFrameChange
  }, [onFrameChange])

  useEffect(() => {
    onPlaybackCompleteRef.current = onPlaybackComplete
  }, [onPlaybackComplete])

  const transformsByFrame = useMemo(() => {
    if (trajectory.length === 0) {
      return []
    }
    const frames = trajectory
      .map((q) => {
        try {
          return ABB_IRB120_FK_All(q)
        } catch {
          return null
        }
      })
      .filter((frame): frame is ReturnType<typeof ABB_IRB120_FK_All> => frame !== null)
    return frames
  }, [trajectory])

  const createFrameArrows = useCallback((length: number) => {
    const frame = new THREE.Group()
    const origin = new THREE.Vector3(0, 0, 0)
    const headLength = Math.max(0.035, length * 0.25)
    const headWidth = Math.max(0.018, length * 0.14)

    const xArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), origin, length, 0xff3b30, headLength, headWidth)
    const yArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 1, 0), origin, length, 0x22c55e, headLength, headWidth)
    const zArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), origin, length, 0x2563eb, headLength, headWidth)

    frame.add(xArrow, yArrow, zArrow)
    return frame
  }, [])

  useEffect(() => {
    const mount = mountRef.current
    if (!mount) {
      return
    }

    const width = mount.clientWidth
    const height = Math.max(320, mount.clientHeight)

    const scene = new THREE.Scene()
    scene.background = new THREE.Color(0x1f2c80)
    scene.fog = new THREE.Fog(0x2d3b91, 1.4, 3.6)

    const camera = new THREE.PerspectiveCamera(45, width / height, 0.01, 10)
    camera.position.set(1.05, 0.92, 1.15)
    camera.lookAt(cameraTarget.current)

    const renderer = new THREE.WebGLRenderer({ antialias: true })
    renderer.outputColorSpace = THREE.SRGBColorSpace
    renderer.setSize(width, height)
    mount.appendChild(renderer.domElement)

    const controls = new OrbitControls(camera, renderer.domElement)
    controls.enableDamping = true
    controls.dampingFactor = 0.08
    controls.target.copy(cameraTarget.current)
    controls.minDistance = 0.45
    controls.maxDistance = 3.2
    controls.update()

    const grid = new THREE.GridHelper(2.2, 22, 0x93a4ff, 0x5b6ac8)
    grid.position.y = -0.002
    scene.add(grid)

    const floor = new THREE.Mesh(
      new THREE.CircleGeometry(1.2, 64),
      new THREE.MeshStandardMaterial({ color: 0x4453ae, roughness: 0.92, metalness: 0.02 }),
    )
    floor.rotation.x = -Math.PI / 2
    floor.position.y = -0.001
    scene.add(floor)

    const ambient = new THREE.AmbientLight(0xf8f9ff, 0.66)
    scene.add(ambient)

    const keyLight = new THREE.DirectionalLight(0xffffff, 0.88)
    keyLight.position.set(1.2, 2.1, 1.2)
    scene.add(keyLight)

    const fillLight = new THREE.DirectionalLight(0xbdd0ff, 0.44)
    fillLight.position.set(-1.3, 1.4, -0.7)
    scene.add(fillLight)

    const staticBaseMaterial = new THREE.MeshStandardMaterial({
      color: 0xeff1ff,
      roughness: 0.35,
      metalness: 0.1,
      emissive: 0x11142c,
      emissiveIntensity: 0.05,
    })
    const bodyMaterial = new THREE.MeshStandardMaterial({
      color: 0xf7f8ff,
      roughness: 0.29,
      metalness: 0.08,
      emissive: 0x0f1328,
      emissiveIntensity: 0.05,
    })
    const jointMaterial = new THREE.MeshStandardMaterial({ color: 0xcfd6ee, roughness: 0.5, metalness: 0.12 })
    const darkMaterial = new THREE.MeshStandardMaterial({ color: 0x2f3749, roughness: 0.42, metalness: 0.32 })

    const pedestal = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.132, 0.24, 36), staticBaseMaterial)
    pedestal.position.set(0, 0.12, 0)
    scene.add(pedestal)

    const baseSkirt = new THREE.Mesh(new THREE.CylinderGeometry(0.155, 0.165, 0.03, 36), staticBaseMaterial)
    baseSkirt.position.set(0, 0.015, 0)
    scene.add(baseSkirt)

    const baseSideBox = new THREE.Mesh(new THREE.BoxGeometry(0.11, 0.12, 0.16), staticBaseMaterial)
    baseSideBox.position.set(0.12, 0.09, 0)
    scene.add(baseSideBox)

    const trailMaterial = new THREE.LineBasicMaterial({ color: 0xff6b00 })
    const trailGeometry = new THREE.BufferGeometry().setFromPoints([])
    const trailLine = new THREE.Line(trailGeometry, trailMaterial)
    scene.add(trailLine)

    const linkGeometry = new THREE.CylinderGeometry(1, 1, 1, 30)
    const linkMeshes: THREE.Mesh[] = []
    for (let idx = 0; idx < 6; idx += 1) {
      const material = idx < 4 ? bodyMaterial : jointMaterial
      const linkMesh = new THREE.Mesh(linkGeometry, material)
      scene.add(linkMesh)
      linkMeshes.push(linkMesh)
    }

    const jointGeometry = new THREE.SphereGeometry(1, 22, 20)
    const jointScales = [0.084, 0.08, 0.073, 0.061, 0.052, 0.046, 0.036]
    const jointMeshes: THREE.Mesh[] = []
    for (let idx = 0; idx < 7; idx += 1) {
      const jointMesh = new THREE.Mesh(jointGeometry, jointMaterial)
      jointMesh.scale.setScalar(jointScales[idx])
      scene.add(jointMesh)
      jointMeshes.push(jointMesh)
    }

    const shoulderCap = new THREE.Mesh(new THREE.CylinderGeometry(0.065, 0.065, 0.058, 24), bodyMaterial)
    scene.add(shoulderCap)
    jointMeshes.push(shoulderCap)

    const toolFlange = new THREE.Mesh(new THREE.CylinderGeometry(0.038, 0.038, 0.03, 24), darkMaterial)
    scene.add(toolFlange)

    const markerGeometry = new THREE.SphereGeometry(0.025, 16, 16)
    const markerMaterial = new THREE.MeshStandardMaterial({ color: 0xff6b00 })
    const eeMarker = new THREE.Mesh(markerGeometry, markerMaterial)
    eeMarker.visible = false
    scene.add(eeMarker)

    const baseAxes = createFrameArrows(0.24)
    baseAxes.position.set(0, 0.02, 0)
    scene.add(baseAxes)

    const toolAxes = createFrameArrows(0.24)
    scene.add(toolAxes)

    sceneRef.current = scene
    cameraRef.current = camera
    rendererRef.current = renderer
    controlsRef.current = controls
    trailLineRef.current = trailLine
    eeMarkerRef.current = eeMarker
    linkMeshesRef.current = linkMeshes
    jointHousingRef.current = jointMeshes
    toolAxesRef.current = toolAxes
    toolFlangeRef.current = toolFlange

    const onResize = () => {
      if (!mountRef.current || !cameraRef.current || !rendererRef.current) {
        return
      }
      const nextW = mountRef.current.clientWidth
      const nextH = Math.max(320, mountRef.current.clientHeight)
      cameraRef.current.aspect = nextW / nextH
      cameraRef.current.updateProjectionMatrix()
      rendererRef.current.setSize(nextW, nextH)
      controlsRef.current?.update()
    }

    window.addEventListener('resize', onResize)

    const renderStatic = () => {
      if (rendererRef.current && sceneRef.current && cameraRef.current) {
        controlsRef.current?.update()
        rendererRef.current.render(sceneRef.current, cameraRef.current)
      }
    }
    controls.addEventListener('change', renderStatic)
    renderStatic()

    return () => {
      window.removeEventListener('resize', onResize)
      controls.removeEventListener('change', renderStatic)
      trailGeometry.dispose()
      trailMaterial.dispose()
      jointGeometry.dispose()
      linkGeometry.dispose()
      markerGeometry.dispose()
      markerMaterial.dispose()
      staticBaseMaterial.dispose()
      bodyMaterial.dispose()
      jointMaterial.dispose()
      darkMaterial.dispose()
      controls.dispose()
      renderer.dispose()
      if (mount.contains(renderer.domElement)) {
        mount.removeChild(renderer.domElement)
      }
    }
  }, [createFrameArrows])

  useEffect(() => {
    const camera = cameraRef.current
    const controls = controlsRef.current
    if (!camera || !controls) {
      return
    }

    const target = cameraTarget.current
    if (viewPreset === 'front') {
      camera.position.set(0, 0.86, 1.95)
    } else if (viewPreset === 'side') {
      camera.position.set(1.95, 0.86, 0)
    } else {
      camera.position.set(1.05, 0.92, 1.15)
    }

    controls.target.copy(target)
    camera.lookAt(target)
    controls.update()
    if (rendererRef.current && sceneRef.current) {
      rendererRef.current.render(sceneRef.current, camera)
    }
  }, [viewPreset])

  const updateRobotMeshes = useCallback((vectors: THREE.Vector3[]) => {
    const joints = jointHousingRef.current
    const links = linkMeshesRef.current
    const yAxis = new THREE.Vector3(0, 1, 0)
    const linkRadii = [0.074, 0.062, 0.056, 0.046, 0.038, 0.029]

    for (let idx = 0; idx < 7 && idx < vectors.length; idx += 1) {
      joints[idx].position.copy(vectors[idx])
    }

    if (joints[7] && vectors[1]) {
      joints[7].position.copy(vectors[1])
      joints[7].rotation.z = Math.PI / 2
    }

    for (let idx = 0; idx < links.length && idx + 1 < vectors.length; idx += 1) {
      const start = vectors[idx]
      const end = vectors[idx + 1]
      const direction = new THREE.Vector3().subVectors(end, start)
      const length = direction.length()
      const midpoint = new THREE.Vector3().addVectors(start, end).multiplyScalar(0.5)

      links[idx].position.copy(midpoint)
      if (length > 1e-8) {
        direction.normalize()
        links[idx].quaternion.setFromUnitVectors(yAxis, direction)
      }
      const radius = linkRadii[idx] ?? 0.03
      links[idx].scale.set(radius, Math.max(length * 0.9, 1e-6), radius)
    }
  }, [])

  const drawFrame = useCallback((frameIndex: number) => {
    if (!rendererRef.current || !sceneRef.current || !cameraRef.current || transformsByFrame.length === 0) {
      return
    }

    try {
      const clampedIndex = ((frameIndex % transformsByFrame.length) + transformsByFrame.length) % transformsByFrame.length
      const frame = transformsByFrame[clampedIndex]
      const vectors = frame.map((transform) => {
        const tx = transform[0][3]
        const ty = transform[1][3]
        const tz = transform[2][3]
        return mapPositionKinToWorld(tx, ty, tz)
      })

      const hasInvalidPoint = vectors.some((vec) => !isFiniteNumber(vec.x) || !isFiniteNumber(vec.y) || !isFiniteNumber(vec.z))
      if (hasInvalidPoint) {
        if (lastGoodVectorsRef.current) {
          updateRobotMeshes(lastGoodVectorsRef.current)
        }
        rendererRef.current.render(sceneRef.current, cameraRef.current)
        return
      }

      lastGoodVectorsRef.current = vectors
      updateRobotMeshes(vectors)

      const trail = transformsByFrame.slice(0, clampedIndex + 1).map((matrices) => {
        const ee = matrices[matrices.length - 1]
        return mapPositionKinToWorld(ee[0][3], ee[1][3], ee[2][3])
      })
      const trailGeometry = trailLineRef.current?.geometry as THREE.BufferGeometry | undefined
      trailGeometry?.setFromPoints(trail)

      const ee = vectors[vectors.length - 1]
      if (eeMarkerRef.current) {
        eeMarkerRef.current.visible = true
        eeMarkerRef.current.position.set(ee.x, ee.y, ee.z)
      }

      const toolAxes = toolAxesRef.current
      const toolFlange = toolFlangeRef.current
      const finalFrame = frame[frame.length - 1]
      if (toolAxes && finalFrame) {
        const toolPosition = mapPositionKinToWorld(finalFrame[0][3], finalFrame[1][3], finalFrame[2][3])
        const toolQuaternion = mapRotationKinToWorld([
          [finalFrame[0][0], finalFrame[0][1], finalFrame[0][2]],
          [finalFrame[1][0], finalFrame[1][1], finalFrame[1][2]],
          [finalFrame[2][0], finalFrame[2][1], finalFrame[2][2]],
        ])
        toolAxes.position.copy(toolPosition)
        toolAxes.quaternion.copy(toolQuaternion)

        if (toolFlange) {
          toolFlange.position.copy(toolPosition)
          toolFlange.quaternion.copy(toolQuaternion)
        }
      }

      onFrameChangeRef.current?.(clampedIndex, transformsByFrame.length)
      controlsRef.current?.update()
      rendererRef.current.render(sceneRef.current, cameraRef.current)
    } catch {
      rendererRef.current.render(sceneRef.current, cameraRef.current)
    }
  }, [isFiniteNumber, mapPositionKinToWorld, mapRotationKinToWorld, transformsByFrame, updateRobotMeshes])

  useEffect(() => {
    if (!trailLineRef.current || !rendererRef.current || !sceneRef.current || !cameraRef.current) {
      return
    }

    playbackIndexRef.current = 0
    completedRef.current = false
    drawFrame(0)
  }, [drawFrame, transformsByFrame])

  useEffect(() => {
    if (!playing || transformsByFrame.length === 0) {
      return
    }

    if (completedRef.current) {
      playbackIndexRef.current = 0
      completedRef.current = false
      drawFrame(0)
    }

    const frameIntervalMs = 40
    const timer = window.setInterval(() => {
      if (playbackIndexRef.current >= transformsByFrame.length - 1) {
        completedRef.current = true
        window.clearInterval(timer)
        onPlaybackCompleteRef.current?.()
        return
      }

      playbackIndexRef.current += 1
      drawFrame(playbackIndexRef.current)
    }, frameIntervalMs)

    return () => window.clearInterval(timer)
  }, [drawFrame, playing, transformsByFrame])

  useEffect(() => {
    if (playing || transformsByFrame.length === 0) {
      return
    }

    drawFrame(playbackIndexRef.current)
  }, [drawFrame, playing, transformsByFrame])

  return <div className="viewer" ref={mountRef} />
}
