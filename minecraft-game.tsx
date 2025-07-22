"use client"

import { useRef, useState, useCallback, useEffect, useMemo } from "react"
import { Canvas, useFrame, useThree } from "@react-three/fiber"
import { PointerLockControls, KeyboardControls, useKeyboardControls } from "@react-three/drei"
import * as THREE from "three"
import { create } from "zustand"

// Noise function for terrain generation
function noise(x: number, z: number): number {
  return Math.sin(x * 0.1) * Math.cos(z * 0.1) * 5 + Math.sin(x * 0.05) * Math.cos(z * 0.05) * 10
}

// Block types
const BLOCK_TYPES = {
  AIR: 0,
  GRASS: 1,
  DIRT: 2,
  STONE: 3,
  WOOD: 4,
  LEAVES: 5,
} as const

const BLOCK_COLORS = {
  [BLOCK_TYPES.GRASS]: "#4a7c59",
  [BLOCK_TYPES.DIRT]: "#8b4513",
  [BLOCK_TYPES.STONE]: "#696969",
  [BLOCK_TYPES.WOOD]: "#daa520",
  [BLOCK_TYPES.LEAVES]: "#228b22",
}

// World store
interface WorldStore {
  blocks: Map<string, number>
  getBlock: (x: number, y: number, z: number) => number
  setBlock: (x: number, y: number, z: number, type: number) => void
  generateChunk: (chunkX: number, chunkZ: number) => void
}

const useWorldStore = create<WorldStore>((set, get) => ({
  blocks: new Map(),
  getBlock: (x: number, y: number, z: number) => {
    const key = `${x},${y},${z}`
    return get().blocks.get(key) || BLOCK_TYPES.AIR
  },
  setBlock: (x: number, y: number, z: number, type: number) => {
    set((state) => {
      const newBlocks = new Map(state.blocks)
      const key = `${x},${y},${z}`
      if (type === BLOCK_TYPES.AIR) {
        newBlocks.delete(key)
      } else {
        newBlocks.set(key, type)
      }
      return { blocks: newBlocks }
    })
  },
  generateChunk: (chunkX: number, chunkZ: number) => {
    const { blocks, setBlock } = get()
    const CHUNK_SIZE = 16

    for (let x = 0; x < CHUNK_SIZE; x++) {
      for (let z = 0; z < CHUNK_SIZE; z++) {
        const worldX = chunkX * CHUNK_SIZE + x
        const worldZ = chunkZ * CHUNK_SIZE + z
        const height = Math.floor(noise(worldX, worldZ)) + 32

        // Generate terrain layers
        for (let y = 0; y <= height; y++) {
          let blockType = BLOCK_TYPES.STONE
          if (y === height) {
            blockType = BLOCK_TYPES.GRASS
          } else if (y > height - 3) {
            blockType = BLOCK_TYPES.DIRT
          }

          setBlock(worldX, y, worldZ, blockType)
        }

        // Add some trees
        if (Math.random() < 0.02 && height > 30) {
          const treeHeight = 4 + Math.floor(Math.random() * 3)
          // Tree trunk
          for (let y = height + 1; y <= height + treeHeight; y++) {
            setBlock(worldX, y, worldZ, BLOCK_TYPES.WOOD)
          }
          // Tree leaves
          for (let dx = -2; dx <= 2; dx++) {
            for (let dz = -2; dz <= 2; dz++) {
              for (let dy = 0; dy <= 2; dy++) {
                if (Math.abs(dx) + Math.abs(dz) + Math.abs(dy) <= 3) {
                  setBlock(worldX + dx, height + treeHeight + dy, worldZ + dz, BLOCK_TYPES.LEAVES)
                }
              }
            }
          }
        }
      }
    }
  },
}))

// Block component
function Block({ position, type, onBreak }: { position: [number, number, number]; type: number; onBreak: () => void }) {
  const meshRef = useRef<THREE.Mesh>(null)
  const [hovered, setHovered] = useState(false)

  const color = BLOCK_COLORS[type as keyof typeof BLOCK_COLORS] || "#ffffff"

  return (
    <mesh
      ref={meshRef}
      position={position}
      onPointerEnter={() => setHovered(true)}
      onPointerLeave={() => setHovered(false)}
      onClick={onBreak}
    >
      <boxGeometry args={[1, 1, 1]} />
      <meshLambertMaterial color={hovered ? "#ffffff" : color} transparent={hovered} opacity={hovered ? 0.8 : 1} />
    </mesh>
  )
}

// Player component with movement and interaction
function Player() {
  const { camera } = useThree()
  const [, getKeys] = useKeyboardControls()
  const { getBlock, setBlock } = useWorldStore()
  const [selectedBlock, setSelectedBlock] = useState(BLOCK_TYPES.GRASS)

  const velocity = useRef(new THREE.Vector3())
  const direction = useRef(new THREE.Vector3())
  const isOnGround = useRef(false)

  // Raycaster for block interaction
  const raycaster = useMemo(() => new THREE.Raycaster(), [])

  useFrame((state, delta) => {
    const { forward, backward, left, right, jump } = getKeys()

    // Movement
    const speed = 5
    direction.current.set(0, 0, 0)

    if (forward) direction.current.z -= 1
    if (backward) direction.current.z += 1
    if (left) direction.current.x -= 1
    if (right) direction.current.x += 1

    direction.current.normalize()
    direction.current.applyQuaternion(camera.quaternion)
    direction.current.y = 0
    direction.current.normalize()

    velocity.current.x = direction.current.x * speed
    velocity.current.z = direction.current.z * speed

    // Gravity and jumping
    if (jump && isOnGround.current) {
      velocity.current.y = 8
      isOnGround.current = false
    }

    velocity.current.y -= 25 * delta // gravity

    // Apply movement with collision detection
    const newPosition = camera.position.clone()
    newPosition.add(velocity.current.clone().multiplyScalar(delta))

    // Simple collision detection
    const playerBox = new THREE.Box3().setFromCenterAndSize(newPosition, new THREE.Vector3(0.8, 1.8, 0.8))

    let collision = false
    const minX = Math.floor(playerBox.min.x)
    const maxX = Math.floor(playerBox.max.x)
    const minY = Math.floor(playerBox.min.y)
    const maxY = Math.floor(playerBox.max.y)
    const minZ = Math.floor(playerBox.min.z)
    const maxZ = Math.floor(playerBox.max.z)

    for (let x = minX; x <= maxX; x++) {
      for (let y = minY; y <= maxY; y++) {
        for (let z = minZ; z <= maxZ; z++) {
          if (getBlock(x, y, z) !== BLOCK_TYPES.AIR) {
            const blockBox = new THREE.Box3().setFromCenterAndSize(
              new THREE.Vector3(x + 0.5, y + 0.5, z + 0.5),
              new THREE.Vector3(1, 1, 1),
            )
            if (playerBox.intersectsBox(blockBox)) {
              collision = true
              break
            }
          }
        }
        if (collision) break
      }
      if (collision) break
    }

    if (!collision) {
      camera.position.copy(newPosition)
    } else {
      velocity.current.y = 0
    }

    // Ground detection
    const groundY = Math.floor(camera.position.y - 0.9)
    isOnGround.current =
      getBlock(Math.floor(camera.position.x), groundY, Math.floor(camera.position.z)) !== BLOCK_TYPES.AIR

    if (isOnGround.current && velocity.current.y < 0) {
      velocity.current.y = 0
    }
  })

  // Block interaction
  const handleClick = useCallback(
    (event: MouseEvent) => {
      if (document.pointerLockElement) {
        raycaster.setFromCamera(new THREE.Vector2(0, 0), camera)

        // Find intersected block
        const intersections: Array<{ point: THREE.Vector3; distance: number; blockPos: THREE.Vector3 }> = []

        for (let i = 0; i < 100; i++) {
          const point = raycaster.ray.origin.clone().add(raycaster.ray.direction.clone().multiplyScalar(i * 0.1))
          const blockPos = new THREE.Vector3(Math.floor(point.x), Math.floor(point.y), Math.floor(point.z))

          if (getBlock(blockPos.x, blockPos.y, blockPos.z) !== BLOCK_TYPES.AIR) {
            intersections.push({ point, distance: i * 0.1, blockPos })
            break
          }
        }

        if (intersections.length > 0) {
          const hit = intersections[0]

          if (event.button === 0) {
            // Left click - break block
            setBlock(hit.blockPos.x, hit.blockPos.y, hit.blockPos.z, BLOCK_TYPES.AIR)
          } else if (event.button === 2) {
            // Right click - place block
            const placePos = hit.point.clone().sub(raycaster.ray.direction.clone().multiplyScalar(0.1))
            const blockPos = new THREE.Vector3(Math.floor(placePos.x), Math.floor(placePos.y), Math.floor(placePos.z))

            if (getBlock(blockPos.x, blockPos.y, blockPos.z) === BLOCK_TYPES.AIR) {
              setBlock(blockPos.x, blockPos.y, blockPos.z, selectedBlock)
            }
          }
        }
      }
    },
    [camera, raycaster, getBlock, setBlock, selectedBlock],
  )

  useEffect(() => {
    document.addEventListener("mousedown", handleClick)
    return () => document.removeEventListener("mousedown", handleClick)
  }, [handleClick])

  // Block selection with number keys
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      const key = Number.parseInt(event.key)
      if (key >= 1 && key <= 5) {
        setSelectedBlock(key)
      }
    }

    document.addEventListener("keydown", handleKeyDown)
    return () => document.removeEventListener("keydown", handleKeyDown)
  }, [])

  return null
}

// World renderer
function World() {
  const { blocks, generateChunk } = useWorldStore()
  const { camera } = useThree()
  const [renderedChunks, setRenderedChunks] = useState(new Set<string>())

  useFrame(() => {
    const chunkX = Math.floor(camera.position.x / 16)
    const chunkZ = Math.floor(camera.position.z / 16)
    const renderDistance = 2

    for (let dx = -renderDistance; dx <= renderDistance; dx++) {
      for (let dz = -renderDistance; dz <= renderDistance; dz++) {
        const cx = chunkX + dx
        const cz = chunkZ + dz
        const chunkKey = `${cx},${cz}`

        if (!renderedChunks.has(chunkKey)) {
          generateChunk(cx, cz)
          setRenderedChunks((prev) => new Set(prev).add(chunkKey))
        }
      }
    }
  })

  const visibleBlocks = useMemo(() => {
    const result: Array<{ position: [number, number, number]; type: number; key: string }> = []
    const playerPos = new THREE.Vector3(0, 32, 0) // Default position

    blocks.forEach((type, key) => {
      const [x, y, z] = key.split(",").map(Number)
      const distance = Math.sqrt((x - playerPos.x) ** 2 + (y - playerPos.y) ** 2 + (z - playerPos.z) ** 2)

      if (distance < 50 && type !== BLOCK_TYPES.AIR) {
        result.push({ position: [x, y, z], type, key })
      }
    })

    return result
  }, [blocks])

  return (
    <>
      {visibleBlocks.map(({ position, type, key }) => (
        <Block
          key={key}
          position={position}
          type={type}
          onBreak={() => useWorldStore.getState().setBlock(position[0], position[1], position[2], BLOCK_TYPES.AIR)}
        />
      ))}
    </>
  )
}

// HUD component
function HUD() {
  const [selectedBlock, setSelectedBlock] = useState(1)

  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      const key = Number.parseInt(event.key)
      if (key >= 1 && key <= 5) {
        setSelectedBlock(key)
      }
    }

    document.addEventListener("keydown", handleKeyDown)
    return () => document.removeEventListener("keydown", handleKeyDown)
  }, [])

  return (
    <div className="fixed bottom-4 left-1/2 transform -translate-x-1/2 flex gap-2 bg-black/50 p-2 rounded">
      {[1, 2, 3, 4, 5].map((blockType) => (
        <div
          key={blockType}
          className={`w-12 h-12 border-2 rounded cursor-pointer ${
            selectedBlock === blockType ? "border-white" : "border-gray-500"
          }`}
          style={{ backgroundColor: BLOCK_COLORS[blockType as keyof typeof BLOCK_COLORS] }}
          onClick={() => setSelectedBlock(blockType)}
        >
          <div className="text-white text-xs text-center mt-1">{blockType}</div>
        </div>
      ))}
    </div>
  )
}

// Instructions component
function Instructions() {
  const [showInstructions, setShowInstructions] = useState(true)

  if (!showInstructions) {
    return (
      <button
        className="fixed top-4 left-4 bg-black/50 text-white p-2 rounded"
        onClick={() => setShowInstructions(true)}
      >
        Show Controls
      </button>
    )
  }

  return (
    <div className="fixed top-4 left-4 bg-black/80 text-white p-4 rounded max-w-xs">
      <button className="float-right text-xl" onClick={() => setShowInstructions(false)}>
        ×
      </button>
      <h3 className="font-bold mb-2">Controls:</h3>
      <ul className="text-sm space-y-1">
        <li>WASD - Move</li>
        <li>Mouse - Look around</li>
        <li>Space - Jump</li>
        <li>Left Click - Break block</li>
        <li>Right Click - Place block</li>
        <li>1-5 - Select block type</li>
        <li>Click to lock cursor</li>
      </ul>
    </div>
  )
}

export default function MinecraftGame() {
  return (
    <div className="w-full h-screen bg-sky-400">
      <KeyboardControls
        map={[
          { name: "forward", keys: ["ArrowUp", "w", "W"] },
          { name: "backward", keys: ["ArrowDown", "s", "S"] },
          { name: "left", keys: ["ArrowLeft", "a", "A"] },
          { name: "right", keys: ["ArrowRight", "d", "D"] },
          { name: "jump", keys: ["Space"] },
        ]}
      >
        <Canvas
          camera={{ fov: 75, near: 0.1, far: 1000, position: [0, 35, 0] }}
          onContextMenu={(e) => e.preventDefault()}
        >
          <PointerLockControls />
          <ambientLight intensity={0.6} />
          <directionalLight position={[50, 50, 50]} intensity={0.8} castShadow />
          <fog attach="fog" args={["#87CEEB", 50, 200]} />

          <Player />
          <World />
        </Canvas>

        <HUD />
        <Instructions />
      </KeyboardControls>
    </div>
  )
}
