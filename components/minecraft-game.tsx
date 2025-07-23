"use client"

import React, { useRef, useCallback, useEffect, useMemo, memo } from "react"
import { useThree, invalidate } from "@react-three/fiber"
import { useKeyboardControls } from "@react-three/drei"
import * as THREE from "three"
import { create } from "zustand"
import { Canvas } from "@react-three/fiber"
import { KeyboardControls } from "@react-three/drei"

// Block types
const BLOCK_TYPES = {
  AIR: 0,
  GRASS: 1,
  DIRT: 2,
  STONE: 3,
  WOOD: 4,
  LEAVES: 5,
} as const

// Chunk constants
const CHUNK_SIZE = 16
const CHUNK_HEIGHT = 64
const RENDER_DISTANCE = 6 // Increased for better tree visibility
const WORLD_HEIGHT = 64

// Performance store using Zustand
interface GameStore {
  chunks: Map<string, ChunkData>
  dirtyChunks: Set<string>
  playerPosition: THREE.Vector3
  selectedBlock: number
  isLocked: boolean
  gameStarted: boolean
  loadedChunks: Set<string>

  setChunk: (key: string, chunk: ChunkData) => void
  markChunkDirty: (key: string) => void
  setPlayerPosition: (pos: THREE.Vector3) => void
  setSelectedBlock: (block: number) => void
  setIsLocked: (locked: boolean) => void
  setGameStarted: (started: boolean) => void
  getBlock: (x: number, y: number, z: number) => number
  setBlock: (x: number, y: number, z: number, type: number) => void
  addLoadedChunk: (key: string) => void
}

const useGameStore = create<GameStore>((set, get) => ({
  chunks: new Map(),
  dirtyChunks: new Set(),
  playerPosition: new THREE.Vector3(0, 35, 0),
  selectedBlock: BLOCK_TYPES.GRASS,
  isLocked: false,
  gameStarted: false,
  loadedChunks: new Set(),

  setChunk: (key, chunk) =>
    set((state) => {
      const newChunks = new Map(state.chunks)
      newChunks.set(key, chunk)
      return { chunks: newChunks }
    }),

  markChunkDirty: (key) =>
    set((state) => {
      const chunk = state.chunks.get(key)
      if (!chunk || !chunk.blocks || chunk.blocks.length === 0) {
        console.error(`Cannot mark chunk ${key} as dirty: invalid chunk data`)
        return state
      }

      const newDirtyChunks = new Set(state.dirtyChunks)
      newDirtyChunks.add(key)
      console.log(`Marked chunk ${key} as dirty (blocks: ${chunk.blocks.length})`)
      return { dirtyChunks: newDirtyChunks }
    }),

  setPlayerPosition: (pos) => set({ playerPosition: pos }),
  setSelectedBlock: (block) => set({ selectedBlock: block }),
  setIsLocked: (locked) => set({ isLocked: locked }),
  setGameStarted: (started) => set({ gameStarted: started }),
  addLoadedChunk: (key) =>
    set((state) => ({
      loadedChunks: new Set(state.loadedChunks).add(key),
    })),

  // Proper block access across chunks
  getBlock: (x: number, y: number, z: number) => {
    const chunkX = Math.floor(x / CHUNK_SIZE)
    const chunkZ = Math.floor(z / CHUNK_SIZE)
    const chunkKey = `${chunkX},${chunkZ}`

    const chunk = get().chunks.get(chunkKey)
    if (!chunk || !chunk.blocks) return BLOCK_TYPES.AIR

    const localX = ((x % CHUNK_SIZE) + CHUNK_SIZE) % CHUNK_SIZE
    const localZ = ((z % CHUNK_SIZE) + CHUNK_SIZE) % CHUNK_SIZE

    if (y < 0 || y >= CHUNK_HEIGHT) return BLOCK_TYPES.AIR

    const index = localX + localZ * CHUNK_SIZE + y * CHUNK_SIZE * CHUNK_SIZE
    return chunk.blocks[index] || BLOCK_TYPES.AIR
  },

  setBlock: (x: number, y: number, z: number, type: number) => {
    const chunkX = Math.floor(x / CHUNK_SIZE)
    const chunkZ = Math.floor(z / CHUNK_SIZE)
    const chunkKey = `${chunkX},${chunkZ}`

    const chunk = get().chunks.get(chunkKey)
    if (!chunk || !chunk.blocks || !chunk.generated) {
      console.warn(`Cannot set block: chunk ${chunkKey} not ready`)
      return
    }

    const localX = ((x % CHUNK_SIZE) + CHUNK_SIZE) % CHUNK_SIZE
    const localZ = ((z % CHUNK_SIZE) + CHUNK_SIZE) % CHUNK_SIZE

    if (y < 0 || y >= CHUNK_HEIGHT) {
      console.warn(`Cannot set block: Y coordinate ${y} out of bounds`)
      return
    }

    const index = localX + localZ * CHUNK_SIZE + y * CHUNK_SIZE * CHUNK_SIZE
    const oldType = chunk.blocks[index]

    if (oldType !== type) {
      // FIXED: Ensure we're modifying the actual chunk data
      chunk.blocks[index] = type
      chunk.dirty = true

      console.log(`Block changed at ${x},${y},${z} from ${oldType} to ${type}`)
      console.log(`Chunk ${chunkKey} blocks array length:`, chunk.blocks.length)

      get().markChunkDirty(chunkKey)
    }
  },
}))

// Chunk data structure
interface ChunkData {
  blocks: Uint16Array
  mesh?: {
    vertices: Float32Array
    indices: Uint32Array
    uvs: Float32Array
    normals: Float32Array
  }
  dirty: boolean
  x: number
  z: number
  generated: boolean
}

// Custom game loop hook
function useGameLoop(callback: (delta: number) => void) {
  const requestRef = useRef<number>()
  const previousTimeRef = useRef<number>()

  const animate = useCallback(
    (time: number) => {
      if (previousTimeRef.current !== undefined) {
        const deltaTime = (time - previousTimeRef.current) / 1000
        callback(Math.min(deltaTime, 1 / 30)) // Cap at 30fps minimum
      }
      previousTimeRef.current = time
      requestRef.current = requestAnimationFrame(animate)
    },
    [callback],
  )

  useEffect(() => {
    requestRef.current = requestAnimationFrame(animate)
    return () => {
      if (requestRef.current) {
        cancelAnimationFrame(requestRef.current)
      }
    }
  }, [animate])
}

// FIXED: Web Worker with proper tree generation that doesn't span chunks
class WorldWorker {
  private worker: Worker | null = null
  private messageId = 0
  private pendingMessages = new Map<number, (data: any) => void>()

  constructor() {
    if (typeof Worker !== "undefined") {
      this.initWorker()
    }
  }

  private initWorker() {
    const workerCode = `
      // Perlin noise implementation
      class PerlinNoise {
        constructor(seed = 0) {
          this.seed = seed;
          this.p = new Array(512);
          this.permutation = [151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,190,6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,88,237,149,56,87,174,20,125,136,171,168,68,175,74,165,71,134,139,48,27,166,77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,102,143,54,65,25,63,161,1,216,80,73,209,76,132,187,208,89,18,169,200,196,135,130,116,188,159,86,164,100,109,198,173,186,3,64,52,217,226,250,124,123,5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,223,183,170,213,119,248,152,2,44,154,163,70,221,153,101,155,167,43,172,9,129,22,39,253,19,98,108,110,79,113,224,232,178,185,112,104,218,246,97,228,251,34,242,193,238,210,144,12,191,179,162,241,81,51,145,235,249,14,239,107,49,192,214,31,181,199,106,157,184,84,204,176,115,121,50,45,127,4,150,254,138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180];
          
          for (let i = 0; i < 256; i++) {
            this.p[256 + i] = this.p[i] = this.permutation[i];
          }
        }
        
        fade(t) {
          return t * t * t * (t * (t * 6 - 15) + 10);
        }
        
        lerp(t, a, b) {
          return a + t * (b - a);
        }
        
        grad(hash, x, y, z) {
          const h = hash & 15;
          const u = h < 8 ? x : y;
          const v = h < 4 ? y : h === 12 || h === 14 ? x : z;
          return ((h & 1) === 0 ? u : -u) + ((h & 2) === 0 ? v : -v);
        }
        
        noise(x, y, z) {
          const X = Math.floor(x) & 255;
          const Y = Math.floor(y) & 255;
          const Z = Math.floor(z) & 255;
          
          x -= Math.floor(x);
          y -= Math.floor(y);
          z -= Math.floor(z);
          
          const u = this.fade(x);
          const v = this.fade(y);
          const w = this.fade(z);
          
          const A = this.p[X] + Y;
          const AA = this.p[A] + Z;
          const AB = this.p[A + 1] + Z;
          const B = this.p[X + 1] + Y;
          const BA = this.p[B] + Z;
          const BB = this.p[B + 1] + Z;
          
          return this.lerp(w,
            this.lerp(v,
              this.lerp(u, this.grad(this.p[AA], x, y, z),
                          this.grad(this.p[BA], x - 1, y, z)),
              this.lerp(u, this.grad(this.p[AB], x, y - 1, z),
                          this.grad(this.p[BB], x - 1, y - 1, z))),
            this.lerp(v,
              this.lerp(u, this.grad(this.p[AA + 1], x, y, z - 1),
                          this.grad(this.p[BA + 1], x - 1, y, z - 1)),
              this.lerp(u, this.grad(this.p[AB + 1], x, y - 1, z - 1),
                          this.grad(this.p[BB + 1], x - 1, y - 1, z - 1))));
        }
      }
      
      const perlin = new PerlinNoise(12345);
      
      // FIXED: Generate chunk data with trees that don't span chunk boundaries
      function generateChunk(chunkX, chunkZ) {
        const CHUNK_SIZE = 16;
        const CHUNK_HEIGHT = 64;
        const blocks = new Uint16Array(CHUNK_SIZE * CHUNK_HEIGHT * CHUNK_SIZE);
        
        // First pass: Generate terrain
        for (let x = 0; x < CHUNK_SIZE; x++) {
          for (let z = 0; z < CHUNK_SIZE; z++) {
            const worldX = chunkX * CHUNK_SIZE + x;
            const worldZ = chunkZ * CHUNK_SIZE + z;
            
            // Multi-octave Perlin noise for terrain
            let height = 0;
            height += perlin.noise(worldX * 0.01, 0, worldZ * 0.01) * 32;
            height += perlin.noise(worldX * 0.02, 0, worldZ * 0.02) * 16;
            height += perlin.noise(worldX * 0.04, 0, worldZ * 0.04) * 8;
            height = Math.floor(height + 32);
            
            // Generate terrain layers
            for (let y = 0; y < CHUNK_HEIGHT && y <= height + 5; y++) {
              const index = x + z * CHUNK_SIZE + y * CHUNK_SIZE * CHUNK_SIZE;
              
              if (y <= height) {
                if (y === height) {
                  blocks[index] = 1; // Grass
                } else if (y > height - 4) {
                  blocks[index] = 2; // Dirt
                } else {
                  blocks[index] = 3; // Stone
                }
              }
            }
          }
        }
        
        // Second pass: Generate trees (FIXED for natural distribution)
        const treePositions = []; // Track tree positions to avoid clustering

        for (let x = 3; x < CHUNK_SIZE - 3; x++) { // More space from edges
          for (let z = 3; z < CHUNK_SIZE - 3; z++) { // More space from edges
            const worldX = chunkX * CHUNK_SIZE + x;
            const worldZ = chunkZ * CHUNK_SIZE + z;
            
            // FIXED: Multi-layer noise for natural tree distribution
            const treeNoise1 = perlin.noise(worldX * 0.02, 0, worldZ * 0.02); // Large scale
            const treeNoise2 = perlin.noise(worldX * 0.08, 100, worldZ * 0.08); // Medium scale  
            const treeNoise3 = perlin.noise(worldX * 0.15, 200, worldZ * 0.15); // Small scale
            
            // Combine noise layers for natural distribution
            const combinedNoise = (treeNoise1 * 0.5) + (treeNoise2 * 0.3) + (treeNoise3 * 0.2);
            
            // FIXED: More selective tree placement with natural threshold
            if (combinedNoise > 0.15 && combinedNoise < 0.4) {
              // Check minimum distance from other trees (natural spacing)
              let tooClose = false;
              for (const treePos of treePositions) {
                const distance = Math.sqrt((x - treePos.x) ** 2 + (z - treePos.z) ** 2);
                if (distance < 4) { // Minimum 4 block spacing
                  tooClose = true;
                  break;
                }
              }
              
              if (!tooClose) {
                // Find ground height
                let groundHeight = 0;
                for (let y = CHUNK_HEIGHT - 1; y >= 0; y--) {
                  const index = x + z * CHUNK_SIZE + y * CHUNK_SIZE * CHUNK_SIZE;
                  if (blocks[index] === 1) { // Grass block
                    groundHeight = y;
                    break;
                  }
                }
                
                // Only place trees on suitable terrain
                if (groundHeight > 28 && groundHeight < CHUNK_HEIGHT - 10) {
                  // FIXED: More natural tree height variation
                  const heightNoise = perlin.noise(worldX * 0.3, 300, worldZ * 0.3);
                  const treeHeight = 4 + Math.floor(Math.abs(heightNoise) * 4); // 4-7 blocks tall
                  
                  // Generate tree trunk
                  for (let y = groundHeight + 1; y <= groundHeight + treeHeight && y < CHUNK_HEIGHT; y++) {
                    const index = x + z * CHUNK_SIZE + y * CHUNK_SIZE * CHUNK_SIZE;
                    blocks[index] = 4; // Wood
                  }
                  
                  // FIXED: More natural leaf generation with varied patterns
                  const leafTop = groundHeight + treeHeight;
                  const leafSize = 2 + Math.floor(Math.abs(perlin.noise(worldX * 0.4, 400, worldZ * 0.4)) * 2); // 2-3 radius
                  
                  for (let dx = -leafSize; dx <= leafSize; dx++) {
                    for (let dz = -leafSize; dz <= leafSize; dz++) {
                      for (let dy = -1; dy <= 2; dy++) {
                        const leafX = x + dx;
                        const leafZ = z + dz;
                        const leafY = leafTop + dy;
                        
                        // FIXED: Natural leaf shape with distance-based placement
                        const leafDistance = Math.sqrt(dx * dx + dz * dz + (dy * 0.5) ** 2);
                        const maxLeafDistance = leafSize + (dy === 0 ? 0.5 : dy); // Wider at middle layer
                        
                        // Add some randomness to leaf placement for natural look
                        const leafNoise = perlin.noise((worldX + dx) * 0.5, 500, (worldZ + dz) * 0.5);
                        const leafThreshold = 0.3 - (leafDistance / maxLeafDistance) * 0.4;
                        
                        if (leafDistance <= maxLeafDistance && leafNoise > leafThreshold &&
                            leafX >= 0 && leafX < CHUNK_SIZE && 
                            leafZ >= 0 && leafZ < CHUNK_SIZE && 
                            leafY < CHUNK_HEIGHT && leafY > groundHeight) {
                          
                          const index = leafX + leafZ * CHUNK_SIZE + leafY * CHUNK_SIZE * CHUNK_SIZE;
                          if (blocks[index] === 0) { // Only place on air
                            blocks[index] = 5; // Leaves
                          }
                        }
                      }
                    }
                  }
                  
                  // Track this tree position
                  treePositions.push({ x, z });
                }
              }
            }
          }
        }
        
        return blocks;
      }
      
      // Greedy meshing with proper normals and UV mapping
      function greedyMesh(blocks, chunkX, chunkZ) {
        const CHUNK_SIZE = 16;
        const CHUNK_HEIGHT = 64;
        const vertices = [];
        const indices = [];
        const uvs = [];
        const normals = [];
        let vertexIndex = 0;
        
        // UV coordinates for texture atlas (4x4 grid)
        const getUV = (blockType) => {
          const texSize = 0.25; // 1/4 for 4x4 atlas
          let u = 0, v = 0;
          
          switch(blockType) {
            case 1: u = 0; v = 0; break; // Grass
            case 2: u = 1; v = 0; break; // Dirt  
            case 3: u = 2; v = 0; break; // Stone
            case 4: u = 3; v = 0; break; // Wood
            case 5: u = 0; v = 1; break; // Leaves
          }
          
          return [u * texSize, v * texSize, texSize];
        };
        
        // Face directions with proper normals and UV coordinates
        const faces = [
          { 
            dir: [1, 0, 0], 
            corners: [[1, 1, 1], [1, 0, 1], [1, 0, 0], [1, 1, 0]], 
            normal: [1, 0, 0],
            uvCorners: [[1, 1], [1, 0], [0, 0], [0, 1]]
          }, // +X
          { 
            dir: [-1, 0, 0], 
            corners: [[0, 1, 0], [0, 0, 0], [0, 0, 1], [0, 1, 1]], 
            normal: [-1, 0, 0],
            uvCorners: [[1, 1], [1, 0], [0, 0], [0, 1]]
          }, // -X
          { 
            dir: [0, 1, 0], 
            corners: [[1, 1, 1], [1, 1, 0], [0, 1, 0], [0, 1, 1]], 
            normal: [0, 1, 0],
            uvCorners: [[1, 1], [1, 0], [0, 0], [0, 1]]
          }, // +Y
          { 
            dir: [0, -1, 0], 
            corners: [[0, 0, 1], [0, 0, 0], [1, 0, 0], [1, 0, 1]], 
            normal: [0, -1, 0],
            uvCorners: [[0, 1], [0, 0], [1, 0], [1, 1]]
          }, // -Y
          { 
            dir: [0, 0, 1], 
            corners: [[0, 1, 1], [0, 0, 1], [1, 0, 1], [1, 1, 1]], 
            normal: [0, 0, 1],
            uvCorners: [[1, 1], [1, 0], [0, 0], [0, 1]]
          }, // +Z
          { 
            dir: [0, 0, -1], 
            corners: [[1, 1, 0], [1, 0, 0], [0, 0, 0], [0, 1, 0]], 
            normal: [0, 0, -1],
            uvCorners: [[1, 1], [1, 0], [0, 0], [0, 1]]
          }  // -Z
        ];
        
        function getBlock(x, y, z) {
          if (x < 0 || x >= CHUNK_SIZE || y < 0 || y >= CHUNK_HEIGHT || z < 0 || z >= CHUNK_SIZE) {
            return 0; // Air outside chunk
          }
          return blocks[x + z * CHUNK_SIZE + y * CHUNK_SIZE * CHUNK_SIZE];
        }
        
        // Generate faces with culling, proper UVs and normals
        for (let x = 0; x < CHUNK_SIZE; x++) {
          for (let y = 0; y < CHUNK_HEIGHT; y++) {
            for (let z = 0; z < CHUNK_SIZE; z++) {
              const block = getBlock(x, y, z);
              if (block === 0) continue;
              
              faces.forEach((face, faceIndex) => {
                const [dx, dy, dz] = face.dir;
                const neighbor = getBlock(x + dx, y + dy, z + dz);
                
                // FIXED: Special handling for leaves - make them slightly transparent
                if (block === 5 && neighbor === 5) return; // Don't render leaf-to-leaf faces
                if (neighbor !== 0 && !(block === 5 && neighbor !== 5)) return; // Face culling
                
                const [baseU, baseV, texSize] = getUV(block);
                
                // Add face vertices with proper UVs and normals
                face.corners.forEach((corner, cornerIndex) => {
                  vertices.push(
                    x + corner[0],
                    y + corner[1], 
                    z + corner[2]
                  );
                  
                  // Proper UV mapping
                  const uvCorner = face.uvCorners[cornerIndex];
                  const u = baseU + uvCorner[0] * texSize;
                  const v = baseV + uvCorner[1] * texSize;
                  uvs.push(u, v);
                  
                  // Add normals for proper lighting
                  normals.push(face.normal[0], face.normal[1], face.normal[2]);
                });
                
                // Add face indices (two triangles per face)
                const base = vertexIndex;
                indices.push(
                  base, base + 1, base + 2,
                  base, base + 2, base + 3
                );
                vertexIndex += 4;
              });
            }
          }
        }
        
        return {
          vertices: new Float32Array(vertices),
          indices: new Uint32Array(indices),
          uvs: new Float32Array(uvs),
          normals: new Float32Array(normals)
        };
      }
      
      // Worker message handler
      self.onmessage = function(e) {
        const { type, data, id } = e.data;
        
        if (type === 'generateChunk') {
          const { chunkX, chunkZ } = data;
          const blocks = generateChunk(chunkX, chunkZ);
          const mesh = greedyMesh(blocks, chunkX, chunkZ);
          
          self.postMessage({
            type: 'chunkGenerated',
            id,
            data: {
              chunkX,
              chunkZ,
              blocks,
              mesh
            }
          }, [blocks.buffer, mesh.vertices.buffer, mesh.indices.buffer, mesh.uvs.buffer, mesh.normals.buffer]);
        }

        if (type === 'generateMesh') {
          const { blocks, chunkX, chunkZ } = data;
          const mesh = greedyMesh(blocks, chunkX, chunkZ);
          
          self.postMessage({
            type: 'meshGenerated',
            id,
            data: mesh
          }, [mesh.vertices.buffer, mesh.indices.buffer, mesh.uvs.buffer, mesh.normals.buffer]);
        }
      };
    `

    const blob = new Blob([workerCode], { type: "application/javascript" })
    this.worker = new Worker(URL.createObjectURL(blob))

    this.worker.onmessage = (e) => {
      const { id, data } = e.data
      const callback = this.pendingMessages.get(id)
      if (callback) {
        callback(data)
        this.pendingMessages.delete(id)
      }
    }
  }

  generateChunk(chunkX: number, chunkZ: number): Promise<any> {
    return new Promise((resolve) => {
      if (!this.worker) {
        // Fallback for when workers aren't available
        resolve(this.generateChunkFallback(chunkX, chunkZ))
        return
      }

      const id = this.messageId++
      this.pendingMessages.set(id, resolve)

      this.worker.postMessage({
        type: "generateChunk",
        id,
        data: { chunkX, chunkZ },
      })
    })
  }

  private generateChunkFallback(chunkX: number, chunkZ: number) {
    // Simple fallback generation
    const blocks = new Uint16Array(CHUNK_SIZE * CHUNK_HEIGHT * CHUNK_SIZE)

    for (let x = 0; x < CHUNK_SIZE; x++) {
      for (let z = 0; z < CHUNK_SIZE; z++) {
        const height =
          Math.floor(Math.sin((chunkX * CHUNK_SIZE + x) * 0.1) * Math.cos((chunkZ * CHUNK_SIZE + z) * 0.1) * 5) + 32

        for (let y = 0; y <= height && y < CHUNK_HEIGHT; y++) {
          const index = x + z * CHUNK_SIZE + y * CHUNK_SIZE * CHUNK_SIZE
          if (y === height) blocks[index] = BLOCK_TYPES.GRASS
          else if (y > height - 3) blocks[index] = BLOCK_TYPES.DIRT
          else blocks[index] = BLOCK_TYPES.STONE
        }
      }
    }

    return { chunkX, chunkZ, blocks, mesh: null }
  }

  // Add this method to the WorldWorker class
  // FIXED: Don't transfer blocks buffer to preserve original data
  generateMesh(blocks: Uint16Array, chunkX: number, chunkZ: number): Promise<any> {
    return new Promise((resolve) => {
      if (!this.worker) {
        resolve(null)
        return
      }

      const id = this.messageId++
      this.pendingMessages.set(id, resolve)

      // FIXED: Clone the blocks array instead of transferring to preserve original
      const blocksCopy = new Uint16Array(blocks)

      this.worker.postMessage({
        type: "generateMesh",
        id,
        data: { blocks: blocksCopy, chunkX, chunkZ },
      })
      // REMOVED: [blocks.buffer] transfer - this was corrupting the original blocks array
    })
  }
}

// Global worker instance
let worldWorker: WorldWorker | null = null

// MINECRAFT-STYLE PIXELATED TEXTURE ATLAS CREATION
function createMinecraftTextureAtlas(): THREE.Texture {
  const canvas = document.createElement("canvas")
  canvas.width = 512 // Higher resolution for crisp pixels
  canvas.height = 512
  const ctx = canvas.getContext("2d")!

  // Disable image smoothing for pixelated look
  ctx.imageSmoothingEnabled = false

  const blockSize = 128 // 128x128 per texture for crisp detail
  const blocksPerRow = 4

  // Clear canvas
  ctx.fillStyle = "#000000"
  ctx.fillRect(0, 0, 512, 512)

  // GRASS BLOCK TEXTURE (0,0) - Classic Minecraft grass
  const grassX = 0,
    grassY = 0
  // Base grass green
  ctx.fillStyle = "#7CB342"
  ctx.fillRect(grassX, grassY, blockSize, blockSize)

  // Add grass texture pattern
  const grassColors = ["#8BC34A", "#689F38", "#558B2F", "#9CCC65"]
  for (let i = 0; i < 800; i++) {
    ctx.fillStyle = grassColors[Math.floor(Math.random() * grassColors.length)]
    const size = Math.random() < 0.7 ? 2 : 4
    ctx.fillRect(
      grassX + Math.floor(Math.random() * (blockSize - size)),
      grassY + Math.floor(Math.random() * (blockSize - size)),
      size,
      size,
    )
  }

  // Add some darker grass blades
  ctx.fillStyle = "#33691E"
  for (let i = 0; i < 200; i++) {
    const x = grassX + Math.floor(Math.random() * (blockSize - 2))
    const y = grassY + Math.floor(Math.random() * (blockSize - 8))
    ctx.fillRect(x, y, 1, 6)
    ctx.fillRect(x + 1, y + 2, 1, 4)
  }

  // DIRT BLOCK TEXTURE (1,0) - Rich brown dirt
  const dirtX = blockSize,
    dirtY = 0
  // Base dirt brown
  ctx.fillStyle = "#8D6E63"
  ctx.fillRect(dirtX, dirtY, blockSize, blockSize)

  // Add dirt texture with various browns
  const dirtColors = ["#A1887F", "#6D4C41", "#5D4037", "#795548", "#BCAAA4"]
  for (let i = 0; i < 1000; i++) {
    ctx.fillStyle = dirtColors[Math.floor(Math.random() * dirtColors.length)]
    const size = Math.random() < 0.5 ? 2 : Math.random() < 0.8 ? 4 : 6
    ctx.fillRect(
      dirtX + Math.floor(Math.random() * (blockSize - size)),
      dirtY + Math.floor(Math.random() * (blockSize - size)),
      size,
      size,
    )
  }

  // Add some small rocks/pebbles
  ctx.fillStyle = "#4E342E"
  for (let i = 0; i < 100; i++) {
    const x = dirtX + Math.floor(Math.random() * (blockSize - 3))
    const y = dirtY + Math.floor(Math.random() * (blockSize - 3))
    ctx.fillRect(x, y, 3, 3)
    ctx.fillRect(x + 1, y + 1, 1, 1)
  }

  // STONE BLOCK TEXTURE (2,0) - Classic cobblestone-like
  const stoneX = blockSize * 2,
    stoneY = 0
  // Base stone gray
  ctx.fillStyle = "#9E9E9E"
  ctx.fillRect(stoneX, stoneY, blockSize, blockSize)

  // Add stone texture with grays
  const stoneColors = ["#BDBDBD", "#757575", "#616161", "#424242", "#E0E0E0"]
  for (let i = 0; i < 1200; i++) {
    ctx.fillStyle = stoneColors[Math.floor(Math.random() * stoneColors.length)]
    const size = Math.random() < 0.3 ? 2 : Math.random() < 0.7 ? 4 : 6
    ctx.fillRect(
      stoneX + Math.floor(Math.random() * (blockSize - size)),
      stoneY + Math.floor(Math.random() * (blockSize - size)),
      size,
      size,
    )
  }

  // Add cracks and details
  ctx.fillStyle = "#37474F"
  for (let i = 0; i < 50; i++) {
    const x = stoneX + Math.floor(Math.random() * (blockSize - 8))
    const y = stoneY + Math.floor(Math.random() * (blockSize - 2))
    ctx.fillRect(x, y, 8, 1)
    ctx.fillRect(x + 2, y + 1, 4, 1)
  }

  // WOOD BLOCK TEXTURE (3,0) - Oak wood with rings
  const woodX = blockSize * 3,
    woodY = 0
  // Base wood brown
  ctx.fillStyle = "#FF9800"
  ctx.fillRect(woodX, woodY, blockSize, blockSize)

  // Add wood grain horizontal lines
  const woodColors = ["#F57C00", "#E65100", "#FF8F00", "#FFB74D"]
  for (let y = 0; y < blockSize; y += 2) {
    ctx.fillStyle = woodColors[Math.floor(Math.random() * woodColors.length)]
    ctx.fillRect(woodX, woodY + y, blockSize, 1)

    // Add some variation
    if (Math.random() < 0.3) {
      ctx.fillRect(woodX, woodY + y + 1, blockSize, 1)
    }
  }

  // Add wood knots and details
  ctx.fillStyle = "#BF360C"
  for (let i = 0; i < 20; i++) {
    const x = woodX + Math.floor(Math.random() * (blockSize - 8))
    const y = woodY + Math.floor(Math.random() * (blockSize - 8))
    // Draw small oval knots
    ctx.fillRect(x + 2, y, 4, 2)
    ctx.fillRect(x + 1, y + 1, 6, 4)
    ctx.fillRect(x + 2, y + 5, 4, 2)
  }

  // LEAVES BLOCK TEXTURE (0,1) - Dense foliage
  const leavesX = 0,
    leavesY = blockSize
  // Base leaf green
  ctx.fillStyle = "#2E7D32"
  ctx.fillRect(leavesX, leavesY, blockSize, blockSize)

  // Add leaf texture with various greens
  const leafColors = ["#388E3C", "#4CAF50", "#66BB6A", "#1B5E20", "#43A047"]
  for (let i = 0; i < 1500; i++) {
    ctx.fillStyle = leafColors[Math.floor(Math.random() * leafColors.length)]
    const size = Math.random() < 0.6 ? 2 : Math.random() < 0.9 ? 3 : 4
    ctx.fillRect(
      leavesX + Math.floor(Math.random() * (blockSize - size)),
      leavesY + Math.floor(Math.random() * (blockSize - size)),
      size,
      size,
    )
  }

  // Add some leaf shapes and details
  ctx.fillStyle = "#1B5E20"
  for (let i = 0; i < 300; i++) {
    const x = leavesX + Math.floor(Math.random() * (blockSize - 4))
    const y = leavesY + Math.floor(Math.random() * (blockSize - 4))
    // Draw small leaf shapes
    ctx.fillRect(x + 1, y, 2, 1)
    ctx.fillRect(x, y + 1, 4, 2)
    ctx.fillRect(x + 1, y + 3, 2, 1)
  }

  // Add some lighter highlights
  ctx.fillStyle = "#81C784"
  for (let i = 0; i < 200; i++) {
    const x = leavesX + Math.floor(Math.random() * (blockSize - 2))
    const y = leavesY + Math.floor(Math.random() * (blockSize - 2))
    ctx.fillRect(x, y, 1, 1)
  }

  // Create texture with proper settings for pixelated look
  const texture = new THREE.CanvasTexture(canvas)
  texture.magFilter = THREE.NearestFilter // Crucial for pixelated look
  texture.minFilter = THREE.NearestFilter // Crucial for pixelated look
  texture.wrapS = THREE.RepeatWrapping
  texture.wrapT = THREE.RepeatWrapping
  texture.flipY = false
  texture.generateMipmaps = false // Disable mipmaps for crisp pixels

  return texture
}

// FIXED: Optimized chunk component with stable rendering
const ChunkMesh = memo(({ chunk, textureAtlas }: { chunk: ChunkData; textureAtlas: THREE.Texture }) => {
  const meshRef = useRef<THREE.Mesh>(null)
  const geometryRef = useRef<THREE.BufferGeometry | null>(null)

  useEffect(() => {
    if (!chunk.mesh || !meshRef.current) return

    // Dispose old geometry
    if (geometryRef.current) {
      geometryRef.current.dispose()
    }

    const geometry = new THREE.BufferGeometry()
    geometry.setAttribute("position", new THREE.BufferAttribute(chunk.mesh.vertices, 3))
    geometry.setAttribute("uv", new THREE.BufferAttribute(chunk.mesh.uvs, 2))
    geometry.setAttribute("normal", new THREE.BufferAttribute(chunk.mesh.normals, 3))
    geometry.setIndex(new THREE.BufferAttribute(chunk.mesh.indices, 1))

    geometryRef.current = geometry
    meshRef.current.geometry = geometry
  }, [chunk.mesh])

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (geometryRef.current) {
        geometryRef.current.dispose()
      }
    }
  }, [])

  if (!chunk.mesh || !chunk.generated) return null

  return (
    <mesh
      ref={meshRef}
      position={[chunk.x * CHUNK_SIZE, 0, chunk.z * CHUNK_SIZE]}
      frustumCulled={false} // FIXED: Disable aggressive frustum culling for trees
    >
      <bufferGeometry />
      <meshLambertMaterial map={textureAtlas} transparent={false} alphaTest={0.1} />
    </mesh>
  )
})

ChunkMesh.displayName = "ChunkMesh"

// ADVANCED MOVEMENT CONFIG
// Base walk speed approximates vanilla Minecraft (~4.3 m/s)
const BASE_SPEED = 4.317

const MOVEMENT_CONFIG = {
  // Core physics
  accel_ground: 50,
  accel_air: 150,
  max_air_speed: BASE_SPEED * (30 / 400), // Only enforced when debug flag is true
  air_drag: 0.02,
  ground_friction: 8.0,

  // Bunny hop
  bhop_window: 0.05, // 50ms timing window
  bhop_preserve_speed: true,

  // Slide mechanics (Titanfall-style)
  slide_threshold: BASE_SPEED * (350 / 400), // Speed to trigger slide
  slide_boost: BASE_SPEED * (70 / 400),
  slide_decay: 0.97,
  slide_max_duration: 0.4, // 400ms
  slide_height_multiplier: 0.5,

  // Wallrun
  wallrun_attach_distance: 1.2,
  wallrun_force: BASE_SPEED * (30 / 400),
  wallrun_duration: 0.9, // 900ms
  wall_jump_out_force: BASE_SPEED * (300 / 400),
  wall_jump_fwd_force: BASE_SPEED * (150 / 400),
  wallrun_up_bias: 5,

  // Surf ramps
  surf_min_normal: 0.1, // dot(normal, up) thresholds
  surf_max_normal: 0.7,
  walkable_normal: 0.7,

  // Camera effects
  tilt_max: 10, // degrees
  tilt_speed: 5,

  // Debug
  debug_enabled: false,
  debug_max_speed_cap: false,
}

// Movement states
enum MovementState {
  GROUND = "Ground",
  AIR = "Air",
  SLIDE = "Slide",
  WALLRUN = "Wallrun",
  SURF = "Surf",
}

// Advanced Player component with movement tech
const Player = memo(({ isLocked }: { isLocked: boolean }) => {
  const { camera } = useThree()
  const [, getKeys] = useKeyboardControls()
  const { setPlayerPosition, getBlock, setBlock, selectedBlock } = useGameStore()

  // Movement state
  const velocity = useRef(new THREE.Vector3(0, 0, 0))
  const movementState = useRef<MovementState>(MovementState.AIR)
  const isOnGround = useRef(false)
  const groundNormal = useRef(new THREE.Vector3(0, 1, 0))

  // Bunny hop timing
  const lastGroundTime = useRef(0)
  const jumpBufferTime = useRef(0)
  const preserveSpeedThisFrame = useRef(false)

  // Slide state
  const slideStartTime = useRef(0)
  const isSliding = useRef(false)
  const slideDirection = useRef(new THREE.Vector3())

  // Wallrun state
  const wallrunStartTime = useRef(0)
  const isWallrunning = useRef(false)
  const wallNormal = useRef(new THREE.Vector3())
  const wallrunSide = useRef<"left" | "right">("left")

  // Surf state
  const isSurfing = useRef(false)
  const surfNormal = useRef(new THREE.Vector3(0, 1, 0))

  // Camera tilt
  const currentTilt = useRef(0)

  // Debug info
  const debugInfo = useRef({
    speed: 0,
    horizontalSpeed: 0,
    state: MovementState.AIR,
    onGround: false,
  })

  const raycaster = useMemo(() => new THREE.Raycaster(), [])

  // Player dimensions
  const PLAYER_WIDTH = 0.6
  const PLAYER_HEIGHT = 1.8
  const PLAYER_EYE_HEIGHT = 1.62

  // Utility functions
  const getHorizontalSpeed = useCallback((vel: THREE.Vector3) => {
    return Math.sqrt(vel.x * vel.x + vel.z * vel.z)
  }, [])

  const getHorizontalVelocity = useCallback((vel: THREE.Vector3) => {
    return new THREE.Vector3(vel.x, 0, vel.z)
  }, [])

  const projectVectorOntoPlane = useCallback((vector: THREE.Vector3, normal: THREE.Vector3) => {
    const projected = vector.clone()
    const dot = vector.dot(normal)
    projected.sub(normal.clone().multiplyScalar(dot))
    return projected
  }, [])

  // Ground/surface detection with advanced surface classification
  const detectSurface = useCallback(
    (position: THREE.Vector3) => {
      const rayOrigin = position.clone()
      rayOrigin.y -= PLAYER_EYE_HEIGHT - 0.1

      const rayDirection = new THREE.Vector3(0, -1, 0)
      const maxDistance = 0.2

      // Cast multiple rays for better surface detection
      const rayPositions = [
        rayOrigin.clone(),
        rayOrigin.clone().add(new THREE.Vector3(0.2, 0, 0)),
        rayOrigin.clone().add(new THREE.Vector3(-0.2, 0, 0)),
        rayOrigin.clone().add(new THREE.Vector3(0, 0, 0.2)),
        rayOrigin.clone().add(new THREE.Vector3(0, 0, -0.2)),
      ]

      let closestHit = null
      let closestDistance = Number.POSITIVE_INFINITY

      for (const rayPos of rayPositions) {
        for (let distance = 0.01; distance <= maxDistance; distance += 0.01) {
          const testPoint = rayPos.clone().add(rayDirection.clone().multiplyScalar(distance))
          const blockX = Math.floor(testPoint.x)
          const blockY = Math.floor(testPoint.y)
          const blockZ = Math.floor(testPoint.z)

          if (getBlock(blockX, blockY, blockZ) !== BLOCK_TYPES.AIR) {
            if (distance < closestDistance) {
              closestDistance = distance
              // Calculate surface normal (simplified - assumes block faces)
              const blockCenter = new THREE.Vector3(blockX + 0.5, blockY + 0.5, blockZ + 0.5)
              const hitPoint = testPoint.clone()
              const diff = hitPoint.sub(blockCenter)

              let normal = new THREE.Vector3(0, 1, 0) // Default to up
              const absX = Math.abs(diff.x)
              const absY = Math.abs(diff.y)
              const absZ = Math.abs(diff.z)

              if (absY >= absX && absY >= absZ) {
                normal = new THREE.Vector3(0, Math.sign(diff.y), 0)
              } else if (absX >= absZ) {
                normal = new THREE.Vector3(Math.sign(diff.x), 0, 0)
              } else {
                normal = new THREE.Vector3(0, 0, Math.sign(diff.z))
              }

              closestHit = {
                distance,
                normal,
                point: testPoint,
                blockPos: new THREE.Vector3(blockX, blockY, blockZ),
              }
            }
            break
          }
        }
      }

      return closestHit
    },
    [getBlock],
  )

  // Wallrun detection
  const detectWall = useCallback(
    (position: THREE.Vector3, direction: THREE.Vector3) => {
      const rayOrigin = position.clone()
      const rayDirections = [
        direction.clone().normalize(),
        new THREE.Vector3(-direction.z, 0, direction.x).normalize(), // Left
        new THREE.Vector3(direction.z, 0, -direction.x).normalize(), // Right
      ]

      for (const rayDir of rayDirections) {
        for (let distance = 0.1; distance <= MOVEMENT_CONFIG.wallrun_attach_distance; distance += 0.1) {
          const testPoint = rayOrigin.clone().add(rayDir.clone().multiplyScalar(distance))
          const blockX = Math.floor(testPoint.x)
          const blockY = Math.floor(testPoint.y)
          const blockZ = Math.floor(testPoint.z)

          if (getBlock(blockX, blockY, blockZ) !== BLOCK_TYPES.AIR) {
            // Calculate wall normal
            const blockCenter = new THREE.Vector3(blockX + 0.5, blockY + 0.5, blockZ + 0.5)
            const normal = testPoint.clone().sub(blockCenter).normalize()

            // Check if it's a valid wall (not floor/ceiling)
            if (Math.abs(normal.dot(new THREE.Vector3(0, 1, 0))) < 0.1) {
              const side = rayDir.equals(rayDirections[1]) ? "left" : "right"
              return { normal, distance, side }
            }
          }
        }
      }
      return null
    },
    [getBlock],
  )

  // Quake-style air strafing acceleration
  const applyAirAcceleration = useCallback(
    (currentVel: THREE.Vector3, wishDir: THREE.Vector3, accel: number, delta: number) => {
      if (wishDir.length() === 0) return currentVel

      const wishSpeed = wishDir.length()
      wishDir.normalize()

      const currentSpeed = currentVel.dot(wishDir)
      const addSpeed = wishSpeed - currentSpeed

      if (addSpeed <= 0) return currentVel

      let accelSpeed = accel * delta * wishSpeed
      if (accelSpeed > addSpeed) {
        accelSpeed = addSpeed
      }

      const newVel = currentVel.clone()
      newVel.add(wishDir.clone().multiplyScalar(accelSpeed))

      // Only cap speed if debug flag is enabled
      if (MOVEMENT_CONFIG.debug_max_speed_cap) {
        const horizontalSpeed = getHorizontalSpeed(newVel)
        if (horizontalSpeed > MOVEMENT_CONFIG.max_air_speed) {
          const horizontalVel = getHorizontalVelocity(newVel)
          horizontalVel.normalize().multiplyScalar(MOVEMENT_CONFIG.max_air_speed)
          newVel.x = horizontalVel.x
          newVel.z = horizontalVel.z
        }
      }

      return newVel
    },
    [getHorizontalSpeed, getHorizontalVelocity],
  )

  // Apply ground friction (Coulomb friction)
  const applyGroundFriction = useCallback(
    (vel: THREE.Vector3, delta: number) => {
      if (preserveSpeedThisFrame.current) {
        preserveSpeedThisFrame.current = false
        return vel // Skip friction for bhop
      }

      const horizontalVel = getHorizontalVelocity(vel)
      const speed = horizontalVel.length()

      if (speed > 0) {
        const friction = MOVEMENT_CONFIG.ground_friction * delta
        const newSpeed = Math.max(0, speed - friction)
        horizontalVel.normalize().multiplyScalar(newSpeed)

        return new THREE.Vector3(horizontalVel.x, vel.y, vel.z)
      }

      return vel
    },
    [getHorizontalVelocity],
  )

  // Apply air drag
  const applyAirDrag = useCallback((vel: THREE.Vector3, delta: number) => {
    const dragFactor = Math.pow(1 - MOVEMENT_CONFIG.air_drag, delta * 60) // Frame-rate independent
    return vel.clone().multiplyScalar(dragFactor)
  }, [])

  // Camera tilt based on horizontal velocity
  const updateCameraTilt = useCallback(
    (vel: THREE.Vector3, delta: number) => {
      const horizontalVel = getHorizontalVelocity(vel)
      const speed = horizontalVel.length()

      // Calculate desired tilt based on velocity
      const maxTiltSpeed = 800 // ups for max tilt
      const tiltAmount = Math.min(speed / maxTiltSpeed, 1) * MOVEMENT_CONFIG.tilt_max

      // Determine tilt direction based on strafe direction
      const forward = new THREE.Vector3()
      camera.getWorldDirection(forward)
      forward.y = 0
      forward.normalize()

      const right = new THREE.Vector3().crossVectors(forward, new THREE.Vector3(0, 1, 0))
      const strafeAmount = horizontalVel.dot(right)

      const targetTilt = strafeAmount > 0 ? tiltAmount : -tiltAmount

      // Smooth tilt interpolation
      const tiltDiff = targetTilt - currentTilt.current
      currentTilt.current += tiltDiff * MOVEMENT_CONFIG.tilt_speed * delta

      // Apply tilt to camera (would need to be implemented based on your camera system)
      // camera.rotation.z = THREE.MathUtils.degToRad(currentTilt.current)
    },
    [camera, getHorizontalVelocity],
  )

  // Main movement update loop
  useGameLoop(
    useCallback(
      (delta: number) => {
        if (!isLocked) return

        const keys = getKeys()
        const currentPos = camera.position.clone()
        const currentTime = performance.now() / 1000

        // Input handling
        const inputVector = new THREE.Vector3()
        if (keys.forward) inputVector.z -= 1
        if (keys.backward) inputVector.z += 1
        if (keys.left) inputVector.x -= 1
        if (keys.right) inputVector.x += 1

        // Transform input to world space
        const cameraDirection = new THREE.Vector3()
        camera.getWorldDirection(cameraDirection)
        const forward = new THREE.Vector3(cameraDirection.x, 0, cameraDirection.z).normalize()
        const right = new THREE.Vector3().crossVectors(forward, new THREE.Vector3(0, 1, 0)).normalize()

        const worldInput = new THREE.Vector3()
        worldInput.add(forward.clone().multiplyScalar(-inputVector.z))
        worldInput.add(right.clone().multiplyScalar(inputVector.x))
        if (worldInput.length() > 0) worldInput.normalize()

        // Jump input buffering
        if (keys.jump) {
          jumpBufferTime.current = currentTime
        }

        // Surface detection
        const surfaceHit = detectSurface(currentPos)
        const wasOnGround = isOnGround.current
        isOnGround.current = surfaceHit && surfaceHit.distance <= 0.1

        if (surfaceHit) {
          groundNormal.current = surfaceHit.normal
        }

        // Update movement state
        const prevState = movementState.current

        if (isOnGround.current) {
          const normalDotUp = groundNormal.current.dot(new THREE.Vector3(0, 1, 0))

          if (normalDotUp >= MOVEMENT_CONFIG.walkable_normal) {
            // Walkable ground
            if (isSliding.current) {
              movementState.current = MovementState.SLIDE
            } else {
              movementState.current = MovementState.GROUND
            }
            isSurfing.current = false
          } else if (normalDotUp > MOVEMENT_CONFIG.surf_min_normal) {
            // Surf ramp
            movementState.current = MovementState.SURF
            isSurfing.current = true
            surfNormal.current = groundNormal.current
            isSliding.current = false
          }
        } else {
          // Check for wallrun
          const wallHit = detectWall(currentPos, getHorizontalVelocity(velocity.current))

          if (wallHit && !isWallrunning.current && getHorizontalSpeed(velocity.current) > 200) {
            // Start wallrun
            isWallrunning.current = true
            wallrunStartTime.current = currentTime
            wallNormal.current = wallHit.normal
            wallrunSide.current = wallHit.side
            movementState.current = MovementState.WALLRUN
          } else if (
            isWallrunning.current &&
            (currentTime - wallrunStartTime.current > MOVEMENT_CONFIG.wallrun_duration || !wallHit)
          ) {
            // End wallrun
            isWallrunning.current = false
            movementState.current = MovementState.AIR
          } else if (!isWallrunning.current) {
            movementState.current = MovementState.AIR
          }

          isSurfing.current = false
          isSliding.current = false
        }

        // Ground landing detection for bhop
        if (!wasOnGround && isOnGround.current) {
          lastGroundTime.current = currentTime

          // Check bhop timing window
          if (currentTime - jumpBufferTime.current <= MOVEMENT_CONFIG.bhop_window) {
            preserveSpeedThisFrame.current = true
          }
        }

        // Slide mechanics
        if (
          keys.crouch &&
          !isSliding.current &&
          isOnGround.current &&
          getHorizontalSpeed(velocity.current) >= MOVEMENT_CONFIG.slide_threshold
        ) {
          // Start slide
          isSliding.current = true
          slideStartTime.current = currentTime
          slideDirection.current = getHorizontalVelocity(velocity.current).normalize()
          movementState.current = MovementState.SLIDE
        }

        if (isSliding.current) {
          const slideDuration = currentTime - slideStartTime.current

          if (keys.jump && slideDuration <= MOVEMENT_CONFIG.slide_max_duration) {
            // Slide jump boost
            const horizontalVel = getHorizontalVelocity(velocity.current)
            const boostVel = horizontalVel.clone().normalize().multiplyScalar(MOVEMENT_CONFIG.slide_boost)
            velocity.current.add(boostVel)
            velocity.current.y = 10 // Jump velocity
            isSliding.current = false
            movementState.current = MovementState.AIR
          } else if (slideDuration > MOVEMENT_CONFIG.slide_max_duration || !keys.crouch) {
            // End slide
            isSliding.current = false
          }
        }

        // Apply movement based on state
        switch (movementState.current) {
          case MovementState.GROUND:
            // Ground movement with friction
            if (!preserveSpeedThisFrame.current) {
              velocity.current = applyGroundFriction(velocity.current, delta)
            }

            // Ground acceleration
            if (worldInput.length() > 0) {
              velocity.current = applyAirAcceleration(
                velocity.current,
                worldInput
                  .clone()
                  .multiplyScalar(BASE_SPEED), // Desired ground speed
                MOVEMENT_CONFIG.accel_ground,
                delta,
              )
            }

            // Jumping
            if (keys.jump && currentTime - jumpBufferTime.current <= MOVEMENT_CONFIG.bhop_window) {
              velocity.current.y = 10
              isOnGround.current = false
              movementState.current = MovementState.AIR
            }
            break

          case MovementState.AIR:
            // Air strafing
            if (worldInput.length() > 0) {
              velocity.current = applyAirAcceleration(
                velocity.current,
                worldInput
                  .clone()
                  .multiplyScalar(BASE_SPEED), // Air strafe speed
                MOVEMENT_CONFIG.accel_air,
                delta,
              )
            }

            // Air drag
            velocity.current = applyAirDrag(velocity.current, delta)

            // Gravity
            velocity.current.y -= 32 * delta
            break

          case MovementState.SLIDE:
            // Slide decay
            const horizontalVel = getHorizontalVelocity(velocity.current)
            horizontalVel.multiplyScalar(MOVEMENT_CONFIG.slide_decay)
            velocity.current.x = horizontalVel.x
            velocity.current.z = horizontalVel.z

            // Gravity still applies
            velocity.current.y -= 32 * delta
            break

          case MovementState.WALLRUN:
            // Zero vertical gravity during wallrun
            velocity.current.y = Math.max(velocity.current.y, -5) // Slight downward drift

            // Project velocity onto wall plane
            velocity.current = projectVectorOntoPlane(velocity.current, wallNormal.current)

            // Apply wallrun force
            const wallForward = projectVectorOntoPlane(forward, wallNormal.current).normalize()
            velocity.current.add(wallForward.multiplyScalar(MOVEMENT_CONFIG.wallrun_force * delta))

            // Upward bias
            velocity.current.y += MOVEMENT_CONFIG.wallrun_up_bias * delta

            // Wall jump
            if (keys.jump) {
              const jumpVel = wallNormal.current.clone().multiplyScalar(MOVEMENT_CONFIG.wall_jump_out_force)
              jumpVel.add(wallForward.multiplyScalar(MOVEMENT_CONFIG.wall_jump_fwd_force))
              jumpVel.y = 10 // Vertical component
              velocity.current.add(jumpVel)

              isWallrunning.current = false
              movementState.current = MovementState.AIR
            }
            break

          case MovementState.SURF:
            // Project velocity onto surf plane
            velocity.current = projectVectorOntoPlane(velocity.current, surfNormal.current)

            // Apply gravity component parallel to ramp
            const gravityVec = new THREE.Vector3(0, -32, 0)
            const rampGravity = projectVectorOntoPlane(gravityVec, surfNormal.current)
            velocity.current.add(rampGravity.multiplyScalar(delta))

            // Air strafing on ramps
            if (worldInput.length() > 0) {
              const surfInput = projectVectorOntoPlane(worldInput, surfNormal.current)
              velocity.current = applyAirAcceleration(
                velocity.current,
                surfInput.multiplyScalar(BASE_SPEED),
                MOVEMENT_CONFIG.accel_air,
                delta,
              )
            }

            // Jumping off ramp
            if (keys.jump) {
              velocity.current.add(surfNormal.current.clone().multiplyScalar(10))
              movementState.current = MovementState.AIR
              isSurfing.current = false
            }
            break
        }

        // Apply movement with collision detection
        const newPosition = currentPos.clone()
        newPosition.add(velocity.current.clone().multiplyScalar(delta))

        // Simple collision detection (reuse existing system)
        const playerBox = new THREE.Box3().setFromCenterAndSize(
          newPosition,
          new THREE.Vector3(PLAYER_WIDTH, PLAYER_HEIGHT, PLAYER_WIDTH),
        )

        let collision = false
        const minX = Math.floor(playerBox.min.x)
        const maxX = Math.floor(playerBox.max.x)
        const minY = Math.floor(playerBox.min.y)
        const maxY = Math.floor(playerBox.max.y)
        const minZ = Math.floor(playerBox.min.z)
        const maxZ = Math.floor(playerBox.max.z)

        for (let x = minX; x <= maxX && !collision; x++) {
          for (let y = minY; y <= maxY && !collision; y++) {
            for (let z = minZ; z <= maxZ && !collision; z++) {
              if (getBlock(x, y, z) !== BLOCK_TYPES.AIR) {
                collision = true
              }
            }
          }
        }

        if (!collision) {
          camera.position.copy(newPosition)
        } else {
          // Handle collision - stop velocity in collision direction
          velocity.current.multiplyScalar(0.5)
        }

        // Update camera tilt
        updateCameraTilt(velocity.current, delta)

        // Update debug info
        debugInfo.current = {
          speed: velocity.current.length(),
          horizontalSpeed: getHorizontalSpeed(velocity.current),
          state: movementState.current,
          onGround: isOnGround.current,
        }

        setPlayerPosition(camera.position.clone())
        invalidate()
      },
      [
        camera,
        getKeys,
        isLocked,
        setPlayerPosition,
        getBlock,
        detectSurface,
        detectWall,
        applyAirAcceleration,
        applyGroundFriction,
        applyAirDrag,
        updateCameraTilt,
        getHorizontalSpeed,
        getHorizontalVelocity,
        projectVectorOntoPlane,
      ],
    ),
  )

  // Block interaction (keep existing implementation)
  useEffect(() => {
    if (!isLocked) return

    const handleMouseDown = (event: MouseEvent) => {
      event.preventDefault()

      raycaster.setFromCamera(new THREE.Vector2(0, 0), camera)

      let hitBlock: { x: number; y: number; z: number } | null = null
      let hitNormal: THREE.Vector3 | null = null

      const maxDistance = 8
      const stepSize = 0.02

      for (let distance = 0.1; distance < maxDistance; distance += stepSize) {
        const point = raycaster.ray.origin.clone().add(raycaster.ray.direction.clone().multiplyScalar(distance))

        const blockX = Math.floor(point.x)
        const blockY = Math.floor(point.y)
        const blockZ = Math.floor(point.z)

        const blockType = getBlock(blockX, blockY, blockZ)
        if (blockType !== BLOCK_TYPES.AIR) {
          hitBlock = { x: blockX, y: blockY, z: blockZ }

          const prevPoint = raycaster.ray.origin
            .clone()
            .add(raycaster.ray.direction.clone().multiplyScalar(distance - stepSize))
          const blockCenter = new THREE.Vector3(blockX + 0.5, blockY + 0.5, blockZ + 0.5)

          const diff = prevPoint.sub(blockCenter)
          const absX = Math.abs(diff.x)
          const absY = Math.abs(diff.y)
          const absZ = Math.abs(diff.z)

          if (absX >= absY && absX >= absZ) {
            hitNormal = new THREE.Vector3(Math.sign(diff.x), 0, 0)
          } else if (absY >= absZ) {
            hitNormal = new THREE.Vector3(0, Math.sign(diff.y), 0)
          } else {
            hitNormal = new THREE.Vector3(0, 0, Math.sign(diff.z))
          }
          break
        }
      }

      if (hitBlock && hitNormal) {
        if (event.button === 0) {
          setBlock(hitBlock.x, hitBlock.y, hitBlock.z, BLOCK_TYPES.AIR)
          const chunkX = Math.floor(hitBlock.x / CHUNK_SIZE)
          const chunkZ = Math.floor(hitBlock.z / CHUNK_SIZE)
          const chunkKey = `${chunkX},${chunkZ}`
          useGameStore.getState().markChunkDirty(chunkKey)
        } else if (event.button === 2) {
          const placeX = hitBlock.x + hitNormal.x
          const placeY = hitBlock.y + hitNormal.y
          const placeZ = hitBlock.z + hitNormal.z

          const existingBlock = getBlock(placeX, placeY, placeZ)
          if (existingBlock === BLOCK_TYPES.AIR) {
            const playerPos = camera.position
            const playerFeet = new THREE.Vector3(playerPos.x, playerPos.y - PLAYER_EYE_HEIGHT, playerPos.z)
            const playerHead = new THREE.Vector3(
              playerPos.x,
              playerPos.y - PLAYER_EYE_HEIGHT + PLAYER_HEIGHT,
              playerPos.z,
            )

            const playerBox = new THREE.Box3().setFromPoints([
              new THREE.Vector3(playerPos.x - PLAYER_WIDTH / 2, playerFeet.y, playerPos.z - PLAYER_WIDTH / 2),
              new THREE.Vector3(playerPos.x + PLAYER_WIDTH / 2, playerHead.y, playerPos.z + PLAYER_WIDTH / 2),
            ])

            const blockBox = new THREE.Box3().setFromCenterAndSize(
              new THREE.Vector3(placeX + 0.5, placeY + 0.5, placeZ + 0.5),
              new THREE.Vector3(1, 1, 1),
            )

            if (!playerBox.intersectsBox(blockBox)) {
              setBlock(placeX, placeY, placeZ, selectedBlock)
              const chunkX = Math.floor(placeX / CHUNK_SIZE)
              const chunkZ = Math.floor(placeZ / CHUNK_SIZE)
              const chunkKey = `${chunkX},${chunkZ}`
              useGameStore.getState().markChunkDirty(chunkKey)
            }
          }
        }
      }
    }

    const handleContextMenu = (e: MouseEvent) => e.preventDefault()

    document.addEventListener("mousedown", handleMouseDown)
    document.addEventListener("contextmenu", handleContextMenu)

    return () => {
      document.removeEventListener("mousedown", handleMouseDown)
      document.removeEventListener("contextmenu", handleContextMenu)
    }
  }, [camera, raycaster, getBlock, setBlock, selectedBlock, isLocked])

  // Debug display (F1 to toggle)
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === "F1") {
        MOVEMENT_CONFIG.debug_enabled = !MOVEMENT_CONFIG.debug_enabled
      }
    }

    window.addEventListener("keydown", handleKeyDown)
    return () => window.removeEventListener("keydown", handleKeyDown)
  }, [])

  return (
    <>
      {/* Debug Panel */}
      {MOVEMENT_CONFIG.debug_enabled && (
        <div className="fixed top-20 right-4 bg-black/90 text-green-400 p-4 rounded font-mono text-sm border border-green-500">
          <div className="text-green-300 font-bold mb-2">🚀 MOVEMENT DEBUG</div>
          <div>Speed: {debugInfo.current.speed.toFixed(1)} ups</div>
          <div>H-Speed: {debugInfo.current.horizontalSpeed.toFixed(1)} ups</div>
          <div>State: {debugInfo.current.state}</div>
          <div>Ground: {debugInfo.current.onGround ? "YES" : "NO"}</div>
          <div className="mt-2 text-xs text-gray-400">
            <div>Slide Threshold: {MOVEMENT_CONFIG.slide_threshold}</div>
            <div>Air Accel: {MOVEMENT_CONFIG.accel_air}</div>
            <div>Bhop Window: {MOVEMENT_CONFIG.bhop_window * 1000}ms</div>
          </div>
        </div>
      )}
    </>
  )
})

Player.displayName = "Player"

// Optimized pointer lock controls
const PointerLockControls = memo(
  ({
    isLocked,
    setIsLocked,
  }: {
    isLocked: boolean
    setIsLocked: (locked: boolean) => void
  }) => {
    const { camera, gl } = useThree()
    const yaw = useRef(0)
    const pitch = useRef(0)

    useEffect(() => {
      const canvas = gl.domElement

      const handlePointerLockChange = () => {
        const locked = document.pointerLockElement === canvas
        setIsLocked(locked)
      }

      const handleMouseMove = (event: MouseEvent) => {
        if (!isLocked || !document.pointerLockElement) return

        const sensitivity = 0.002
        yaw.current -= event.movementX * sensitivity
        pitch.current -= event.movementY * sensitivity
        pitch.current = Math.max(-Math.PI / 2 + 0.1, Math.min(Math.PI / 2 - 0.1, pitch.current))

        const euler = new THREE.Euler(pitch.current, yaw.current, 0, "YXZ")
        camera.quaternion.setFromEuler(euler)

        invalidate()
      }

      document.addEventListener("pointerlockchange", handlePointerLockChange)
      canvas.addEventListener("mousemove", handleMouseMove)

      return () => {
        document.removeEventListener("pointerlockchange", handlePointerLockChange)
        canvas.removeEventListener("mousemove", handleMouseMove)
      }
    }, [camera, gl.domElement, isLocked, setIsLocked])

    return null
  },
)

PointerLockControls.displayName = "PointerLockControls"

// FIXED: World manager with chunk mesh updates
const WorldManager = memo(() => {
  const { chunks, setChunk, playerPosition, addLoadedChunk, loadedChunks, dirtyChunks } = useGameStore()
  const textureAtlas = useMemo(() => createMinecraftTextureAtlas(), [])
  const loadingChunks = useRef(new Set<string>())

  // Initialize worker
  useEffect(() => {
    if (!worldWorker) {
      worldWorker = new WorldWorker()
    }
  }, [])

  // Handle dirty chunks - regenerate meshes
  useEffect(() => {
    if (!worldWorker || dirtyChunks.size === 0) return

    dirtyChunks.forEach((chunkKey) => {
      const chunk = chunks.get(chunkKey)
      if (chunk && chunk.blocks && chunk.generated) {
        console.log(`Regenerating mesh for dirty chunk ${chunkKey}`)

        // FIXED: Create a copy of blocks to preserve original data
        const [chunkX, chunkZ] = chunkKey.split(",").map(Number)
        const blocksCopy = new Uint16Array(chunk.blocks)

        // Use worker to regenerate mesh with copied data
        worldWorker
          .generateMesh(blocksCopy, chunkX, chunkZ)
          .then((meshData) => {
            if (meshData) {
              const updatedChunk: ChunkData = {
                ...chunk,
                mesh: meshData,
                dirty: false,
              }
              setChunk(chunkKey, updatedChunk)

              // Clear from dirty chunks
              useGameStore.setState((state) => {
                const newDirtyChunks = new Set(state.dirtyChunks)
                newDirtyChunks.delete(chunkKey)
                return { dirtyChunks: newDirtyChunks }
              })

              invalidate()
            }
          })
          .catch((error) => {
            console.error(`Failed to regenerate mesh for chunk ${chunkKey}:`, error)
            // Clear from dirty chunks even on error to prevent infinite retry
            useGameStore.setState((state) => {
              const newDirtyChunks = new Set(state.dirtyChunks)
              newDirtyChunks.delete(chunkKey)
              return { dirtyChunks: newDirtyChunks }
            })
          })
      }
    })
  }, [dirtyChunks, chunks, setChunk])

  // Stable chunk loading with proper tracking
  useEffect(() => {
    if (!worldWorker) return

    const playerChunkX = Math.floor(playerPosition.x / CHUNK_SIZE)
    const playerChunkZ = Math.floor(playerPosition.z / CHUNK_SIZE)

    // Load chunks around player
    for (let dx = -RENDER_DISTANCE; dx <= RENDER_DISTANCE; dx++) {
      for (let dz = -RENDER_DISTANCE; dz <= RENDER_DISTANCE; dz++) {
        const chunkX = playerChunkX + dx
        const chunkZ = playerChunkZ + dz
        const chunkKey = `${chunkX},${chunkZ}`

        // Only load if not already loaded or loading
        if (!chunks.has(chunkKey) && !loadingChunks.current.has(chunkKey)) {
          loadingChunks.current.add(chunkKey)

          worldWorker
            .generateChunk(chunkX, chunkZ)
            .then((data) => {
              const chunk: ChunkData = {
                blocks: data.blocks,
                mesh: data.mesh,
                dirty: false,
                x: data.chunkX,
                z: data.chunkZ,
                generated: true,
              }

              setChunk(chunkKey, chunk)
              addLoadedChunk(chunkKey)
              loadingChunks.current.delete(chunkKey)
              invalidate()
            })
            .catch((error) => {
              console.error("Failed to generate chunk:", error)
              loadingChunks.current.delete(chunkKey)
            })
        }
      }
    }
  }, [playerPosition, chunks, setChunk, addLoadedChunk])

  // Render visible chunks with less aggressive culling
  const visibleChunks = useMemo(() => {
    const visible: ChunkData[] = []
    const playerChunkX = Math.floor(playerPosition.x / CHUNK_SIZE)
    const playerChunkZ = Math.floor(playerPosition.z / CHUNK_SIZE)

    chunks.forEach((chunk, key) => {
      if (!chunk.generated || !chunk.mesh) return

      const distance = Math.abs(chunk.x - playerChunkX) + Math.abs(chunk.z - playerChunkZ)
      if (distance <= RENDER_DISTANCE + 1) {
        visible.push(chunk)
      }
    })

    return visible
  }, [chunks, playerPosition])

  return (
    <>
      {visibleChunks.map((chunk) => (
        <ChunkMesh key={`${chunk.x},${chunk.z}`} chunk={chunk} textureAtlas={textureAtlas} />
      ))}
    </>
  )
})

WorldManager.displayName = "WorldManager"

const HUD = memo(
  ({
    selectedBlock,
    setSelectedBlock,
    isLocked,
  }: {
    selectedBlock: number
    setSelectedBlock: (block: number) => void
    isLocked: boolean
  }) => {
    const blockInfo = [
      { type: BLOCK_TYPES.GRASS, name: "Grass", key: "1", color: "#7CB342" },
      { type: BLOCK_TYPES.DIRT, name: "Dirt", key: "2", color: "#8D6E63" },
      { type: BLOCK_TYPES.STONE, name: "Stone", key: "3", color: "#9E9E9E" },
      { type: BLOCK_TYPES.WOOD, name: "Wood", key: "4", color: "#FF9800" },
    ]

    return (
      <div className="fixed bottom-4 left-1/2 transform -translate-x-1/2 flex gap-2 bg-black/80 p-4 rounded-lg border-2 border-gray-600">
        {blockInfo.map((block) => (
          <button
            key={block.type}
            className={`w-16 h-16 rounded border-2 flex flex-col items-center justify-center text-white font-bold transition-all ${
              selectedBlock === block.type
                ? "border-yellow-400 bg-yellow-400/20 scale-110 shadow-lg shadow-yellow-400/50"
                : "border-gray-500 hover:border-gray-300 bg-gray-800/50"
            }`}
            onClick={() => setSelectedBlock(block.type)}
          >
            <div className="text-xs font-mono">{block.key}</div>
            <div className="text-xs font-mono">{block.name}</div>
          </button>
        ))}
        {!isLocked && (
          <div className="ml-4 flex items-center text-white text-sm font-mono">
            <span>Click to play!</span>
          </div>
        )}
      </div>
    )
  },
)

HUD.displayName = "HUD"

const Instructions = memo(({ isLocked }: { isLocked: boolean }) => {
  const [show, setShow] = React.useState(true)

  if (!show) {
    return (
      <button
        className="fixed top-4 left-4 bg-black/80 text-white px-4 py-2 rounded border border-gray-600 font-mono"
        onClick={() => setShow(true)}
      >
        Help
      </button>
    )
  }

  return (
    <div className="fixed top-4 left-4 bg-black/90 text-white p-4 rounded-lg max-w-sm border-2 border-gray-600">
      <div className="flex justify-between items-center mb-3">
        <h3 className="font-bold font-mono text-green-400">🎮 ADVANCED MINECRAFT</h3>
        <button onClick={() => setShow(false)} className="text-xl hover:text-red-400">
          ×
        </button>
      </div>
      <div className="text-sm space-y-1 font-mono">
        <div className="text-green-300">✅ Pixelated Textures</div>
        <div className="text-green-300">✅ Advanced Movement Tech</div>
        <div className="text-green-300">✅ Quake Air-Strafing</div>
        <div className="text-green-300">✅ Titanfall Slide-Hop</div>
        <div className="text-green-300">✅ CS:GO Surf Physics</div>
        <div className="text-green-300">✅ Wallrun Mechanics</div>
        <div className="mt-3 text-yellow-300">
          <strong>WASD</strong> - Move | <strong>Shift</strong> - Sprint
          <br />
          <strong>Space</strong> - Jump/Bhop | <strong>Ctrl</strong> - Slide
          <br />
          <strong>Mouse</strong> - Air strafe while jumping
          <br />
          <strong>F1</strong> - Toggle debug panel
          <br />
          <strong>1-4</strong> - Select blocks
        </div>
        <div className="mt-2 text-xs text-cyan-300">
          <div>💡 Chain slides + jumps for speed</div>
          <div>💡 Strafe on ramps to surf</div>
          <div>💡 Run at walls to wallrun</div>
          <div>💡 Time jumps for bunny hops</div>
        </div>
      </div>
    </div>
  )
})

const ClickToPlay = memo(
  ({
    onActivate,
    isLocked,
  }: {
    onActivate: () => void
    isLocked: boolean
  }) => {
    if (isLocked) return null

    return (
      <div
        className="fixed inset-0 bg-black/50 flex items-center justify-center cursor-pointer z-10"
        onClick={onActivate}
      >
        <div className="bg-black/90 text-white p-8 rounded-lg text-center border-2 border-green-500">
          <h2 className="text-3xl font-bold mb-4 font-mono text-green-400">🎮 MINECRAFT CLONE</h2>
          <p className="text-lg mb-2 text-green-300">✅ Authentic Pixelated Textures</p>
          <p className="text-lg mb-2 text-green-300">✅ Nostalgic Voxel Styling</p>
          <p className="text-lg mb-2 text-green-300">✅ Natural Tree Generation</p>
          <p className="text-lg mb-4 text-green-300">✅ Optimized Performance</p>
          <p className="text-xl font-mono text-yellow-300">Click anywhere to start!</p>
        </div>
      </div>
    )
  },
)

// Main game component
export default function MinecraftGame() {
  const { selectedBlock, setSelectedBlock, isLocked, setIsLocked, gameStarted, setGameStarted } = useGameStore()

  const activatePointerLock = useCallback(() => {
    const canvas = document.querySelector("canvas")
    if (canvas && document.pointerLockElement !== canvas) {
      canvas.requestPointerLock?.()
    }
    if (!gameStarted) {
      setGameStarted(true)
    }
  }, [gameStarted, setGameStarted])

  // Keyboard controls
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      if (!isLocked) return

      switch (event.key) {
        case "1":
          setSelectedBlock(BLOCK_TYPES.GRASS)
          break
        case "2":
          setSelectedBlock(BLOCK_TYPES.DIRT)
          break
        case "3":
          setSelectedBlock(BLOCK_TYPES.STONE)
          break
        case "4":
          setSelectedBlock(BLOCK_TYPES.WOOD)
          break
        case "5":
          setSelectedBlock(BLOCK_TYPES.LEAVES)
          break
        case "Escape":
          if (document.pointerLockElement) {
            document.exitPointerLock()
          }
          break
      }
    }

    if (gameStarted) {
      window.addEventListener("keydown", handleKeyDown)
      return () => window.removeEventListener("keydown", handleKeyDown)
    }
  }, [gameStarted, isLocked, setSelectedBlock])

  return (
    <div className="w-full h-screen relative">
      <KeyboardControls
        map={[
          { name: "forward", keys: ["ArrowUp", "w", "W"] },
          { name: "backward", keys: ["ArrowDown", "s", "S"] },
          { name: "left", keys: ["ArrowLeft", "a", "A"] },
          { name: "right", keys: ["ArrowRight", "d", "D"] },
          { name: "jump", keys: ["Space"] },
          { name: "shift", keys: ["Shift"] },
          { name: "crouch", keys: ["ControlLeft", "ControlRight", "c", "C"] },
        ]}
      >
        <Canvas
          frameloop="demand" // Performance optimization
          camera={{
            fov: 75,
            near: 0.1,
            far: 300, // Increased for better tree visibility
            position: [0, 35, 0],
          }}
          gl={{
            antialias: false,
            alpha: false,
            powerPreference: "high-performance",
          }}
          onContextMenu={(e) => e.preventDefault()}
        >
          <color attach="background" args={["#87CEEB"]} />
          <PointerLockControls isLocked={isLocked} setIsLocked={setIsLocked} />
          {/* Proper lighting setup */}
          <ambientLight intensity={0.4} />
          <directionalLight position={[10, 20, 10]} intensity={1.0} castShadow={false} color="#ffffff" />
          <fog attach="fog" args={["#87CEEB", 80, 200]} />

          {gameStarted && (
            <>
              <Player isLocked={isLocked} />
              <WorldManager />
            </>
          )}
        </Canvas>

        <ClickToPlay onActivate={activatePointerLock} isLocked={isLocked} />
        <div className="fixed inset-0 pointer-events-none flex items-center justify-center">
          <div className="relative">
            <div className="absolute w-4 h-0.5 bg-white -translate-x-2 -translate-y-0.25"></div>
            <div className="absolute w-0.5 h-4 bg-white -translate-x-0.25 -translate-y-2"></div>
          </div>
        </div>
        <HUD selectedBlock={selectedBlock} setSelectedBlock={setSelectedBlock} isLocked={isLocked} />
        <Instructions isLocked={isLocked} />
      </KeyboardControls>
    </div>
  )
}
