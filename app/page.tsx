"use client"

import { Suspense } from "react"
import dynamic from "next/dynamic"

// Lazy load the heavy game component
const MinecraftGame = dynamic(() => import("../components/minecraft-game"), {
  ssr: false,
  loading: () => (
    <div className="w-full h-screen bg-gradient-to-b from-blue-400 to-green-400 flex items-center justify-center">
      <div className="text-white text-2xl">Loading optimized Minecraft...</div>
    </div>
  ),
})

export default function Page() {
  return (
    <Suspense fallback={<div>Loading...</div>}>
      <MinecraftGame />
    </Suspense>
  )
}
