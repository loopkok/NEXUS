// Module-level singleton WebSocket hook (one connection per app).
// Uses useSyncExternalStore so 30Hz updates don't trigger deep re-renders.
import { useSyncExternalStore } from 'react'
import type { UiState } from '../types'
import { mapUiState, type NormalisedState } from '../lib/mapUiState'

let ws: WebSocket | null = null
let state: NormalisedState | null = null
const listeners = new Set<() => void>()

function emit() {
  listeners.forEach((l) => l())
}

function connect() {
  if (ws) return
  const proto = location.protocol === 'https:' ? 'wss' : 'ws'
  ws = new WebSocket(`${proto}://${location.host}/ws/telemetry`)
  ws.onmessage = (e) => {
    try {
      const msg = JSON.parse(e.data) as UiState
      if (msg.type === 'ui_state') {
        state = mapUiState(msg)
        emit()
      }
    } catch {
      /* ignore malformed frames */
    }
  }
  ws.onclose = () => {
    ws = null
    setTimeout(connect, 3000)
  }
  ws.onerror = () => {
    ws?.close()
  }
}

function subscribe(cb: () => void) {
  if (!ws) connect()
  listeners.add(cb)
  return () => listeners.delete(cb)
}

function getSnapshot(): NormalisedState | null {
  return state
}

export function useRealtime(): NormalisedState | null {
  return useSyncExternalStore(subscribe, getSnapshot, getSnapshot)
}
