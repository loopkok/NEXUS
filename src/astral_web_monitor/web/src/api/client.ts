// REST client mirroring rob_station's api/client.js convention.
import type { ApiEnvelope, Health, Preset } from '../types'

const base = import.meta.env.VITE_API_BASE ?? ''

async function post<T>(path: string, body?: unknown): Promise<ApiEnvelope<T>> {
  const res = await fetch(base + path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: body ? JSON.stringify(body) : undefined,
  })
  return res.json()
}

async function get<T>(path: string): Promise<ApiEnvelope<T>> {
  const res = await fetch(base + path)
  return res.json()
}

export const api = {
  health: () => get<Health>('/api/v1/health'),
  presets: () => get<Preset[]>('/api/v1/presets'),
  state: () => get<unknown>('/api/v1/state'),
  start: (preset: string) => post<unknown>('/api/v1/start', { preset }),
  stop: () => post<unknown>('/api/v1/stop'),
  pause: () => post<unknown>('/api/v1/pause'),
  resume: () => post<unknown>('/api/v1/resume'),
  teleopStart: () => post<unknown>('/api/v1/teleop/start'),
}
