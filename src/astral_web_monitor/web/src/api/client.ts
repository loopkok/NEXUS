// REST client mirroring rob_station's api/client.js convention.
import type { ApiEnvelope, HealthData, Preset, VideoStatusData } from '../types'

const base = import.meta.env.VITE_API_BASE ?? ''

async function post<T>(path: string, body?: unknown): Promise<ApiEnvelope<T>> {
  const res = await fetch(base + path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: body ? JSON.stringify(body) : undefined,
  })
  // HTTP errors (409/503) arrive as {detail}; normalise into the envelope shape
  // so callers can always read .ok / .message.
  if (!res.ok) {
    let detail = `HTTP ${res.status}`
    try {
      const j = await res.json()
      detail = j.detail ?? detail
    } catch {
      /* keep status */
    }
    return { ok: false, message: detail, data: null as unknown as T }
  }
  return res.json()
}

async function get<T>(path: string): Promise<ApiEnvelope<T>> {
  const res = await fetch(base + path)
  return res.json()
}

export const api = {
  health: () => get<HealthData>('/api/v1/health'),
  presets: () => get<Preset[]>('/api/v1/presets'),
  state: () => get<unknown>('/api/v1/state'),
  start: (preset: string) => post<unknown>('/api/v1/start', { preset }),
  stop: () => post<unknown>('/api/v1/stop'),
  pause: () => post<unknown>('/api/v1/pause'),
  resume: () => post<unknown>('/api/v1/resume'),
  teleopStart: () => post<unknown>('/api/v1/teleop/start'),
  restart: () => post<unknown>('/api/v1/restart'),
  // Robot hardware mode (driver services — real hardware actions)
  robotReady: () => post<unknown>('/api/v1/robot/ready'),
  robotHome: () => post<unknown>('/api/v1/robot/home'),
  robotEstop: () => post<unknown>('/api/v1/robot/estop'),
  robotDamping: () => post<unknown>('/api/v1/robot/damping'),
  robotPosition: () => post<unknown>('/api/v1/robot/position'),
  // Video return gate (quest3_video_streamer — streamer owns the cameras)
  videoStatus: () => get<VideoStatusData>('/api/v1/video/status'),
  videoPush: (enabled: boolean) => post<unknown>('/api/v1/video/push', { enabled }),
  videoCameras: (cameras: string[]) => post<unknown>('/api/v1/video/cameras', { cameras }),
  // MJPEG live preview URL for <img src> (streamed, not fetched)
  videoFeedUrl: (label: string) => `${base}/api/v1/video/feed/${encodeURIComponent(label)}`,
  // Data collection recorder (astral_data_collect; works even when the
  // recorder was started from CLI — the control surface is pure topics)
  collectControl: (cmd: string) => post<unknown>('/api/v1/collect/control', { cmd }),
  collectTask: (text: string) => post<unknown>('/api/v1/collect/task', { text }),
  // 数采节点泳道（独立于遥操预设生命周期）
  collectLaunchStart: () => post<unknown>('/api/v1/collect/launch/start'),
  collectLaunchStop: () => post<unknown>('/api/v1/collect/launch/stop'),
  collectLaunchRestart: () => post<unknown>('/api/v1/collect/launch/restart'),
  logs: () => get<{ teleop: string[]; collect: string[] }>('/api/v1/logs'),
}
