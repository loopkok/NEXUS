// Shared types for the ui_state WebSocket frame and REST envelopes.

export interface JointSlot {
  values: number[]
  ts: number
  stale: boolean
}

export type HealthStatus = 'ok' | 'slow' | 'stale' | 'down'

export interface EntityHealth {
  stale: boolean
  state_hz: number
  cmd_hz: number
  expected_hz: number
  slow: boolean
  status: HealthStatus
}

export interface HealthSummary {
  overall: HealthStatus
  entities: Record<string, EntityHealth>
}

export interface VideoGateState {
  push_enabled: boolean
  configured: string[]
  active: string[]
}

export interface UiState {
  type: 'ui_state'
  ts: number
  teleop: {
    state: TeleopState
    preset: string
    uptime_s: number
    pid: number | null
  }
  joints: Record<string, JointSlot>
  rates_hz: Record<string, number>
  state_rates_hz?: Record<string, number>
  health?: HealthSummary
  video_gate?: VideoGateState | null
  log_tail: string[]
}

export type TeleopState =
  | 'stopped'
  | 'starting'
  | 'running'
  | 'stopping'
  | 'paused'
  | 'start_failed'

export interface Preset {
  name: string
  package: string
  launch: string
  args: Record<string, string>
  description: string
}

export interface ApiEnvelope<T = unknown> {
  ok: boolean
  message: string
  data: T
}

export interface HealthData {
  ros_ok: boolean
  launch_state: TeleopState
  launch_pid: number | null
  uptime_s: number
  ws_clients: number
  health: HealthSummary
}

export interface VideoCameraInfo {
  label: string
  device: string
  source: string
  preset: string
  exists: boolean
  sysfs_name: string
}

export interface VideoStatusData {
  configured: VideoCameraInfo[]
  gate: VideoGateState | null
  online: boolean
}
