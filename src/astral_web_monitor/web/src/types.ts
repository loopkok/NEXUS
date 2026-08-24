// Shared types for the ui_state WebSocket frame and REST envelopes.

export interface JointSlot {
  values: number[]
  ts: number
  stale: boolean
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

export interface Health {
  ros_ok: boolean
  launch_state: TeleopState
  launch_pid: number | null
  uptime_s: number
  ws_clients: number
}
