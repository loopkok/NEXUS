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

export interface LatencyStage {
  value_ms: number
  stale: boolean
  ok: boolean
  hz?: number
  expected_hz?: number
}

export interface LatencySummary {
  stages: Record<string, LatencyStage>
}

export interface VideoGateState {
  push_enabled: boolean
  configured: string[]
  active: string[]
}

// 数采节点泳道（独立于遥操预设的 LaunchManager）状态。
export interface CollectLaunchInfo {
  state: TeleopState
  preset: string
  uptime_s: number
  pid: number | null
  log_tail?: string[]
}

// astral_data_collect 的 /data_collect/state latched JSON 镜像。
// stale 由 monitor 后端加注：latched 消息在节点死后仍残留，靠龄期判活。
export interface DataCollectState {
  state: 'IDLE' | 'RECORDING' | 'PAUSED' | 'SAVING'
  session: string
  episode_index: number
  elapsed_s: number
  task_next: string
  samples_per_s: Record<string, number>
  dropped: Record<string, number>
  // 录制期参考相机实率低于 dataset_fps 一半时的告警文案（无告警为 null）
  low_fps_warning?: string | null
  // 录制启动 ~2s 数值/图像全 0（源未就绪空录）的告警文案（无告警为 null）
  empty_warning?: string | null
  // 状态不合法被忽略的控制指令计数（如 SAVING 期按 start；键盘/VR 无
  // disabled 视觉，靠它留痕）
  ignored?: Record<string, number>
  // monitor 后端加注：/data_collect/state 发布者数量（>1 = 有残留/双开节点）
  node_count?: number
  schema?: Record<string, unknown>
  stale?: boolean
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
  latency?: LatencySummary
  video_gate?: VideoGateState | null
  data_collect?: DataCollectState | null
  collect_launch?: CollectLaunchInfo
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
