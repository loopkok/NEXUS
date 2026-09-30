export type Component = { name: string; kind: string; dim: number; driver: string; feedback: string; home: boolean }
export type Capabilities = { components: Component[]; home: boolean; viewer: boolean; cameras: string[] }
export type DriverOperation = { id: string; verb: string; status: string; message: string; started_at: number }
export type Job = { profile_sha256: string; id: string; kind: string; status: string; step: string; session: string; logs: string[]; artifacts: Record<string, string> }
export type RobotSnapshot = {
  launch_state: string
  profile: { profile_id: string; profile_sha256: string } | null
  settings: { profile?: string; session?: string; dry_run?: boolean; with_inputs?: boolean; with_cameras?: boolean; with_recording?: boolean; with_policy?: boolean; viewer?: boolean; model?: 'act' | 'pi05'; server_host?: string; server_port?: number; model_manifest?: string; data_root?: string }
  capabilities: Capabilities | null
  operations: DriverOperation[]
  run_id: string | null
  jobs: Job[]
  robot: {
    control: { mode?: string; fault?: string; ready_components?: string[] }
    data_collect?: { state?: string; stale?: boolean; [key: string]: unknown }
    infer?: { state?: string; fault?: string; stale?: boolean }
    joints: Record<string, { values: number[]; stale: boolean; age_s?: number }>
    diagnostics?: { streams: Record<string, { hz: number; age_s: number | null }>; command_publishers: Record<string, Array<{ node: string; reliability: string; durability: string }> | null>; joint_error: Record<string, number> }
  } | null
}
