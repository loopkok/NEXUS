// Normalise the raw ui_state frame into a camelCase view model.
import type { UiState } from '../types'

export interface NormalisedJoint {
  values: number[]
  stale: boolean
  ageS: number
}

export interface NormalisedState {
  ts: number
  teleopState: string
  preset: string
  uptimeS: number
  pid: number | null
  joints: Record<string, NormalisedJoint>
  ratesHz: Record<string, number>
  logTail: string[]
}

export function mapUiState(msg: UiState, now: number = Date.now() / 1000): NormalisedState {
  const joints: Record<string, NormalisedJoint> = {}
  for (const [k, v] of Object.entries(msg.joints)) {
    joints[k] = {
      values: v.values,
      stale: v.stale,
      ageS: v.ts ? Math.max(0, now - v.ts) : -1,
    }
  }
  return {
    ts: msg.ts,
    teleopState: msg.teleop.state,
    preset: msg.teleop.preset,
    uptimeS: msg.teleop.uptime_s,
    pid: msg.teleop.pid,
    joints,
    ratesHz: msg.rates_hz,
    logTail: msg.log_tail,
  }
}
