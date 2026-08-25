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
  stateRatesHz: Record<string, number>
  health: {
    overall: string
    entities: Record<string, {
      stale: boolean
      stateHz: number
      cmdHz: number
      expectedHz: number
      slow: boolean
      status: string
    }>
  }
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
  const rawHealth = msg.health ?? { overall: 'ok', entities: {} }
  const entities: NormalisedState['health']['entities'] = {}
  for (const [k, v] of Object.entries(rawHealth.entities)) {
    entities[k] = {
      stale: v.stale,
      stateHz: v.state_hz,
      cmdHz: v.cmd_hz,
      expectedHz: v.expected_hz,
      slow: v.slow,
      status: v.status,
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
    stateRatesHz: msg.state_rates_hz ?? {},
    health: { overall: rawHealth.overall, entities },
    logTail: msg.log_tail,
  }
}
