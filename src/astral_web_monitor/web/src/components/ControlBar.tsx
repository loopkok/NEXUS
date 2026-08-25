import type { NormalisedState } from '../lib/mapUiState'
import { api } from '../api/client'
import type { Preset, TeleopState } from '../types'
import { StatusBadge } from './StatusBadge'

interface Props {
  state: NormalisedState | null
  presets: Preset[]
  selected: string
  onSelect: (name: string) => void
  onAction: () => void
}

export function ControlBar({ state, presets, selected, onSelect, onAction }: Props) {
  const teleopState = (state?.teleopState ?? 'stopped') as TeleopState
  const canStart = teleopState === 'stopped' || teleopState === 'start_failed'
  const canStop = teleopState === 'running' || teleopState === 'paused' || teleopState === 'starting'
  const canPause = teleopState === 'running'
  const canResume = teleopState === 'paused'
  // /teleop/start (capture vr_init + arm) is meaningful once the teleop launch
  // is running (arm nodes are up, waiting for the start signal).
  const canTeleopStart = teleopState === 'running' || teleopState === 'paused'

  const run = async (fn: () => Promise<{ ok: boolean; message: string }>) => {
    const res = await fn()
    if (!res.ok) alert(res.message)
    onAction()
  }

  return (
    <div style={barStyle}>
      <StatusBadge state={teleopState} />
      {state?.preset && <span style={presetStyle}>预设: {state.preset}</span>}
      {state && state.uptimeS > 0 && (
        <span style={uptimeStyle}>运行 {formatUptime(state.uptimeS)}</span>
      )}

      <div style={spacer} />

      <select
        value={selected}
        onChange={(e) => onSelect(e.target.value)}
        disabled={!canStart}
        style={selectStyle}
      >
        {presets.map((p) => (
          <option key={p.name} value={p.name}>
            {p.name}
          </option>
        ))}
      </select>

      <button style={btn('#22c55e')} disabled={!canStart} onClick={() => run(() => api.start(selected))}>
        启动
      </button>
      <button style={btn('#f59e0b')} disabled={!canTeleopStart} onClick={() => run(() => api.teleopStart())}>
        开始遥操
      </button>
      <button style={btn('#3b82f6')} disabled={!canPause} onClick={() => run(() => api.pause())}>
        暂停
      </button>
      <button style={btn('#22c55e')} disabled={!canResume} onClick={() => run(() => api.resume())}>
        恢复
      </button>
      <button style={btn('#ef4444')} disabled={!canStop} onClick={() => run(() => api.stop())}>
        停止
      </button>
    </div>
  )
}

function formatUptime(s: number): string {
  if (s < 60) return `${s.toFixed(0)}s`
  const m = Math.floor(s / 60)
  const sec = Math.floor(s % 60)
  if (m < 60) return `${m}m${sec}s`
  const h = Math.floor(m / 60)
  return `${h}h${m % 60}m`
}

const barStyle: React.CSSProperties = {
  display: 'flex',
  alignItems: 'center',
  gap: '12px',
  padding: '12px 16px',
  background: '#1f2937',
  borderBottom: '1px solid #374151',
  flexWrap: 'wrap',
}
const spacer: React.CSSProperties = { flex: 1 }
const presetStyle: React.CSSProperties = { color: '#9ca3af', fontSize: '13px' }
const uptimeStyle: React.CSSProperties = { color: '#6b7280', fontSize: '13px' }
const selectStyle: React.CSSProperties = {
  background: '#374151',
  color: '#e5e7eb',
  border: '1px solid #4b5563',
  borderRadius: '6px',
  padding: '6px 10px',
  fontSize: '13px',
}
const btn = (color: string): React.CSSProperties => ({
  background: color,
  color: 'white',
  border: 'none',
  borderRadius: '6px',
  padding: '7px 16px',
  fontSize: '13px',
  fontWeight: 600,
  cursor: 'pointer',
})
