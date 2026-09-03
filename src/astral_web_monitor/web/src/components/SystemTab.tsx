import { useState } from 'react'
import type { NormalisedState } from '../lib/mapUiState'
import type { Preset } from '../types'
import { api } from '../api/client'
import { pushToast } from '../hooks/useToast'
import { LatencyPanel } from './LatencyPanel'
import { LogConsole } from './LogConsole'
import { VideoCard } from './VideoCard'

interface Props {
  state: NormalisedState | null
  presets: Preset[]
  onAction: () => void
}

export function SystemTab({ state, presets, onAction }: Props) {
  const [selected, setSelected] = useState('')
  // 数采预设走独立泳道（监控 tab 卡片上的节点启停），不占遥操主泳道。
  const teleopPresets = presets.filter((p) => p.package !== 'astral_data_collect')
  const eff = selected || teleopPresets[0]?.name || ''
  const teleopState = state?.teleopState ?? 'stopped'
  const canStart = teleopState === 'stopped' || teleopState === 'start_failed'
  const canStop = teleopState === 'running' || teleopState === 'paused' || teleopState === 'starting'
  const canRestart = teleopState === 'running' || teleopState === 'paused'

  async function run(fn: () => Promise<{ ok: boolean; message: string }>, okMsg?: string) {
    const res = await fn()
    if (res.ok) pushToast(okMsg ?? res.message, 'success')
    else pushToast(res.message, 'error')
    onAction()
  }

  function stop() {
    if (!confirm('确认停止 launch？（SIGINT，30s 超时 SIGKILL）')) return
    void run(() => api.stop(), '已发送停止信号')
  }
  function restart() {
    if (!confirm('确认重启当前预设？（停止后重新启动）')) return
    void run(() => api.restart(), '重启中')
  }

  return (
    <div style={wrapStyle}>
      <div style={cardStyle}>
        <div style={titleStyle}>预设管理</div>
        <div style={rowStyle}>
          <select
            value={eff}
            onChange={(e) => setSelected(e.target.value)}
            disabled={!canStart}
            style={selectStyle}
          >
            {teleopPresets.map((p) => (
              <option key={p.name} value={p.name}>{p.name}</option>
            ))}
          </select>
          <button style={btn('#22c55e')} disabled={!canStart || !eff} onClick={() => run(() => api.start(eff), `启动 ${eff}`)}>
            启动
          </button>
          <button style={btn('#ef4444')} disabled={!canStop} onClick={stop}>
            停止
          </button>
          <button style={btn('#3b82f6')} disabled={!canRestart} onClick={restart}>
            重启
          </button>
          <button
            style={homeBtn}
            title="先确保电机使能，再让双臂沿 init_pose → init_waypoints → 零位 慢速收回"
            disabled={!canRestart}
            onClick={() => {
              if (!confirm('确认 HOME？双臂将从当前位姿经 init_pose → init_waypoints 收回零位（期间会 disarm，需重新启动/一键就绪再遥操）')) return
              void run(() => api.teleopHome(), 'HOME 已下发')
            }}
          >
            HOME
          </button>
        </div>
        <div style={hintStyle}>
          非侵入：仅通过子进程管理 <code>ros2 launch</code>（启动/停止/重启），不直接控制硬件。
          CLI 启动的遥操无预设记录，不支持重启（请用停止+启动）。
        </div>
      </div>

      <div style={cardStyle}>
        <div style={titleStyle}>机器人模式（driver 服务）</div>
        <div style={rowStyle}>
          <button style={btn('#22c55e')} onClick={() => run(() => api.robotReady(), '一键就绪 OK')}>
            一键就绪
          </button>
          <button style={btn('#3b82f6')} onClick={() => run(() => api.robotHome(), '全关节归零')}>
            归零
          </button>
          <button style={btn('#3b82f6')} onClick={() => run(() => api.robotPosition(), '位置保持')}>
            位置保持
          </button>
          <button style={btn('#f59e0b')} onClick={() => { if (confirm('阻尼释放后可手动拖拽臂，确认？')) run(() => api.robotDamping(), '阻尼释放') }}>
            阻尼释放
          </button>
          <button style={estopBtn} onClick={() => { if (confirm('确认急停断电？')) run(() => api.robotEstop(), '已断电') }}>
            ⛔ 急停(断电)
          </button>
        </div>
        <div style={hintStyle}>
          调用 <code>astral_robot_control</code> driver 已有的 Trigger 服务（~/ready · ~/home · ~/position · ~/damping · ~/estop）。
          典型流程：遥操中 → <b>停止</b>（臂保持末位姿）→ <b>阻尼释放</b>（手动拖回 home）→ <b>位置保持</b>或<b>归零</b>。
          急停=真断电（disable），臂失去保持力；恢复需重新<b>一键就绪</b>。
        </div>
      </div>

      <LatencyPanel state={state} />

      <VideoCard state={state} />

      <LogConsole state={state} />
    </div>
  )
}

const wrapStyle: React.CSSProperties = { display: 'flex', flexDirection: 'column', gap: '14px' }
const cardStyle: React.CSSProperties = {
  background: '#1f2937',
  border: '1px solid #374151',
  borderRadius: '10px',
  padding: '14px',
  display: 'flex',
  flexDirection: 'column',
  gap: '10px',
}
const titleStyle: React.CSSProperties = { color: '#e5e7eb', fontWeight: 700, fontSize: '14px' }
const rowStyle: React.CSSProperties = { display: 'flex', gap: '8px', flexWrap: 'wrap', alignItems: 'center' }
const selectStyle: React.CSSProperties = {
  background: '#374151',
  color: '#e5e7eb',
  border: '1px solid #4b5563',
  borderRadius: '6px',
  padding: '6px 10px',
  fontSize: '13px',
  minWidth: '180px',
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
const estopBtn: React.CSSProperties = {
  background: '#dc2626',
  color: 'white',
  border: '2px solid #fca5a5',
  borderRadius: '6px',
  padding: '7px 16px',
  fontSize: '13px',
  fontWeight: 800,
  cursor: 'pointer',
}
const homeBtn: React.CSSProperties = {
  background: '#7c3aed',
  color: 'white',
  border: 'none',
  borderRadius: '6px',
  padding: '7px 16px',
  fontSize: '13px',
  fontWeight: 600,
  cursor: 'pointer',
  marginLeft: '4px',
}
const hintStyle: React.CSSProperties = { color: '#6b7280', fontSize: '12px', lineHeight: 1.5 }
