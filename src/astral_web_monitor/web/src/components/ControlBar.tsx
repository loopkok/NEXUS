import type { NormalisedState } from '../lib/mapUiState'
import { api } from '../api/client'
import type { RobotSnapshot } from '../lib/robotTypes'
import type { TeleopState } from '../types'
import { Icon, stateLabel } from './ConsoleWidgets'
import { pushToast } from '../hooks/useToast'

interface Props {
  state: NormalisedState | null
  robot: RobotSnapshot | null
  onAction: () => void
}

export function ControlBar({ state, robot, onAction }: Props) {
  const canonical = !!robot?.profile
  const canonicalRunning = canonical && ['running', 'starting', 'paused'].includes(robot!.launch_state)
  const teleopState = (state?.teleopState ?? 'stopped') as TeleopState
  const busy = robot?.operations?.some((op) => op.status === 'running') ?? false
  const legacyRunning = !canonical && ['running', 'paused', 'starting'].includes(teleopState)
  const canPause = teleopState === 'running' && !busy
  const canResume = teleopState === 'paused'
  // E-STOP + 开始遥操 are unconditional on the backend (arm node is the
  // authority). Enable whenever the monitor is alive.
  const alive = state != null && (canonicalRunning || !canonical && (['running', 'paused', 'starting'].includes(teleopState) || state.health.overall === 'ok'))

  async function run(fn: () => Promise<{ ok: boolean; message: string }>, okMsg?: string) {
    const res = await fn()
    if (res.ok) {
      pushToast(okMsg ?? res.message, 'success')
    } else {
      pushToast(res.message, 'error')
    }
    onAction()
  }

  function estop() {
    if (!confirm('确认请求驱动急停？电机失能可能失去保持力，各组件结果以驱动反馈为准。')) return
    void run(() => canonical ? api.nexusDriver('estop') : api.robotEstop())
  }
  function damping() {
    if (!confirm('确认阻尼释放？会先自动 disarm 遥操，切换运动模式=阻尼，可手动拖拽臂回 home（电机仍上电）。')) return
    void run(() => api.robotDamping(), '阻尼释放 (可手动拖拽)')
  }

  return (
    <div className="global-controls">
<span className="toolbar-state"><i />{stateLabel(teleopState)}</span>

      {state && state.uptimeS > 0 && (
        <span className="uptime">运行 {formatUptime(state.uptimeS)}</span>
      )}



      {/* E-STOP: real hardware power-off via driver ~/estop. */}
      <button
        className="global-estop"
        disabled={!alive}
        onClick={estop}
        title="请求当前机器人驱动急停；软件封锁与硬件断电由驱动能力决定"
      >
        <Icon name="power" size={15} />急停
      </button>
      {legacyRunning && <button
        className="toolbar-button"
        disabled={!alive}
        onClick={damping}
        title="阻尼释放：调 driver ~/damping → motion_mode=0，可手动拖拽臂"
      >
        阻尼释放
      </button>}
      {legacyRunning && <button className="toolbar-button" disabled={!alive} onClick={() => run(() => api.teleopStart())}>
        开始遥操
      </button>}
      {canPause && <button className="toolbar-button" disabled={!canPause} onClick={() => run(() => api.pause())}>
        暂停
      </button>}
      {canResume && <button className="toolbar-button" disabled={!canResume} onClick={() => run(() => api.resume())}>
        {canonical ? '重锚并恢复' : '恢复'}
      </button>}
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
