import { useEffect, useState } from 'react'
import { useRealtime, useWsConnected } from './hooks/useRealtime'
import { usePresets } from './hooks/usePresets'
import { ControlBar } from './components/ControlBar'
import { Tabs } from './components/Tabs'
import { SystemTab } from './components/SystemTab'
import type { RobotSnapshot } from './lib/robotTypes'
import { ToastHost } from './components/ToastHost'
import { Icon } from './components/ConsoleWidgets'
import './console.css'
import './macos.css'
import { ErrorBoundary } from './components/ErrorBoundary'
import { api } from './api/client'
import { pushSample } from './hooks/historyStore'

const TABS = [
  { id: 'system', label: '系统' },
  { id: 'teleop', label: '遥操' },
  { id: 'data', label: '数采' },
  { id: 'training', label: '数据与训练' },
  { id: 'inference', label: '推理' },
  { id: 'diagnostics', label: '日志与诊断' },
]

export default function App() {
  const state = useRealtime()
  const wsConnected = useWsConnected()
  const { presets } = usePresets()
  const [active, setActive] = useState('system')
  const [robot, setRobot] = useState<RobotSnapshot | null>(null)

  // Feed the chart ring buffer on every telemetry frame.
  useEffect(() => {
    if (!state) return
    pushSample({
      ts: state.ts,
      ratesHz: state.ratesHz,
      joints: Object.fromEntries(
        Object.entries(state.joints).map(([k, v]) => [k, v.values]),
      ),
    })
  }, [state])

  const canonical = !!robot?.profile && ['running', 'starting', 'paused'].includes(robot.launch_state)
  const feedbackHealthy = canonical ? !!robot?.capabilities?.components.every((c) => {
    const joint = robot.robot?.joints[c.name]
    return joint && !joint.stale && joint.values.length === c.dim
  }) && !robot?.robot?.control.fault : state?.health?.overall === 'ok'

  return (
    <ErrorBoundary>
      <div className="console-shell">
        <ToastHost />
        <aside className="console-sidebar"><div className="window-lights" aria-hidden="true"><i /><i /><i /></div>
          <div className="brand"><span className="brand-mark"><Icon name="robot" size={25} /></span><div><h1>NEXUS</h1><span>ROBOTICS WORKSPACE</span></div></div>
          <div className="nav-caption">工作空间</div>
          <Tabs tabs={TABS} active={active} onChange={setActive} />
          <div className="sidebar-bottom">
            <div className="connection-title">系统连接</div>
            <StatusPill ok={wsConnected} okLabel="Web 在线" badLabel="Web 断开" colorOk="#a7e8cf" />
            <StatusPill ok={!!state} okLabel="ROS 已连接" badLabel="ROS 未连接" colorOk="#a7e8cf" />
            <StatusPill ok={feedbackHealthy} okLabel="机器人反馈正常"
              badLabel={canonical ? '等待有效反馈' : '机器人待机'} colorOk="#a7e8cf" neutral={!canonical && !feedbackHealthy} warn={canonical && !feedbackHealthy} />
            <small>遥操 · 数据 · 学习</small>
          </div>
        </aside>
        <div className="console-workspace">
          <header className="workspace-header"><div><span className="eyebrow">NEXUS / ROBOTICS WORKSPACE</span><h2>{TABS.find((item) => item.id === active)?.label}<span className="header-subtitle">{({system:"机器人工作空间",teleop:"实时动作与控制",data:"记录每一次交互",training:"从数据到策略",inference:"部署与人在环",diagnostics:"运行状态与诊断"} as Record<string,string>)[active]}</span></h2></div>
            <ControlBar state={state} robot={robot} onAction={() => void api.health()} />
          </header>
          <main className="console-main">
            <SystemTab section={active} state={state} robot={robot} onSnapshot={setRobot} presets={presets} onAction={() => void api.health()} />
          </main>
          <footer className="workspace-footer"><span>NEXUS / 多机器人遥操与学习框架</span><span>ROS 2 · 配置驱动</span></footer>
        </div>
      </div>
    </ErrorBoundary>
  )
}

function StatusPill({
  ok,
  okLabel,
  badLabel,
  colorOk,
  warn,
  neutral,
}: {
  ok: boolean
  okLabel: string
  badLabel: string
  colorOk: string
  neutral?: boolean
  warn?: boolean
}) {
  const color = ok && !warn ? colorOk : warn ? '#f59e0b' : neutral ? '#8195ab' : '#ef4444'
  return (
    <span style={pillStyle(color)}>
      <span style={dotStyle(color)} />
      {ok ? okLabel : badLabel}
    </span>
  )
}

function pillStyle(color: string): React.CSSProperties {
  return {
    display: 'inline-flex',
    alignItems: 'center',
    gap: '6px',
    padding: '3px 10px',
    borderRadius: '9999px',
    background: color + '0d',
    color,
    fontSize: '12px',
    fontWeight: 600,
  border: `1px solid ${color}22`,
  }
}
function dotStyle(color: string): React.CSSProperties {
  return { width: '7px', height: '7px', borderRadius: '50%', background: color }
}
