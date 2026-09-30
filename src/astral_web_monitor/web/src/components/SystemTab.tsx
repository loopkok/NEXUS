import type { NormalisedState } from '../lib/mapUiState'
import type { RobotSnapshot } from '../lib/robotTypes'
import type { Preset } from '../types'
import { RobotConsole } from './RobotConsole'
import { LegacySystemTab } from './LegacySystemTab'
import { MonitorTab } from './MonitorTab'
import { HealthPanel } from './HealthPanel'

type Props = { section: string; state: NormalisedState | null; robot: RobotSnapshot | null; presets: Preset[]; onSnapshot: (value: RobotSnapshot) => void; onAction: () => void }

export function SystemTab({ section, state, robot, presets, onSnapshot, onAction }: Props) {
  const canonical = !!robot?.profile && ['running', 'starting', 'paused'].includes(robot.launch_state)
  return <>
    <RobotConsole section={section} state={state} onSnapshot={onSnapshot} />
    {section === 'system' && !canonical && <details>
      <summary>旧版预设兼容入口</summary>
      <LegacySystemTab state={state} presets={presets} onAction={onAction} />
    </details>}
    {!canonical && ['data', 'inference'].includes(section) && <details>
      <summary>旧版数采与推理控制</summary><MonitorTab state={state} />
    </details>}
    {section === 'diagnostics' && !canonical && <details>
      <summary>旧版话题诊断</summary><HealthPanel state={state} />
    </details>}
  </>
}
