import { useRealtime } from './hooks/useRealtime'
import { usePresets } from './hooks/usePresets'
import { ControlBar } from './components/ControlBar'
import { JointPanel } from './components/JointPanel'
import { LogConsole } from './components/LogConsole'
import { useState } from 'react'
import { api } from './api/client'

export default function App() {
  const state = useRealtime()
  const { presets } = usePresets()
  const [selected, setSelected] = useState('')

  const effectiveSelected = selected || presets[0]?.name || ''
  const j = state?.joints ?? {}

  return (
    <div style={appStyle}>
      <header style={headerStyle}>
        <h1 style={titleStyle}>Astral Web Monitor</h1>
        <span style={subStyle}>非侵入式遥操作监控</span>
      </header>

      <ControlBar
        state={state}
        presets={presets}
        selected={effectiveSelected}
        onSelect={setSelected}
        onAction={() => void api.health()}
      />

      <main style={mainStyle}>
        <div style={gridStyle}>
          <JointPanel title="左臂 (7-DoF)" joint={j.left_arm} rateHz={state?.ratesHz.left_arm_cmd} />
          <JointPanel title="右臂 (7-DoF)" joint={j.right_arm} rateHz={state?.ratesHz.right_arm_cmd} />
          <JointPanel title="左夹爪" joint={j.left_gripper} rateHz={state?.ratesHz.left_gripper_cmd} />
          <JointPanel title="右灵巧手 (20-DoF)" joint={j.right_hand} rateHz={state?.ratesHz.right_hand_cmd} />
        </div>

        {state && (
          <div style={ratesStyle}>
            <strong>指令频率:</strong>{' '}
            {Object.entries(state.ratesHz).map(([k, v]) => (
              <span key={k} style={rateChip}>
                {k}: <b>{v.toFixed(1)}</b> Hz
              </span>
            ))}
          </div>
        )}

        <LogConsole state={state} />
      </main>
    </div>
  )
}

const appStyle: React.CSSProperties = {
  minHeight: '100vh',
  background: '#0b0f17',
  color: '#e5e7eb',
  fontFamily: 'system-ui, -apple-system, "Segoe UI", Roboto, sans-serif',
  display: 'flex',
  flexDirection: 'column',
}
const headerStyle: React.CSSProperties = {
  display: 'flex',
  alignItems: 'baseline',
  gap: '12px',
  padding: '16px 20px',
  borderBottom: '1px solid #1f2937',
}
const titleStyle: React.CSSProperties = { margin: 0, fontSize: '20px', fontWeight: 700 }
const subStyle: React.CSSProperties = { color: '#6b7280', fontSize: '13px' }
const mainStyle: React.CSSProperties = {
  flex: 1,
  padding: '20px',
  display: 'flex',
  flexDirection: 'column',
  gap: '16px',
  maxWidth: '1400px',
  width: '100%',
  margin: '0 auto',
}
const gridStyle: React.CSSProperties = {
  display: 'grid',
  gridTemplateColumns: 'repeat(auto-fit, minmax(280px, 1fr))',
  gap: '14px',
}
const ratesStyle: React.CSSProperties = {
  display: 'flex',
  flexWrap: 'wrap',
  gap: '12px',
  alignItems: 'center',
  fontSize: '13px',
  color: '#9ca3af',
  padding: '10px 14px',
  background: '#1f2937',
  borderRadius: '8px',
}
const rateChip: React.CSSProperties = {
  background: '#374151',
  padding: '3px 10px',
  borderRadius: '6px',
  color: '#d1d5db',
}
