import { useEffect, useRef } from 'react'
import type { NormalisedState } from '../lib/mapUiState'

export function LogConsole({ state }: { state: NormalisedState | null }) {
  const ref = useRef<HTMLPreElement>(null)
  const lines = state?.logTail ?? []

  useEffect(() => {
    if (ref.current) ref.current.scrollTop = ref.current.scrollHeight
  }, [lines.length])

  return (
    <div style={wrapStyle}>
      <div style={headerStyle}>Launch 日志</div>
      <pre ref={ref} style={preStyle}>
        {lines.length === 0 ? '（等待启动...）' : lines.join('\n')}
      </pre>
    </div>
  )
}

const wrapStyle: React.CSSProperties = {
  background: '#111827',
  border: '1px solid #374151',
  borderRadius: '10px',
  display: 'flex',
  flexDirection: 'column',
  overflow: 'hidden',
  height: '240px',
}
const headerStyle: React.CSSProperties = {
  padding: '8px 14px',
  color: '#9ca3af',
  fontSize: '13px',
  fontWeight: 600,
  borderBottom: '1px solid #374151',
  background: '#1f2937',
}
const preStyle: React.CSSProperties = {
  flex: 1,
  margin: 0,
  padding: '10px 14px',
  color: '#d1d5db',
  fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace',
  fontSize: '12px',
  lineHeight: '1.5',
  overflow: 'auto',
  whiteSpace: 'pre-wrap',
  wordBreak: 'break-all',
}
