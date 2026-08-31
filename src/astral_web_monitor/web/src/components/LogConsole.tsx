import { useEffect, useRef } from 'react'
import type { NormalisedState } from '../lib/mapUiState'
import { api } from '../api/client'
import { pushToast } from '../hooks/useToast'

export function LogConsole({ state }: { state: NormalisedState | null }) {
  const ref = useRef<HTMLPreElement>(null)
  const lines = state?.logTail ?? []

  useEffect(() => {
    if (ref.current) ref.current.scrollTop = ref.current.scrollHeight
  }, [lines.length])

  async function copy() {
    const text = lines.join('\n')
    if (!text) {
      pushToast('暂无日志', 'info')
      return
    }
    try {
      await navigator.clipboard.writeText(text)
      pushToast(`已复制 ${lines.length} 行`, 'success')
    } catch {
      pushToast('复制失败', 'error')
    }
  }

  async function download() {
    let teleop = lines
    let collect: string[] = []
    const res = await api.logs()
    if (res.ok && res.data) {
      teleop = res.data.teleop ?? teleop
      collect = res.data.collect ?? []
    }
    const parts: string[] = []
    if (teleop.length) {
      parts.push('# teleop launch', ...teleop)
    }
    if (collect.length) {
      parts.push('', '# data collect', ...collect)
    }
    const text = parts.join('\n')
    if (!text.trim()) {
      pushToast('暂无日志', 'info')
      return
    }
    const stamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, 19)
    const blob = new Blob([text], { type: 'text/plain;charset=utf-8' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = `launch-logs-${stamp}.txt`
    a.click()
    URL.revokeObjectURL(url)
    pushToast(`已下载 ${teleop.length + collect.length} 行`, 'success')
  }

  return (
    <div style={wrapStyle}>
      <div style={headerStyle}>
        <span>Launch 日志</span>
        <span style={btnRow}>
          <button type="button" style={hdrBtn} onClick={() => void copy()}>复制</button>
          <button type="button" style={hdrBtn} onClick={() => void download()}>下载</button>
        </span>
      </div>
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
  height: '70vh',
}
const headerStyle: React.CSSProperties = {
  padding: '8px 14px',
  color: '#9ca3af',
  fontSize: '13px',
  fontWeight: 600,
  borderBottom: '1px solid #374151',
  background: '#1f2937',
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'space-between',
  gap: '8px',
}
const btnRow: React.CSSProperties = { display: 'flex', gap: '6px' }
const hdrBtn: React.CSSProperties = {
  background: '#374151',
  color: '#e5e7eb',
  border: '1px solid #4b5563',
  borderRadius: '6px',
  padding: '4px 10px',
  fontSize: '12px',
  fontWeight: 600,
  cursor: 'pointer',
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
