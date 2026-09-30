import { useEffect, useId, useState } from 'react'
import type { Capabilities, RobotSnapshot } from '../lib/robotTypes'
import { componentLabel, Icon, stateLabel } from './ConsoleWidgets'

// Decorative component illustration. It deliberately does not represent FK or collision geometry.
function RobotIllustration({ caps }: { caps?: Capabilities | null }) {
  const id = useId().replace(/:/g, '')
  const arms = caps?.components.filter(c => c.kind === 'arm') ?? []
  return <svg className="robot-illustration" viewBox="0 0 560 270" role="img" aria-label="机器人组件示意，非运动学视图">
    <defs>
      <linearGradient id={`${id}-metal`} x1="0" y1="0" x2="1" y2="1"><stop stopColor="#e4e8f5" /><stop offset=".32" stopColor="#8c98b8" /><stop offset=".58" stopColor="#46506c" /><stop offset=".82" stopColor="#a1adca" /><stop offset="1" stopColor="#303850" /></linearGradient>
      <linearGradient id={`${id}-dark`} x2="0" y2="1"><stop stopColor="#48516a" /><stop offset="1" stopColor="#171d30" /></linearGradient>
      <radialGradient id={`${id}-halo`}><stop stopColor="#66d9bd" stopOpacity=".22" /><stop offset="1" stopColor="#66d9bd" stopOpacity="0" /></radialGradient>
      <filter id={`${id}-shadow`}><feDropShadow dx="0" dy="12" stdDeviation="7" floodColor="#05060d" floodOpacity=".6" /></filter>
    </defs>
    <ellipse cx="320" cy="166" rx="210" ry="110" fill={`url(#${id}-halo)`} />
    <g stroke="#9fa9d8" strokeOpacity=".08" fill="none">{[0,1,2,3,4,5].map(i => <path key={i} d={`M${50+i*45} 242L${190+i*45} 186M${60+i*50} 190L${330+i*25} 250`} />)}<ellipse cx="295" cy="226" rx="198" ry="32" /><ellipse cx="295" cy="226" rx="155" ry="24" /></g>
    <ellipse cx="280" cy="233" rx="146" ry="16" fill="#090d19" opacity=".55" />
    {arms.map((arm, i) => {
      const x = arms.length === 1 ? 280 : 155 + i * (250 / Math.max(1,arms.length-1))
      const hand = caps?.components.find(c => c.kind !== 'arm' && (c.name.startsWith(arm.name.split('_')[0])))
      return <g key={arm.name} transform={`translate(${x} 0) scale(${i % 2 ? -1 : 1} 1)`} filter={`url(#${id}-shadow)`}>
        <path d="M-30 217L31 217L39 229L-37 229Z" fill={`url(#${id}-dark)`} stroke="#5d6885" />
        <rect x="-21" y="184" width="43" height="37" rx="8" fill={`url(#${id}-metal)`} />
        <path d="M0 189L-24 132L-42 125" fill="none" stroke="#161d30" strokeWidth="31" strokeLinecap="round" />
        <path d="M0 189L-24 132L-42 125" fill="none" stroke={`url(#${id}-metal)`} strokeWidth="23" strokeLinecap="round" />
        <path d="M-40 126L-18 68L24 54" fill="none" stroke="#151c30" strokeWidth="27" strokeLinecap="round" />
        <path d="M-40 126L-18 68L24 54" fill="none" stroke={`url(#${id}-metal)`} strokeWidth="19" strokeLinecap="round" />
        <path d="M24 54L59 89" fill="none" stroke={`url(#${id}-metal)`} strokeWidth="16" strokeLinecap="round" />
        {[[0,186],[-40,126],[-18,68],[24,54],[59,89]].map(([cx,cy],j) => <g key={j}><circle cx={cx} cy={cy} r={j < 3 ? 15 : 11} fill={`url(#${id}-dark)`} stroke="#9eaaca" strokeWidth="1.3" /><circle cx={cx} cy={cy} r={j < 3 ? 7 : 4} fill="#1e293e" stroke="#607690" /><path d={`M${cx-5} ${cy-10}h10`} stroke="#78e4c1" strokeWidth="2" strokeLinecap="round" /></g>)}
        {hand && <g transform="translate(64 102) rotate(-28)"><rect x="-11" y="-8" width="23" height="25" rx="6" fill={`url(#${id}-metal)`} />{Array.from({length:hand.kind === 'hand' ? 4 : 2},(_,j) => <path key={j} d={`M${-8+j*(hand.kind === 'hand' ? 5 : 15)} 12v${20-j%2*5}l3 6`} fill="none" stroke="#a5b5cc" strokeWidth="4" strokeLinecap="round" />)}</g>}
      </g>
    })}
    {!arms.length && <text x="280" y="140" textAnchor="middle" fill="#8a91ac" fontSize="14">选择机器人配置</text>}
  </svg>
}

function Sparkline({ values }: { values: number[] }) {
  if (values.length < 2) return <svg viewBox="0 0 100 28" aria-hidden="true"><path d="M1 23H99" stroke="currentColor" strokeDasharray="3 4" opacity=".3" /></svg>
  const max = Math.max(1,...values)
  const points = values.map((v,i) => `${i*100/(values.length-1)},${25-v/max*20}`).join(' ')
  return <svg viewBox="0 0 100 28" aria-hidden="true"><polyline points={`0,28 ${points} 100,28`} fill="currentColor" opacity=".08" /><polyline points={points} fill="none" stroke="currentColor" strokeWidth="1.4" strokeLinejoin="round" /></svg>
}

export function RobotOverview({ caps, snapshot, active, name, error }: { caps?: Capabilities | null; snapshot: RobotSnapshot | null; active: boolean; name: string; error: string }) {
  const [history, setHistory] = useState<Record<string,number[]>>({})
  const profileKey = snapshot?.profile?.profile_sha256 ?? name
  useEffect(() => { setHistory({}) },[profileKey])
  useEffect(() => {
    if (!active || !snapshot) return
    setHistory(prev => Object.fromEntries((caps?.components ?? []).map(c => {
      const metric = snapshot.robot?.diagnostics?.streams[`feedback/${c.name}`]
      const hz = metric && metric.age_s !== null && metric.age_s < .5 ? metric.hz : 0
      return [c.name,[...(prev[c.name] ?? []),hz].slice(-40)]
    })))
  },[snapshot,active,caps])
  const online = caps?.components.filter(c => { const j = snapshot?.robot?.joints[c.name]; return active && j && !j.stale && j.values.length === c.dim }).length ?? 0
  const dim = caps?.components.reduce((sum,c) => sum+c.dim,0) ?? 0
  const mode = snapshot?.robot?.control.mode
  return <>
    <section className="robot-hero">
      <div className="hero-copy"><span className="eyebrow">YOUR ROBOT WORKSPACE</span><div className="hero-title"><h2>{name}</h2><span className="platform-chip">ROS 2</span></div><p>从人类动作，到机器人学习。</p>
        <div className="hero-state"><span className={`state-tag ${active ? 'is-online' : ''}`}><i />{stateLabel(snapshot?.launch_state)}</span><span>{active ? snapshot?.settings.dry_run ? '接口调试' : caps?.viewer ? '仿真运行' : '真机运行' : '等待启动'}<span className="hero-state-separator">/</span>{mode ?? 'IDLE'}</span></div>
        <div className="hero-stats"><div><span>组件连接</span><strong>{online}<small> / {caps?.components.length ?? 0}</small></strong></div><div><span>关节维度</span><strong>{dim}<small> DOF</small></strong></div><div><span>视觉通道</span><strong>{caps?.cameras.length ?? 0}<small> CAM</small></strong></div></div>
      </div>
      <div className="hero-visual"><div className="stage-orbit" /><RobotIllustration caps={caps} /><span className="illustration-caption">组件示意 · 非运动学视图</span><span className="stage-coordinate">ROBOT / WORLD</span></div>
    </section>
    {(snapshot?.robot?.control.fault || error) && <div role="alert" className="notice-error">{error || snapshot?.robot?.control.fault}</div>}
    <div className="component-grid">{caps?.components.map((c,i) => {
      const joint = snapshot?.robot?.joints[c.name]
      const connected = active && joint && !joint.stale && joint.values.length === c.dim
      const metric = snapshot?.robot?.diagnostics?.streams[`feedback/${c.name}`]
      return <article key={c.name} className={`component-tile tile-${i%4}`}><div className="tile-top"><span className="component-icon"><Icon name={c.kind === 'arm' ? 'robot' : 'teleop'} size={21} /></span><span className={`component-status ${connected ? 'is-online' : ''}`}><i />{connected ? '在线' : '待连接'}</span></div><strong>{componentLabel(c)}</strong><div className="tile-meta"><span>{c.dim} 自由度</span><span>{connected && metric ? `${metric.hz.toFixed(0)} Hz` : '等待反馈'}</span></div><Sparkline values={active ? history[c.name] ?? [] : []} /><span className="tile-name">{c.name}</span></article>
    })}</div>
  </>
}
