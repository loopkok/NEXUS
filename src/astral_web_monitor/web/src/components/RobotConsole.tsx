import { useEffect, useState } from 'react'
import '../console.css'
import type { NormalisedState } from '../lib/mapUiState'
import type { Capabilities, RobotSnapshot } from '../lib/robotTypes'
import { api } from '../api/client'
import { pushToast } from '../hooks/useToast'
import { Icon, FeatureSwitch, componentLabel } from './ConsoleWidgets'
import { LogConsole } from './LogConsole'
import { RobotOverview } from './RobotOverview'

type ProfileItem = { instance: string; id: string; state_dim: number; sha256: string; capabilities: Capabilities }
type Props = { section: string; state: NormalisedState | null; onSnapshot: (value: RobotSnapshot) => void }

export function RobotConsole({ section, state, onSnapshot }: Props) {
  const [profiles, setProfiles] = useState<ProfileItem[]>([])
  const [selected, setSelected] = useState('')
  const [session, setSession] = useState('default_task')
  const [dryRun, setDryRun] = useState(true)
  const [inputs, setInputs] = useState(true)
  const [cameras, setCameras] = useState(false)
  const [recording, setRecording] = useState(false)
  const [policy, setPolicy] = useState(false)
  const [viewer, setViewer] = useState(false)
  const [manifest, setManifest] = useState('')
  const [model, setModel] = useState<'act' | 'pi05'>('act')
  const [serverHost, setServerHost] = useState('127.0.0.1')
  const [serverPort, setServerPort] = useState(8001)
  const [task, setTask] = useState('')
  const [steps, setSteps] = useState(1000)
  const [snapshot, setSnapshot] = useState<RobotSnapshot | null>(null)
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState('')
  const [validated, setValidated] = useState('')

  useEffect(() => {
    let alive = true
    void api.nexusProfiles().then((res) => {
      if (alive && res.ok) {
        setProfiles(res.data)
        setSelected((current) => current || res.data[0]?.id || '')
      }
    }).catch((e) => { if (alive) setError(String(e)) })
    let hydrated = false
    const refresh = async () => {
      try {
        const res = await api.nexusState()
        if (!alive) return
        if (!res.ok) { setError(res.message); return }
        setSnapshot(res.data)
        onSnapshot(res.data)
        if (!hydrated && res.data.settings.profile) {
          const cfg = res.data.settings
          setSelected(cfg.profile!)
          setSession(cfg.session ?? 'default_task')
          setDryRun(cfg.dry_run ?? true)
          setInputs(cfg.with_inputs ?? true)
          setCameras(cfg.with_cameras ?? false)
          setRecording(cfg.with_recording ?? false)
          setPolicy(cfg.with_policy ?? false)
          setViewer(cfg.viewer ?? false)
          setModel(cfg.model ?? 'act')
          setServerHost(cfg.server_host ?? '127.0.0.1')
          setServerPort(cfg.server_port ?? 8001)
          setManifest(cfg.model_manifest ?? '')
        }
        hydrated = true
      } catch (e) { if (alive) setError(`控制台连接失败：${String(e)}`) }
    }
    void refresh()
    const timer = window.setInterval(() => void refresh(), 1000)
    return () => { alive = false; window.clearInterval(timer) }
  }, [onSnapshot])

  const chosen = profiles.find((p) => p.id === selected)
  const running = ['running', 'starting', 'paused'].includes(snapshot?.launch_state ?? '')
  const active = ['running', 'paused'].includes(snapshot?.launch_state ?? '') && !!snapshot?.profile
  const caps = active ? snapshot?.capabilities : chosen?.capabilities
  const mode = snapshot?.robot?.control.mode ?? '未连接'
  const ready = caps?.components.every((c) => {
    const joint = snapshot?.robot?.joints[c.name]
    return joint && !joint.stale && joint.values.length === c.dim
  }) ?? false
  const operationRunning = snapshot?.operations.some((o) => o.status === 'running') ?? false
  const recordingState = snapshot?.robot?.data_collect?.state
  const recordingActive = ['RECORDING', 'PAUSED', 'SAVING'].includes(recordingState ?? '')
  const sim = !!chosen?.capabilities.viewer
  const controlled = active && ready && !busy && !operationRunning
  const driverIdle = ['IDLE', 'PAUSED'].includes(mode)
  const capture = snapshot?.robot?.data_collect
  const captureElapsed = typeof capture?.elapsed_s === 'number' ? `${capture.elapsed_s.toFixed(1)} s` : '—'
  const captureEpisode = typeof capture?.episode_index === 'number' && capture.episode_index >= 0 ? String(capture.episode_index + 1) : '—'

  function selectProfile(id: string) {
    const item = profiles.find((p) => p.id === id)
    setSelected(id)
    setValidated('')
    setDryRun(!item?.capabilities.viewer)
    setViewer(!!item?.capabilities.viewer)
    setCameras(false)
    setRecording(false)
    setPolicy(false)
  }

  async function act(fn: () => Promise<{ ok: boolean; message: string }>) {
    setBusy(true)
    try {
      const res = await fn()
      setError(res.ok ? '' : res.message)
      pushToast(res.message, res.ok ? 'success' : 'error')
    } catch (e) { setError(String(e)); pushToast(String(e), 'error') }
    finally { setBusy(false) }
  }

  async function emergencyStop() {
    if (!confirm('确认请求驱动急停？各组件行为由驱动决定，电机失能可能失去保持力。')) return
    try {
      const result = await api.nexusDriver('estop')
      pushToast(result.message, result.ok ? 'success' : 'error')
    } catch (e) { setError(String(e)) }
  }

  const controlPanel = <section className="panel control-panel">
    <div className="panel-heading"><span className="eyebrow">CONTROL</span><h2>驱动与遥操</h2><p>按顺序完成准备，再开始跟随输入。</p></div>
    <div className="control-steps">
      <div className="control-step"><span className={`step-number ${ready ? 'is-done' : ''}`}>01</span><div><strong>检查反馈</strong><small>{ready ? '所有组件反馈在线' : '确认机械臂与末端连接'}</small></div><button disabled={!active || busy || operationRunning} onClick={() => void act(() => api.nexusDriver('ready'))}>检查</button></div>
      <div className="control-step"><span className="step-number">02</span><div><strong>驱动使能</strong><small>开启关节控制</small></div><button disabled={!controlled || !driverIdle} onClick={() => void act(() => api.nexusDriver('enable'))}>使能</button></div>
      {caps?.home && <div className="control-step"><span className="step-number">03</span><div><strong>机器人归位</strong><small>移动到预设关节姿态</small></div><button disabled={!controlled || !driverIdle || recordingActive} onClick={() => { if (confirm('确认移动到配置中的归位姿态？请确认运动空间安全。')) void act(() => api.nexusDriver('home')) }}>归位</button></div>}
    </div>
    <button className="btn-primary wide" disabled={!controlled || mode === 'HOMING' || mode === 'ESTOP'} onClick={() => void act(api.teleopStart)}><Icon name="teleop" size={18} />开始遥操 / 重锚<Icon name="arrow" size={18} /></button>
    <div className="control-secondary"><button disabled={!active || operationRunning || snapshot?.launch_state !== 'running'} onClick={() => void act(api.pause)}>暂停遥操</button><button disabled={!controlled || snapshot?.launch_state !== 'paused'} onClick={() => void act(api.resume)}>重锚并恢复</button></div>
    {(snapshot?.operations ?? []).slice(-2).map((o) => <div className={`operation-result ${o.status}`} key={o.id} role="status"><span className="status-dot" /><div><strong>{({ready:'检查',enable:'使能',home:'归位',estop:'急停'} as Record<string,string>)[o.verb] ?? o.verb} · {o.status === 'running' ? '执行中' : o.status === 'succeeded' ? '完成' : '失败'}</strong><small>{o.message}</small></div></div>)}
    <div className="control-note"><Icon name="shield" size={16} /><small>归位仅作用于支持该能力的驱动。急停行为以驱动反馈为准。</small></div>
    <button className="btn-danger wide" disabled={!active} onClick={() => void emergencyStop()}>请求驱动急停</button>
  </section>

  const robotName = profiles.find(p => p.id === snapshot?.profile?.profile_id)?.instance ?? chosen?.instance ?? 'NEXUS'
  const status = <RobotOverview caps={caps} snapshot={snapshot} active={active} name={robotName.toUpperCase()} error={error} />

  return <div className="robot-console" style={page}>
    {status}
    {section === 'system' && <div className="system-grid">
      <section className="panel configuration-panel">
        <div className="panel-heading"><span className="eyebrow">CONFIGURATION</span><h2>机器人配置</h2><p>选择平台与末端，配置本次运行。</p></div>
        {running && <div className="active-config-summary"><span className="workflow-icon"><Icon name="robot" size={25} /></span><div><strong>{robotName.toUpperCase()}</strong><small>{selected}</small></div><span className="state-tag is-online">配置已冻结</span><div className="runtime-options">{[[inputs,'遥操输入'],[cameras,'相机采集'],[recording,'数据采集'],[policy,'模型推理']].map(([enabled,label]) => <span key={String(label)} className={enabled ? 'enabled' : ''}><i />{label}</span>)}</div><div className="runtime-session"><span>任务目录</span><strong>{session}</strong></div></div>}
        <details className={`configuration-disclosure ${running ? 'is-running' : ''}`} open={!running}><summary>查看完整运行配置<Icon name="arrow" size={15} /></summary>
        <div className="field-heading"><span>01</span><h3>选择机器人</h3><small>{profiles.length} 个可用配置</small></div>
        <div className="profile-grid">{profiles.map((p) => <button key={p.id} type="button" className={`profile-card ${p.id === selected ? 'is-selected' : ''}`} aria-label={`选择 ${p.id}`} aria-pressed={p.id === selected} disabled={running} onClick={() => selectProfile(p.id)}>
          <div className="profile-card-top"><span className="profile-symbol"><Icon name="robot" size={24} /></span><span className={`profile-type ${p.capabilities.viewer ? 'simulation' : ''}`}>{p.capabilities.viewer ? '仿真' : '机器人'}</span><span className="selection-dot" /></div>
          <strong>{p.instance.toUpperCase()}</strong><span className="profile-description">{p.capabilities.components.filter(c => c.kind === 'arm').length} 个机械臂 · {p.state_dim}D</span>
          <small>{p.capabilities.components.filter(c => c.kind !== 'arm').map(c => componentLabel(c)).join(' + ') || '无末端执行器'}</small><span className="profile-id">{p.id}</span>
        </button>)}</div>
        <div className="setup-divider" />
        <div className="field-heading"><span>02</span><h3>运行模式</h3></div>
        <div className="mode-selector" role="group" aria-label="运行模式">
          {sim ? <button className="is-selected" disabled><Icon name="system" size={18} /><strong>机器人仿真</strong><small>仿真驱动与可视化</small></button> : <>
            <button className={dryRun ? 'is-selected' : ''} disabled={running} aria-pressed={dryRun} onClick={() => setDryRun(true)}><Icon name="system" size={18} /><strong>接口调试</strong><small>使用假驱动验证流程</small></button>
            <button className={!dryRun ? 'is-selected' : ''} disabled={running} aria-pressed={!dryRun} onClick={() => setDryRun(false)}><Icon name="power" size={18} /><strong>真机运行</strong><small>连接物理机器人驱动</small></button>
          </>}
        </div>
        <div className="field-heading"><span>03</span><h3>运行功能</h3><small>按需开启</small></div>
        <div className="feature-grid">
          <FeatureSwitch title="遥操输入" description="使用配置中的输入源" icon="teleop" checked={inputs} disabled={running} onChange={setInputs} />
          <FeatureSwitch title="相机采集" description={`${chosen?.capabilities.cameras.length ?? 0} 个语义相机通道`} icon="data" checked={cameras} disabled={running || !chosen?.capabilities.cameras.length} onChange={setCameras} />
          <FeatureSwitch title="数据采集" description="录制机器人状态与动作" icon="data" checked={recording} disabled={running} onChange={setRecording} />
          <FeatureSwitch title="模型推理" description="启动策略与人在环控制" icon="inference" checked={policy} disabled={running} onChange={setPolicy} />
          {sim && <FeatureSwitch title="仿真可视化" description="在主机桌面打开仿真窗口" icon="system" checked={viewer} disabled={running} onChange={setViewer} />}
        </div>
        <div className="session-field"><label htmlFor="robot-session">数据任务目录</label><input id="robot-session" value={session} disabled={running} onChange={(e) => setSession(e.target.value)} /><small>本次录制与训练使用的任务名称</small></div>
        </details>
        <div className="setup-actions"><button disabled={busy || running || !selected} onClick={() => void act(async () => { const res = await api.robotValidate(selected); if (res.ok) setValidated(selected); return res })}><Icon name="shield" size={17} />校验配置</button>
          {running ? <button className="btn-danger" disabled={busy || operationRunning || recordingActive} onClick={() => void act(api.stop)}>停止机器人系统</button> : <button className="btn-primary" disabled={busy || !selected} onClick={() => void act(() => api.nexusStart({ profile: selected, session, dry_run: dryRun, with_inputs: inputs, with_cameras: cameras, with_recording: recording, with_policy: policy, viewer, model_manifest: manifest, model, server_host: serverHost, server_port: serverPort }))}>{busy ? '处理中…' : '启动机器人系统'}<Icon name="arrow" size={18} /></button>}
        </div><small className="validation-hint">{running ? '配置在运行期间锁定，停止后可切换机器人' : validated === selected ? '配置校验通过' : '启动前自动校验机器人配置与数据布局'}</small>
        {recordingActive && <p>请先保存当前数据段，再停止机器人系统。</p>}
      </section>
      {controlPanel}
    </div>}
    {section === 'teleop' && controlPanel}
    {section === 'data' && <section className="workflow-panel" style={card}>
      <div className="workflow-heading"><span className="workflow-icon"><Icon name="data" size={24} /></span><div><span className="eyebrow">CAPTURE YOUR DEMONSTRATIONS</span><h2 style={title}>数据采集</h2></div><span className="workflow-badge">RAW / EPISODE</span></div>
      <div className="capture-metrics"><div><span>录制状态</span><strong>{recordingState ?? '未启动'}</strong></div><div><span>当前数据段</span><strong>{captureEpisode}</strong></div><div><span>本段时长</span><strong>{captureElapsed}</strong></div></div>
      <p>状态：{snapshot?.robot?.data_collect?.state ?? '数采节点未启动'}{snapshot?.robot?.data_collect?.stale ? ' · 状态过期' : ''}</p>
      {active && recording && <div style={row}><input value={task} onChange={(e) => setTask(e.target.value)} placeholder="任务描述" />
        <button disabled={!active || !recording} onClick={() => void act(() => api.collectTask(task))}>设置任务</button>
        <button disabled={!active || !recording || busy || recordingActive} onClick={() => void act(() => api.collectControl('start'))}>开始录制</button>
        <button disabled={!active || !recording || busy || !recordingActive} onClick={() => void act(() => api.collectControl('stop'))}>停止并保存</button>
        <button disabled={!active || !recording || busy || recordingState !== 'RECORDING'} onClick={() => void act(() => api.collectControl('pause'))}>暂停录制</button>
        <button disabled={!active || !recording || busy || recordingState !== 'PAUSED'} onClick={() => void act(() => api.collectControl('resume'))}>继续录制</button>
      </div>}
      <p>数据根目录：{snapshot?.settings.data_root ?? '启动后显示'}　任务目录：{snapshot?.settings.session ?? session}</p>
      <details><summary>数采状态与质量信息</summary><pre style={logs}>{JSON.stringify(snapshot?.robot?.data_collect ?? {}, null, 2)}</pre></details>
      <div style={row}>{(snapshot?.settings.with_cameras ? caps?.cameras ?? [] : []).map((role) => <div key={role} style={{ width: 260 }}><p>{role}</p><img style={{ width: '100%' }} src={api.videoFeedUrl(role)} alt={role} /></div>)}</div>
      {!recording && <p>在系统页启用数采节点后启动机器人系统。</p>}
    </section>}
    {['system', 'inference'].includes(section) && <details className="panel advanced-panel" open={section === 'inference' ? true : undefined}><summary><Icon name="inference" size={18} />模型与推理参数<span>高级配置</span></summary><section>

      <div style={row}>
        <div className="model-selector" role="group" aria-label="模型类型">{(['act','pi05'] as const).map(item => <button key={item} className={model === item ? 'is-selected' : ''} disabled={running} aria-pressed={model === item} onClick={() => setModel(item)}>{item === 'act' ? 'ACT' : 'pi0.5'}</button>)}</div>
        <label>GPU 服务地址 <input value={serverHost} disabled={running} onChange={(e) => setServerHost(e.target.value)} /></label>
        <label>端口 <input type="number" min={1} max={65535} value={serverPort} disabled={running} onChange={(e) => setServerPort(Number(e.target.value))} /></label>
        <label>模型布局清单 <input value={manifest} disabled={running} onChange={(e) => setManifest(e.target.value)} placeholder="机器人主机上的清单路径" /></label>
      </div>
      {section === 'inference' && <>
        <div className="inference-identity"><span className="workflow-icon"><Icon name="inference" size={24} /></span><div><small>SELECTED MODEL</small><strong>{model === 'act' ? 'ACT' : 'π₀.₅'}</strong></div><div><small>控制权</small><strong>{snapshot?.robot?.control.mode ?? '未连接'}</strong></div></div>
        <p>推理状态：{snapshot?.robot?.infer?.state ?? '未启动'} · {snapshot?.robot?.infer?.fault || '无已报告故障'}</p>
        {active && policy ? <><div style={row}>{[['policy', '启动策略'], ['pause', '暂停策略'], ['takeover', '人工接管'], ['release', '返回策略'], ['stop', '停止策略']].map(([verb, label]) => <button key={verb} disabled={busy || operationRunning} onClick={() => void act(() => api.inferCmd(verb))}>{label}</button>)}</div>
        <div style={row}><input value={task} onChange={(e) => setTask(e.target.value)} placeholder="推理任务" /><button onClick={() => void act(() => api.inferTask(task))}>设置推理任务</button></div></> : <p>在系统页开启模型推理并启动机器人系统后，显示策略与人在环操作。</p>}
        <small>推理使能前由服务校验状态、动作、关节顺序及相机槽位；控制权以机器人反馈为准。</small>
      </>}
    </section></details>}
    {section === 'training' && <section className="workflow-panel" style={card}>
      <div className="workflow-heading"><span className="workflow-icon"><Icon name="training" size={24} /></span><div><span className="eyebrow">TEACH. TRAIN. DEPLOY.</span><h2 style={title}>数据处理与远端训练</h2></div><span className="workflow-badge">LEARNING PIPELINE</span></div>
      <div className="learning-pipeline">{[['01','data','同步与处理','对齐、质检与数据导出'],['02','training','策略训练','ACT / pi0.5 · 远端 GPU'],['03','inference','模型部署','清单校验与推理服务']].map(([number,icon,label,description]) => <div key={number}><span className="pipeline-number">{number}</span><Icon name={icon} size={25} /><strong>{label}</strong><small>{description}</small></div>)}</div>
      <p>任务目录：{session} · 使用当前机器人配置的冻结布局</p>
      <div style={row}>
        <button disabled={!snapshot?.profile || recordingActive || busy} onClick={() => void act(() => api.nexusJob('sync_process', session, steps))}>同步 → 对齐与质检 → 导出</button>
        <label>训练步数 <input type="number" min={1} value={steps} onChange={(e) => setSteps(Number(e.target.value))} /></label>
        <button disabled={!snapshot?.profile || recordingActive || busy} onClick={() => void act(() => api.nexusJob('train_act', session, steps))}>ACT 训练</button>
        <button disabled={!snapshot?.profile || recordingActive || busy} onClick={() => void act(() => api.nexusJob('train_pi05', session, steps))}>pi0.5 训练</button>
      </div>
      {(snapshot?.jobs ?? []).slice().reverse().map((job) => <div key={job.id} style={card}>
        <div style={row}><strong>{job.kind}</strong><span>{job.session} · {job.status} · {job.step}</span>
          {job.status === 'running' && <button onClick={() => void act(() => api.nexusCancel(job.id))}>取消</button>}
          {['failed', 'cancelled'].includes(job.status) && <button disabled={job.profile_sha256 !== snapshot?.profile?.profile_sha256} title="只有当前机器人布局与原作业一致时才能重新提交" onClick={() => void act(() => api.nexusJob(job.kind, job.session, steps))}>重新提交</button>}
        </div><pre style={logs}>{JSON.stringify(job.artifacts, null, 2)}</pre><pre style={logs}>{job.logs.slice(-20).join('\n')}</pre>
      </div>)}
      {!snapshot?.jobs?.length && <div className="workflow-empty"><Icon name="training" size={25} /><div><strong>还没有训练作业</strong><small>保存演示数据后，提交数据处理或模型训练。进度、日志和产物会显示在这里。</small></div></div>}
    </section>}
    {['teleop', 'diagnostics'].includes(section) && <section className="workflow-panel" style={card}>
      <h2 style={title}>实时控制诊断</h2>
      <table style={{ width: '100%', textAlign: 'left' }}><thead><tr><th>数据流</th><th>频率</th><th>接收年龄</th></tr></thead><tbody>
        {Object.entries(snapshot?.robot?.diagnostics?.streams ?? {}).map(([name, metric]) => <tr key={name}><td>{name}</td><td>{metric.hz.toFixed(1)} Hz</td><td style={metric.age_s === null || metric.age_s > .5 ? alert : undefined}>{metric.age_s === null ? '尚未收到' : `${(metric.age_s * 1000).toFixed(1)} ms`}</td></tr>)}
      </tbody></table>
      {Object.entries(snapshot?.robot?.diagnostics?.command_publishers ?? {}).map(([name, publishers]) => <p key={name} style={publishers?.length !== 1 ? alert : undefined}>{name} 最终命令发布者：{publishers ? publishers.length : '未知'} · {publishers?.map((p) => `${p.node} / ${p.reliability} / ${p.durability}`).join('；')}</p>)}
      {Object.entries(snapshot?.robot?.diagnostics?.joint_error ?? {}).map(([name, value]) => <p key={name}>{name} 最大关节跟随误差：{value.toFixed(4)}（配置中的关节单位）</p>)}
      <small>接收年龄表示距最近一帧的时间，不是端到端遥操延迟。IK 耗时与不可达原因查看运行日志。</small>
    </section>}
    {section === 'diagnostics' && <>
      <section className="workflow-panel" style={card}><h2 style={title}>运行记录</h2><p>运行 ID：{snapshot?.run_id ?? '尚未启动'}</p>
        <a href={api.robotReportUrl()} download>导出本次测试报告（配置、版本、操作、诊断、作业及日志）</a>
      </section>
      <LogConsole state={state} />
    </>}
  </div>
}

const page: React.CSSProperties = { display: 'grid', gap: 16 }
const card: React.CSSProperties = { background: 'var(--panel)', border: '1px solid var(--border)', borderRadius: 14, padding: 24 }
const title: React.CSSProperties = { margin: '0 0 12px', fontSize: 17 }
const row: React.CSSProperties = { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: 10, marginBottom: 12 }
const alert: React.CSSProperties = { color: '#fca5a5' }
const logs: React.CSSProperties = { maxHeight: 260, overflow: 'auto', background: '#080d16', padding: 10, whiteSpace: 'pre-wrap' }
