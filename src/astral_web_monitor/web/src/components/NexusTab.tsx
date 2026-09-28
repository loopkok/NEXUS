import { useEffect, useState } from 'react'
import type { NormalisedState } from '../lib/mapUiState'
import { api } from '../api/client'
import { pushToast } from '../hooks/useToast'

type ProfileItem = { id: string; adapters: string[]; state_dim: number; cameras: string[]; sha256: string }
type Job = { id: string; kind: string; status: string; step: string; logs: string[]; artifacts: Record<string, string> }
type NexusState = { launch_state: string; profile: { profile_id: string; profile_sha256: string } | null; robot: { control: { mode?: string; fault?: string }; data_collect?: { state?: string }; infer?: { state?: string; fault?: string }; joints: Record<string, { values: number[]; stale: boolean }> } | null; jobs: Job[] }

export function NexusTab({ state }: { state: NormalisedState | null }) {
  const [profiles, setProfiles] = useState<ProfileItem[]>([])
  const [selected, setSelected] = useState('astral_gripper_wuji')
  const [session, setSession] = useState('default_task')
  const [dryRun, setDryRun] = useState(true)
  const [cameras, setCameras] = useState(false)
  const [manifest, setManifest] = useState('')
  const [model, setModel] = useState<'act' | 'pi05'>('act')
  const [serverHost, setServerHost] = useState('127.0.0.1')
  const [serverPort, setServerPort] = useState(8001)
  const [task, setTask] = useState('')
  const [steps, setSteps] = useState(1000)
  const [snapshot, setSnapshot] = useState<NexusState | null>(null)
  const [busy, setBusy] = useState(false)

  useEffect(() => {
    void api.nexusProfiles().then((res) => {
      if (res.ok) setProfiles(res.data)
    })
    const refresh = () => void api.nexusState().then((res) => {
      if (res.ok) setSnapshot(res.data)
    })
    refresh()
    const timer = window.setInterval(refresh, 2000)
    return () => window.clearInterval(timer)
  }, [])

  const chosen = profiles.find((p) => p.id === selected)
  const running = snapshot?.launch_state === 'running' || snapshot?.launch_state === 'starting'

  async function act(fn: () => Promise<{ ok: boolean; message: string }>, success: string) {
    setBusy(true)
    try {
      const res = await fn()
      pushToast(res.ok ? success : res.message, res.ok ? 'success' : 'error')
    } catch (e) {
      pushToast(String(e), 'error')
    } finally {
      setBusy(false)
    }
  }

  async function job(kind: string) {
    try {
      const res = await api.nexusJob(kind, session, steps)
      pushToast(res.ok ? `作业已提交：${res.data.job_id}` : res.message, res.ok ? 'success' : 'error')
    } catch (e) {
      pushToast(String(e), 'error')
    }
  }

  return <div style={page}>
    <section style={card}>
      <h2 style={title}>机器人装配</h2>
      <div style={row}>
        <label>Profile <select value={selected} disabled={running} onChange={(e) => setSelected(e.target.value)}>
          {profiles.map((p) => <option key={p.id} value={p.id}>{p.id} · {p.adapters.join('+')} · {p.state_dim}D</option>)}
        </select></label>
        <label>Session <input value={session} disabled={running} onChange={(e) => setSession(e.target.value)} /></label>
        <label><input type="checkbox" checked={dryRun} disabled={running} onChange={(e) => setDryRun(e.target.checked)} /> 仿真驱动</label>
        <label><input type="checkbox" checked={cameras} disabled={running} onChange={(e) => setCameras(e.target.checked)} /> 启动相机</label>
      </div>
      <div style={row}>
        <label>模型 <select value={model} disabled={running} onChange={(e) => setModel(e.target.value as 'act' | 'pi05')}><option value="act">ACT</option><option value="pi05">pi0.5</option></select></label>
        <label>GPU 服务地址 <input value={serverHost} disabled={running} onChange={(e) => setServerHost(e.target.value)} /></label>
        <label>端口 <input type="number" min={1} max={65535} value={serverPort} disabled={running} onChange={(e) => setServerPort(Number(e.target.value))} style={{ width: 90 }} /></label>
        <label style={{ flex: 1 }}>模型布局清单 <input style={{ width: '100%' }} value={manifest} disabled={running} onChange={(e) => setManifest(e.target.value)} placeholder="留空则由 profile 生成，推理时仍校验 GPU 服务的模型哈希" /></label>
      </div>
      <div style={row}>
        <button disabled={busy || running} onClick={() => void act(() => api.nexusStart({ profile: selected, session, dry_run: dryRun, with_cameras: cameras, model_manifest: manifest, model, server_host: serverHost, server_port: serverPort }), 'NEXUS 已启动')}>校验并启动</button>
        <button disabled={busy || !running} onClick={() => void act(api.stop, '已请求停止')}>停止装配</button>
        <strong>状态：{snapshot?.launch_state ?? '未知'}</strong>
      </div>
      <small>布局 SHA256：{snapshot?.profile?.profile_sha256 ?? chosen?.sha256 ?? '—'}</small>
    </section>

    <section style={card}>
      <h2 style={title}>遥操与控制权</h2>
      <fieldset disabled={!running} style={{ border: 0, padding: 0, margin: 0 }}>
      <div style={row}>
        <button onClick={() => void act(() => api.nexusDriver('ready'), '驱动反馈就绪')}>检查驱动</button>
        <button onClick={() => void act(() => api.nexusDriver('enable'), '驱动使能完成')}>使能</button>
        <button onClick={() => void act(() => api.nexusDriver('home'), '归位命令完成')}>归位</button>
        <button onClick={() => void act(() => api.nexusDriver('estop'), '急停已下发')}>急停</button>
        <button onClick={() => void act(api.teleopStart, '遥操重锚已请求')}>开始遥操 / 重锚</button>
        <button onClick={() => void act(() => api.inferCmd('policy'), '策略启动已请求')}>启动推理</button>
        <button onClick={() => void act(() => api.inferCmd('pause'), '已暂停')}>暂停</button>
        <button onClick={() => void act(() => api.inferCmd('takeover'), '人工接管已请求')}>人工接管</button>
        <button onClick={() => void act(() => api.inferCmd('release'), '控制权返回已请求')}>返回策略</button>
        <button onClick={() => void act(() => api.inferCmd('stop'), '推理已停止')}>停止推理</button>
      </div>
      <div>控制权：{snapshot?.robot?.control?.mode ?? '未连接'}　控制故障：{snapshot?.robot?.control?.fault || '—'}</div>
      <div>推理状态：{state?.infer?.state ?? '未连接'}　故障：{String(state?.infer?.fault ?? state?.infer?.error ?? '—')}</div>
      <div style={row}>
        <input value={task} onChange={(e) => setTask(e.target.value)} placeholder="任务文本" />
        <button onClick={() => void act(() => api.collectTask(task), '下一段任务已设置')}>设置数采任务</button>
        <button onClick={() => void act(() => api.inferTask(task), '推理任务已设置')}>设置推理任务</button>
      </div>
      <div style={row}>
        <button onClick={() => void act(() => api.collectControl('start'), '开始录制')}>录制</button>
        <button onClick={() => void act(() => api.collectControl('stop'), '录制已保存')}>保存</button>
        <button onClick={() => void act(() => api.collectControl('pause'), '录制暂停')}>暂停录制</button>
        <span>数采：{snapshot?.robot?.data_collect?.state ?? state?.dataCollect?.state ?? '未连接'}</span>
      </div>
      </fieldset>
    </section>

    <section style={card}>
      <h2 style={title}>状态与相机</h2>
      <div style={row}>
        {Object.entries(snapshot?.robot?.joints ?? {}).map(([name, item]) =>
          <span key={name} style={chip}>{name}: {item.values.length}D {item.stale ? '· 过期' : '· 在线'}</span>)}
      </div>
      <div style={row}>
        {(chosen?.cameras ?? []).map((role) => <div key={role} style={{ width: 240 }}>
          <div>{role}</div><img style={{ width: 240, background: '#05080f' }} src={api.videoFeedUrl(role)} alt={role} />
        </div>)}
      </div>
    </section>

    <section style={card}>
      <h2 style={title}>远端 GPU 作业</h2>
      <div style={row}>
        <button onClick={() => void job('sync_process')}>同步 → 对齐质检 → ACT/pi0.5 导出</button>
        <label>训练步数 <input type="number" min={1} value={steps} onChange={(e) => setSteps(Number(e.target.value))} style={{ width: 100 }} /></label>
        <button onClick={() => void job('train_act')}>ACT 训练</button>
        <button onClick={() => void job('train_pi05')}>pi0.5 训练</button>
      </div>
      {(snapshot?.jobs ?? []).slice().reverse().map((item) => <div key={item.id} style={jobCard}>
        <div style={row}><strong>{item.kind}</strong><span>{item.status} · {item.step}</span>
          {item.status === 'running' && <button onClick={() => void act(() => api.nexusCancel(item.id), '取消已请求')}>取消</button>}
        </div>
        <div>{Object.entries(item.artifacts).map(([k, v]) => `${k}: ${v}`).join('　')}</div>
        <pre style={logs}>{item.logs.slice(-12).join('\n')}</pre>
      </div>)}
    </section>
  </div>
}

const page: React.CSSProperties = { padding: 20, display: 'grid', gap: 16 }
const card: React.CSSProperties = { background: '#111827', border: '1px solid #263244', borderRadius: 10, padding: 18 }
const title: React.CSSProperties = { margin: '0 0 12px', fontSize: 17 }
const row: React.CSSProperties = { display: 'flex', flexWrap: 'wrap', alignItems: 'center', gap: 10, marginBottom: 12 }
const chip: React.CSSProperties = { background: '#203047', padding: '6px 10px', borderRadius: 6 }
const jobCard: React.CSSProperties = { borderTop: '1px solid #334155', paddingTop: 12, marginTop: 12 }
const logs: React.CSSProperties = { maxHeight: 180, overflow: 'auto', background: '#080d16', padding: 10, whiteSpace: 'pre-wrap' }
