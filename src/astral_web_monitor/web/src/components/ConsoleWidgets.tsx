import type { Component } from '../lib/robotTypes'

export function Icon({ name, size = 20 }: { name: string; size?: number }) {
  const paths: Record<string, string> = {
    system: 'M3 3h7v7H3z M14 3h7v7h-7z M3 14h7v7H3z M14 14h7v7h-7z',
    teleop: 'M8 12V5a2 2 0 0 1 4 0v7 M12 9a2 2 0 0 1 4 0v4 M16 11a2 2 0 0 1 4 0v5c0 4-3 6-6 6h-2c-2 0-4-1-5-3l-4-6a2 2 0 0 1 3-2l2 2',
    data: 'M4 5h4l2-2h4l2 2h4v15H4z M16 12a4 4 0 1 1-8 0 4 4 0 0 1 8 0',
    training: 'M3 18V6 M3 18h18 M6 14l4-4 4 2 6-7',
    inference: 'M8 4l12 8-12 8z',
    diagnostics: 'M3 12h4l3-8 4 16 3-8h4',
    robot: 'M4 21h16 M8 21v-5l5-5 M7 5a3 3 0 1 0 6 0 3 3 0 0 0-6 0 M12 7l6 4 M18 11l3-2 M18 11l2 3 M10 8l-5 8 M5 16h6',
    arrow: 'M4 12h16 M14 6l6 6-6 6',
    power: 'M12 2v10 M6 5a9 9 0 1 0 12 0',
    shield: 'M12 3l8 3v6c0 5-8 9-8 9s-8-4-8-9V6z M8 12l3 3 5-6',
  }
  return <svg width={size} height={size} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true"><path d={paths[name] ?? paths.system} /></svg>
}

export function FeatureSwitch({ title, description, checked, disabled, onChange, icon }: {
  title: string; description: string; checked: boolean; disabled?: boolean; onChange: (value: boolean) => void; icon: string
}) {
  return <button type="button" role="switch" aria-label={title} aria-checked={checked} disabled={disabled}
    className={`feature-switch ${checked ? 'is-on' : ''}`} onClick={() => onChange(!checked)}>
    <span className="feature-icon"><Icon name={icon} /></span>
    <span className="feature-copy"><strong>{title}</strong><small>{description}</small></span>
    <span className="switch-track"><span /></span>
  </button>
}

export function componentLabel(component: Component) {
  const kind = ({ arm: '机械臂', hand: '灵巧手', gripper: '夹爪' } as Record<string, string>)[component.kind] ?? component.kind
  const side = component.name.startsWith('left_') ? '左' : component.name.startsWith('right_') ? '右' : ''
  return side ? side + kind : component.name
}

export function stateLabel(state?: string) {
  return ({ stopped: '待机', starting: '启动中', running: '运行中', paused: '已暂停', stopping: '停止中', start_failed: '启动失败', IDLE: '待机保持', TELEOP: '遥操', POLICY: '策略控制', PLAYBACK: '回放', PAUSED: '暂停保持', ESTOP: '急停锁定', HOMING: '归位中' } as Record<string, string>)[state ?? ''] ?? state ?? '未连接'
}
