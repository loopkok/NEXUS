import { Icon } from './ConsoleWidgets'
interface Tab {
  id: string
  label: string
}

interface Props {
  tabs: Tab[]
  active: string
  onChange: (id: string) => void
}

export function Tabs({ tabs, active, onChange }: Props) {
  return (
    <nav className="console-nav" role="tablist" aria-label="控制台导航">
      {tabs.map((t) => {
        const on = t.id === active
        return (
          <button
            key={t.id}
            role="tab"
            aria-selected={on}
            className={`nav-item ${on ? 'is-active' : ''}`}
            onClick={() => onChange(t.id)}
          >
            <Icon name={t.id} /><span>{t.label}</span>
          </button>
        )
      })}
    </nav>
  )
}
