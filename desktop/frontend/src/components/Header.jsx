export default function Header({ theme, toggleTheme, missionTime, packetCount, connected }) {
  return (
    <header className="header">
      <div className="header-left">
        <img
          className="header-logo"
          src={theme === 'dark' ? '/logo-dark.png' : '/logo-light.png'}
          alt="NAZARX"
          draggable={false}
        />
        <img className="header-flag" src="/flag-uzbekistan.png" alt="Flag" draggable={false} />
        <div className="header-title">
          <h1>NazarX Ground Station</h1>
          <span className={`header-status ${connected ? 'connected' : 'disconnected'}`}>
            {connected ? 'Connected' : 'Disconnected'}
          </span>
        </div>
      </div>
      <div className="header-right">
        <div className="header-stat">
          <span className="header-stat-label">Mission Time</span>
          <span className="header-stat-value">{missionTime}</span>
        </div>
        <div className="header-stat">
          <span className="header-stat-label">Packets</span>
          <span className="header-stat-value">{packetCount}</span>
        </div>
        <button className="theme-toggle" onClick={toggleTheme} title="Toggle theme">
          {theme === 'dark' ? '\u263E' : '\u2600'}
        </button>
      </div>
    </header>
  );
}
