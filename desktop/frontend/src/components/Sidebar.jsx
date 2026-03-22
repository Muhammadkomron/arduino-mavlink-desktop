import { useState, useEffect, useCallback } from 'react';

const TEAM_ID = import.meta.env.VITE_TEAM_ID || '1003';

export default function Sidebar({ telemetry, connected }) {
  const [ports, setPorts] = useState([]);
  const [selectedPort, setSelectedPort] = useState('');
  const [baud, setBaud] = useState(57600);

  const refreshPorts = useCallback(async () => {
    if (window.go) {
      try {
        const p = await window.go.backend.App.ListPorts();
        setPorts(p || []);
        if (p && p.length > 0 && !selectedPort) {
          setSelectedPort(p[0]);
        }
      } catch { /* ignore */ }
    }
  }, [selectedPort]);

  useEffect(() => {
    refreshPorts();
    const interval = setInterval(refreshPorts, 5000);
    return () => clearInterval(interval);
  }, [refreshPorts]);

  const handleConnect = async () => {
    if (!window.go) return;
    try {
      if (connected) {
        await window.go.backend.App.Disconnect();
      } else {
        await window.go.backend.App.Connect(selectedPort, baud);
      }
    } catch (err) {
      console.error('Connection error:', err);
    }
  };

  return (
    <aside className="sidebar">
      <section className="panel">
        <h3>Connection</h3>
        <div className="port-row">
          <select
            className="port-select"
            value={selectedPort}
            onChange={e => setSelectedPort(e.target.value)}
          >
            {ports.length === 0 && <option value="">No ports found</option>}
            {ports.map(p => <option key={p} value={p}>{p}</option>)}
          </select>
          <button className="refresh-ports-btn" onClick={refreshPorts} title="Refresh ports">
            &#x21BB;
          </button>
        </div>
        <div className="port-info">
          Port: {selectedPort || '—'} | Baud: {baud}
        </div>
        <button
          className={`btn btn-connect ${connected ? 'connected' : ''}`}
          onClick={handleConnect}
        >
          {connected ? 'Disconnect' : 'Connect'}
        </button>
      </section>

      <section className="panel">
        <h3>Team Info</h3>
        <div className="info-row"><span className="label">Team ID</span><span className="value">{TEAM_ID}</span></div>
        <div className="info-row"><span className="label">Packets</span><span className="value">{telemetry.msgCount}</span></div>
        <div className="info-row"><span className="label">Mode</span><span className="value">{telemetry.mode}</span></div>
        <div className="info-row"><span className="label">State</span><span className="value">{telemetry.state}</span></div>
      </section>

      <section className="panel">
        <h3>Telemetry</h3>
        <div className="info-row"><span className="label">Voltage</span><span className="value">{telemetry.voltage.toFixed(2)} V</span></div>
        <div className="info-row"><span className="label">Altitude</span><span className="value">{telemetry.altitude.toFixed(1)} m</span></div>
        <div className="info-row"><span className="label">Pressure</span><span className="value">{telemetry.pressure.toFixed(0)} hPa</span></div>
        <div className="info-row"><span className="label">Temp</span><span className="value">{telemetry.temperature.toFixed(1)} °C</span></div>
      </section>

      <section className="panel">
        <h3>Gyro</h3>
        <div className="info-row"><span className="label">Roll</span><span className="value">{telemetry.roll.toFixed(2)}</span></div>
        <div className="info-row"><span className="label">Pitch</span><span className="value">{telemetry.pitch.toFixed(2)}</span></div>
        <div className="info-row"><span className="label">Yaw</span><span className="value">{telemetry.yaw.toFixed(2)}</span></div>
        <div className="info-row"><span className="label">Rate</span><span className="value">{telemetry.gyroRate.toFixed(1)} °/s</span></div>
      </section>

      <section className="panel">
        <h3>Accel</h3>
        <div className="info-row"><span className="label">X</span><span className="value">{telemetry.accelX.toFixed(1)} m/s²</span></div>
        <div className="info-row"><span className="label">Y</span><span className="value">{telemetry.accelY.toFixed(1)} m/s²</span></div>
        <div className="info-row"><span className="label">Z</span><span className="value">{telemetry.accelZ.toFixed(1)} m/s²</span></div>
      </section>

      <section className="panel">
        <h3>Magnetometer</h3>
        <div className="info-row"><span className="label">X</span><span className="value">{telemetry.magX.toFixed(1)} &micro;T</span></div>
        <div className="info-row"><span className="label">Y</span><span className="value">{telemetry.magY.toFixed(1)} &micro;T</span></div>
        <div className="info-row"><span className="label">Z</span><span className="value">{telemetry.magZ.toFixed(1)} &micro;T</span></div>
      </section>

      <section className="panel">
        <h3>GPS</h3>
        <div className="info-row"><span className="label">Lat</span><span className="value">{telemetry.gpsLat.toFixed(4)}</span></div>
        <div className="info-row"><span className="label">Lon</span><span className="value">{telemetry.gpsLon.toFixed(4)}</span></div>
        <div className="info-row"><span className="label">Alt</span><span className="value">{telemetry.gpsAlt.toFixed(1)}</span></div>
        <div className="info-row"><span className="label">Sats</span><span className="value">{telemetry.gpsSats}</span></div>
      </section>
    </aside>
  );
}
