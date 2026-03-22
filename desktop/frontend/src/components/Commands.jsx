import { useState, useCallback, useRef } from 'react';

const TEAM_ID = import.meta.env.VITE_TEAM_ID || '1003';

export default function Commands({ connected }) {
  const [timeVal, setTimeVal] = useState(() => {
    const now = new Date();
    return now.toTimeString().slice(0, 8);
  });
  const [dateVal, setDateVal] = useState(() => {
    const now = new Date();
    return now.toISOString().slice(0, 10);
  });
  const [pressVal, setPressVal] = useState('');
  const [telemetryOn, setTelemetryOn] = useState(false);
  const [simEnabled, setSimEnabled] = useState(false);
  const csvFileRef = useRef(null);

  const send = useCallback(async (cmd) => {
    if (!connected || !window.go) return;
    try {
      await window.go.backend.App.SendCommand(cmd);
    } catch (err) {
      console.error('Command error:', err);
    }
  }, [connected]);

  const handleCsvUpload = (e) => {
    const file = e.target.files[0];
    if (file) {
      console.log('CSV file selected:', file.name);
    }
  };

  return (
    <div className="commands-card">
      <h4>Commands</h4>
      <div className="cmd-grid">
        <button
          className={`btn ${telemetryOn ? 'btn-danger' : 'btn-success'}`}
          onClick={() => {
            const next = !telemetryOn;
            send(`CMD,${TEAM_ID},${next ? 'ON' : 'OFF'}`);
            setTelemetryOn(next);
          }}
          disabled={!connected}
        >
          Telemetry {telemetryOn ? 'OFF' : 'ON'}
        </button>

        {!simEnabled ? (
          <button className="btn btn-primary" onClick={() => { send(`CMD,${TEAM_ID},SIM,1`); setSimEnabled(true); }} disabled={!connected}>
            Sim Enable
          </button>
        ) : (
          <div style={{ display: 'flex', gap: '4px' }}>
            <button className="btn btn-danger" style={{ flex: 1 }} onClick={() => { send(`CMD,${TEAM_ID},SIM,0`); setSimEnabled(false); }} disabled={!connected}>
              Disable
            </button>
            <button className="btn btn-success" style={{ flex: 1 }} onClick={() => send(`CMD,${TEAM_ID},SIM,2`)} disabled={!connected}>
              Activate
            </button>
          </div>
        )}

        <button className="btn btn-teal" onClick={async () => {
          if (!connected || !window.go) return;
          try { await window.go.backend.App.ResetMPU(); } catch (err) { console.error('ResetMPU error:', err); }
        }} disabled={!connected}>
          Calibrate
        </button>

        <div className="cmd-input-group">
          <input className="cmd-input" type="time" step="1" value={timeVal} onChange={e => setTimeVal(e.target.value)} />
          <button className="btn btn-primary" onClick={() => { send(`CMD,${TEAM_ID},ST,${timeVal}`); setTimeVal(''); }} disabled={!connected}>Send</button>
        </div>

        <div className="cmd-input-group">
          <input className="cmd-input" type="date" value={dateVal} onChange={e => setDateVal(e.target.value)} />
          <button className="btn btn-primary" onClick={() => {
            // Convert YYYY-MM-DD to DD.MM.YYYY
            const parts = dateVal.split('-');
            const formatted = parts.length === 3 ? `${parts[2]}.${parts[1]}.${parts[0]}` : dateVal;
            send(`CMD,${TEAM_ID},SD,${formatted}`);
            setDateVal('');
          }} disabled={!connected}>Send</button>
        </div>

        <div className="cmd-input-group">
          <input className="cmd-input" type="text" placeholder="kPa" value={pressVal} onChange={e => setPressVal(e.target.value)} />
          <button className="btn btn-warning" onClick={() => { send(`CMD,${TEAM_ID},SIM,${pressVal}`); setPressVal(''); }} disabled={!connected}>Send</button>
          <input ref={csvFileRef} type="file" accept=".csv" onChange={handleCsvUpload} style={{ display: 'none' }} />
          <button className="btn btn-purple" onClick={() => csvFileRef.current?.click()} disabled={!connected}>CSV</button>
        </div>
      </div>
    </div>
  );
}
