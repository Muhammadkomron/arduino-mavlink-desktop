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
  const [simActive, setSimActive] = useState(false);
  const [csvPlaying, setCsvPlaying] = useState(false);
  const csvFileRef = useRef(null);
  const csvTimersRef = useRef([]);

  const send = useCallback(async (cmd) => {
    if (!connected || !window.go) return;
    try {
      await window.go.backend.App.SendCommand(cmd);
    } catch (err) {
      console.error('Command error:', err);
    }
  }, [connected]);

  const stopCsvPlayback = useCallback(() => {
    csvTimersRef.current.forEach(t => clearTimeout(t));
    csvTimersRef.current = [];
    setCsvPlaying(false);
  }, []);

  const handleCsvUpload = (e) => {
    const file = e.target.files[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = (evt) => {
      const lines = evt.target.result.split('\n').filter(l => l.trim());
      // Skip header if present
      const startIdx = lines[0].toLowerCase().includes('pressure') ? 1 : 0;
      const values = lines.slice(startIdx).map(line => {
        const val = parseFloat(line.split(',')[0]);
        return isNaN(val) ? null : val;
      }).filter(v => v !== null);

      if (values.length === 0) return;

      // Clear any previous playback
      stopCsvPlayback();
      setCsvPlaying(true);

      // Send each pressure value with 1s interval
      values.forEach((val, i) => {
        const timer = setTimeout(() => {
          // CSV values are in Pa (large numbers) or kPa (< 200)
          const pa = val < 200 ? val * 1000 : val;
          send(`CMD,${TEAM_ID},SIMP,${pa.toFixed(2)}`);
          // Stop playing indicator after last value
          if (i === values.length - 1) setCsvPlaying(false);
        }, i * 1000);
        csvTimersRef.current.push(timer);
      });
    };
    reader.readAsText(file);
    e.target.value = '';  // Reset so same file can be re-selected
  };

  const sendPressure = useCallback(() => {
    if (!pressVal) return;
    const val = parseFloat(pressVal);
    if (isNaN(val)) return;
    // Input is always in kPa, convert to Pa
    const pa = val * 1000;
    send(`CMD,${TEAM_ID},SIMP,${pa.toFixed(2)}`);
    setPressVal('');
  }, [pressVal, send]);

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
        ) : !simActive ? (
          <div style={{ display: 'flex', gap: '4px' }}>
            <button className="btn btn-danger" style={{ flex: 1 }} onClick={() => {
              send(`CMD,${TEAM_ID},SIM,0`);
              setSimEnabled(false);
            }} disabled={!connected}>
              Disable
            </button>
            <button className="btn btn-success" style={{ flex: 1 }} onClick={() => {
              send(`CMD,${TEAM_ID},SIM,2`);
              setSimActive(true);
            }} disabled={!connected}>
              Activate
            </button>
          </div>
        ) : (
          <button className="btn btn-danger" onClick={() => {
            send(`CMD,${TEAM_ID},SIM,0`);
            setSimEnabled(false);
            setSimActive(false);
            stopCsvPlayback();
          }} disabled={!connected}>
            Sim Disable
          </button>
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
          <input
            className="cmd-input"
            type="text"
            placeholder={simActive ? 'Pressure (kPa)' : 'Sim not active'}
            value={pressVal}
            onChange={e => setPressVal(e.target.value)}
            disabled={!simActive}
            onKeyDown={e => { if (e.key === 'Enter') sendPressure(); }}
          />
          <button className="btn btn-warning" onClick={sendPressure} disabled={!connected || !simActive}>Send</button>
          <input ref={csvFileRef} type="file" accept=".csv" onChange={handleCsvUpload} style={{ display: 'none' }} />
          {csvPlaying ? (
            <button className="btn btn-danger" onClick={stopCsvPlayback}>Stop</button>
          ) : (
            <button className="btn btn-purple" onClick={() => csvFileRef.current?.click()} disabled={!connected || !simActive}>CSV</button>
          )}
        </div>
      </div>
    </div>
  );
}
