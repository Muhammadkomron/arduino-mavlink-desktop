import { useState, useCallback, useRef } from 'react';

export default function Commands({ connected }) {
  const [timeVal, setTimeVal] = useState('');
  const [dateVal, setDateVal] = useState('');
  const [pressVal, setPressVal] = useState('');
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
        <button className="btn btn-success" onClick={() => send('CX,1003,CX,ON')} disabled={!connected}>
          Telemetry ON
        </button>
        <button className="btn btn-danger" onClick={() => send('CX,1003,CX,OFF')} disabled={!connected}>
          Telemetry OFF
        </button>
        <button className="btn btn-primary" onClick={() => send('CX,1003,SIM,ENABLE')} disabled={!connected}>
          Sim Enable
        </button>
        <button className="btn btn-danger" onClick={() => send('CX,1003,SIM,DISABLE')} disabled={!connected}>
          Sim Disable
        </button>
        <button className="btn btn-success" onClick={() => send('CX,1003,SIM,ACTIVATE')} disabled={!connected}>
          Sim Activate
        </button>
        <button className="btn btn-secondary" onClick={() => send('CX,1003,CAL')} disabled={!connected}>
          Calibrate
        </button>

        <div className="cmd-input-group">
          <input
            className="cmd-input"
            type="text"
            placeholder="HH:MM:SS"
            value={timeVal}
            onChange={e => setTimeVal(e.target.value)}
          />
          <button
            className="btn btn-primary"
            onClick={() => { send(`CX,1003,ST,${timeVal}`); setTimeVal(''); }}
            disabled={!connected}
          >
            Set Time
          </button>
        </div>

        <div className="cmd-input-group">
          <input
            className="cmd-input"
            type="text"
            placeholder="DD.MM.YYYY"
            value={dateVal}
            onChange={e => setDateVal(e.target.value)}
          />
          <button
            className="btn btn-primary"
            onClick={() => { send(`CX,1003,SD,${dateVal}`); setDateVal(''); }}
            disabled={!connected}
          >
            Set Date
          </button>
        </div>

        <div className="cmd-input-group">
          <input
            className="cmd-input"
            type="text"
            placeholder="Pressure (kPa)"
            value={pressVal}
            onChange={e => setPressVal(e.target.value)}
          />
          <button
            className="btn btn-warning"
            onClick={() => { send(`CX,1003,SIM,${pressVal}`); setPressVal(''); }}
            disabled={!connected}
          >
            Send
          </button>
          <input
            ref={csvFileRef}
            type="file"
            accept=".csv"
            onChange={handleCsvUpload}
            style={{ display: 'none' }}
          />
          <button
            className="btn btn-purple"
            onClick={() => csvFileRef.current?.click()}
            disabled={!connected}
          >
            Select File
          </button>
        </div>
      </div>
    </div>
  );
}
