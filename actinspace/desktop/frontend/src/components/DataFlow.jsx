import { useRef, useEffect } from 'react';

export default function DataFlow({ logs }) {
  const scrollRef = useRef(null);

  useEffect(() => {
    if (scrollRef.current) {
      scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
    }
  }, [logs]);

  return (
    <div className="dataflow-card">
      <h4>Telemetry Data Flow</h4>
      <div className="dataflow-log" ref={scrollRef}>
        {logs.length === 0 ? (
          <div className="dataflow-empty">Waiting for telemetry data...</div>
        ) : (
          logs.map((log, i) => (
            <div className="dataflow-entry" key={i}>
              <span className="dataflow-time">{log.timestamp}</span>
              <span className={`dataflow-dir-${log.direction}`}>
                {log.direction === 'sent' ? '\u25B2' : '\u25BC'}
              </span>
              <span className="dataflow-msg">{log.message}</span>
            </div>
          ))
        )}
      </div>
    </div>
  );
}
