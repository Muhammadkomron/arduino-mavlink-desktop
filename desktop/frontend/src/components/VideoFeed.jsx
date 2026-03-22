import { useState, useRef, useEffect, useCallback } from 'react';

const ExpandIcon = () => (
  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <polyline points="15 3 21 3 21 9" />
    <polyline points="9 21 3 21 3 15" />
    <line x1="21" y1="3" x2="14" y2="10" />
    <line x1="3" y1="21" x2="10" y2="14" />
  </svg>
);

const CollapseIcon = () => (
  <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
    <polyline points="4 14 10 14 10 20" />
    <polyline points="20 10 14 10 14 4" />
    <line x1="14" y1="10" x2="21" y2="3" />
    <line x1="3" y1="21" x2="10" y2="14" />
  </svg>
);

export default function VideoFeed({ standalone }) {
  const videoRef = useRef(null);
  const [devices, setDevices] = useState([]);
  const [selectedDevice, setSelectedDevice] = useState('');
  const [streaming, setStreaming] = useState(false);
  const streamRef = useRef(null);

  useEffect(() => {
    async function getDevices() {
      try {
        await navigator.mediaDevices.getUserMedia({ video: true });
        const allDevices = await navigator.mediaDevices.enumerateDevices();
        const videoDevices = allDevices.filter(d => d.kind === 'videoinput');
        setDevices(videoDevices);
        if (videoDevices.length > 0 && !selectedDevice) {
          setSelectedDevice(videoDevices[0].deviceId);
        }
      } catch {
        // No camera access
      }
    }
    getDevices();
  }, []);

  const startStream = useCallback(async () => {
    if (!selectedDevice) return;
    try {
      if (streamRef.current) {
        streamRef.current.getTracks().forEach(t => t.stop());
      }
      const stream = await navigator.mediaDevices.getUserMedia({
        video: { deviceId: { exact: selectedDevice } },
      });
      streamRef.current = stream;
      if (videoRef.current) {
        videoRef.current.srcObject = stream;
      }
      setStreaming(true);
    } catch (err) {
      console.error('Failed to start video:', err);
    }
  }, [selectedDevice]);

  const stopStream = useCallback(() => {
    if (streamRef.current) {
      streamRef.current.getTracks().forEach(t => t.stop());
      streamRef.current = null;
    }
    if (videoRef.current) {
      videoRef.current.srcObject = null;
    }
    setStreaming(false);
  }, []);

  useEffect(() => {
    return () => {
      if (streamRef.current) {
        streamRef.current.getTracks().forEach(t => t.stop());
      }
    };
  }, []);

  const [expanded, setExpanded] = useState(false);

  // Poll child window status to detect native close
  useEffect(() => {
    if (!window.go?.backend?.App?.IsWindowOpen) return;
    window.go.backend.App.IsWindowOpen('video').then(setExpanded).catch(() => {});
    const poll = setInterval(() => {
      window.go.backend.App.IsWindowOpen('video').then(setExpanded).catch(() => {});
    }, 2000);
    return () => clearInterval(poll);
  }, []);

  const toggleWindow = useCallback(async () => {
    if (window.go?.backend?.App?.ToggleWindow) {
      const isNowOpen = await window.go.backend.App.ToggleWindow('video');
      setExpanded(isNowOpen);
    } else {
      window.open(`${window.location.origin}/#video`, 'video-feed', 'width=900,height=600');
    }
  }, []);

  if (standalone) {
    return (
      <div className="standalone-view">
        <div className="standalone-view-body" style={{ background: '#000' }}>
          {streaming ? (
            <video ref={videoRef} autoPlay muted playsInline style={{ width: '100%', height: '100%', objectFit: 'contain' }} />
          ) : (
            <div className="video-placeholder">
              {devices.length > 0 ? 'Select a camera and click Start' : 'No video devices found'}
            </div>
          )}
        </div>
        <div className="standalone-view-footer">
          <select
            value={selectedDevice}
            onChange={e => setSelectedDevice(e.target.value)}
            style={{ padding: '4px 8px', border: '1px solid var(--border)', borderRadius: '4px', background: 'var(--bg-input)', color: 'var(--text-primary)', fontSize: '12px' }}
          >
            {devices.length === 0 && <option value="">No cameras</option>}
            {devices.map(d => (
              <option key={d.deviceId} value={d.deviceId}>
                {d.label || `Camera ${d.deviceId.slice(0, 8)}`}
              </option>
            ))}
          </select>
          {!streaming ? (
            <button className="btn btn-primary" onClick={startStream} style={{ fontSize: '11px', padding: '4px 12px' }}>Start</button>
          ) : (
            <button className="btn btn-danger" onClick={stopStream} style={{ fontSize: '11px', padding: '4px 12px' }}>Stop</button>
          )}
        </div>
      </div>
    );
  }

  return (
    <div className="viz-card">
      <div className="viz-card-header">
        <h4>Video Feed</h4>
        <button className={`viz-expand-btn${expanded ? ' expanded' : ''}`} onClick={toggleWindow} title={expanded ? 'Close window' : 'Open in new window'}>
          {expanded ? <CollapseIcon /> : <ExpandIcon />}
        </button>
      </div>
      <div className="viz-content">
        <div className="video-container">
          {streaming ? (
            <video ref={videoRef} autoPlay muted playsInline />
          ) : (
            <div className="video-placeholder">
              {devices.length > 0 ? 'Select a camera and click Start' : 'No video devices found'}
            </div>
          )}
        </div>
      </div>
      <div className="video-select">
        <select
          value={selectedDevice}
          onChange={e => setSelectedDevice(e.target.value)}
          style={{ flex: 1, padding: '4px 6px', border: '1px solid var(--border)', borderRadius: '4px', background: 'var(--bg-input)', color: 'var(--text-primary)', fontSize: '10px' }}
        >
          {devices.length === 0 && <option value="">No cameras</option>}
          {devices.map(d => (
            <option key={d.deviceId} value={d.deviceId}>
              {d.label || `Camera ${d.deviceId.slice(0, 8)}`}
            </option>
          ))}
        </select>
        {!streaming ? (
          <button className="btn btn-primary" onClick={startStream} style={{ fontSize: '10px', padding: '4px 8px' }}>Start</button>
        ) : (
          <button className="btn btn-danger" onClick={stopStream} style={{ fontSize: '10px', padding: '4px 8px' }}>Stop</button>
        )}
      </div>
    </div>
  );
}
