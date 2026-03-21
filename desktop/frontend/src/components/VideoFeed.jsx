import { useState, useRef, useEffect, useCallback } from 'react';

export default function VideoFeed() {
  const videoRef = useRef(null);
  const overlayRef = useRef(null);
  const dragRef = useRef({ dragging: false, x: 0, y: 0 });
  const [devices, setDevices] = useState([]);
  const [selectedDevice, setSelectedDevice] = useState('');
  const [streaming, setStreaming] = useState(false);
  const [expanded, setExpanded] = useState(false);
  const [overlayPos, setOverlayPos] = useState({ x: 100, y: 100 });
  const streamRef = useRef(null);

  // Enumerate video devices
  useEffect(() => {
    async function getDevices() {
      try {
        // Request permission first
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
    setExpanded(false);
  }, []);

  // Assign stream to video element when expanding/collapsing
  useEffect(() => {
    if (videoRef.current && streamRef.current) {
      videoRef.current.srcObject = streamRef.current;
    }
  }, [expanded]);

  // Drag logic for overlay
  const onMouseDown = useCallback((e) => {
    if (e.target.closest('.video-overlay-close')) return;
    dragRef.current = { dragging: true, x: e.clientX - overlayPos.x, y: e.clientY - overlayPos.y };
    const onMove = (ev) => {
      if (dragRef.current.dragging) {
        setOverlayPos({ x: ev.clientX - dragRef.current.x, y: ev.clientY - dragRef.current.y });
      }
    };
    const onUp = () => {
      dragRef.current.dragging = false;
      document.removeEventListener('mousemove', onMove);
      document.removeEventListener('mouseup', onUp);
    };
    document.addEventListener('mousemove', onMove);
    document.addEventListener('mouseup', onUp);
  }, [overlayPos]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (streamRef.current) {
        streamRef.current.getTracks().forEach(t => t.stop());
      }
    };
  }, []);

  return (
    <>
      <div className="viz-card">
        <div className="viz-card-header">
          <h4>Video Feed</h4>
          {streaming && !expanded && (
            <button
              className="video-expand-btn"
              onClick={() => setExpanded(true)}
              title="Expand video"
              style={{ opacity: 1, position: 'static' }}
            >
              &#x2197;
            </button>
          )}
        </div>
        <div className="viz-content">
          <div className="video-container">
            {streaming && !expanded ? (
              <video ref={videoRef} autoPlay muted playsInline />
            ) : expanded ? (
              <div className="video-placeholder">Video in floating window</div>
            ) : (
              <div className="video-placeholder">
                {devices.length > 0 ? 'Select a camera and click Start' : 'No video devices found'}
              </div>
            )}
            {streaming && !expanded && (
              <button className="video-expand-btn" onClick={() => setExpanded(true)} title="Pop out">
                &#x2197;
              </button>
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
            <button className="btn btn-primary" onClick={startStream} style={{ fontSize: '10px', padding: '4px 8px' }}>
              Start
            </button>
          ) : (
            <button className="btn btn-danger" onClick={stopStream} style={{ fontSize: '10px', padding: '4px 8px' }}>
              Stop
            </button>
          )}
        </div>
      </div>

      {/* Floating video overlay */}
      {expanded && streaming && (
        <div
          className="video-overlay"
          ref={overlayRef}
          style={{
            left: overlayPos.x,
            top: overlayPos.y,
            width: 640,
            height: 400,
          }}
        >
          <div className="video-overlay-header" onMouseDown={onMouseDown}>
            <span>Video Feed</span>
            <button className="video-overlay-close" onClick={() => setExpanded(false)}>
              &#x2715;
            </button>
          </div>
          <video
            ref={el => {
              if (el && streamRef.current) {
                el.srcObject = streamRef.current;
              }
            }}
            autoPlay
            muted
            playsInline
            style={{ width: '100%', height: 'calc(100% - 32px)', objectFit: 'contain' }}
          />
        </div>
      )}
    </>
  );
}
