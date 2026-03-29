import { useState, useEffect, useCallback, useRef } from 'react';

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

const CARD = { 0:'N', 45:'NE', 90:'E', 135:'SE', 180:'S', 225:'SW', 270:'W', 315:'NW' };

function getCardinal(h) {
  h = ((h%360)+360)%360;
  if (h>=337.5||h<22.5) return 'N';
  if (h<67.5) return 'NE';
  if (h<112.5) return 'E';
  if (h<157.5) return 'SE';
  if (h<202.5) return 'S';
  if (h<247.5) return 'SW';
  if (h<292.5) return 'W';
  return 'NW';
}

function CompassHUD({ heading, magX, magY, magZ, standalone }) {
  const canvasRef = useRef(null);
  const containerRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    const container = containerRef.current;
    if (!canvas || !container) return;
    const dpr = window.devicePixelRatio || 1;

    // Size the canvas
    // Standalone: fit within window, square canvas
    // Dashboard: fixed 290px square (matches viz-card height)
    let w, h;
    if (standalone) {
      const size = Math.min(window.innerWidth, window.innerHeight, 600);
      w = size;
      h = size;
    } else {
      w = 290;
      h = 290;
    }
    canvas.width = w;
    canvas.height = h;
    // Don't set style.width/height — let the canvas render at its intrinsic pixel size
    // The container centers it via flexbox
    canvas.style.width = '';
    canvas.style.height = '';
    const ctx = canvas.getContext('2d');

    const hdg = ((heading % 360) + 360) % 360;
    const rad = (hdg * Math.PI) / 180;
    const accent = '#4a9eff';
    const white = '#e0e0e0';
    const dim = 'rgba(255,255,255,0.3)';
    const bg = '#141420';

    ctx.fillStyle = bg;
    ctx.fillRect(0, 0, w, h);

    // ════════════ HEADING TAPE (top strip) ════════════
    const tapeH = Math.round(h * 0.1);
    const tapeCx = w / 2;
    const ppd = w / 140; // pixels per degree — scales with width

    ctx.fillStyle = 'rgba(0,0,0,0.5)';
    ctx.fillRect(0, 0, w, tapeH);

    // Bottom edge line
    ctx.strokeStyle = 'rgba(74,158,255,0.2)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, tapeH);
    ctx.lineTo(w, tapeH);
    ctx.stroke();

    const vis = (w/2) / ppd;
    for (let d = Math.floor(hdg-vis)-5; d <= Math.ceil(hdg+vis)+5; d++) {
      const nd = ((d%360)+360)%360;
      const px = tapeCx + (d - hdg) * ppd;
      if (px < -20 || px > w+20) continue;
      if (d%5 !== 0) continue;

      const isC = nd in CARD;
      const isM = nd%15 === 0;
      let th = isC ? tapeH*0.6 : (isM ? tapeH*0.38 : tapeH*0.2);

      ctx.beginPath();
      ctx.moveTo(px, tapeH);
      ctx.lineTo(px, tapeH - th);
      ctx.strokeStyle = isC ? accent : dim;
      ctx.lineWidth = isC ? Math.max(1.5, w*0.004) : 1;
      ctx.stroke();

      if (isC) {
        const fs = Math.round(w * 0.032);
        ctx.font = `bold ${fs}px sans-serif`;
        ctx.fillStyle = accent;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'bottom';
        ctx.fillText(CARD[nd], px, tapeH - th - 2);
      } else if (isM) {
        const fs = Math.round(w * 0.022);
        ctx.font = `${fs}px sans-serif`;
        ctx.fillStyle = dim;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'bottom';
        ctx.fillText(nd.toString(), px, tapeH - th - 1);
      }
    }

    // Center marker — small triangle from bottom
    const tmW = Math.round(w * 0.014);
    const tmH = Math.round(w * 0.018);
    ctx.beginPath();
    ctx.moveTo(tapeCx-tmW, tapeH);
    ctx.lineTo(tapeCx+tmW, tapeH);
    ctx.lineTo(tapeCx, tapeH-tmH);
    ctx.closePath();
    ctx.fillStyle = accent;
    ctx.fill();

    // Edge fade
    const fw = Math.round(w * 0.07);
    let gl = ctx.createLinearGradient(0,0,fw,0);
    gl.addColorStop(0, bg); gl.addColorStop(1, 'transparent');
    ctx.fillStyle = gl;
    ctx.fillRect(0, 0, fw, tapeH);
    let gr = ctx.createLinearGradient(w-fw,0,w,0);
    gr.addColorStop(0, 'transparent'); gr.addColorStop(1, bg);
    ctx.fillStyle = gr;
    ctx.fillRect(w-fw, 0, fw, tapeH);

    // ════════════ CIRCULAR COMPASS DIAL ════════════
    const dialCx = w / 2;
    // Layout: tape(10%) + gap + dial + heading + mag
    const availH = h - tapeH;
    const R = Math.round(availH * 0.25);
    const dialCy = tapeH + Math.round(availH * 0.03) + R;

    // Outer glow ring
    ctx.beginPath();
    ctx.arc(dialCx, dialCy, R+4, 0, Math.PI*2);
    ctx.strokeStyle = 'rgba(74,158,255,0.12)';
    ctx.lineWidth = Math.max(3, w * 0.008);
    ctx.stroke();

    // Main ring
    ctx.beginPath();
    ctx.arc(dialCx, dialCy, R, 0, Math.PI*2);
    ctx.strokeStyle = 'rgba(255,255,255,0.15)';
    ctx.lineWidth = Math.max(1.5, w * 0.004);
    ctx.stroke();

    // Rotating ticks and labels
    ctx.save();
    ctx.translate(dialCx, dialCy);
    ctx.rotate(-rad);

    for (let d = 0; d < 360; d += 5) {
      const a = (d - 90) * Math.PI / 180;
      const isC = d in CARD;
      const isM = d%15 === 0;
      const outer = R - 2;
      const inner = isC ? R-R*0.16 : (isM ? R-R*0.11 : R-R*0.06);

      ctx.beginPath();
      ctx.moveTo(outer*Math.cos(a), outer*Math.sin(a));
      ctx.lineTo(inner*Math.cos(a), inner*Math.sin(a));
      ctx.strokeStyle = isC ? (d===0 ? '#ff4444' : accent) : dim;
      ctx.lineWidth = isC ? Math.max(2, w*0.004) : 1;
      ctx.stroke();

      if (isC) {
        const lr = R - R*0.26;
        const lx = lr * Math.cos(a);
        const ly = lr * Math.sin(a);
        const fs = Math.round(R * 0.13);
        ctx.font = `bold ${fs}px sans-serif`;
        ctx.fillStyle = d===0 ? '#ff4444' : white;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(CARD[d], lx, ly);
      }
    }
    ctx.restore();

    // Fixed aircraft icon in center
    const as = Math.round(R * 0.17);
    ctx.save();
    ctx.translate(dialCx, dialCy);
    // Fuselage
    ctx.beginPath();
    ctx.moveTo(0, -as);
    ctx.lineTo(-as*0.2, as*0.5);
    ctx.lineTo(0, as*0.35);
    ctx.lineTo(as*0.2, as*0.5);
    ctx.closePath();
    ctx.fillStyle = accent;
    ctx.globalAlpha = 0.9;
    ctx.fill();
    ctx.globalAlpha = 1;
    // Wings
    ctx.beginPath();
    ctx.moveTo(-as*1.1, as*0.1);
    ctx.lineTo(0, -as*0.2);
    ctx.lineTo(as*1.1, as*0.1);
    ctx.strokeStyle = accent;
    ctx.lineWidth = Math.max(2, R*0.02);
    ctx.stroke();
    // Tail
    ctx.beginPath();
    ctx.moveTo(-as*0.5, as*0.45);
    ctx.lineTo(0, as*0.3);
    ctx.lineTo(as*0.5, as*0.45);
    ctx.strokeStyle = accent;
    ctx.lineWidth = Math.max(1.5, R*0.015);
    ctx.stroke();
    ctx.restore();

    // Fixed heading bug — small triangle at top of dial
    const bugW = Math.round(R * 0.06);
    const bugH = Math.round(R * 0.08);
    ctx.beginPath();
    ctx.moveTo(dialCx, dialCy - R - 4);
    ctx.lineTo(dialCx-bugW, dialCy - R - 4 - bugH);
    ctx.lineTo(dialCx+bugW, dialCy - R - 4 - bugH);
    ctx.closePath();
    ctx.fillStyle = '#ff4444';
    ctx.fill();

    // ════════════ HEADING READOUT below dial ════════════
    const dialBottom = dialCy + R;
    const magBarH = Math.round(h * 0.06);
    const readSpace = h - dialBottom - magBarH;
    const readY = dialBottom + Math.round(readSpace * 0.15);
    const hdgStr = Math.round(hdg).toString().padStart(3, '0') + '\u00B0';
    const hdgFs = Math.min(Math.max(14, Math.round(readSpace * 0.35)), 32);
    ctx.font = `bold ${hdgFs}px monospace`;
    ctx.fillStyle = '#ffffff';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    ctx.fillText(hdgStr, dialCx, readY);

    const cardinal = getCardinal(hdg);
    ctx.font = `${Math.min(Math.max(9, Math.round(readSpace*0.18)), 16)}px sans-serif`;
    ctx.fillStyle = accent;
    ctx.fillText(cardinal, dialCx, readY + hdgFs + 2);

    // ════════════ MAG DATA ════════════
    const magY_pos = h - Math.round(magBarH * 0.5);
    const mFs = Math.max(8, Math.round(w * 0.025));
    ctx.font = `${mFs}px monospace`;
    ctx.textBaseline = 'middle';
    const mags = [
      {l:'MAG X', v:magX, c:'#ff6b6b'},
      {l:'MAG Y', v:magY, c:'#6bcb77'},
      {l:'MAG Z', v:magZ, c:'#4a9eff'},
    ];
    const seg = w/3;
    mags.forEach((m,i) => {
      const mx = seg*i + seg/2;
      ctx.fillStyle = dim;
      ctx.textAlign = 'right';
      ctx.fillText(m.l, mx-3, magY_pos);
      ctx.fillStyle = m.c;
      ctx.textAlign = 'left';
      ctx.fillText((m.v||0).toFixed(1), mx+3, magY_pos);
    });

  }, [heading, magX, magY, magZ, standalone]);

  const cStyle = standalone
    ? { display:'inline-block' }
    : { width:'100%', height:'100%', display:'flex', justifyContent:'center', alignItems:'center' };
  return (
    <div ref={containerRef} style={cStyle}>
      <canvas ref={canvasRef} style={{ display:'block' }} />
    </div>
  );
}

export default function Compass({ heading, magX, magY, magZ, standalone }) {
  const [expanded, setExpanded] = useState(false);

  useEffect(() => {
    if (!window.go?.backend?.App?.IsWindowOpen) return;
    window.go.backend.App.IsWindowOpen('compass').then(setExpanded).catch(()=>{});
    const poll = setInterval(() => {
      window.go.backend.App.IsWindowOpen('compass').then(setExpanded).catch(()=>{});
    }, 2000);
    return () => clearInterval(poll);
  }, []);

  const toggleWindow = useCallback(async () => {
    if (window.go?.backend?.App?.ToggleWindow) {
      const isNowOpen = await window.go.backend.App.ToggleWindow('compass');
      setExpanded(isNowOpen);
    } else {
      window.open(`${window.location.origin}/#compass`, 'compass', 'width=560,height=480');
    }
  }, []);

  if (standalone) {
    return (
      <div className="standalone-view">
        <div className="standalone-view-body" style={{ display:'flex', alignItems:'center', justifyContent:'center', background:'#141420', width:'100%', height:'100vh' }}>
          <CompassHUD heading={heading} magX={magX} magY={magY} magZ={magZ} standalone />
        </div>
      </div>
    );
  }

  return (
    <div className="viz-card">
      <div className="viz-card-header">
        <h4>Compass</h4>
        <button className={`viz-expand-btn${expanded?' expanded':''}`} onClick={toggleWindow} title={expanded?'Close window':'Open in new window'}>
          {expanded ? <CollapseIcon /> : <ExpandIcon />}
        </button>
      </div>
      <div className="viz-content" style={{ padding:0, background:'#141420', borderRadius:'0 0 8px 8px', overflow:'hidden', flex:1 }}>
        <CompassHUD heading={heading} magX={magX} magY={magY} magZ={magZ} />
      </div>
    </div>
  );
}
