import { useEffect, useRef, useState, useCallback } from 'react';
import L from 'leaflet';

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

export default function GPSMap({ lat, lon, alt, sats, standalone }) {
  const mapRef = useRef(null);
  const mapInstanceRef = useRef(null);
  const markerRef = useRef(null);
  const pathRef = useRef(null);
  const [pathPoints, setPathPoints] = useState([]);

  const createMapIcon = () => L.divIcon({
    className: 'gps-marker',
    html: `<div style="
      width: 14px; height: 14px;
      background: #4a9eff;
      border: 2px solid #fff;
      border-radius: 50%;
      box-shadow: 0 0 8px rgba(74,158,255,0.6);
    "></div>`,
    iconSize: [14, 14],
    iconAnchor: [7, 7],
  });

  useEffect(() => {
    if (!mapRef.current || mapInstanceRef.current) return;

    const map = L.map(mapRef.current, {
      center: [41.3111, 69.2797],
      zoom: 15,
      zoomControl: true,
      attributionControl: false,
    });

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19,
    }).addTo(map);

    markerRef.current = L.marker([41.3111, 69.2797], { icon: createMapIcon() }).addTo(map);
    pathRef.current = L.polyline([], {
      color: '#4a9eff',
      weight: 2,
      opacity: 0.7,
    }).addTo(map);

    mapInstanceRef.current = map;
    setTimeout(() => map.invalidateSize(), 100);

    return () => {
      map.remove();
      mapInstanceRef.current = null;
    };
  }, []);

  useEffect(() => {
    if (!mapInstanceRef.current || !markerRef.current) return;
    if (lat === 0 && lon === 0) return;

    const pos = [lat, lon];
    markerRef.current.setLatLng(pos);
    mapInstanceRef.current.setView(pos, mapInstanceRef.current.getZoom());

    setPathPoints(prev => {
      const next = [...prev, pos];
      if (pathRef.current) pathRef.current.setLatLngs(next);
      return next.length > 500 ? next.slice(-500) : next;
    });
  }, [lat, lon]);

  const [expanded, setExpanded] = useState(false);

  // Poll child window status to detect native close
  useEffect(() => {
    if (!window.go?.backend?.App?.IsWindowOpen) return;
    window.go.backend.App.IsWindowOpen('gps').then(setExpanded).catch(() => {});
    const poll = setInterval(() => {
      window.go.backend.App.IsWindowOpen('gps').then(setExpanded).catch(() => {});
    }, 2000);
    return () => clearInterval(poll);
  }, []);

  const toggleWindow = useCallback(async () => {
    if (window.go?.backend?.App?.ToggleWindow) {
      const isNowOpen = await window.go.backend.App.ToggleWindow('gps');
      setExpanded(isNowOpen);
    } else {
      window.open(`${window.location.origin}/#gps`, 'gps-tracking', 'width=900,height=600');
    }
  }, []);

  if (standalone) {
    return (
      <div className="standalone-view">
        <div className="standalone-view-body">
          <div ref={mapRef} style={{ width: '100%', height: '100%' }} />
        </div>
        <div className="standalone-view-footer">
          <span>Lat: {lat.toFixed(6)}</span>
          <span>Lon: {lon.toFixed(6)}</span>
          <span>Alt: {alt.toFixed(1)}m</span>
          <span>Sats: {sats}</span>
        </div>
      </div>
    );
  }

  return (
    <div className="viz-card">
      <div className="viz-card-header">
        <h4>GPS Tracking</h4>
        <button className={`viz-expand-btn${expanded ? ' expanded' : ''}`} onClick={toggleWindow} title={expanded ? 'Close window' : 'Open in new window'}>
          {expanded ? <CollapseIcon /> : <ExpandIcon />}
        </button>
      </div>
      <div className="viz-content">
        <div ref={mapRef} className="gps-map-container" />
      </div>
      <div className="gps-stats">
        <span>Lat: {lat.toFixed(6)}</span>
        <span>Lon: {lon.toFixed(6)}</span>
        <span>Alt: {alt.toFixed(1)}m</span>
        <span>Sats: {sats}</span>
      </div>
    </div>
  );
}
