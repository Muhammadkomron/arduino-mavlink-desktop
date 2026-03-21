import { useEffect, useRef, useState } from 'react';
import L from 'leaflet';

export default function GPSMap({ lat, lon, alt, sats }) {
  const mapRef = useRef(null);
  const mapInstanceRef = useRef(null);
  const markerRef = useRef(null);
  const pathRef = useRef(null);
  const [pathPoints, setPathPoints] = useState([]);

  // Initialize map
  useEffect(() => {
    if (!mapRef.current || mapInstanceRef.current) return;

    const map = L.map(mapRef.current, {
      center: [41.3111, 69.2797], // Default: Tashkent
      zoom: 15,
      zoomControl: true,
      attributionControl: false,
    });

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19,
    }).addTo(map);

    // Custom marker
    const icon = L.divIcon({
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

    markerRef.current = L.marker([41.3111, 69.2797], { icon }).addTo(map);
    pathRef.current = L.polyline([], {
      color: '#4a9eff',
      weight: 2,
      opacity: 0.7,
    }).addTo(map);

    mapInstanceRef.current = map;

    // Fix map size after render
    setTimeout(() => map.invalidateSize(), 100);

    return () => {
      map.remove();
      mapInstanceRef.current = null;
    };
  }, []);

  // Update position
  useEffect(() => {
    if (!mapInstanceRef.current || !markerRef.current) return;
    if (lat === 0 && lon === 0) return;

    const pos = [lat, lon];
    markerRef.current.setLatLng(pos);
    mapInstanceRef.current.setView(pos, mapInstanceRef.current.getZoom());

    setPathPoints(prev => {
      const next = [...prev, pos];
      if (pathRef.current) {
        pathRef.current.setLatLngs(next);
      }
      return next.length > 500 ? next.slice(-500) : next;
    });
  }, [lat, lon]);

  return (
    <div className="viz-card">
      <div className="viz-card-header">
        <h4>GPS Tracking</h4>
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
