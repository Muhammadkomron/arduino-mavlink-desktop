import { useState, useEffect, useCallback, useRef } from 'react';
import Header from './components/Header';
import Sidebar from './components/Sidebar';
import ChartCard from './components/ChartCard';
import GPSMap from './components/GPSMap';
import Rocket3D from './components/Rocket3D';
import VideoFeed from './components/VideoFeed';
import DataFlow from './components/DataFlow';
import Commands from './components/Commands';

const DEFAULT_TELEMETRY = {
  counter: 0, temperature: 0, humidity: 0, pressure: 1013.0,
  altitude: 0, voltage: 0, accelX: 0, accelY: 0, accelZ: 0,
  gyroX: 0, gyroY: 0, gyroZ: 0, roll: 0, pitch: 0, yaw: 0,
  gyroRate: 0, magX: 0, magY: 0, magZ: 0,
  gpsLat: 0, gpsLon: 0, gpsAlt: 0, gpsSats: 0,
  gpsFix: 0, msgCount: 0, packetsLost: 0, timestamp: '',
  connected: false, mode: 'Flight', state: 'Launch Wait',
};

const SSE_URL = 'http://localhost:9877/telemetry';
const SSE_THEME_URL = 'http://localhost:9877/theme';

// Hook: subscribe to SSE telemetry stream (for standalone browser windows)
function useSSETelemetry(setter) {
  useEffect(() => {
    const es = new EventSource(SSE_URL);
    es.onmessage = (e) => {
      try {
        setter(JSON.parse(e.data));
      } catch { /* ignore */ }
    };
    return () => es.close();
  }, [setter]);
}

// Hook: subscribe to SSE theme changes (for standalone child windows)
function useSSETheme(setter) {
  useEffect(() => {
    const es = new EventSource(SSE_THEME_URL);
    es.onmessage = (e) => {
      const theme = e.data.trim();
      if (theme === 'dark' || theme === 'light') {
        setter(theme);
        document.documentElement.setAttribute('data-theme', theme);
      }
    };
    es.onerror = () => { /* reconnects automatically */ };
    return () => es.close();
  }, [setter]);
}

function SplashScreen({ onFinish }) {
  const [fadeOut, setFadeOut] = useState(false);
  useEffect(() => {
    const timer = setTimeout(() => setFadeOut(true), 5000);
    const remove = setTimeout(onFinish, 5600);
    return () => { clearTimeout(timer); clearTimeout(remove); };
  }, [onFinish]);

  return (
    <div className={`splash-screen ${fadeOut ? 'splash-fade-out' : ''}`}>
      <div className="splash-content">
        <div className="splash-logo-row">
          <img src="/logo-dark.png" alt="NazarX" className="splash-logo" />
          <img src="/flag-uzbekistan.png" alt="Uzbekistan" className="splash-flag" />
        </div>
        <p className="splash-subtitle">Ground Station</p>
      </div>
      <div className="splash-progress">
        <div className="splash-progress-bar" />
      </div>
    </div>
  );
}

// Standalone fullscreen views for child windows
function StandaloneGPS() {
  const [telemetry, setTelemetry] = useState(DEFAULT_TELEMETRY);
  const [theme, setTheme] = useState(() => localStorage.getItem('theme') || 'dark');

  useEffect(() => {
    document.documentElement.setAttribute('data-theme', theme);
    document.title = 'GPS Tracking - NazarX';
  }, [theme]);

  useSSETelemetry(setTelemetry);
  useSSETheme(setTheme);

  return (
    <div className="standalone-window">
      <GPSMap lat={telemetry.gpsLat} lon={telemetry.gpsLon} alt={telemetry.gpsAlt} sats={telemetry.gpsSats} standalone />
    </div>
  );
}

function Standalone3D() {
  const [telemetry, setTelemetry] = useState(DEFAULT_TELEMETRY);
  const [theme, setTheme] = useState(() => localStorage.getItem('theme') || 'dark');

  useEffect(() => {
    document.documentElement.setAttribute('data-theme', theme);
    document.title = '3D Orientation - NazarX';
  }, [theme]);

  useSSETelemetry(setTelemetry);
  useSSETheme(setTheme);

  return (
    <div className="standalone-window">
      <Rocket3D roll={telemetry.roll} pitch={telemetry.pitch} yaw={telemetry.yaw} theme={theme} standalone />
    </div>
  );
}

function StandaloneVideo() {
  const [theme, setTheme] = useState(() => localStorage.getItem('theme') || 'dark');

  useEffect(() => {
    document.documentElement.setAttribute('data-theme', theme);
    document.title = 'Video Feed - NazarX';
  }, [theme]);

  useSSETheme(setTheme);

  return (
    <div className="standalone-window">
      <VideoFeed standalone />
    </div>
  );
}

export default function App() {
  const [viewMode, setViewMode] = useState(null);
  const [checked, setChecked] = useState(false);

  useEffect(() => {
    // Check URL hash first (browser fallback)
    const hash = window.location.hash.replace('#', '');
    if (hash) {
      setViewMode(hash);
      setChecked(true);
      return;
    }

    // Ask Go backend for CANSAT_VIEW env var (native child windows)
    if (window.go?.backend?.App?.GetViewMode) {
      window.go.backend.App.GetViewMode().then((mode) => {
        setViewMode(mode || '');
        setChecked(true);
      }).catch(() => {
        setViewMode('');
        setChecked(true);
      });
    } else {
      // No Wails runtime — main dashboard
      setViewMode('');
      setChecked(true);
    }
  }, []);

  if (!checked) return null; // Wait for view mode detection

  if (viewMode === 'gps') return <StandaloneGPS />;
  if (viewMode === '3d') return <Standalone3D />;
  if (viewMode === 'video') return <StandaloneVideo />;

  return <MainDashboard />;
}

function MainDashboard() {
  const [loading, setLoading] = useState(true);
  const [theme, setTheme] = useState(() => localStorage.getItem('theme') || 'dark');
  const [telemetry, setTelemetry] = useState(DEFAULT_TELEMETRY);
  const [connected, setConnected] = useState(false);
  const [dataFlowLogs, setDataFlowLogs] = useState([]);
  const [missionTime, setMissionTime] = useState('00:00');

  // Chart history
  const [altHistory, setAltHistory] = useState([]);
  const [voltHistory, setVoltHistory] = useState([]);
  const [pressHistory, setPressHistory] = useState([]);
  const [tempHistory, setTempHistory] = useState([]);
  const [rollHistory, setRollHistory] = useState([]);
  const [pitchHistory, setPitchHistory] = useState([]);
  const [gyroHistory, setGyroHistory] = useState([]);
  const [yawHistory, setYawHistory] = useState([]);

  const MAX_POINTS = 60;

  const toggleTheme = useCallback(() => {
    setTheme(prev => {
      const next = prev === 'dark' ? 'light' : 'dark';
      localStorage.setItem('theme', next);
      // Broadcast theme to child windows via SSE
      if (window.go?.backend?.App?.SetTheme) {
        window.go.backend.App.SetTheme(next);
      }
      return next;
    });
  }, []);

  useEffect(() => {
    document.documentElement.setAttribute('data-theme', theme);
  }, [theme]);

  // Wails event listeners
  useEffect(() => {
    let cleanupTelemetry, cleanupConnection, cleanupDataflow;

    if (window.runtime) {
      cleanupTelemetry = window.runtime.EventsOn('telemetry', (data) => {
        setTelemetry(data);
        setConnected(data.connected);

        const now = new Date().toLocaleTimeString('en', { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' });
        const addPoint = (setter, value) => {
          setter(prev => {
            const next = [...prev, { x: now, y: value }];
            return next.length > MAX_POINTS ? next.slice(-MAX_POINTS) : next;
          });
        };
        addPoint(setAltHistory, data.altitude);
        addPoint(setVoltHistory, data.voltage);
        addPoint(setPressHistory, data.pressure / 10);
        addPoint(setTempHistory, data.temperature);
        addPoint(setRollHistory, data.roll);
        addPoint(setPitchHistory, data.pitch);
        addPoint(setGyroHistory, data.gyroRate);
        addPoint(setYawHistory, data.yaw);
      });

      cleanupConnection = window.runtime.EventsOn('connection', (data) => {
        setConnected(data.connected);
      });

      cleanupDataflow = window.runtime.EventsOn('dataflow', (data) => {
        setDataFlowLogs(prev => {
          const next = [...prev, data];
          return next.length > 100 ? next.slice(-100) : next;
        });
      });
    }

    // Mission time ticker
    const timer = setInterval(async () => {
      if (window.go && connected) {
        try {
          const t = await window.go.backend.App.GetMissionTime();
          setMissionTime(t);
        } catch { /* ignore */ }
      }
    }, 1000);

    return () => {
      clearInterval(timer);
      if (cleanupTelemetry) cleanupTelemetry();
      if (cleanupConnection) cleanupConnection();
      if (cleanupDataflow) cleanupDataflow();
    };
  }, [connected]);

  if (loading) return <SplashScreen onFinish={() => setLoading(false)} />;

  return (
    <div className="app">
      <Header
        theme={theme}
        toggleTheme={toggleTheme}
        missionTime={missionTime}
        packetCount={telemetry.msgCount}
        connected={connected}
      />
      <div className="main-layout">
        <Sidebar telemetry={telemetry} connected={connected} />
        <main className="content">
          <Commands connected={connected} />
          <div className="chart-row">
            <ChartCard title="Altitude" data={altHistory} value={`${telemetry.altitude.toFixed(1)} m`} color="#4a9eff" unit="m" />
            <ChartCard title="Voltage" data={voltHistory} value={`${telemetry.voltage.toFixed(1)} V`} color="#ffd93d" unit="V" />
            <ChartCard title="Pressure" data={pressHistory} value={`${(telemetry.pressure / 10).toFixed(1)} kPa`} color="#6bcb77" unit="kPa" />
            <ChartCard title="Temperature" data={tempHistory} value={`${telemetry.temperature.toFixed(1)} °C`} color="#ff6b6b" unit="°C" />
          </div>
          <div className="chart-row">
            <ChartCard title="Gyro Rate" data={gyroHistory} value={`${telemetry.gyroRate.toFixed(1)} °/s`} color="#1abc9c" unit="°/s" />
            <ChartCard title="Roll" data={rollHistory} value={`${telemetry.roll.toFixed(1)} °`} color="#9b59b6" unit="°" />
            <ChartCard title="Pitch" data={pitchHistory} value={`${telemetry.pitch.toFixed(1)} °`} color="#e67e22" unit="°" />
            <ChartCard title="Yaw" data={yawHistory} value={`${telemetry.yaw.toFixed(1)} °`} color="#e74c3c" unit="°" />
          </div>
          <div className="viz-row">
            <GPSMap lat={telemetry.gpsLat} lon={telemetry.gpsLon} alt={telemetry.gpsAlt} sats={telemetry.gpsSats} />
            <Rocket3D roll={telemetry.roll} pitch={telemetry.pitch} yaw={telemetry.yaw} theme={theme} />
            <VideoFeed />
          </div>
          <DataFlow logs={dataFlowLogs} />
        </main>
      </div>
    </div>
  );
}
