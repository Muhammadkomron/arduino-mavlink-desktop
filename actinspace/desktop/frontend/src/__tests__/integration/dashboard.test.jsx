import { render, screen, fireEvent, act } from '@testing-library/react';
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import App from '../../App';

// Mock heavy components
vi.mock('../../components/Rocket3D', () => ({
  default: ({ roll, pitch, yaw }) => (
    <div data-testid="rocket3d">
      R:{roll.toFixed(1)} P:{pitch.toFixed(1)} Y:{yaw.toFixed(1)}
    </div>
  ),
}));

vi.mock('../../components/GPSMap', () => ({
  default: ({ lat, lon, alt, sats }) => (
    <div data-testid="gpsmap">
      Lat:{lat} Lon:{lon} Alt:{alt} Sats:{sats}
    </div>
  ),
}));

vi.mock('../../components/ChartCard', () => ({
  default: ({ title, value }) => (
    <div data-testid={`chart-${title.toLowerCase().replace(/\s+/g, '-')}`}>
      <h4>{title}</h4>
      <span>{value}</span>
    </div>
  ),
}));

describe('Dashboard Integration', () => {
  let telemetryCallback;
  let connectionCallback;
  let dataflowCallback;

  beforeEach(() => {
    vi.useFakeTimers();
    telemetryCallback = null;
    connectionCallback = null;
    dataflowCallback = null;

    window.runtime.EventsOn.mockImplementation((event, cb) => {
      if (event === 'telemetry') telemetryCallback = cb;
      if (event === 'connection') connectionCallback = cb;
      if (event === 'dataflow') dataflowCallback = cb;
      return vi.fn();
    });

    window.go.backend.App.ListPorts.mockResolvedValue([]);
    window.go.backend.App.GetMissionTime.mockResolvedValue('00:00');
    window.go.backend.App.IsConnected.mockResolvedValue(false);
  });

  afterEach(() => {
    vi.useRealTimers();
  });

  function renderAndSkipSplash() {
    const result = render(<App />);
    act(() => {
      vi.advanceTimersByTime(6000);
    });
    return result;
  }

  it('renders all main dashboard sections', () => {
    renderAndSkipSplash();

    expect(screen.getByText('Commands')).toBeInTheDocument();
    expect(screen.getByText('Telemetry Data Flow')).toBeInTheDocument();
    expect(screen.getByTestId('rocket3d')).toBeInTheDocument();
    expect(screen.getByTestId('gpsmap')).toBeInTheDocument();
    expect(screen.getByText('Video Feed')).toBeInTheDocument();
    expect(screen.getByText('Connection')).toBeInTheDocument();
  });

  it('updates telemetry values when event is received', async () => {
    renderAndSkipSplash();

    await act(async () => {
      if (telemetryCallback) {
        telemetryCallback({
          counter: 5, temperature: 28.5, pressure: 1015.0, altitude: 200.0,
          voltage: 11.8, humidity: 55.0, roll: 5.0, pitch: -3.0, yaw: 90.0,
          accelX: 0.5, accelY: -0.1, accelZ: 9.8, gyroX: 2.0, gyroY: -1.0,
          gyroZ: 0.5, gyroRate: 2.0, gpsLat: 41.3111, gpsLon: 69.2797,
          gpsAlt: 200.0, gpsSats: 10, gpsFix: 3, msgCount: 5, packetsLost: 0,
          timestamp: '12:00:05', connected: true, mode: 'Flight', state: 'Ascent',
        });
      }
    });

    expect(screen.getByTestId('gpsmap').textContent).toContain('41.3111');
    expect(screen.getByTestId('rocket3d').textContent).toContain('R:5.0');
  });

  it('shows connection status change', async () => {
    renderAndSkipSplash();

    expect(screen.getByText('Disconnected')).toBeInTheDocument();

    await act(async () => {
      if (connectionCallback) {
        connectionCallback({ connected: true, port: '/dev/ttyUSB0', baud: 57600 });
      }
    });

    expect(screen.getByText('Connected')).toBeInTheDocument();
  });

  it('disables commands when disconnected and enables when connected', async () => {
    renderAndSkipSplash();

    const telOnBtn = screen.getByText('Telemetry ON');
    expect(telOnBtn).toBeDisabled();

    await act(async () => {
      if (connectionCallback) {
        connectionCallback({ connected: true });
      }
    });

    expect(screen.getByText('Telemetry ON')).not.toBeDisabled();
  });

  it('adds data flow entries when dataflow event fires', async () => {
    renderAndSkipSplash();

    expect(screen.getByText('Waiting for telemetry data...')).toBeInTheDocument();

    await act(async () => {
      if (dataflowCallback) {
        dataflowCallback({
          direction: 'received',
          message: 'HB #1 | T:25.0°C',
          timestamp: '12:00:00.000',
        });
      }
    });

    expect(screen.getByText('HB #1 | T:25.0°C')).toBeInTheDocument();
  });

  it('registers all required Wails event listeners', () => {
    renderAndSkipSplash();

    const registeredEvents = window.runtime.EventsOn.mock.calls.map(c => c[0]);
    expect(registeredEvents).toContain('telemetry');
    expect(registeredEvents).toContain('connection');
    expect(registeredEvents).toContain('dataflow');
  });
});
