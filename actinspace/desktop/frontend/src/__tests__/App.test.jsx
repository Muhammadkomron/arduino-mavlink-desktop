import { render, screen, act } from '@testing-library/react';
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import App from '../App';

// Mock heavy components that require WebGL/Canvas
vi.mock('../components/Rocket3D', () => ({
  default: ({ roll, pitch, yaw }) => (
    <div data-testid="rocket3d">
      R:{roll.toFixed(1)} P:{pitch.toFixed(1)} Y:{yaw.toFixed(1)}
    </div>
  ),
}));

vi.mock('../components/GPSMap', () => ({
  default: ({ lat, lon, alt, sats }) => (
    <div data-testid="gpsmap">
      Lat:{lat} Lon:{lon} Alt:{alt} Sats:{sats}
    </div>
  ),
}));

vi.mock('../components/ChartCard', () => ({
  default: ({ title, value }) => (
    <div data-testid={`chart-${title.toLowerCase()}`}>
      <h4>{title}</h4>
      <span>{value}</span>
    </div>
  ),
}));

describe('App', () => {
  beforeEach(() => {
    vi.useFakeTimers();
  });

  afterEach(() => {
    vi.useRealTimers();
  });

  function renderAndSkipSplash() {
    const result = render(<App />);
    // Fast-forward past splash screen (5s display + 0.6s fade)
    act(() => {
      vi.advanceTimersByTime(6000);
    });
    return result;
  }

  it('shows splash screen initially', () => {
    render(<App />);
    expect(screen.getByText('Ground Station')).toBeInTheDocument();
    expect(screen.getByAltText('NazarX')).toBeInTheDocument();
  });

  it('renders the main layout after splash', () => {
    const { container } = renderAndSkipSplash();
    expect(container.querySelector('.app')).toBeInTheDocument();
  });

  it('shows disconnected by default', () => {
    renderAndSkipSplash();
    expect(screen.getByText('Disconnected')).toBeInTheDocument();
  });

  it('renders all chart cards', () => {
    renderAndSkipSplash();
    expect(screen.getAllByText('Altitude').length).toBeGreaterThanOrEqual(1);
    expect(screen.getAllByText('Voltage').length).toBeGreaterThanOrEqual(1);
    expect(screen.getAllByText('Pressure').length).toBeGreaterThanOrEqual(1);
    expect(screen.getByText('Temperature')).toBeInTheDocument();
    expect(screen.getByText('Gyro Rate')).toBeInTheDocument();
  });

  it('renders GPS map', () => {
    renderAndSkipSplash();
    expect(screen.getByTestId('gpsmap')).toBeInTheDocument();
  });

  it('renders 3D rocket view', () => {
    renderAndSkipSplash();
    expect(screen.getByTestId('rocket3d')).toBeInTheDocument();
  });

  it('renders video feed', () => {
    renderAndSkipSplash();
    expect(screen.getByText('Video Feed')).toBeInTheDocument();
  });

  it('renders commands panel', () => {
    renderAndSkipSplash();
    expect(screen.getByText('Commands')).toBeInTheDocument();
  });

  it('renders data flow panel', () => {
    renderAndSkipSplash();
    expect(screen.getByText('Telemetry Data Flow')).toBeInTheDocument();
  });

  it('renders sidebar with connection panel', () => {
    renderAndSkipSplash();
    expect(screen.getByText('Connection')).toBeInTheDocument();
  });

  it('registers Wails event listeners', () => {
    renderAndSkipSplash();
    expect(window.runtime.EventsOn).toHaveBeenCalledWith('telemetry', expect.any(Function));
    expect(window.runtime.EventsOn).toHaveBeenCalledWith('connection', expect.any(Function));
    expect(window.runtime.EventsOn).toHaveBeenCalledWith('dataflow', expect.any(Function));
  });

  it('shows default telemetry values', () => {
    renderAndSkipSplash();
    expect(screen.getAllByText('0.0 m').length).toBeGreaterThanOrEqual(1);
  });
});
