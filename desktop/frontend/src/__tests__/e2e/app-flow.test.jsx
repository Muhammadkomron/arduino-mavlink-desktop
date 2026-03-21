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

describe('App E2E Flow', () => {
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

    window.go.backend.App.ListPorts.mockResolvedValue(['/dev/ttyUSB0', '/dev/ttyACM0']);
    window.go.backend.App.Connect.mockResolvedValue();
    window.go.backend.App.Disconnect.mockResolvedValue();
    window.go.backend.App.SendCommand.mockResolvedValue();
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

  it('completes a full connect → receive telemetry → send command → disconnect cycle', async () => {
    renderAndSkipSplash();

    // --- Step 1: Verify initial state ---
    expect(screen.getByText('Disconnected')).toBeInTheDocument();
    expect(screen.getByText('Telemetry ON')).toBeDisabled();

    // --- Step 2: Click Connect ---
    const connectBtn = screen.getByText('Connect');
    await act(async () => {
      fireEvent.click(connectBtn);
    });

    expect(window.go.backend.App.Connect).toHaveBeenCalled();

    // --- Step 3: Simulate connection success event ---
    await act(async () => {
      connectionCallback({ connected: true, port: '/dev/ttyUSB0', baud: 57600 });
    });

    expect(screen.getByText('Connected')).toBeInTheDocument();
    expect(screen.getByText('Telemetry ON')).not.toBeDisabled();

    // --- Step 4: Receive telemetry data ---
    await act(async () => {
      telemetryCallback({
        counter: 1, temperature: 22.5, pressure: 1013.0, altitude: 50.0,
        voltage: 12.1, humidity: 40.0, roll: 2.0, pitch: -1.5, yaw: 45.0,
        accelX: 0.0, accelY: 0.0, accelZ: 9.81, gyroX: 0.1, gyroY: 0.0,
        gyroZ: 0.0, gyroRate: 0.1, gpsLat: 41.311, gpsLon: 69.279,
        gpsAlt: 50.0, gpsSats: 6, gpsFix: 3, msgCount: 1, packetsLost: 0,
        timestamp: '10:00:01', connected: true, mode: 'Flight', state: 'Ascent',
      });
    });

    expect(screen.getByTestId('rocket3d').textContent).toContain('R:2.0');
    expect(screen.getByTestId('gpsmap').textContent).toContain('41.311');

    // --- Step 5: Send a command ---
    await act(async () => {
      fireEvent.click(screen.getByText('Telemetry ON'));
    });

    expect(window.go.backend.App.SendCommand).toHaveBeenCalledWith('CX,1003,CX,ON');

    // --- Step 6: Receive data flow log ---
    await act(async () => {
      dataflowCallback({
        direction: 'sent',
        message: 'CX,1003,CX,ON',
        timestamp: '10:00:02.000',
      });
    });

    expect(screen.getByText('CX,1003,CX,ON')).toBeInTheDocument();

    // --- Step 7: Disconnect ---
    const disconnectBtn = screen.getByText('Disconnect');
    await act(async () => {
      fireEvent.click(disconnectBtn);
    });

    expect(window.go.backend.App.Disconnect).toHaveBeenCalled();

    await act(async () => {
      connectionCallback({ connected: false });
    });

    expect(screen.getByText('Disconnected')).toBeInTheDocument();
  });

  it('handles Set Time command with user input', async () => {
    renderAndSkipSplash();

    await act(async () => {
      connectionCallback({ connected: true });
    });

    const timeInput = screen.getByPlaceholderText('HH:MM:SS');
    fireEvent.change(timeInput, { target: { value: '14:30:00' } });
    fireEvent.click(screen.getByText('Set Time'));

    expect(window.go.backend.App.SendCommand).toHaveBeenCalledWith('CX,1003,ST,14:30:00');
  });

  it('handles Set Date command with user input', async () => {
    renderAndSkipSplash();

    await act(async () => {
      connectionCallback({ connected: true });
    });

    const dateInput = screen.getByPlaceholderText('DD.MM.YYYY');
    fireEvent.change(dateInput, { target: { value: '21.03.2026' } });
    fireEvent.click(screen.getByText('Set Date'));

    expect(window.go.backend.App.SendCommand).toHaveBeenCalledWith('CX,1003,SD,21.03.2026');
  });

  it('handles simulation mode commands in sequence', async () => {
    renderAndSkipSplash();

    await act(async () => {
      connectionCallback({ connected: true });
    });

    fireEvent.click(screen.getByText('Sim Enable'));
    expect(window.go.backend.App.SendCommand).toHaveBeenCalledWith('CX,1003,SIM,ENABLE');

    fireEvent.click(screen.getByText('Sim Activate'));
    expect(window.go.backend.App.SendCommand).toHaveBeenCalledWith('CX,1003,SIM,ACTIVATE');

    const pressInput = screen.getByPlaceholderText('Pressure (kPa)');
    fireEvent.change(pressInput, { target: { value: '101.325' } });
    fireEvent.click(screen.getAllByText('Send')[0]);
    expect(window.go.backend.App.SendCommand).toHaveBeenCalledWith('CX,1003,SIM,101.325');

    fireEvent.click(screen.getByText('Sim Disable'));
    expect(window.go.backend.App.SendCommand).toHaveBeenCalledWith('CX,1003,SIM,DISABLE');
  });

  it('updates multiple telemetry cycles correctly', async () => {
    renderAndSkipSplash();

    await act(async () => {
      connectionCallback({ connected: true });
    });

    // Cycle 1
    await act(async () => {
      telemetryCallback({
        counter: 1, temperature: 20.0, pressure: 1010.0, altitude: 0.0,
        voltage: 12.5, humidity: 50.0, roll: 0.0, pitch: 0.0, yaw: 0.0,
        accelX: 0, accelY: 0, accelZ: 9.81, gyroX: 0, gyroY: 0, gyroZ: 0,
        gyroRate: 0, gpsLat: 41.0, gpsLon: 69.0, gpsAlt: 0, gpsSats: 4,
        gpsFix: 2, msgCount: 1, packetsLost: 0, timestamp: '10:00:00',
        connected: true, mode: 'Flight', state: 'Launch Wait',
      });
    });

    expect(screen.getByTestId('gpsmap').textContent).toContain('Lat:41');

    // Cycle 2 — values change
    await act(async () => {
      telemetryCallback({
        counter: 2, temperature: 22.0, pressure: 1005.0, altitude: 100.0,
        voltage: 12.3, humidity: 45.0, roll: 5.0, pitch: -2.0, yaw: 30.0,
        accelX: 0.5, accelY: -0.1, accelZ: 9.7, gyroX: 1.0, gyroY: 0.5,
        gyroZ: -0.2, gyroRate: 1.0, gpsLat: 41.312, gpsLon: 69.280,
        gpsAlt: 100.0, gpsSats: 8, gpsFix: 3, msgCount: 2, packetsLost: 0,
        timestamp: '10:00:01', connected: true, mode: 'Flight', state: 'Ascent',
      });
    });

    expect(screen.getByTestId('rocket3d').textContent).toContain('R:5.0');
    expect(screen.getByTestId('gpsmap').textContent).toContain('41.312');
  });

  it('handles theme toggle during active session', async () => {
    renderAndSkipSplash();

    await act(async () => {
      connectionCallback({ connected: true });
    });

    const themeBtn = screen.getByTitle('Toggle theme');
    fireEvent.click(themeBtn);

    expect(screen.getByText('Connected')).toBeInTheDocument();
    expect(screen.getByText('Telemetry ON')).not.toBeDisabled();

    fireEvent.click(screen.getByText('Calibrate'));
    expect(window.go.backend.App.SendCommand).toHaveBeenCalledWith('CX,1003,CAL');
  });
});
