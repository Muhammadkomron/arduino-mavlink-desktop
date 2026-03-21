import { render, screen, fireEvent } from '@testing-library/react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import Sidebar from '../components/Sidebar';

const defaultTelemetry = {
  counter: 0, temperature: 25.5, humidity: 60.0, pressure: 1013.25,
  altitude: 150.0, voltage: 12.5, accelX: 0.1, accelY: -0.2, accelZ: 9.8,
  gyroX: 1.5, gyroY: -0.5, gyroZ: 0.3, roll: 10.5, pitch: -5.2, yaw: 180.0,
  gyroRate: 1.5, gpsLat: 41.3111, gpsLon: 69.2797, gpsAlt: 450.0, gpsSats: 12,
  gpsFix: 3, msgCount: 100, packetsLost: 2, timestamp: '12:00:00',
  connected: false, mode: 'Flight', state: 'Launch Wait',
};

describe('Sidebar', () => {
  beforeEach(() => {
    window.go.backend.App.ListPorts.mockResolvedValue([]);
  });

  it('renders connection panel', () => {
    render(<Sidebar telemetry={defaultTelemetry} connected={false} />);
    expect(screen.getByText('Connection')).toBeInTheDocument();
  });

  it('shows Connect button when disconnected', () => {
    render(<Sidebar telemetry={defaultTelemetry} connected={false} />);
    expect(screen.getByText('Connect')).toBeInTheDocument();
  });

  it('shows Disconnect button when connected', () => {
    render(<Sidebar telemetry={defaultTelemetry} connected={true} />);
    expect(screen.getByText('Disconnect')).toBeInTheDocument();
  });

  it('displays team info', () => {
    render(<Sidebar telemetry={defaultTelemetry} connected={false} />);
    expect(screen.getByText('Team Info')).toBeInTheDocument();
    expect(screen.getByText('1003')).toBeInTheDocument();
    expect(screen.getByText('Flight')).toBeInTheDocument();
    expect(screen.getByText('Launch Wait')).toBeInTheDocument();
  });

  it('displays telemetry values', async () => {
    const { container } = render(<Sidebar telemetry={defaultTelemetry} connected={false} />);
    // Use container queries to avoid act() warnings from async port refresh
    expect(container.textContent).toContain('12.50 V');
    expect(container.textContent).toContain('150.0 m');
    expect(container.textContent).toContain('1013.3 hPa');
    expect(container.textContent).toContain('25.5 °C');
  });

  it('displays gyro values', () => {
    render(<Sidebar telemetry={defaultTelemetry} connected={false} />);
    expect(screen.getByText('Gyro')).toBeInTheDocument();
    expect(screen.getByText('10.50')).toBeInTheDocument(); // roll
    expect(screen.getByText('-5.20')).toBeInTheDocument(); // pitch
    expect(screen.getByText('180.00')).toBeInTheDocument(); // yaw
  });

  it('displays accel values', () => {
    render(<Sidebar telemetry={defaultTelemetry} connected={false} />);
    expect(screen.getByText('Accel')).toBeInTheDocument();
    expect(screen.getByText('0.1 m/s²')).toBeInTheDocument();
    expect(screen.getByText('-0.2 m/s²')).toBeInTheDocument();
    expect(screen.getByText('9.8 m/s²')).toBeInTheDocument();
  });

  it('displays GPS values', () => {
    render(<Sidebar telemetry={defaultTelemetry} connected={false} />);
    expect(screen.getByText('GPS')).toBeInTheDocument();
    expect(screen.getByText('41.3111')).toBeInTheDocument();
    expect(screen.getByText('69.2797')).toBeInTheDocument();
    expect(screen.getByText('450.0')).toBeInTheDocument();
  });

  it('displays packet count from telemetry', () => {
    render(<Sidebar telemetry={defaultTelemetry} connected={false} />);
    expect(screen.getByText('100')).toBeInTheDocument();
  });

  it('shows "No ports found" when no ports available', () => {
    render(<Sidebar telemetry={defaultTelemetry} connected={false} />);
    expect(screen.getByText('No ports found')).toBeInTheDocument();
  });

  it('has a refresh ports button', () => {
    render(<Sidebar telemetry={defaultTelemetry} connected={false} />);
    expect(screen.getByTitle('Refresh ports')).toBeInTheDocument();
  });

  it('calls ListPorts when refresh button is clicked', () => {
    render(<Sidebar telemetry={defaultTelemetry} connected={false} />);
    fireEvent.click(screen.getByTitle('Refresh ports'));
    expect(window.go.backend.App.ListPorts).toHaveBeenCalled();
  });
});
