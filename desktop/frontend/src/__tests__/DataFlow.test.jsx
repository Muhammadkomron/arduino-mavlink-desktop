import { render, screen } from '@testing-library/react';
import { describe, it, expect } from 'vitest';
import DataFlow from '../components/DataFlow';

describe('DataFlow', () => {
  it('shows waiting message when no logs', () => {
    render(<DataFlow logs={[]} />);
    expect(screen.getByText('Waiting for telemetry data...')).toBeInTheDocument();
  });

  it('renders log entries', () => {
    const logs = [
      { timestamp: '12:00:00.000', direction: 'received', message: 'HB #1 | T:25.0°C' },
      { timestamp: '12:00:01.000', direction: 'sent', message: 'CX,1003,CX,ON' },
    ];
    render(<DataFlow logs={logs} />);
    expect(screen.getByText('HB #1 | T:25.0°C')).toBeInTheDocument();
    expect(screen.getByText('CX,1003,CX,ON')).toBeInTheDocument();
  });

  it('displays timestamps', () => {
    const logs = [
      { timestamp: '14:30:15.123', direction: 'received', message: 'test' },
    ];
    render(<DataFlow logs={logs} />);
    expect(screen.getByText('14:30:15.123')).toBeInTheDocument();
  });

  it('shows direction indicators', () => {
    const logs = [
      { timestamp: '12:00:00', direction: 'received', message: 'msg1' },
      { timestamp: '12:00:01', direction: 'sent', message: 'msg2' },
    ];
    render(<DataFlow logs={logs} />);
    // ▼ for received, ▲ for sent
    expect(screen.getByText('▼')).toBeInTheDocument();
    expect(screen.getByText('▲')).toBeInTheDocument();
  });

  it('renders the title', () => {
    render(<DataFlow logs={[]} />);
    expect(screen.getByText('Telemetry Data Flow')).toBeInTheDocument();
  });
});
