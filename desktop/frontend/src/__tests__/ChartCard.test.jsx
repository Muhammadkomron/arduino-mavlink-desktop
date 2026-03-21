import { render, screen } from '@testing-library/react';
import { describe, it, expect } from 'vitest';
import ChartCard from '../components/ChartCard';

describe('ChartCard', () => {
  it('renders the title', () => {
    render(<ChartCard title="Altitude" data={[]} value="150.5 m" color="#4a9eff" unit="m" />);
    expect(screen.getByText('Altitude')).toBeInTheDocument();
  });

  it('displays the current value', () => {
    render(<ChartCard title="Voltage" data={[]} value="12.5 V" color="#ffd93d" unit="V" />);
    expect(screen.getByText('12.5 V')).toBeInTheDocument();
  });

  it('renders with data points', () => {
    const data = [
      { x: '12:00:00', y: 100 },
      { x: '12:00:01', y: 105 },
      { x: '12:00:02', y: 110 },
    ];
    render(<ChartCard title="Pressure" data={data} value="1013.0 kPa" color="#6bcb77" unit="kPa" />);
    expect(screen.getByText('1013.0 kPa')).toBeInTheDocument();
  });

  it('renders canvas element', () => {
    const { container } = render(
      <ChartCard title="Temp" data={[]} value="25.0 °C" color="#ff6b6b" unit="°C" />
    );
    expect(container.querySelector('canvas')).toBeInTheDocument();
  });

  it('renders with different chart types', () => {
    const charts = [
      { title: 'Roll', value: '10.0 °', color: '#9b59b6', unit: '°' },
      { title: 'Pitch', value: '-5.0 °', color: '#e67e22', unit: '°' },
      { title: 'Gyro Rate', value: '1.5 °/s', color: '#1abc9c', unit: '°/s' },
      { title: 'Yaw', value: '180.0 °', color: '#e74c3c', unit: '°' },
    ];
    charts.forEach(chart => {
      const { unmount } = render(
        <ChartCard title={chart.title} data={[]} value={chart.value} color={chart.color} unit={chart.unit} />
      );
      expect(screen.getByText(chart.title)).toBeInTheDocument();
      expect(screen.getByText(chart.value)).toBeInTheDocument();
      unmount();
    });
  });
});
