import { useRef, useEffect, useMemo } from 'react';
import { Chart, registerables } from 'chart.js';

Chart.register(...registerables);

export default function ChartCard({ title, data, value, color, unit }) {
  const canvasRef = useRef(null);
  const chartRef = useRef(null);

  const chartColor = useMemo(() => color || '#4a9eff', [color]);

  useEffect(() => {
    const ctx = canvasRef.current?.getContext('2d');
    if (!ctx) return;

    if (chartRef.current) {
      chartRef.current.destroy();
    }

    const gradient = ctx.createLinearGradient(0, 0, 0, 80);
    gradient.addColorStop(0, chartColor + '40');
    gradient.addColorStop(1, chartColor + '05');

    chartRef.current = new Chart(ctx, {
      type: 'line',
      data: {
        labels: data.map(d => d.x),
        datasets: [{
          data: data.map(d => d.y),
          borderColor: chartColor,
          backgroundColor: gradient,
          borderWidth: 1.5,
          pointRadius: 0,
          fill: true,
          tension: 0.3,
        }],
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: { duration: 0 },
        plugins: { legend: { display: false }, tooltip: { enabled: false } },
        scales: {
          x: {
            display: false,
            grid: { display: false },
          },
          y: {
            display: false,
            grid: { color: 'var(--chart-grid)' },
          },
        },
        interaction: { intersect: false },
      },
    });

    return () => {
      if (chartRef.current) {
        chartRef.current.destroy();
        chartRef.current = null;
      }
    };
  }, [data, chartColor]);

  return (
    <div className="chart-card">
      <h4>{title}</h4>
      <div className="chart-canvas-wrap">
        <canvas ref={canvasRef} />
      </div>
      <span className="chart-value">{value}</span>
    </div>
  );
}
