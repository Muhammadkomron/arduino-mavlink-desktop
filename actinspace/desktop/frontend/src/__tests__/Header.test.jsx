import { render, screen, fireEvent } from '@testing-library/react';
import { describe, it, expect, vi } from 'vitest';
import Header from '../components/Header';

describe('Header', () => {
  const defaultProps = {
    theme: 'dark',
    toggleTheme: vi.fn(),
    missionTime: '05:30',
    packetCount: 42,
    connected: false,
  };

  it('renders the title', () => {
    render(<Header {...defaultProps} />);
    expect(screen.getByText('NazarX Ground Station')).toBeInTheDocument();
  });

  it('shows disconnected status when not connected', () => {
    render(<Header {...defaultProps} />);
    expect(screen.getByText('Disconnected')).toBeInTheDocument();
  });

  it('shows connected status when connected', () => {
    render(<Header {...defaultProps} connected={true} />);
    expect(screen.getByText('Connected')).toBeInTheDocument();
  });

  it('displays mission time', () => {
    render(<Header {...defaultProps} />);
    expect(screen.getByText('05:30')).toBeInTheDocument();
  });

  it('displays packet count', () => {
    render(<Header {...defaultProps} />);
    expect(screen.getByText('42')).toBeInTheDocument();
  });

  it('calls toggleTheme when theme button is clicked', () => {
    const toggleTheme = vi.fn();
    render(<Header {...defaultProps} toggleTheme={toggleTheme} />);
    fireEvent.click(screen.getByTitle('Toggle theme'));
    expect(toggleTheme).toHaveBeenCalledOnce();
  });

  it('shows dark theme logo for dark theme', () => {
    render(<Header {...defaultProps} theme="dark" />);
    const logo = screen.getByAltText('NAZARX');
    expect(logo.getAttribute('src')).toBe('/logo-dark.png');
  });

  it('shows light theme logo for light theme', () => {
    render(<Header {...defaultProps} theme="light" />);
    const logo = screen.getByAltText('NAZARX');
    expect(logo.getAttribute('src')).toBe('/logo-light.png');
  });

  it('shows moon icon in dark theme', () => {
    render(<Header {...defaultProps} theme="dark" />);
    expect(screen.getByTitle('Toggle theme').textContent).toBe('☾');
  });

  it('shows sun icon in light theme', () => {
    render(<Header {...defaultProps} theme="light" />);
    expect(screen.getByTitle('Toggle theme').textContent).toBe('☀');
  });

  it('applies connected CSS class when connected', () => {
    render(<Header {...defaultProps} connected={true} />);
    const status = screen.getByText('Connected');
    expect(status.className).toContain('connected');
  });

  it('applies disconnected CSS class when disconnected', () => {
    render(<Header {...defaultProps} connected={false} />);
    const status = screen.getByText('Disconnected');
    expect(status.className).toContain('disconnected');
  });
});
