import { render, screen, fireEvent } from '@testing-library/react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import Commands from '../components/Commands';

describe('Commands', () => {
  beforeEach(() => {
    window.go.backend.App.SendCommand.mockClear();
  });

  it('renders the title', () => {
    render(<Commands connected={false} />);
    expect(screen.getByText('Commands')).toBeInTheDocument();
  });

  it('disables all buttons when disconnected', () => {
    render(<Commands connected={false} />);
    const buttons = screen.getAllByRole('button');
    buttons.forEach(btn => {
      expect(btn).toBeDisabled();
    });
  });

  it('enables command buttons when connected', () => {
    render(<Commands connected={true} />);
    expect(screen.getByText('Telemetry ON')).not.toBeDisabled();
    expect(screen.getByText('Telemetry OFF')).not.toBeDisabled();
    expect(screen.getByText('Sim Enable')).not.toBeDisabled();
    expect(screen.getByText('Sim Disable')).not.toBeDisabled();
    expect(screen.getByText('Sim Activate')).not.toBeDisabled();
    expect(screen.getByText('Calibrate')).not.toBeDisabled();
  });

  it('sends telemetry ON command', async () => {
    render(<Commands connected={true} />);
    fireEvent.click(screen.getByText('Telemetry ON'));
    expect(window.go.backend.App.SendCommand).toHaveBeenCalledWith('CX,1003,CX,ON');
  });

  it('sends telemetry OFF command', async () => {
    render(<Commands connected={true} />);
    fireEvent.click(screen.getByText('Telemetry OFF'));
    expect(window.go.backend.App.SendCommand).toHaveBeenCalledWith('CX,1003,CX,OFF');
  });

  it('sends SIM ENABLE command', async () => {
    render(<Commands connected={true} />);
    fireEvent.click(screen.getByText('Sim Enable'));
    expect(window.go.backend.App.SendCommand).toHaveBeenCalledWith('CX,1003,SIM,ENABLE');
  });

  it('sends SIM DISABLE command', async () => {
    render(<Commands connected={true} />);
    fireEvent.click(screen.getByText('Sim Disable'));
    expect(window.go.backend.App.SendCommand).toHaveBeenCalledWith('CX,1003,SIM,DISABLE');
  });

  it('sends SIM ACTIVATE command', async () => {
    render(<Commands connected={true} />);
    fireEvent.click(screen.getByText('Sim Activate'));
    expect(window.go.backend.App.SendCommand).toHaveBeenCalledWith('CX,1003,SIM,ACTIVATE');
  });

  it('sends CAL command', async () => {
    render(<Commands connected={true} />);
    fireEvent.click(screen.getByText('Calibrate'));
    expect(window.go.backend.App.SendCommand).toHaveBeenCalledWith('CX,1003,CAL');
  });

  it('sends set time command with input value', async () => {
    render(<Commands connected={true} />);
    const timeInput = screen.getByPlaceholderText('HH:MM:SS');
    fireEvent.change(timeInput, { target: { value: '12:30:00' } });
    fireEvent.click(screen.getByText('Set Time'));
    expect(window.go.backend.App.SendCommand).toHaveBeenCalledWith('CX,1003,ST,12:30:00');
  });

  it('sends set date command with input value', async () => {
    render(<Commands connected={true} />);
    const dateInput = screen.getByPlaceholderText('DD.MM.YYYY');
    fireEvent.change(dateInput, { target: { value: '21.03.2026' } });
    fireEvent.click(screen.getByText('Set Date'));
    expect(window.go.backend.App.SendCommand).toHaveBeenCalledWith('CX,1003,SD,21.03.2026');
  });

  it('sends pressure value via SIM command', async () => {
    render(<Commands connected={true} />);
    const pressInput = screen.getByPlaceholderText('Pressure (kPa)');
    fireEvent.change(pressInput, { target: { value: '101.325' } });
    fireEvent.click(screen.getAllByText('Send')[0]);
    expect(window.go.backend.App.SendCommand).toHaveBeenCalledWith('CX,1003,SIM,101.325');
  });

  it('clears input after sending time', async () => {
    render(<Commands connected={true} />);
    const timeInput = screen.getByPlaceholderText('HH:MM:SS');
    fireEvent.change(timeInput, { target: { value: '12:30:00' } });
    fireEvent.click(screen.getByText('Set Time'));
    expect(timeInput.value).toBe('');
  });

  it('renders all expected input fields', () => {
    render(<Commands connected={false} />);
    expect(screen.getByPlaceholderText('HH:MM:SS')).toBeInTheDocument();
    expect(screen.getByPlaceholderText('DD.MM.YYYY')).toBeInTheDocument();
    expect(screen.getByPlaceholderText('Pressure (kPa)')).toBeInTheDocument();
  });

  it('renders Select File button', () => {
    render(<Commands connected={true} />);
    expect(screen.getByText('Select File')).toBeInTheDocument();
    expect(screen.getByText('Select File')).not.toBeDisabled();
  });
});
