import { render, screen } from '@testing-library/react';
import { describe, it, expect } from 'vitest';
import VideoFeed from '../components/VideoFeed';

describe('VideoFeed', () => {
  it('renders the title', () => {
    render(<VideoFeed />);
    expect(screen.getByText('Video Feed')).toBeInTheDocument();
  });

  it('shows no cameras message when no devices', () => {
    render(<VideoFeed />);
    expect(screen.getByText('No cameras')).toBeInTheDocument();
  });

  it('shows start button', () => {
    render(<VideoFeed />);
    expect(screen.getByText('Start')).toBeInTheDocument();
  });

  it('renders camera select dropdown', () => {
    const { container } = render(<VideoFeed />);
    const select = container.querySelector('select');
    expect(select).toBeInTheDocument();
  });
});
