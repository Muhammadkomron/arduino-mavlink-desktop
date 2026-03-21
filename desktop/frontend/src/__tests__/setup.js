import '@testing-library/jest-dom';

// Mock window.runtime (Wails runtime)
window.runtime = {
  EventsOn: vi.fn(() => vi.fn()),
  EventsEmit: vi.fn(),
};

// Mock window.go (Wails Go bindings)
window.go = {
  backend: {
    App: {
      ListPorts: vi.fn(() => Promise.resolve([])),
      Connect: vi.fn(() => Promise.resolve()),
      Disconnect: vi.fn(() => Promise.resolve()),
      SendCommand: vi.fn(() => Promise.resolve()),
      GetMissionTime: vi.fn(() => Promise.resolve('00:00')),
      IsConnected: vi.fn(() => Promise.resolve(false)),
    },
  },
};

// Mock canvas getContext for Chart.js
HTMLCanvasElement.prototype.getContext = vi.fn(() => ({
  createLinearGradient: vi.fn(() => ({
    addColorStop: vi.fn(),
  })),
  canvas: { width: 300, height: 150 },
  clearRect: vi.fn(),
  fillRect: vi.fn(),
  beginPath: vi.fn(),
  moveTo: vi.fn(),
  lineTo: vi.fn(),
  stroke: vi.fn(),
  fill: vi.fn(),
  arc: vi.fn(),
  closePath: vi.fn(),
  measureText: vi.fn(() => ({ width: 0 })),
  setTransform: vi.fn(),
  resetTransform: vi.fn(),
  save: vi.fn(),
  restore: vi.fn(),
  scale: vi.fn(),
  translate: vi.fn(),
  rotate: vi.fn(),
  drawImage: vi.fn(),
  getImageData: vi.fn(() => ({ data: new Uint8ClampedArray(4) })),
  putImageData: vi.fn(),
  createPattern: vi.fn(),
  font: '',
  textAlign: '',
  textBaseline: '',
  fillStyle: '',
  strokeStyle: '',
  lineWidth: 1,
  lineCap: '',
  lineJoin: '',
  globalAlpha: 1,
  globalCompositeOperation: '',
}));

// Mock ResizeObserver
global.ResizeObserver = vi.fn().mockImplementation(() => ({
  observe: vi.fn(),
  unobserve: vi.fn(),
  disconnect: vi.fn(),
}));

// Mock navigator.mediaDevices
Object.defineProperty(navigator, 'mediaDevices', {
  value: {
    getUserMedia: vi.fn(() => Promise.reject(new Error('No camera in test'))),
    enumerateDevices: vi.fn(() => Promise.resolve([])),
  },
  writable: true,
});
