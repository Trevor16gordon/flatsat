import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// Deployed as static files behind whatever fronts it (nginx, a CDN, or
// the future FastAPI service). `base` stays relative so the build works
// unchanged whether it is served from / or from a subpath.
export default defineConfig({
  base: './',
  plugins: [react()],
  server: { port: 5173 },
  build: { outDir: 'dist', sourcemap: true },
});
