// vite.config.js
import { defineConfig } from 'vite';

export default defineConfig({
    base: '',
    build: {
        sourcemap: true,
        compress: {
            keep_fnames: true,  // Keep original function names
            // Add more specific configurations if needed
        },

    },
});