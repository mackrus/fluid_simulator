import * as ort from 'onnxruntime-web';

export class MLModule {
    constructor() {
        this.session = null;
        this.initialized = false;
        this.loading = false;
        this.busy = false;
        this.lastBoostTime = 0;
    }

    async init() {
        if (this.initialized || this.loading) return;
        this.loading = true;
        try {
            this.session = await ort.InferenceSession.create('./ml/surrogate.onnx', {
                executionProviders: ['wasm'],
            });
            this.inputBuffer = new Float32Array(1 * 3 * 64 * 64);
            this.initialized = true;
            console.log('Neural Surrogate Model loaded successfully.');
        } catch (e) {
            console.error('Failed to load Neural Surrogate Model:', e);
        } finally {
            this.loading = false;
        }
    }

    /**
     * Runs the neural surrogate model to predict the next fluid state.
     * @param {Float32Array} m - Density (smoke)
     * @param {Float32Array} u - Velocity X
     * @param {Float32Array} v - Velocity Y
     * @param {number} numX - Original grid width
     * @param {number} numY - Original grid height
     */
    async predict(m, u, v, numX, numY) {
        if (!this.initialized || this.busy) return null;
        this.busy = true;

        try {
            // Carefully copy the 64x64 interior (skipping boundaries)
            for (let i = 0; i < 64; i++) {
                for (let j = 0; j < 64; j++) {
                    const srcIdx = (i + 1) * numY + (j + 1);
                    const dstIdx = i * 64 + j;
                    this.inputBuffer[dstIdx] = m[srcIdx];
                    this.inputBuffer[4096 + dstIdx] = u[srcIdx];
                    this.inputBuffer[8192 + dstIdx] = v[srcIdx];
                }
            }

            const tensor = new ort.Tensor('float32', this.inputBuffer, [1, 3, 64, 64]);
            const results = await this.session.run({ input: tensor });
            const output = results.output.data;

            // Sanitize output (NaN/Infinity check)
            for (let i = 0; i < output.length; i++) {
                if (isNaN(output[i]) || !isFinite(output[i])) {
                    output[i] = 0.0;
                }
            }

            return {
                m: output.subarray(0, 4096),
                u: output.subarray(4096, 8192),
                v: output.subarray(8192, 12288)
            };
        } finally {
            this.busy = false;
        }
    }
}
