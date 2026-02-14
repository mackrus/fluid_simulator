import { MLModule } from './ml_module.js';

/**
 * @typedef {Object} SceneConfig
 * @property {number} dt - Time step
 * @property {number} numIters - Number of iterations for the pressure solver
 * @property {number} frameNr - Current frame number
 * @property {number} overRelaxation - Relaxation factor for the solver
 * @property {number} obstacleX - X position of the obstacle
 * @property {number} obstacleY - Y position of the obstacle
 * @property {number} obstacleRadius - Radius of the obstacle
 * @property {boolean} paused - Whether the simulation is paused
 * @property {number} sceneNr - Current scene identifier
 * @property {boolean} showObstacle - Whether to render the obstacle
 */

const FIELD_TYPES = {
    U: 0,
    V: 1,
    S: 2
};

class Fluid {
    constructor(density, numX, numY, h) {
        this.density = density;
        this.numX = numX + 2;
        this.numY = numY + 2;
        this.numCells = this.numX * this.numY;
        this.h = h;
        
        this.u = new Float32Array(this.numCells);
        this.v = new Float32Array(this.numCells);
        this.newU = new Float32Array(this.numCells);
        this.newV = new Float32Array(this.numCells);
        this.p = new Float32Array(this.numCells);
        this.s = new Float32Array(this.numCells);
        this.m = new Float32Array(this.numCells);
        this.newM = new Float32Array(this.numCells);
        
        this.m.fill(1.0);
        this.ml = new MLModule();
        this.neuralEnabled = false;
    }

    async toggleNeural(enabled) {
        this.neuralEnabled = enabled;
        if (enabled && !this.ml.initialized) {
            await this.ml.init();
        }
    }

    async simulateNeural(dt) {
        const result = await this.ml.predict(this.m, this.u, this.v, this.numX, this.numY);
        if (result && this.neuralEnabled) {
            this.ml.lastBoostTime = Date.now();
            for (let i = 0; i < 64; i++) {
                for (let j = 0; j < 64; j++) {
                    const srcIdx = i * 64 + j;
                    const dstIdx = (i + 1) * this.numY + (j + 1);
                    
                    if (this.s[dstIdx] > 0.0) {
                        // 100% Replacement - No blending
                        this.m[dstIdx] = result.m[srcIdx];
                        this.u[dstIdx] = result.u[srcIdx];
                        this.v[dstIdx] = result.v[srcIdx];
                    }
                }
            }
        }
    }

    solveIncompressibility(numIters, dt, overRelaxation) {
        const n = this.numY;
        const cp = (this.density * this.h) / dt;

        for (let iter = 0; iter < numIters; iter++) {
            for (let i = 1; i < this.numX - 1; i++) {
                for (let j = 1; j < this.numY - 1; j++) {
                    const idx = i * n + j;
                    if (this.s[idx] === 0.0) continue;

                    const sx0 = this.s[(i - 1) * n + j];
                    const sx1 = this.s[(i + 1) * n + j];
                    const sy0 = this.s[i * n + j - 1];
                    const sy1 = this.s[i * n + j + 1];
                    const sSum = sx0 + sx1 + sy0 + sy1;

                    if (sSum === 0.0) continue;

                    const div = this.u[(i + 1) * n + j] - this.u[idx] +
                                this.v[i * n + j + 1] - this.v[idx];

                    let p = -div / sSum;
                    p *= overRelaxation;
                    
                    if (isNaN(p) || !isFinite(p)) p = 0.0;
                    
                    this.p[idx] += cp * p;

                    this.u[idx] -= sx0 * p;
                    this.u[(i + 1) * n + j] += sx1 * p;
                    this.v[idx] -= sy0 * p;
                    this.v[i * n + j + 1] += sy1 * p;
                }
            }
        }
    }

    sampleField(x, y, fieldType) {
        const n = this.numY;
        const h = this.h;
        const h1 = 1.0 / h;
        const h2 = 0.5 * h;

        x = Math.max(Math.min(x, this.numX * h), h);
        y = Math.max(Math.min(y, this.numY * h), h);

        let dx = 0.0;
        let dy = 0.0;
        let f;

        switch (fieldType) {
            case FIELD_TYPES.U: f = this.u; dy = h2; break;
            case FIELD_TYPES.V: f = this.v; dx = h2; break;
            case FIELD_TYPES.S: f = this.m; dx = h2; dy = h2; break;
        }

        const x0 = Math.min(Math.floor((x - dx) * h1), this.numX - 1);
        const tx = ((x - dx) - x0 * h) * h1;
        const x1 = Math.min(x0 + 1, this.numX - 1);

        const y0 = Math.min(Math.floor((y - dy) * h1), this.numY - 1);
        const ty = ((y - dy) - y0 * h) * h1;
        const y1 = Math.min(y0 + 1, this.numY - 1);

        const sx = 1.0 - tx;
        const sy = 1.0 - ty;

        return sx * sy * f[x0 * n + y0] +
               tx * sy * f[x1 * n + y0] +
               tx * ty * f[x1 * n + y1] +
               sx * ty * f[x0 * n + y1];
    }

    avgU(i, j) {
        const n = this.numY;
        return (this.u[i * n + j - 1] + this.u[i * n + j] +
                this.u[(i + 1) * n + j - 1] + this.u[(i + 1) * n + j]) * 0.25;
    }

    avgV(i, j) {
        const n = this.numY;
        return (this.v[(i - 1) * n + j] + this.v[i * n + j] +
                this.v[(i - 1) * n + j + 1] + this.v[i * n + j + 1]) * 0.25;
    }

    advectVel(dt) {
        this.newU.set(this.u);
        this.newV.set(this.v);

        const n = this.numY;
        const h = this.h;
        const h2 = 0.5 * h;

        for (let i = 1; i < this.numX; i++) {
            for (let j = 1; j < this.numY; j++) {
                const idx = i * n + j;

                // u component
                if (this.s[idx] !== 0.0 && this.s[(i - 1) * n + j] !== 0.0 && j < this.numY - 1) {
                    let x = i * h;
                    let y = j * h + h2;
                    let u = this.u[idx];
                    let v = this.avgV(i, j);
                    x = x - dt * u;
                    y = y - dt * v;
                    this.newU[idx] = this.sampleField(x, y, FIELD_TYPES.U);
                }
                // v component
                if (this.s[idx] !== 0.0 && this.s[i * n + j - 1] !== 0.0 && i < this.numX - 1) {
                    let x = i * h + h2;
                    let y = j * h;
                    let u = this.avgU(i, j);
                    let v = this.v[idx];
                    x = x - dt * u;
                    y = y - dt * v;
                    this.newV[idx] = this.sampleField(x, y, FIELD_TYPES.V);
                }
            }
        }

        this.u.set(this.newU);
        this.v.set(this.newV);
    }

    advectSmoke(dt) {
        this.newM.set(this.m);

        const n = this.numY;
        const h = this.h;
        const h2 = 0.5 * h;

        for (let i = 1; i < this.numX - 1; i++) {
            for (let j = 1; j < this.numY - 1; j++) {
                const idx = i * n + j;
                if (this.s[idx] !== 0.0) {
                    const u = (this.u[idx] + this.u[(i + 1) * n + j]) * 0.5;
                    const v = (this.v[idx] + this.v[i * n + j + 1]) * 0.5;
                    const x = i * h + h2 - dt * u;
                    const y = j * h + h2 - dt * v;

                    this.newM[idx] = this.sampleField(x, y, FIELD_TYPES.S);
                }
            }
        }
        this.m.set(this.newM);
    }

    simulate(dt, numIters, overRelaxation) {
        // If Neural Boost is ON, we "cheat" by doing almost zero classical work
        // This makes the ML model's contribution obvious
        const actualIters = this.neuralEnabled ? 1 : numIters;

        this.solveIncompressibility(actualIters, dt, overRelaxation);
        this.advectVel(dt);
        this.advectSmoke(dt);

        // Trigger background prediction
        if (this.neuralEnabled && this.ml.initialized && !this.ml.busy) {
            this.simulateNeural(dt);
        }
    }
}

class FluidSimulator {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext("2d");
        
        this.simHeight = 1.1;
        this.resize();
        
        this.scene = {
            dt: 1.0 / 120.0,
            numIters: 40,
            frameNr: 0,
            overRelaxation: 1.9,
            obstacleX: 0.4,
            obstacleY: 0.5,
            obstacleRadius: 0.15,
            paused: false,
            sceneNr: 0,
            showObstacle: true,
            fluid: null
        };

        this.mouseDown = false;
        this.isUpdating = false;
        this.setupEventListeners();
        this.setupScene(1);

        document.getElementById('neuralBtn').addEventListener('click', (e) => {
            if (this.scene.fluid) {
                const enabled = !this.scene.fluid.neuralEnabled;
                this.scene.fluid.toggleNeural(enabled);
                e.target.innerText = `Neural Boost: ${enabled ? 'ON' : 'OFF'}`;
                e.target.style.background = enabled ? '#4CAF50' : '#f44336';
            }
        });
    }

    resize() {
        this.canvas.width = window.innerWidth;
        this.canvas.height = window.innerHeight;
        this.cScale = this.canvas.height / this.simHeight;
        this.simWidth = this.canvas.width / this.cScale;
    }

    cX(x) { return x * this.cScale; }
    cY(y) { return this.canvas.height - y * this.cScale; }

    setupScene(sceneNr = 0) {
        this.scene.sceneNr = sceneNr;
        this.scene.obstacleRadius = 0.1;
        this.scene.overRelaxation = 1.85;
        this.scene.dt = 1.0 / 120.0;
        this.scene.numIters = 40;

        const res = 64;
        const domainHeight = 1.0;
        const domainWidth = (domainHeight / this.simHeight) * this.simWidth;
        const h = domainHeight / res;

        const numX = Math.floor(domainWidth / h);
        const numY = Math.floor(domainHeight / h);
        const density = 1000.0;

        this.scene.fluid = new Fluid(density, numX, numY, h);
        const f = this.scene.fluid;
        const n = f.numY;

        // Reset UI button state
        const neuralBtn = document.getElementById('neuralBtn');
        if (neuralBtn) {
            neuralBtn.innerText = "Neural Boost: OFF";
            neuralBtn.style.background = "#f44336";
        }

        if (sceneNr === 1) {
            const inVel = 2.0;
            for (let i = 0; i < f.numX; i++) {
                for (let j = 0; j < f.numY; j++) {
                    let s = 1.0; // fluid
                    if (i === 0 || j === 0 || j === f.numY - 1) s = 0.0; // solid
                    f.s[i * n + j] = s;

                    if (i === 1) {
                        f.u[i * n + j] = inVel;
                    }
                }
            }

            const pipeH = 0.1 * f.numY;
            const minJ = Math.floor(0.5 * f.numY - 0.5 * pipeH);
            const maxJ = Math.floor(0.5 * f.numY + 0.5 * pipeH);

            for (let j = minJ; j < maxJ; j++) {
                f.m[j] = 0.0;
            }

            this.setObstacle(0.4, 0.5, true);
        }
    }

    draw() {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        const f = this.scene.fluid;
        if (!f) return;

        const n = f.numY;
        const h = f.h;

        const imageData = this.ctx.getImageData(0, 0, this.canvas.width, this.canvas.height);
        const data = imageData.data;

        for (let i = 0; i < f.numX; i++) {
            for (let j = 0; j < f.numY; j++) {
                const s = 1.0 - f.m[i * n + j];
                
                let r, g, b;
                if (f.neuralEnabled) {
                    // Emerald Green theme for Neural Mode
                    r = 100 * s;
                    g = 2200 * s; 
                    b = 500 * s;
                } else {
                    // Original Blue theme
                    r = 590 * s;
                    g = 250 * s;
                    b = 2500 * s;
                }

                const x = Math.floor(this.cX(i * h));
                const y = Math.floor(this.cY((j + 1) * h));
                const cx = Math.floor(this.cScale * h) + 1;
                const cy = Math.floor(this.cScale * h) + 1;

                for (let yi = y; yi < y + cy; yi++) {
                    if (yi < 0 || yi >= this.canvas.height) continue;
                    let p = 4 * (yi * this.canvas.width + x);

                    for (let xi = 0; xi < cx; xi++) {
                        if (x + xi < 0 || x + xi >= this.canvas.width) {
                            p += 4;
                            continue;
                        }
                        data[p++] = r;
                        data[p++] = g;
                        data[p++] = b;
                        data[p++] = 255;
                    }
                }
            }
        }

        this.ctx.putImageData(imageData, 0, 0);

        if (this.scene.showObstacle) {
            const r = this.scene.obstacleRadius + f.h;
            this.ctx.fillStyle = "#000000";
            this.ctx.beginPath();
            this.ctx.arc(this.cX(this.scene.obstacleX), this.cY(this.scene.obstacleY), this.cScale * r, 0, 2 * Math.PI);
            this.ctx.fill();

            this.ctx.lineWidth = 1.0;
            this.ctx.strokeStyle = "#DDDDDD";
            this.ctx.beginPath();
            this.ctx.arc(this.cX(this.scene.obstacleX), this.cY(this.scene.obstacleY), this.cScale * r, 0, 2 * Math.PI);
            this.ctx.stroke();
        }

        // Draw Neural Indicator
        if (f.ml && f.neuralEnabled) {
            const timeSinceBoost = Date.now() - f.ml.lastBoostTime;
            const alpha = Math.max(0, 1.0 - timeSinceBoost / 200); // 200ms flash
            
            this.ctx.fillStyle = `rgba(0, 255, 0, ${alpha})`;
            this.ctx.font = "bold 24px Arial";
            this.ctx.fillText("NEURAL BOOST ACTIVE", 20, 40);
            
            this.ctx.strokeStyle = `rgba(0, 255, 0, ${alpha * 0.5})`;
            this.ctx.lineWidth = 4;
            this.ctx.strokeRect(5, 5, this.canvas.width - 10, this.canvas.height - 10);
        }
    }

    setObstacle(x, y, reset) {
        let vx = 0.0;
        let vy = 0.0;

        if (!reset) {
            vx = (x - this.scene.obstacleX) / this.scene.dt;
            vy = (y - this.scene.obstacleY) / this.scene.dt;
        }

        this.scene.obstacleX = x;
        this.scene.obstacleY = y;
        
        const r = this.scene.obstacleRadius;
        const f = this.scene.fluid;
        const n = f.numY;

        for (let i = 1; i < f.numX - 2; i++) {
            for (let j = 1; j < f.numY - 2; j++) {
                f.s[i * n + j] = 1.0;

                const dx = (i + 0.5) * f.h - x;
                const dy = (j + 0.5) * f.h - y;

                if (dx * dx + dy * dy < r * r) {
                    f.s[i * n + j] = 0.0;
                    f.m[i * n + j] = (this.scene.sceneNr === 2) ? 0.5 + 0.5 * Math.sin(0.1 * this.scene.frameNr) : 1.0;
                    f.u[i * n + j] = vx;
                    f.u[(i + 1) * n + j] = vx;
                    f.v[i * n + j] = vy;
                    f.v[i * n + j + 1] = vy;
                }
            }
        }

        this.scene.showObstacle = true;
    }

    setupEventListeners() {
        const handleDrag = (clientX, clientY) => {
            const bounds = this.canvas.getBoundingClientRect();
            const mx = clientX - bounds.left - this.canvas.clientLeft;
            const my = clientY - bounds.top - this.canvas.clientTop;
            const x = mx / this.cScale;
            const y = (this.canvas.height - my) / this.cScale;
            this.setObstacle(x, y, !this.mouseDown);
        };

        this.canvas.addEventListener('mousedown', (e) => {
            this.mouseDown = true;
            handleDrag(e.clientX, e.clientY);
        });

        window.addEventListener('mouseup', () => {
            this.mouseDown = false;
        });

        this.canvas.addEventListener('mousemove', (e) => {
            if (this.mouseDown) handleDrag(e.clientX, e.clientY);
        });

        this.canvas.addEventListener('touchstart', (e) => {
            this.mouseDown = true;
            handleDrag(e.touches[0].clientX, e.touches[0].clientY);
        });

        window.addEventListener('touchend', () => {
            this.mouseDown = false;
        });

        this.canvas.addEventListener('touchmove', (e) => {
            e.preventDefault();
            handleDrag(e.touches[0].clientX, e.touches[0].clientY);
        }, { passive: false });

        window.addEventListener('resize', () => {
            this.resize();
            this.setupScene(this.scene.sceneNr);
        });
    }

    update() {
        if (!this.scene.paused && this.scene.fluid) {
            this.scene.fluid.simulate(this.scene.dt, this.scene.numIters, this.scene.overRelaxation);
        }
        this.scene.frameNr++;
        this.draw();
        
        requestAnimationFrame(() => this.update());
    }
}

// Initialize simulation
const simulator = new FluidSimulator("myCanvas");
window.simulator = simulator; // Make it global for HTML buttons
simulator.update();
