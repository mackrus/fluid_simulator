// refactor number 1 of main.js

class FluidField {
    constructor(canvasId, simHeight) {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas.getContext("2d");
        this.canvas.width = window.innerWidth;
        this.canvas.height = window.innerHeight;
        this.canvas.focus();

        this.simHeight = simHeight;
        this.cScale = this.canvas.height / this.simHeight;
        this.simWidth = this.canvas.width / this.cScale;

        this.scene = {
            dt: 1.0 / 120.0,
            numIters: 100,
            frameNr: 0,
            overRelaxation: 1.9,
            obstacleX: 2.0,
            obstacleY: 0.0,
            obstacleRadius: 0.15,
            paused: false,
            sceneNr: 0,
            fluid: null,
            showObstacle: false  // Added this property
        };

        this.setupScene(1);
    }

    cX(x) {
        return x * this.cScale;
    }

    cY(y) {
        return this.canvas.height - y * this.cScale;
    }

    // ... Other utility functions ...

    setupScene(sceneNr = 0) {
        this.scene.sceneNr = sceneNr;
        this.scene.obstacleRadius = 0.1;
        this.scene.overRelaxation = 1.85;

        this.scene.dt = 1.0 / 120.0;
        this.scene.numIters = 20;

        var res = 210;

        var domainHeight = 1.0;
        var domainWidth = domainHeight / this.simHeight * this.simWidth;
        var h = domainHeight / res;

        var numX = Math.floor(domainWidth / h);
        var numY = Math.floor(domainHeight / h);

        var density = 1000.0;

        var f = this.scene.fluid = new FluidField(density, numX, numY, h);

        var n = f.numY;

        if (sceneNr === 1) { // vortex shedding
            var inVel = 4.0;
            for (var i = 0; i < f.numX; i++) {
                for (var j = 0; j < f.numY; j++) {
                    var s = 1.9;	// fluid
                    if (i === 0 || j === 0 || j === f.numY - 1)
                        s = 0.001;	// solid
                    f.s[i * n + j] = s

                    if (i === 1) {
                        f.u[i * n + j] = inVel;
                    }
                }
            }

            var pipeH = 0.1 * f.numY;
            var minJ = Math.floor(0.5 * f.numY - 0.5 * pipeH);
            var maxJ = Math.floor(0.5 * f.numY + 0.5 * pipeH);

            for (var j = minJ; j < maxJ; j++)
                f.m[j] = 0.0;

            this.setObstacle(0.4, 0.5, true)
        }
    }

    // ... Other simulation-related functions ...

    draw() {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);

        this.ctx.fillStyle = "#000000";
        var f = this.scene.fluid;
        var n = f.numY;

        var cellScale = 1;

        var h = f.h;

        var minP = f.p[0];
        var maxP = f.p[0];

        for (var i = 0; i < f.numCells; i++) {
            minP = Math.min(minP, f.p[i]);
            maxP = Math.max(maxP, f.p[i]);
        }

        var id = this.ctx.getImageData(0, 0, this.canvas.width, this.canvas.height)

        var color = [0, 0, 0, 0]

        for (var i = 0; i < f.numX; i++) {
            for (var j = 0; j < f.numY; j++) {

                var s = 1 - f.m[i * n + j];
                color[0] = 590 * s;
                color[1] = 250 * s;
                color[2] = 2500 * s;

                var x = Math.floor(this.cX(i * h));
                var y = Math.floor(this.cY((j + 1) * h));
                var cx = Math.floor(this.cScale * cellScale * h) + 1;
                var cy = Math.floor(this.cScale * cellScale * h) + 1;

                var r = color[0];
                var g = color[1];
                var b = color[2];

                for (var yi = y; yi < y + cy; yi++) {
                    var p = 4 * (yi * this.canvas.width + x)

                    for (var xi = 0; xi < cx; xi++) {
                        id.data[p++] = r;
                        id.data[p++] = g;
                        id.data[p++] = b;
                        id.data[p++] = 100;
                    }
                }
            }
        }
        this.ctx.putImageData(id, 0, 0);

        if (this.scene.showObstacle) {
            this.ctx.strokeW
            r = this.scene.obstacleRadius + f.h;
            this.ctx.fillStyle = "#000000";  // Hex color for circle
            this.ctx.beginPath();
            this.ctx.arc(
                this.cX(this.scene.obstacleX), this.cY(this.scene.obstacleY), this.cScale * r, 0.0, 2.0 * Math.PI);
            this.ctx.closePath();
            this.ctx.fill();

            this.ctx.lineWidth = 0.4;
            this.ctx.strokeStyle = "#DDDDDD";
            this.ctx.beginPath();
            this.ctx.arc(
                this.cX(this.scene.obstacleX), this.cY(this.scene.obstacleY), this.cScale * r, 0.0, 2.0 * Math.PI);
            this.ctx.closePath();
            this.ctx.stroke();
            this.ctx.lineWidth = 0.05;
        }
    }

    setObstacle(x, y, reset) {
        var vx = 0.0;
        var vy = 0.0;

        if (!reset) {
            vx = (x - this.scene.obstacleX) / this.scene.dt;
            vy = (y - this.scene.obstacleY) / this.scene.dt;
        }

        this.scene.obstacleX = x;
        this.scene.obstacleY = y;
        var r = this.scene.obstacleRadius;
        var f = this.scene.fluid;
        var n = f.numY;
        var cd = Math.sqrt(2) * f.h;

        for (var i = 1; i < f.numX - 2; i++) {
            for (var j = 1; j < f.numY - 2; j++) {

                f.s[i * n + j] = 1.0;

                var dx = (i + 0.5) * f.h - x;
                var dy = (j + 0.5) * f.h - y;

                if (dx * dx + dy * dy < r * r) {
                    f.s[i * n + j] = 0.0;
                    if (this.scene.sceneNr === 2)
                        f.m[i * n + j] = 0.5 + 0.5 * Math.sin(0.1 * this.scene.frameNr)
                    else
                        f.m[i * n + j] = 1.0;
                    f.u[i * n + j] = vx;
                    f.u[(i + 1) * n + j] = vx;
                    f.v[i * n + j] = vy;
                    f.v[i * n + j + 1] = vy;
                }
            }
        }

        this.scene.showObstacle = true;
    }

    startDrag(x, y) {
        let bounds = this.canvas.getBoundingClientRect();

        let mx = x - bounds.left - this.canvas.clientLeft;
        let my = y - bounds.top - this.canvas.clientTop;
        mouseDown = true;

        x = mx / this.cScale;
        y = (this.canvas.height - my) / this.cScale;

        this.setObstacle(x, y, true);
    }

    drag(x, y) {
        if (mouseDown) {
            let bounds = this.canvas.getBoundingClientRect();
            let mx = x - bounds.left - this.canvas.clientLeft;
            let my = y - bounds.top - this.canvas.clientTop;
            x = mx / this.cScale;
            y = (this.canvas.height - my) / this.cScale;
            this.setObstacle(x, y, false);
        }
    }

    endDrag() {
        mouseDown = false;
    }

    simulate() {
        if (!this.scene.paused)
            this.scene.fluid.simulate(this.scene.dt, this.scene.numIters)
        this.scene.frameNr++;
    }

    update() {
        this.simulate();
        this.draw();
        requestAnimationFrame(() => this.update());
    }

    run() {
        this.update();
    }
}

// Usage
const fluidField = new FluidField("myCanvas", 1.01);
fluidField.run();
