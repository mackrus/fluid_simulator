
var canvas = document.getElementById("myCanvas");
var c = canvas.getContext("2d");
canvas.width = window.innerWidth;
canvas.height = window.innerHeight;

canvas.focus();

var simHeight = 1.01;
var cScale = canvas.height / simHeight;
var simWidth = canvas.width / cScale;

var U_FIELD = 0;
var V_FIELD = 1;
var S_FIELD = 2;

var cnt = 0;

function cX(x) {
    return x * cScale;
}

function cY(y) {
    return canvas.height - y * cScale;
}

// ----------------- start of simulator ------------------------------

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
        this.m.fill(1.0)
        var num = numX * numY;
    }


    solveIncompressibility(numIters, dt) {

        var n = this.numY;
        var cp = this.density * this.h / dt;

        for (var iter = 0; iter < numIters; iter++) {

            for (var i = 1; i < this.numX - 1; i++) {
                for (var j = 1; j < this.numY - 1; j++) {

                    if (this.s[i * n + j] == 0.0)
                        continue;

                    var s = this.s[i * n + j];
                    var sx0 = this.s[(i - 1) * n + j];
                    var sx1 = this.s[(i + 1) * n + j];
                    var sy0 = this.s[i * n + j - 1];
                    var sy1 = this.s[i * n + j + 1];
                    var s = sx0 + sx1 + sy0 + sy1;
                    if (s == 0.0)
                        continue;

                    var div = this.u[(i + 1) * n + j] - this.u[i * n + j] +
                        this.v[i * n + j + 1] - this.v[i * n + j];

                    var p = -div / s;
                    p *= scene.overRelaxation;
                    this.p[i * n + j] += cp * p;

                    this.u[i * n + j] -= sx0 * p;
                    this.u[(i + 1) * n + j] += sx1 * p;
                    this.v[i * n + j] -= sy0 * p;
                    this.v[i * n + j + 1] += sy1 * p;
                }
            }
        }
    }


    sampleField(x, y, field) {
        var n = this.numY;
        var h = this.h;
        var h1 = 1.0 / h;
        var h2 = 0.5 * h;

        x = Math.max(Math.min(x, this.numX * h), h);
        y = Math.max(Math.min(y, this.numY * h), h);

        var dx = 0.0;
        var dy = 0.0;

        var f;

        switch (field) {
            case U_FIELD: f = this.u; dy = h2; break;
            case V_FIELD: f = this.v; dx = h2; break;
            case S_FIELD: f = this.m; dx = h2; dy = h2; break

        }


        var x0 = Math.min(Math.floor((x - dx) * h1), this.numX - 1);
        var tx = ((x - dx) - x0 * h) * h1;
        var x1 = Math.min(x0 + 1, this.numX - 1);

        var y0 = Math.min(Math.floor((y - dy) * h1), this.numY - 1);
        var ty = ((y - dy) - y0 * h) * h1;
        var y1 = Math.min(y0 + 1, this.numY - 1);

        var sx = 1.0 - tx;
        var sy = 1.0 - ty;

        var val = sx * sy * f[x0 * n + y0] +
            tx * sy * f[x1 * n + y0] +
            tx * ty * f[x1 * n + y1] +
            sx * ty * f[x0 * n + y1];

        return val;
    }

    avgU(i, j) {
        var n = this.numY;
        var u = (this.u[i * n + j - 1] + this.u[i * n + j] +
            this.u[(i + 1) * n + j - 1] + this.u[(i + 1) * n + j]) * 0.25;
        return u;

    }

    avgV(i, j) {
        var n = this.numY;
        var v = (this.v[(i - 1) * n + j] + this.v[i * n + j] +
            this.v[(i - 1) * n + j + 1] + this.v[i * n + j + 1]) * 0.25;
        return v;
    }

    advectVel(dt) {

        this.newU.set(this.u);
        this.newV.set(this.v);

        var n = this.numY;
        var h = this.h;
        var h2 = 0.5 * h;

        for (var i = 1; i < this.numX; i++) {
            for (var j = 1; j < this.numY; j++) {

                cnt++;

                // u component
                if (this.s[i * n + j] != 0.0 && this.s[(i - 1) * n + j] != 0.0 && j < this.numY - 1) {
                    var x = i * h;
                    var y = j * h + h2;
                    var u = this.u[i * n + j];
                    var v = this.avgV(i, j);
                    //						var v = this.sampleField(x,y, V_FIELD);
                    x = x - dt * u;
                    y = y - dt * v;
                    u = this.sampleField(x, y, U_FIELD);
                    this.newU[i * n + j] = u;
                }
                // v component
                if (this.s[i * n + j] != 0.0 && this.s[i * n + j - 1] != 0.0 && i < this.numX - 1) {
                    var x = i * h + h2;
                    var y = j * h;
                    var u = this.avgU(i, j);
                    //						var u = this.sampleField(x,y, U_FIELD);
                    var v = this.v[i * n + j];
                    x = x - dt * u;
                    y = y - dt * v;
                    v = this.sampleField(x, y, V_FIELD);
                    this.newV[i * n + j] = v;
                }
            }
        }

        this.u.set(this.newU);
        this.v.set(this.newV);
    }

    advectSmoke(dt) {

        this.newM.set(this.m);

        var n = this.numY;
        var h = this.h;
        var h2 = 0.5 * h;

        for (var i = 1; i < this.numX - 1; i++) {
            for (var j = 1; j < this.numY - 1; j++) {

                if (this.s[i * n + j] != 0.0) {
                    var u = (this.u[i * n + j] + this.u[(i + 1) * n + j]) * 0.5;
                    var v = (this.v[i * n + j] + this.v[i * n + j + 1]) * 0.5;
                    var x = i * h + h2 - dt * u;
                    var y = j * h + h2 - dt * v;

                    this.newM[i * n + j] = this.sampleField(x, y, S_FIELD);
                }
            }
        }
        this.m.set(this.newM);
    }

    // ----------------- end of simulator ------------------------------


    simulate(dt, numIters) {

        this.solveIncompressibility(numIters, dt);
        this.advectVel(dt);
        this.advectSmoke(dt);
    }
}

var scene =
{
    dt: 1.0 / 120.0,
    numIters: 100,
    frameNr: 0,
    overRelaxation: 1.9,
    obstacleX: 2.0,
    obstacleY: 0.0,
    obstacleRadius: 0.15,
    paused: false,
    sceneNr: 0,
    fluid: null
};



function setupScene(sceneNr = 0) {
    scene.sceneNr = sceneNr;
    scene.obstacleRadius = 0.1;
    scene.overRelaxation = 1.85;

    scene.dt = 1.0 / 120.0;
    scene.numIters = 20;

    var res = 210;

    var domainHeight = 1.0;
    var domainWidth = domainHeight / simHeight * simWidth;
    var h = domainHeight / res;

    var numX = Math.floor(domainWidth / h);
    var numY = Math.floor(domainHeight / h);

    var density = 1000.0;


    var f = scene.fluid = new Fluid(density, numX, numY, h);


    var n = f.numY;


    if (sceneNr == 1) { // vortex shedding

        var inVel = 4.0;
        for (var i = 0; i < f.numX; i++) {
            for (var j = 0; j < f.numY; j++) {
                var s = 1.9;	// fluid
                if (i == 0 || j == 0 || j == f.numY - 1)
                    s = 0.001;	// solid
                f.s[i * n + j] = s

                if (i == 1) {
                    f.u[i * n + j] = inVel;
                }
            }
        }

        var pipeH = 0.1 * f.numY;
        var minJ = Math.floor(0.5 * f.numY - 0.5 * pipeH);
        var maxJ = Math.floor(0.5 * f.numY + 0.5 * pipeH);

        for (var j = minJ; j < maxJ; j++)
            f.m[j] = 0.0;

        setObstacle(0.4, 0.5, true)
    }

}


// draw -------------------------------------------------------


function draw() {
    c.clearRect(0, 0, canvas.width, canvas.height);

    c.fillStyle = "#000000";
    var f = scene.fluid;
    var n = f.numY;

    var cellScale = 1;

    var h = f.h;

    var minP = f.p[0];
    var maxP = f.p[0];

    for (var i = 0; i < f.numCells; i++) {
        minP = Math.min(minP, f.p[i]);
        maxP = Math.max(maxP, f.p[i]);
    }

    var id = c.getImageData(0, 0, canvas.width, canvas.height)

    var color = [0, 0, 0, 0]

    for (var i = 0; i < f.numX; i++) {
        for (var j = 0; j < f.numY; j++) {


            var s = 1 - f.m[i * n + j];
            color[0] = 590 * s;
            color[1] = 250 * s;
            color[2] = 2500 * s;

            var x = Math.floor(cX(i * h));
            var y = Math.floor(cY((j + 1) * h));
            var cx = Math.floor(cScale * cellScale * h) + 1;
            var cy = Math.floor(cScale * cellScale * h) + 1;

            var r = color[0];
            var g = color[1];
            var b = color[2];

            for (var yi = y; yi < y + cy; yi++) {
                var p = 4 * (yi * canvas.width + x)

                for (var xi = 0; xi < cx; xi++) {
                    id.data[p++] = r;
                    id.data[p++] = g;
                    id.data[p++] = b;
                    id.data[p++] = 100;
                }
            }
        }
    }

    c.putImageData(id, 0, 0);

    if (scene.showObstacle) {

        c.strokeW
        r = scene.obstacleRadius + f.h;
        c.fillStyle = "#000000";  // Hex color for circle
        c.beginPath();
        c.arc(
            cX(scene.obstacleX), cY(scene.obstacleY), cScale * r, 0.0, 2.0 * Math.PI);
        c.closePath();
        c.fill();

        c.lineWidth = 0.4;
        c.strokeStyle = "#DDDDDD";
        c.beginPath();
        c.arc(
            cX(scene.obstacleX), cY(scene.obstacleY), cScale * r, 0.0, 2.0 * Math.PI);
        c.closePath();
        c.stroke();
        c.lineWidth = 0.05;
    }


}

function setObstacle(x, y, reset) {

    var vx = 0.0;
    var vy = 0.0;

    if (!reset) {
        vx = (x - scene.obstacleX) / scene.dt;
        vy = (y - scene.obstacleY) / scene.dt;
    }

    scene.obstacleX = x;
    scene.obstacleY = y;
    var r = scene.obstacleRadius;
    var f = scene.fluid;
    var n = f.numY;
    var cd = Math.sqrt(2) * f.h;

    for (var i = 1; i < f.numX - 2; i++) {
        for (var j = 1; j < f.numY - 2; j++) {

            f.s[i * n + j] = 1.0;

            var dx = (i + 0.5) * f.h - x;
            var dy = (j + 0.5) * f.h - y;


            if (dx * dx + dy * dy < r * r) {
                f.s[i * n + j] = 0.0;
                if (scene.sceneNr == 2)
                    f.m[i * n + j] = 0.5 + 0.5 * Math.sin(0.1 * scene.frameNr)
                else
                    f.m[i * n + j] = 1.0;
                f.u[i * n + j] = vx;
                f.u[(i + 1) * n + j] = vx;
                f.v[i * n + j] = vy;
                f.v[i * n + j + 1] = vy;
            }
        }
    }

    scene.showObstacle = true;
}

// interaction -------------------------------------------------------

var mouseDown = false;

function startDrag(x, y) {
    let bounds = canvas.getBoundingClientRect();

    let mx = x - bounds.left - canvas.clientLeft;
    let my = y - bounds.top - canvas.clientTop;
    mouseDown = true;

    x = mx / cScale;
    y = (canvas.height - my) / cScale;

    setObstacle(x, y, true);
}

function drag(x, y) {
    if (mouseDown) {
        let bounds = canvas.getBoundingClientRect();
        let mx = x - bounds.left - canvas.clientLeft;
        let my = y - bounds.top - canvas.clientTop;
        x = mx / cScale;
        y = (canvas.height - my) / cScale;
        setObstacle(x, y, false);
    }
}

function endDrag() {
    mouseDown = false;
}

canvas.addEventListener('mousedown', event => {
    startDrag(event.x, event.y);
});

canvas.addEventListener('mouseup', event => {
    endDrag();
});

canvas.addEventListener('mousemove', event => {
    drag(event.x, event.y);
});

canvas.addEventListener('touchstart', event => {
    startDrag(event.touches[0].clientX, event.touches[0].clientY)
});

canvas.addEventListener('touchend', event => {
    endDrag()
});

canvas.addEventListener('touchmove', event => {
    event.preventDefault();
    event.stopImmediatePropagation();
    drag(event.touches[0].clientX, event.touches[0].clientY)
}, { passive: false });


// main -------------------------------------------------------

function simulate() {
    if (!scene.paused)
        scene.fluid.simulate(scene.dt, scene.numIters)
    scene.frameNr++;
}

function update() {
    simulate();
    draw();
    requestAnimationFrame(update);
}

setupScene(1);
update();
