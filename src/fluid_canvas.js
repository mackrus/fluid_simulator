// inititation function to set up the canvas element

function init() {

    const canvas = document.getElementById("myCanvas");
    const c = canvas.getContext("2d");
    canvas.width = window.innerWidth - 20;
    canvas.height = window.innerHeight - 100;


    canvas.focus();

    var simHeight = 1.1;
    var cScale = canvas.height / simHeight;
    var simWidth = canvas.width / cScale;

    const U_FIELD = 0;
    const V_FIELD = 1;
    const S_FIELD = 2;

    const cnt = 0;


    function cX(x) {
        return x * cScale;
    }

    function cY(y) {
        return canvas.height - y * cScale;
    }

    // If cX and cY need to be accessible outside init, consider returning them:
    return { cX, cY };

};



