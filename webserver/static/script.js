const canvas = document.getElementById("mapCanvas");
const ctx = canvas.getContext("2d");

const img = new Image();
img.src = "/static/floor.png";


const STATUS_NAVIGATING = 1;
const STATUS_RELOCALIZE_WALL_FOLLOW = 10;
const STATUS_APRIL_TAG_VISION = 20;
const STATUS_PAUSED = 2;
const STATUS_ARRIVED = 3;
const STATUS_IDLE = 0;

// scales the image width/height appropriately as well as waypoint locations.
SCALE = 0.5
let navState = STATUS_IDLE;

let currentPath = [];
let robotPosition = null;
let currentWaypointIndex = 0;

img.onload = () => {
    canvas.width = img.width * SCALE;
    canvas.height = img.height * SCALE;
    ctx.drawImage(img, 0, 0, img.width, img.height,0, 0, canvas.width, canvas.height);
};

setInterval(() => {
    fetch('/status')
    .then(res => res.json())
    .then(data => {

        navState = data.state;

        robotPosition = data.robot_position;
        currentWaypointIndex = data.current_waypoint_index;

        updateUI();
        drawScene();
    });
}, 100);

function buttonPress() {
    if (navState === STATUS_IDLE || navState === STATUS_ARRIVED ) {
        navigate();
    }

    else {
        togglePause();
    }
}


function navigate() {
    const roomInput = document.getElementById("roomInput");
    const btn = document.getElementById("goBtn");

    const room = roomInput.value;

    fetch('/navigate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ room: room })
    })
    .then(res => res.json())
    .then(data => {
    if (data.success) {

        currentPath = data.waypoints;

        drawScene();

        btn.innerText = "Pause";
        btn.onclick = togglePause;

        roomInput.disabled = true;
    }
});
}

function togglePause() {
    const btn = document.getElementById("goBtn");

    fetch('/control', { method: 'POST' })
    .then(res => res.json())
    .then(data => {
        if (data.state === "paused") {
            btn.innerText = "Resume";
        } else {
            btn.innerText = "Pause";
        }
    });
}

function toggleNav() {
    fetch('/toggle', { method: 'POST' })
    .then(res => res.json())
    .then(data => {
        document.getElementById("toggleBtn").innerText =
            data.active ? "Stop" : "Start";
    });
}

function drawScene() {

    ctx.drawImage(
        img,
        0, 0, img.width, img.height,
        0, 0, canvas.width, canvas.height
    );

    if (currentPath.length === 0)
        return;

    ctx.beginPath();
    ctx.strokeStyle = "red";
    ctx.lineWidth = 3;

    for (let i = 0; i < currentPath.length; i++) {

        let x = currentPath[i][0] * SCALE;
        let y = currentPath[i][1] * SCALE;

        if (i === 0)
            ctx.moveTo(x, y);
        else
            ctx.lineTo(x, y);
    }

    ctx.stroke();

    for (let i = 0; i < currentPath.length; i++) {

        let x = currentPath[i][0] * SCALE;
        let y = currentPath[i][1] * SCALE;

        ctx.beginPath();

        // completed waypoint = green
        if (i < currentWaypointIndex)
            ctx.fillStyle = "green";

        // current target = yellow
        else if (i === currentWaypointIndex)
            ctx.fillStyle = "yellow";

        // future waypoint = blue
        else
            ctx.fillStyle = "blue";

        ctx.arc(x, y, 6, 0, 2 * Math.PI);
        ctx.fill();
    }
    if (robotPosition) {

        let rx = robotPosition[0] * SCALE;
        let ry = robotPosition[1] * SCALE;

        ctx.beginPath();
        ctx.fillStyle = "lime";

        ctx.arc(rx, ry, 10, 0, 2 * Math.PI);

        ctx.fill();
    }
}

function updateUI() {
    const btn = document.getElementById("goBtn");
    const input = document.getElementById("roomInput");

    if (navState === STATUS_IDLE) {
        btn.innerText = "Go";
        input.disabled = false;
    }

    else if (navState === STATUS_NAVIGATING) {
        btn.innerText = "Pause";
        input.disabled = true;
    }

    else if (navState === STATUS_PAUSED) {
        btn.innerText = "Resume";
        input.disabled = true;
    }

    else if (navState === STATUS_ARRIVED) {
        btn.innerText = "Go";
        input.disabled = false;

        alert("Arrived at destination");
    }
}
