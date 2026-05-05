const canvas = document.getElementById("mapCanvas");
const ctx = canvas.getContext("2d");

const img = new Image();
img.src = "/static/floor.png";

// scales the image width/height appropriately as well as waypoint locations.
SCALE = 0.5
let navState = "idle";

img.onload = () => {
    canvas.width = img.width * SCALE;
    canvas.height = img.height * SCALE;
    ctx.drawImage(img, 0, 0, img.width, img.height,0, 0, canvas.width, canvas.height);
};

function handleGo() {
	if (navState === "idle") {
		startNavigation();
	} else {
		togglePause();
	}
}

setInterval(() => {
    fetch('/status')
    .then(res => res.json())
    .then(data => {
        if (data.state !== navState) {
            navState = data.state;
            updateUI();
        }

	if (data.state === "arrived") {
	    navState = "arrived";
	    updateUI();
	}
    });
}, 1000);


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
            drawPath(data.waypoints);
		alert("data success test");

           
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

function drawPath(points) {
    const scaledWidth = canvas.width;
    const scaledHeight = canvas.height;

    // redraw map (scaled properly)
    ctx.drawImage(
        img,
        0, 0, img.width, img.height,
        0, 0, scaledWidth, scaledHeight
    );

    ctx.beginPath();
    ctx.strokeStyle = "red";
    ctx.lineWidth = 3;

    for (let i = 0; i < points.length; i++) {
        let x = points[i][0] * SCALE;
        let y = points[i][1] * SCALE;

        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    }

    ctx.stroke();
}

function updateUI() {
    const btn = document.getElementById("goBtn");
    const input = document.getElementById("roomInput");

    if (navState === "idle") {
        btn.innerText = "Go";
        input.disabled = false;
    }

    else if (navState === "navigating") {
        btn.innerText = "Pause";
        input.disabled = true;
    }

    else if (navState === "paused") {
        btn.innerText = "Resume";
        input.disabled = true;
    }

    else if (navState === "arrived") {
        btn.innerText = "Go";
        input.disabled = false;

        // optional: clear path or notify user
        alert("Arrived at destination");
    }
}
