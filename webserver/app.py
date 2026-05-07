from flask import Flask, render_template, request, jsonify
import navigator  # your module

app = Flask(__name__)

navigation_state = "idle"  
# idle | navigating | paused | arrived

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/navigate', methods=['POST'])
def navigate():
    global navigation_state

    data = request.json
    room = data.get('room')
    try:
        waypoints = navigator.get_waypoints(room)

        # Tell MPC to start (you implement this)
        navigator.start_navigation(waypoints)

        navigation_state = "navigating"
        
        return jsonify({
            "success": True,
            "waypoints": waypoints,
            "state": navigation_state
        })
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})

@app.route('/control', methods=['POST'])
def control():
    global navigation_state

    if navigation_state == "navigating":
        navigator.pause()
        navigation_state = "paused"

    elif navigation_state == "paused":
        navigator.resume()
        navigation_state = "navigating"

    return jsonify({"state": navigation_state})

@app.route('/status')
def status():
    global navigation_state

    # Ask MPC if done
    if navigator.is_complete():
        navigation_state = "arrived"

    return jsonify({"state": navigation_state})

@app.route('/toggle', methods=['POST'])
def toggle():
    global navigation_active
    navigation_active = not navigation_active
    return jsonify({"active": navigation_active})


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
