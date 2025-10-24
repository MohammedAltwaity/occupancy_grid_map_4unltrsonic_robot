#!/usr/bin/env python3
"""
Flask + PiCamera2 Streaming and Analysis Server
------------------------------------------------
Features:
- Live video streaming with FPS overlay
- Burst capture with top sharpest frame selection
- Face detection and extraction with margin
- Save best frames and extracted faces
- Select the most frontal (least rotated) face from current burst
- Save results with timestamp filenames
"""

from flask import Flask, Response, render_template_string, request, jsonify
from picamera2 import Picamera2
import cv2
import threading
import time
import os
import numpy as np

# ---------------------------
# CONFIGURATION
# ---------------------------
CAPTURE_BURST_COUNT = 10
CAPTURE_DURATION = 1.2
TOP_N = 5
DEFAULT_MARGIN = 0.1
HAAR_CASCADE_PATH = "./haarcascade_frontalface_default.xml"  # assume cascade is in same folder

# ---------------------------
# FOLDER SETUP
# ---------------------------
os.makedirs("captured_images", exist_ok=True)
os.makedirs("best", exist_ok=True)
os.makedirs("extracted_faces", exist_ok=True)
os.makedirs("final_result", exist_ok=True)

# ---------------------------
# FLASK APP
# ---------------------------
app = Flask(__name__)

# ---------------------------
# CAMERA SETUP
# ---------------------------
picam2 = Picamera2()
config = picam2.create_video_configuration(
    main={"size": (640, 480), "format": "XRGB8888"},
    buffer_count=2
)
picam2.configure(config)
picam2.start()

latest_frame = None
frame_lock = threading.Condition()

# ---------------------------
# HELPER FUNCTIONS
# ---------------------------
def save_frame(frame, folder="captured_images", prefix="frame"):
    """Save a frame with timestamped filename"""
    timestamp = time.strftime("%Y%m%d-%H%M%S-%f")
    filename = f"{prefix}_{timestamp}.jpg"
    path = os.path.join(folder, filename)
    cv2.imwrite(path, frame)
    return path

def image_quality(image):
    """Return image sharpness based on Laplacian variance"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return cv2.Laplacian(gray, cv2.CV_64F).var()

def extract_all_faces(image, margin=DEFAULT_MARGIN):
    """Detect and extract all faces (no rotation)"""
    if not os.path.exists(HAAR_CASCADE_PATH):
        print("❌ Haar cascade not found:", HAAR_CASCADE_PATH)
        return []

    cascade = cv2.CascadeClassifier(HAAR_CASCADE_PATH)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    faces = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    crops = []

    for (x, y, w, h) in faces:
        m_w, m_h = int(w * margin), int(h * margin)
        x1, y1 = max(x - m_w, 0), max(y - m_h, 0)
        x2, y2 = min(x + w + m_w, image.shape[1]), min(y + h + m_h, image.shape[0])
        crops.append(image[y1:y2, x1:x2])

    return crops

def symmetry_score(image):
    """Compute horizontal symmetry score (lower = more frontal)"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flipped = cv2.flip(gray, 1)
    diff = np.abs(gray - flipped)
    mask = gray > 20  # ignore dark background
    score = np.mean(diff[mask]) if np.any(mask) else np.mean(diff)
    return score

def select_most_frontal_face_from_list(face_paths, output_folder="final_result"):
    """Select the most frontal face from the given list of extracted faces"""
    if not face_paths:
        print("[INFO] No extracted faces in current burst.")
        return None

    best_path = None
    best_score = float("inf")

    for f in face_paths:
        img = cv2.imread(f)
        if img is None or img.size == 0:
            continue
        score = symmetry_score(img)
        print(f"[DEBUG] {os.path.basename(f)} → symmetry={score:.3f}")
        if score < best_score:
            best_score = score
            best_path = f

    if best_path:
        img = cv2.imread(best_path)
        timestamp = time.strftime("%Y%m%d-%H%M%S-%f")
        final_path = os.path.join(output_folder, f"best_face_{timestamp}.jpg")
        cv2.imwrite(final_path, img)
        print(f"[INFO] ✅ Most frontal face saved: {final_path}")
        return final_path
    else:
        print("[INFO] No valid frontal face found.")
        return None

def send_images_to_simulated_api(image_paths):
    """Simulated API upload"""
    print("🔹 Sending images to simulated API...")
    for p in image_paths:
        print("   ", p)
    time.sleep(2)
    return {"status": "success", "processed_images": [os.path.basename(p) for p in image_paths]}

# ---------------------------
# CAMERA THREAD
# ---------------------------
def update_camera():
    """Continuously capture frames and update global variable"""
    global latest_frame
    prev_time = time.time()
    while True:
        frame = picam2.capture_array()
        if frame is not None:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            curr_time = time.time()
            fps = 1.0 / (curr_time - prev_time) if curr_time > prev_time else 0
            prev_time = curr_time
            cv2.putText(frame, f"FPS:{fps:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
            with frame_lock:
                latest_frame = frame
                frame_lock.notify_all()
        time.sleep(0.01)

def generate_frames():
    """MJPEG generator for streaming"""
    global latest_frame
    while True:
        with frame_lock:
            if latest_frame is None:
                frame_lock.wait()
                continue
            frame = latest_frame.copy()
        _, buffer = cv2.imencode(".jpg", frame)
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" +
               buffer.tobytes() + b"\r\n")
        time.sleep(0.03)

# ---------------------------
# ROUTES
# ---------------------------
HTML_TEMPLATE = """<!DOCTYPE html>
<html>
<head>
<title>Pi Camera Stream</title>
<style>
body { font-family: 'Segoe UI'; background:#121212; color:#E0E0E0; margin:0; }
.top-bar { display:flex; justify-content:space-between; padding:20px; }
#video { border:2px solid #333; width:400px;height:300px; background:#1E1E1E; }
button { padding:10px 20px; font-size:18px; border-radius:10px; background:#3D5AFE; color:white; border:none; cursor:pointer; margin:40px; }
button:hover { background:#5C6BC0; }
.info { padding:10px 40px; }
#status { font-weight:bold; color:#00E676; margin-bottom:8px; }
#result { background:#1E1E1E; padding:10px; border-radius:6px; font-family:monospace; white-space:pre-wrap; max-width:400px; border:1px solid #333; }
</style>
</head>
<body>
<div class="top-bar">
<img id="video" src="/video_feed" alt="Video Feed">
<button onclick="capture()">Capture & Analyze</button>
</div>
<div class="info">
<p id="status">Waiting...</p>
<div><b>Result:</b></div>
<div id="result">None</div>
</div>
<script>
function capture() {
    document.getElementById("status").innerText="Capturing...";
    document.getElementById("result").innerText="Processing...";
    fetch("/", {method:"POST"})
        .then(r=>r.json())
        .then(data=>{
            document.getElementById("status").innerText="Done.";
            document.getElementById("result").innerText=JSON.stringify(data.result,null,2);
        })
        .catch(err=>{
            document.getElementById("status").innerText="Error.";
            console.error(err);
        });
}
</script>
</body>
</html>"""

@app.route("/", methods=["GET", "POST"])
def index_route():
    if request.method == "POST":
        frames = []
        interval = CAPTURE_DURATION / CAPTURE_BURST_COUNT

        # Capture burst
        for _ in range(CAPTURE_BURST_COUNT):
            frame = picam2.capture_array()
            if frame is not None:
                frames.append(cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR))
            time.sleep(interval)

        # Save raw burst frames
        for i, f in enumerate(frames):
            save_frame(f, folder="captured_images", prefix=f"frame_{i}")

        # Select top N sharpest frames
        frames_sorted = sorted(frames, key=image_quality, reverse=True)[:TOP_N]
        best_paths, extracted_face_paths = [], []

        # Extract faces
        for i, f in enumerate(frames_sorted):
            best_path = save_frame(f, folder="best", prefix=f"best_{i}")
            best_paths.append(best_path)

            faces = extract_all_faces(f, margin=DEFAULT_MARGIN)
            for j, face_crop in enumerate(faces):
                face_path = save_frame(face_crop, folder="extracted_faces", prefix=f"face_{i}_{j}")
                extracted_face_paths.append(face_path)

        # Pick most frontal face from current burst
        final_face = select_most_frontal_face_from_list(extracted_face_paths)
        api_result = send_images_to_simulated_api(best_paths)
        api_result["extracted_faces"] = [os.path.basename(p) for p in extracted_face_paths]
        api_result["most_frontal_face"] = os.path.basename(final_face) if final_face else None

        return jsonify({"result": api_result})

    return render_template_string(HTML_TEMPLATE)

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

# ---------------------------
# MAIN
# ---------------------------
if __name__ == "__main__":
    threading.Thread(target=update_camera, daemon=True).start()
    app.run(host="0.0.0.0", port=5001, threaded=True)
