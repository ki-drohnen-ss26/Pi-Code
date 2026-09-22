"""
Landing-Pad-Erkennung auf der Raspberry Pi AI Camera.

Baut auf test_model.py auf: dort war zu sehen, dass der Sensor vier Tensoren
liefert — Boxen (300,4), Konfidenzen (300,), Klassen (300,) und die Anzahl
gueltiger Funde (1,). Das ist das "pp"-Format, NMS laeuft also schon auf dem
Sensor. Der Pi muss nur noch auslesen und umrechnen.

Zwei Details, die man falsch machen kann und die dann still falsche Boxen
ergeben — beide aus picamera2s eigenem Beispielskript uebernommen:

  Normierung   Die Boxen kommen als Pixel im 320er-Eingangsfenster, nicht
               als 0..1. Deshalb / input_h.
  Reihenfolge  YOLO gibt (x0, y0, x1, y1) aus, convert_inference_coords()
               erwartet aber (y0, x0, y1, x1). Deshalb das Umsortieren.

Schwelle 0.3 statt 0.4: die Quantisierung verschiebt die Konfidenzen nach
unten. Gemessen faellt die Erkennung ferner Pads ab 0.5 deutlich ab.

    python3 detect_pad.py
    python3 detect_pad.py --conf 0.4
    python3 detect_pad.py --preview        # mit Bild, braucht Monitor
    python3 detect_pad.py --stream         # Boxen im Browser: http://<pi>:8000/
"""

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
from picamera2 import Picamera2
from picamera2.devices.imx500 import IMX500

MODEL = "/home/drone/models/network.rpk"
LABELS = "/home/drone/models/labels.txt"
GREEN = (0, 255, 0)


def parse(imx500, picam2, metadata, conf_thr):
    """Vier Sensor-Tensoren -> Liste von Funden."""
    outputs = imx500.get_outputs(metadata, add_batch=True)
    if outputs is None:
        return []

    boxes, scores, classes = outputs[0][0], outputs[1][0], outputs[2][0]
    input_w, input_h = imx500.get_input_size()

    boxes = boxes / input_h              # Pixel im Eingangsfenster -> 0..1
    boxes = boxes[:, [1, 0, 3, 2]]       # (x0,y0,x1,y1) -> (y0,x0,y1,x1)

    frame_w, frame_h = picam2.camera_configuration()["main"]["size"]
    found = []
    for box, score, cls in zip(boxes, scores, classes):
        if score < conf_thr:
            continue
        x, y, w, h = imx500.convert_inference_coords(box, metadata, picam2)
        found.append({
            "conf": float(score),
            "cls": int(cls),
            "px": (int(x), int(y), int(w), int(h)),
            # normiert aufs Bild — das braucht der Landeregler:
            # 0.5/0.5 = mittig, groesseres w/h = naeher am Boden
            "cx": (x + w / 2) / frame_w,
            "cy": (y + h / 2) / frame_h,
            "w": w / frame_w,
            "h": h / frame_h,
        })
    return found


class StreamServer:
    """Haelt das letzte annotierte Bild als JPEG bereit und liefert es per
    MJPEG aus -- im Browser unter http://<pi>:<port>/ ansehen, waehrend
    detect_pad.py gleichzeitig die Kamera fuer die Erkennung offen haelt.
    """

    def __init__(self, port):
        self._frame = None
        self._lock = threading.Lock()
        server = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass  # keine Terminal-Flut, ein Log pro Frame waere zu viel

            def do_GET(self):
                self.send_response(200)
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        with server._lock:
                            frame = server._frame
                        if frame is not None:
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(
                                f"Content-Length: {len(frame)}\r\n\r\n".encode())
                            self.wfile.write(frame)
                            self.wfile.write(b"\r\n")
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self._httpd = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        threading.Thread(target=self._httpd.serve_forever, daemon=True).start()
        print(f"Stream: http://<pi-name-oder-ip>:{port}/\n")

    def update(self, frame_bgr):
        ok, jpg = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self._lock:
                self._frame = jpg.tobytes()


def draw_detections(frame_bgr, dets, labels):
    for d in dets:
        x, y, w, h = d["px"]
        name = labels[d["cls"]] if d["cls"] < len(labels) else d["cls"]
        cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), GREEN, 2)
        cv2.putText(frame_bgr, f"{name} {d['conf']:.2f}", (x, max(y - 8, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2)
    return frame_bgr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--conf", type=float, default=0.3)
    ap.add_argument("--fps", type=int, default=10)
    ap.add_argument("--model", default=MODEL)
    ap.add_argument("--preview", action="store_true")
    ap.add_argument("--stream", action="store_true",
                    help="Boxen im Browser sehen, waehrend das Modell laeuft")
    ap.add_argument("--stream-port", type=int, default=8000)
    args = ap.parse_args()

    try:
        labels = np.genfromtxt(LABELS, dtype=str, delimiter="\n", ndmin=1)
    except OSError:
        labels = np.array(["landingPad"])

    print("Modell laden ...")
    imx500 = IMX500(args.model)
    picam2 = Picamera2(imx500.camera_num)
    config = picam2.create_preview_configuration(
        controls={"FrameRate": args.fps}, buffer_count=12)
    imx500.show_network_fw_progress_bar()
    picam2.start(config, show_preview=args.preview)

    streamer = StreamServer(args.stream_port) if args.stream else None

    print(f"Eingangsgroesse: {imx500.get_input_size()}")
    print(f"Schwelle: {args.conf}   Strg+C zum Beenden\n")

    last_seen, frames = 0.0, 0
    try:
        while True:
            # Ein Request statt capture_metadata(), damit bei --stream auch
            # das Bild greifbar ist, ohne die Kamera ein zweites Mal zu fragen.
            request = picam2.capture_request()
            metadata = request.get_metadata()
            frames += 1
            dets = parse(imx500, picam2, metadata, args.conf)

            if streamer:
                # XBGR8888 -> BGR: die ersten drei Kanaele sind schon BGR,
                # der vierte (X) ist Fuellbyte.
                array = request.make_array("main")[:, :, :3].copy()
                streamer.update(draw_detections(array, dets, labels))
            request.release()

            if dets:
                last_seen = time.time()
                for d in dets:
                    name = labels[d["cls"]] if d["cls"] < len(labels) else d["cls"]
                    print(f"{name}  Mitte {d['cx']:.3f},{d['cy']:.3f}  "
                          f"Groesse {d['w']:.3f}x{d['h']:.3f}  "
                          f"conf {d['conf']:.2f}  px={d['px']}")
            elif frames % 30 == 0:
                gap = time.time() - last_seen if last_seen else -1
                print(f"kein Pad  (zuletzt vor {gap:.0f}s)" if gap >= 0
                      else "kein Pad")
    except KeyboardInterrupt:
        print("\nbeendet.")
    finally:
        picam2.stop()


if __name__ == "__main__":
    main()
