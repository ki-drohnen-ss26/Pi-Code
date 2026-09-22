"""
Drone landing-pad + person detection for Raspberry Pi Zero 2 W (TFLite, no torch).

Corrected rewrite of ~/Downloads/drone_pi.py. Four things in that version cost
accuracy or would break on a re-exported model:

  1. Aspect ratio. It did cv2.resize(frame, (S, S)), squashing a 640x480 frame
     into a square. Training and validation letterbox instead, so the network
     saw a pad 25% narrower than anything it was trained on. Fixed by
     letterboxing here and undoing the letterbox when boxes are mapped back.

  2. The person model was fed the *pad* model's input tensor
     (preprocess(frame, pad_inp) reused for both). It happens to work only
     because both current exports share a shape and dtype; the moment either is
     re-exported at a different size or with int8 I/O it silently misfeeds.
     Each model is now preprocessed against its own input spec.

  3. IMG_SIZE was a module constant used to rescale boxes while the input size
     was read from the tensor, so the two disagree for any export that is not
     320. Size now comes from the model in both places.

  4. decode_yolo looped over all 2100 candidate rows in Python, twice per
     frame. On a Pi Zero 2 W that is the most expensive part of the pipeline
     after the model itself. Now vectorised in numpy.

Also handles int8 input/output quantisation, so both `*_int8.tflite`
(float I/O, quantised weights) and `*_full_integer_quant.tflite` work.

Usage:
    python3 drone_pi.py
    python3 drone_pi.py --save output.avi
    python3 drone_pi.py --stream          # MJPEG at http://<pi-ip>:8080
"""

import argparse
import time

import cv2
import numpy as np

try:
    import tflite_runtime.interpreter as tflite
except ImportError:                                    # dev machine / newer Pi OS
    from ai_edge_litert import interpreter as tflite

PAD_MODEL = "pad_320_int8.tflite"      # run D, exported by export.py
PERSON_MODEL = "yolo11n_int8.tflite"   # stock COCO YOLO11n
CONF = 0.4
IOU_NMS = 0.45
SKIP_FRAMES = 2          # run inference every Nth frame
PERSON_CLASS = 0         # COCO 'person'


class Model:
    """One TFLite detector: owns its own preprocessing and output decoding."""

    def __init__(self, path, num_threads=4):
        self.interp = tflite.Interpreter(model_path=path, num_threads=num_threads)
        self.interp.allocate_tensors()
        self.inp = self.interp.get_input_details()[0]
        self.out = self.interp.get_output_details()[0]

        shape = self.inp["shape"]
        self.chw = shape[1] == 3                       # [1,3,S,S] vs [1,S,S,3]
        self.size = int(shape[2] if self.chw else shape[1])

    def _preprocess(self, frame):
        """Letterbox to SxS, keeping aspect ratio. Returns tensor, scale, pads."""
        S = self.size
        h, w = frame.shape[:2]
        r = min(S / h, S / w)
        nw, nh = round(w * r), round(h * r)
        dw, dh = (S - nw) / 2, (S - nh) / 2

        img = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
        top, left = int(round(dh - 0.1)), int(round(dw - 0.1))
        bottom, right = S - nh - top, S - nw - left
        img = cv2.copyMakeBorder(img, top, bottom, left, right,
                                 cv2.BORDER_CONSTANT, value=(114, 114, 114))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        if self.inp["dtype"] in (np.uint8, np.int8):
            scale, zero = self.inp["quantization"]
            x = (img / 255.0 / scale + zero) if scale else img.astype(np.float32)
            x = x.astype(self.inp["dtype"])
        else:
            x = (img / 255.0).astype(np.float32)

        if self.chw:
            x = x.transpose(2, 0, 1)
        return np.expand_dims(x, 0), r, left, top

    def __call__(self, frame, conf_thr, target_class=None):
        tensor, r, pad_x, pad_y = self._preprocess(frame)
        self.interp.set_tensor(self.inp["index"], tensor)
        self.interp.invoke()
        raw = self.interp.get_tensor(self.out["index"])

        if self.out["dtype"] in (np.uint8, np.int8):
            scale, zero = self.out["quantization"]
            raw = (raw.astype(np.float32) - zero) * (scale or 1.0)

        h, w = frame.shape[:2]
        return self._decode(raw, conf_thr, target_class, r, pad_x, pad_y, w, h)

    def _decode(self, raw, conf_thr, target_class, r, pad_x, pad_y, ow, oh):
        data = np.squeeze(raw).astype(np.float32)
        if data.ndim != 2:
            return []
        if data.shape[0] < data.shape[1]:
            data = data.T                              # -> [num_boxes, 4+nc]

        scores_all = data[:, 4:]
        if target_class is not None:
            cls = np.full(len(data), target_class)
            conf = scores_all[:, target_class]
        else:
            cls = scores_all.argmax(1)
            conf = scores_all[np.arange(len(data)), cls]

        keep = conf >= conf_thr
        if not keep.any():
            return []
        boxes, conf, cls = data[keep, :4], conf[keep], cls[keep]

        # ultralytics TFLite exports emit xywh normalised to the letterboxed
        # square; scale to letterbox pixels, then undo padding and resize.
        if boxes.max() <= 1.5:
            boxes = boxes * self.size
        cx, cy, bw, bh = boxes.T
        x1 = (cx - bw / 2 - pad_x) / r
        y1 = (cy - bh / 2 - pad_y) / r
        x2 = (cx + bw / 2 - pad_x) / r
        y2 = (cy + bh / 2 - pad_y) / r
        x1 = np.clip(x1, 0, ow); x2 = np.clip(x2, 0, ow)
        y1 = np.clip(y1, 0, oh); y2 = np.clip(y2, 0, oh)

        wh = np.stack([x1, y1, x2 - x1, y2 - y1], 1)
        idx = cv2.dnn.NMSBoxes(wh.tolist(), conf.tolist(), conf_thr, IOU_NMS)
        if len(idx) == 0:
            return []
        idx = np.array(idx).flatten()
        return [(int(x1[i]), int(y1[i]), int(x2[i]), int(y2[i]),
                 float(conf[i]), int(cls[i])) for i in idx]


def boxes_overlap(b1, b2):
    return (max(b1[0], b2[0]) < min(b1[2], b2[2])
            and max(b1[1], b2[1]) < min(b1[3], b2[3]))


def draw(frame, boxes, color, label):
    for (x1, y1, x2, y2, conf, _) in boxes:
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, f"{label} {conf:.2f}", (x1, max(y1 - 6, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--save", default=None, help="write annotated video here")
    ap.add_argument("--stream", action="store_true", help="MJPEG on port 8080")
    ap.add_argument("--conf", type=float, default=CONF)
    args = ap.parse_args()

    print("Loading models...")
    pad = Model(PAD_MODEL)
    person = Model(PERSON_MODEL)
    print(f"  pad    {PAD_MODEL} @ {pad.size}px  {pad.inp['dtype'].__name__}")
    print(f"  person {PERSON_MODEL} @ {person.size}px  {person.inp['dtype'].__name__}")

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        raise RuntimeError("Cannot open camera")

    writer = None
    if args.save:
        writer = cv2.VideoWriter(args.save, cv2.VideoWriter_fourcc(*"XVID"),
                                 10, (640, 480))

    frame_bytes = [None]
    if args.stream:
        import threading
        from http.server import BaseHTTPRequestHandler, HTTPServer

        class StreamHandler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                self.send_response(200)
                self.send_header("Content-type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        if frame_bytes[0]:
                            self.wfile.write(
                                b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                                + frame_bytes[0] + b"\r\n")
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        threading.Thread(
            target=HTTPServer(("0.0.0.0", 8080), StreamHandler).serve_forever,
            daemon=True).start()
        print("MJPEG stream: http://<pi-ip>:8080")

    print("Running - Ctrl+C to stop")
    tick, pads, persons = 0, [], []
    t_last, fps = time.time(), 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            tick += 1

            if tick % SKIP_FRAMES == 0:
                pads = pad(frame, args.conf)
                persons = person(frame, args.conf, target_class=PERSON_CLASS)
                now = time.time()
                fps = 0.8 * fps + 0.2 / max(now - t_last, 1e-6)
                t_last = now

            draw(frame, pads, (0, 255, 0), "PAD")
            draw(frame, persons, (0, 0, 255), "PERSON")

            cv2.putText(frame,
                        f"Pads:{len(pads)}  Persons:{len(persons)}  {fps:4.1f} inf/s",
                        (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            if any(boxes_overlap(p[:4], q[:4]) for p in pads for q in persons):
                cv2.putText(frame, "!! ZONE BLOCKED !!", (8, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            if writer:
                writer.write(frame)
            if args.stream:
                _, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                frame_bytes[0] = jpg.tobytes()

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        cap.release()
        if writer:
            writer.release()


if __name__ == "__main__":
    main()
