# ai — landing-pad detector

Training, evaluation, export and on-drone inference for the **landing-pad detector**
used by the automated delivery mission. The deployed model is a single-class YOLO11n
(`landingPad`, 320 px) running on the Sony IMX500 of the Raspberry Pi AI Camera.

Nothing in this folder runs in flight. It is the laptop-side pipeline that produced
`models/network.rpk`, kept here so that the model the aircraft loads can be traced back
to the data and settings that made it.

> **The documentation lives in the project-docs site**, not here:
> **<https://ki-drohnen-ss26.github.io/project-docs/landing-pad/>**
>
> That is where the dataset, the training runs, the robustness measurements and the
> contract with the flight code are written up. This README covers only how to *run*
> what is in this folder.

## Companion repositories

| Where | Role |
|---|---|
| [`project-docs`](https://github.com/ki-drohnen-ss26/project-docs) | all documentation, including this detector's |
| `../camera.py` (`RealCamera`) | the flight code that consumes the detections |
| `../camera_test.py` | the flight repo's own standalone camera check |

## What is in here

### Data preparation

| Script | Purpose |
|---|---|
| `build_dataset.py` | re-split the Roboflow export into contiguous blocks (no temporal leakage) — 175 train / 25 val / 16 test |
| `negatives.py` | harvest pad-free crops as hard negatives (76 → 30 % background) |
| `synth.py` | composite distant pads onto real floor (run E) |
| `build_2class.py` | merge pad + recovered person data (runs G/H) |
| `fpv_aug.py` | motion blur / noise / JPEG augmentation |

### Training

| Script | Run |
|---|---|
| `train.py` | A (old recipe), B and C (drone recipe @ 320 / 416) |
| `train_fpv.py` | D — drone recipe + capture-path degradation |
| `train_synth.py` | E — as D, on the composited set |
| `train_neg.py` | **F — as D plus hard negatives. This is the deployed model.** |
| `train_2class.py` | G / H — single two-class net for the AI Camera |

### Measurement

| Script | Measures |
|---|---|
| `compare.py` | mAP and recall by pad size and by scene |
| `robustness.py` | yaw, altitude and capture-path stress probes |
| `fp_bench.py` | false positives on pad-free images vs recall |
| `person_bench.py` | two-class person recall vs stock YOLO11n |
| `pattern_check.py` | red-X verification (measured: does not work in a sports hall) |
| `pad_pose.py` | contour, centre and yaw from a confirmed detection |

Summary outputs are tracked: `cmp_final.txt`, `rob_final.txt`, `rob_F.txt`, `rob_H.txt`,
`rob_imx.txt`, `fp_imx.txt`. The full per-run dumps under `runs/` and `eval/` are not.

### Export

| Script | Target |
|---|---|
| `export.py` | TFLite INT8, with the quantisation loss measured rather than assumed |
| `export_imx.py` | IMX500 steps 1–2 (Linux) |
| `export_imx_local.py` + `run_imx_local.sh` | the same steps on macOS, ~3.5 min |
| `make_imx_bundle.py` + `imx_colab.ipynb` | bundle and ready-to-run Colab cell for the cloud route |
| `.github/workflows/imx500-rpk.yml` | the ARM-only packaging step on a free `ubuntu-24.04-arm` runner |

### On the drone / on a PC

| Script | Runs where |
|---|---|
| `detect_pad.py` | Pi + AI Camera, via `picamera2` / `IMX500` (loads `models/network.rpk`). `--stream` serves the annotated frames at `http://<pi>:8000/`, which is the only way to see the boxes over SSH |
| `pi_aicam.py` + `live_imx.sh` | Pi + AI Camera, via Sony `modlib` (loads `packerOut.zip`) |
| `drone_pi.py` | Pi CPU, two TFLite detectors per frame |
| `test_pi_inference.py` | checks the TFLite path against ground truth |
| `live.py` | webcam / video / single image on a PC |

### Deployable artefacts

| File | For |
|---|---|
| `models/network.rpk` | the IMX500 sensor — **this is what the aircraft loads** |
| `models/packerOut.zip` | the same model for the `modlib` route |
| `models/labels.txt` | one line, `landingPad` |
| `calib.yaml` | calibration set descriptor for the IMX quantisation |

## Reproducing

```bash
python3 build_dataset.py                     # leak-free split
python3 negatives.py                         # hard negatives
python3 train_neg.py                         # run F — the deployed model
python3 compare.py    runs/*/weights/best.pt
python3 robustness.py runs/*/weights/best.pt
python3 fp_bench.py   runs/*/weights/best.pt
python3 export.py     runs/F_neg_fpv_320/weights/best.pt
./run_imx_local.sh                           # -> packerOut.zip
```

The Roboflow export is **not** in this folder — `build_dataset.py` expects it
alongside. Datasets, training runs and the large regenerable model files are not tracked.

## Environments

Three, and they cannot be merged:

| Environment | For | Why separate |
|---|---|---|
| Ultralytics 8.4.50 + torch 2.11 | runs A–F | the baseline everything was measured on |
| Ultralytics 8.4.90 + torch 2.12.1 | runs G/H, `export.py`, `test_pi_inference.py` | 8.4.50 hits an MPS assigner bug on many-object images; TFLite export needs TensorFlow |
| a virtualenv with **no TensorFlow** | `export_imx_local.py` | Sony's converter pins `protobuf==4.25.5`, TensorFlow needs 5.x, and MCT imports TF merely because it is installed |

Training ran on Apple MPS at ~4 s/epoch (320 px, 175 images); set `device="cpu"` if
MPS is unavailable.

## Deployment settings

The model measures best at **conf 0.4** on TFLite and **conf 0.3** on the quantised
`.rpk`. Do not run it away from its 320 px training resolution — false positives rise
from 0.00 to 1.12 per image at 416.

The flight-code settings that still have to be changed, and the bench check that
verifies them, are documented here:
<https://ki-drohnen-ss26.github.io/project-docs/landing-pad/integration/>

## Superseded

`Untitled8.ipynb`, `export_for_pi.py` and the `Landing_Pad_Person_Detection*.ipynb`
notebooks were the original ad-hoc training and export. Everything they did is
reproducible from the scripts above; they are not included here.
