import os, sys, glob, hashlib
import numpy as np
from PIL import Image

def parse(lf):
    """-> list of (cls, xs, ys) normalised; raises on malformed"""
    out, bad = [], []
    for i, line in enumerate(open(lf), 1):
        t = line.split()
        if not t: continue
        if len(t) < 5 or len(t) % 2 == 0:
            bad.append(f"line {i}: {len(t)} values"); continue
        try: v = [float(x) for x in t]
        except ValueError: bad.append(f"line {i}: non-numeric"); continue
        cls, coords = int(v[0]), v[1:]
        xs, ys = coords[0::2], coords[1::2]
        if len(coords) == 4:            # bbox cx cy w h
            cx, cy, w, h = coords
            xs = [cx-w/2, cx+w/2]; ys = [cy-h/2, cy+h/2]
        out.append((cls, xs, ys))
    return out, bad

def red_mask(img):
    a = np.asarray(img.convert('HSV'), dtype=np.int16)
    h, s, v = a[...,0], a[...,1], a[...,2]
    return (((h < 12) | (h > 235)) & (s > 90) & (v > 70))

def audit(root, splits):
    print(f"\n{'='*78}\n{root}\n{'='*78}")
    findings = {'malformed': [], 'empty': [], 'noimg': [], 'cls': [], 'range': [],
                'multi': [], 'geom': [], 'nored': [], 'redout': []}
    hashes = {}
    n = 0
    for sp in splits:
        for ip in sorted(glob.glob(f"{root}/{sp}/images/*")):
            if not ip.lower().endswith(('.jpg','.jpeg','.png')): continue
            n += 1
            base = os.path.splitext(os.path.basename(ip))[0]
            lp = f"{root}/{sp}/labels/{base}.txt"
            key = f"{sp}/{base}"
            if not os.path.exists(lp):
                findings['noimg'].append(key); continue
            objs, bad = parse(lp)
            if bad: findings['malformed'].append(f"{key}: {'; '.join(bad)}")
            img = Image.open(ip).convert('RGB')
            W, H = img.size
            hashes.setdefault(hashlib.md5(img.tobytes()).hexdigest(), []).append(key)
            rm = red_mask(img)
            total_red = int(rm.sum())
            if not objs:
                findings['empty'].append((key, total_red, W*H))
                continue
            if len(objs) > 1: findings['multi'].append(f"{key}: {len(objs)} Objekte")
            covered = np.zeros((H, W), bool)
            for cls, xs, ys in objs:
                if cls != 0: findings['cls'].append(f"{key}: class={cls}")
                if min(xs+ys) < -0.01 or max(xs+ys) > 1.01:
                    findings['range'].append(f"{key}: min={min(xs+ys):.3f} max={max(xs+ys):.3f}")
                x0,x1 = max(0,min(xs))*W, min(1,max(xs))*W
                y0,y1 = max(0,min(ys))*H, min(1,max(ys))*H
                bw, bh = x1-x0, y1-y0
                if bw < 2 or bh < 2:
                    findings['geom'].append(f"{key}: Box {bw:.0f}x{bh:.0f}px winzig"); continue
                ar = max(bw/bh, bh/bw)
                area = bw*bh/(W*H)
                if ar > 4.0: findings['geom'].append(f"{key}: Seitenverhaeltnis 1:{ar:.1f}")
                if area > 0.97: findings['geom'].append(f"{key}: Box deckt {area*100:.0f}% des Bildes")
                covered[int(y0):int(y1), int(x0):int(x1)] = True
                inside = int(rm[int(y0):int(y1), int(x0):int(x1)].sum())
                if inside < 0.002*bw*bh:
                    findings['nored'].append(f"{key}: nur {inside}px Rot in der Box ({inside/(bw*bh)*100:.2f}%)")
            out_red = int((rm & ~covered).sum())
            if total_red > 0 and out_red > max(400, 0.35*total_red) and out_red > 0.002*W*H:
                findings['redout'].append((key, out_red, total_red, W*H))
    dups = {k: v for k, v in hashes.items() if len(v) > 1}
    print(f"geprueft: {n} Bilder\n")
    labels = {
        'noimg':    'Bild ohne Label-Datei',
        'malformed':'kaputte Label-Zeile',
        'cls':      'falsche Klassen-ID',
        'range':    'Koordinate ausserhalb 0..1',
        'multi':    'mehr als ein Objekt markiert',
        'geom':     'unplausible Box-Geometrie',
        'nored':    'Box enthaelt (fast) kein Rot',
    }
    for k, title in labels.items():
        v = findings[k]
        print(f"[{'OK ' if not v else 'HIT'}] {title}: {len(v)}")
        for x in v[:12]: print(f"        {x}")
        if len(v) > 12: print(f"        ... und {len(v)-12} weitere")
    print(f"[{'OK ' if not findings['empty'] else 'HIT'}] leeres Label: {len(findings['empty'])}")
    for key, red, px in findings['empty'][:12]:
        print(f"        {key}: {red} rote Pixel ({red/px*100:.2f}% des Bildes)")
    print(f"[{'OK ' if not findings['redout'] else 'HIT'}] viel Rot AUSSERHALB aller Boxen: {len(findings['redout'])}")
    for key, o, t, px in sorted(findings['redout'], key=lambda r: -r[1])[:15]:
        print(f"        {key}: {o} von {t} roten Pixeln draussen ({o/px*100:.2f}% des Bildes)")
    print(f"[{'OK ' if not dups else 'HIT'}] bildgleiche Duplikate: {len(dups)} Gruppen")
    for v in list(dups.values())[:10]:
        cross = len({x.split('/')[0] for x in v}) > 1
        print(f"        {'!! SPLIT-UEBERGREIFEND ' if cross else ''}{v}")
    return findings, dups

audit('Landing-Pad-1', ['train','valid','test'])
