import numpy as np
from PIL import Image
import math

def uism(img):
    gray = 0.299*img[...,0] + 0.587*img[...,1] + 0.114*img[...,2]
    k1, k2 = 8, 8
    h, w = gray.shape
    eme = 0.0
    cnt = 0
    for r in range(0, h - k1, k1):
        for c in range(0, w - k2, k2):
            block = gray[r:r+k1, c:c+k2]
            mi, ma = float(block.min()), float(block.max())
            if mi > 0 and ma > 0 and ma > mi:
                eme += math.log(ma / mi)
                cnt += 1
    return 2.0 * eme / max(cnt, 1)

def uiconm(img):
    gray = 0.299*img[...,0] + 0.587*img[...,1] + 0.114*img[...,2]
    k1, k2 = 8, 8
    h, w = gray.shape
    ame = 0.0
    cnt = 0
    for r in range(0, h - k1, k1):
        for c in range(0, w - k2, k2):
            block = gray[r:r+k1, c:c+k2]
            mi, ma = float(block.min()), float(block.max())
            if (ma + mi) > 0 and ma > mi:
                ame += math.log((ma - mi) / (ma + mi) + 1.0)
                cnt += 1
    return ame / max(cnt, 1)

def entropy(img):
    gray = np.round(0.299*img[...,0] + 0.587*img[...,1] + 0.114*img[...,2]).astype(np.uint8)
    hist = np.bincount(gray.ravel(), minlength=256)
    p = hist[hist > 0] / hist.sum()
    return -float(np.sum(p * np.log2(p)))

items = [
    ("Original WB (BRAM)", "white_balanced.png"),
    ("Stretched WB", "white_balanced_stretched.png"),
    ("Current Unsharp (Eq 6)", "unsharp_masking.png"),
    ("Enhanced Unsharp beta=1.0", "eval_uns_b10.png"),
    ("Enhanced Unsharp beta=1.5", "eval_uns_b15.png"),
    ("Original Fused Output", "fused_output.png"),
    ("Enhanced Fused beta=1.0", "eval_fus_b10.png"),
    ("Enhanced Fused beta=1.5", "eval_fus_b15.png"),
    ("Fused from Stretched WB", "eval_fus_from_stretched_wb.png"),
]

hdr = f"{'Image':28s} | {'Contrast (Std)':14s} | {'Sharpness (EME)':15s} | {'UIConM':8s} | {'Entropy':8s}"
print(hdr)
print("-" * len(hdr))
for name, p in items:
    im = np.array(Image.open(p), dtype=np.float32)
    s = float(im.std())
    e = uism(im)
    c = uiconm(im)
    ent = entropy(im)
    print(f"{name:28s} | {s:14.2f} | {e:15.2f} | {c:8.4f} | {ent:8.3f}")
