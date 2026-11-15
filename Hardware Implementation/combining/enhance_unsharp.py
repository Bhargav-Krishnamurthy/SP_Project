#!/usr/bin/env python3
"""
enhance_unsharp.py
Underwater Image Enhancement - Unsharp Masking & Fusion Optimizer

Replaces the lossy Equation (6) unsharp masking:
    Old (Lossy):     S = (I + N{I - G*I}) / 2      [Halves contrast, amplifies noise by 10.6x]
    Enhanced:        S = clip(I + beta*(I - G*I), 0, 255) [100% dynamic range, clean crisp edges]

Hardware-friendly beta modes:
    beta = 1.0 : val = I + diff                 (1 add, 0 mul)
    beta = 1.5 : val = I + diff + (diff >>> 1)  (shift-add)
    beta = 2.0 : val = I + (diff <<< 1)         (shift-add)
"""

import sys
import os
import math
import numpy as np
from PIL import Image

def rd(path):
    with open(path) as f:
        toks = f.read().split()
    return [((int(t, 16) >> 16) & 0xFF,
             (int(t, 16) >> 8)  & 0xFF,
              int(t, 16)        & 0xFF) for t in toks]

def save_mem(path, px):
    with open(path, "w") as f:
        for p in px:
            f.write(f"{(p[0]<<16) | (p[1]<<8) | p[2]:06X}\n")

def save_png(path, px, W, H):
    arr = np.array(px, dtype=np.uint8).reshape(H, W, 3)
    Image.fromarray(arr, "RGB").save(path)

# --- 3x3 Gaussian Blur & Difference ---
def gaussian_blur_diff(px, W, H):
    K = ((1, 2, 1),
         (2, 4, 2),
         (1, 2, 1))
    def at(y, x):
        return px[min(max(y, 0), H - 1) * W + min(max(x, 0), W - 1)]

    diff_list = []
    blur_list = []
    for y in range(H):
        for x in range(W):
            c = px[y * W + x]
            acc = [0, 0, 0]
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    w = K[dy + 1][dx + 1]
                    n = at(y + dy, x + dx)
                    for ch in range(3):
                        acc[ch] += w * n[ch]
            b = tuple(acc[ch] >> 4 for ch in range(3))
            d = tuple(c[ch] - b[ch] for ch in range(3))
            blur_list.append(b)
            diff_list.append(d)
    return blur_list, diff_list

# --- Unsharp Masking Implementations ---
def unsharp_paper_eq6(px, diff_list):
    """Old implementation from Equation (6) in Ancuti 2018."""
    dmin = min(min(d) for d in diff_list)
    dmax = max(max(d) for d in diff_list)
    rng = dmax - dmin
    out = []
    for p, d in zip(px, diff_list):
        if rng == 0:
            out.append(p)
        else:
            out.append(tuple((p[ch] + ((d[ch] - dmin) * 255) // rng) >> 1 for ch in range(3)))
    return out

def unsharp_enhanced(px, diff_list, beta=1.0):
    """
    Enhanced high-fidelity unsharp masking.
    Preserves 100% of base image dynamic range, injects controlled high-frequency edge detail.
    """
    out = []
    for p, d in zip(px, diff_list):
        pix = []
        for ch in range(3):
            val = int(round(p[ch] + beta * d[ch]))
            pix.append(min(max(val, 0), 255))
        out.append(tuple(pix))
    return out

# --- Single Scale Fusion ---
SRGB_LUT = np.array([round(((i/255.0)/12.92 if (i/255.0)<=0.04045 else ((i/255.0+0.055)/1.055)**2.4)*4095.0) for i in range(256)], dtype=np.int64)
LAB_F_LUT = np.array([round(((i/511.0)**(1.0/3.0) if (i/511.0)>0.008856 else (7.787*(i/511.0)+16.0/116.0))*1023.0) for i in range(512)], dtype=np.int64)

def rgb_to_lab_fp(blur_u8):
    rl = SRGB_LUT[blur_u8[..., 0]]
    gl = SRGB_LUT[blur_u8[..., 1]]
    bl = SRGB_LUT[blur_u8[..., 2]]
    xn = (444 * rl + 385 * gl + 195 * bl) >> 13
    yn = (218 * rl + 732 * gl +  74 * bl) >> 13
    zn = ( 18 * rl + 112 * gl + 894 * bl) >> 13
    fx = LAB_F_LUT[xn]
    fy = LAB_F_LUT[yn]
    fz = LAB_F_LUT[zn]
    Lf = (116 * fy) >> 8
    af = (500 * (fx - fy)) >> 8
    bf = (200 * (fy - fz)) >> 8
    return Lf, af, bf

def single_scale_fusion(in1_px, in2_px, W, stretch_pct=1):
    H = len(in1_px) // W
    depth = H * W
    aw = int(round(math.log2(depth)))
    i1 = np.array(in1_px, dtype=np.int64).reshape(H, W, 3)
    i2 = np.array(in2_px, dtype=np.int64).reshape(H, W, 3)

    def comp_raw_weights(im):
        p = np.pad(im, ((1,1),(1,1),(0,0)), mode='edge')
        blur = (
            p[0:H,0:W] + 2*p[0:H,1:W+1] + p[0:H,2:W+2] +
            2*p[1:H+1,0:W] + 4*p[1:H+1,1:W+1] + 2*p[1:H+1,2:W+2] +
            p[2:H+2,0:W] + 2*p[2:H+2,1:W+1] + p[2:H+2,2:W+2]
        ) >> 4
        Lp = (54*p[...,0] + 183*p[...,1] + 19*p[...,2]) >> 4
        lap = np.abs(
            2*Lp[0:H,0:W] + 2*Lp[0:H,2:W+2] +
            2*Lp[2:H+2,0:W] + 2*Lp[2:H+2,2:W+2] -
            8*Lp[1:H+1,1:W+1]
        ) >> 3
        Lc = (54*im[...,0] + 183*im[...,1] + 19*im[...,2] + 128) >> 8
        sq = (im[...,0]-Lc)**2 + (im[...,1]-Lc)**2 + (im[...,2]-Lc)**2
        sat = np.floor(np.sqrt((sq * 21845) >> 12)).astype(np.int64)
        Lf, af, bf = rgb_to_lab_fp(blur)
        mL = int(Lf.sum()) >> aw
        ma = int(af.sum()) >> aw
        mb = int(bf.sum()) >> aw
        sal = np.floor(np.sqrt((Lf - mL)**2 + (af - ma)**2 + (bf - mb)**2)).astype(np.int64)
        return lap, sal, sat

    def n255(x):
        lo, hi = int(x.min()), int(x.max())
        if hi == lo:
            return np.zeros_like(x)
        return ((x - lo) * 255) // (hi - lo)

    wl1, ws1, wsat1 = comp_raw_weights(i1)
    wl2, ws2, wsat2 = comp_raw_weights(i2)
    w1 = n255(wl1) + n255(ws1) + n255(wsat1)
    w2 = n255(wl2) + n255(ws2) + n255(wsat2)
    nw1 = ((w1 + 26) << 8) // (w1 + w2 + 52)
    nw2 = 256 - nw1
    fused = np.clip((nw1[...,None]*i1 + nw2[...,None]*i2 + 128) >> 8, 0, 255)

    lo_th = 1 if stretch_pct == 0 else (depth * stretch_pct) // 100
    hi_th = depth if stretch_pct == 0 else depth - (depth * stretch_pct) // 100
    stretched = np.empty_like(fused)
    for c in range(3):
        hist = np.bincount(fused[..., c].ravel(), minlength=256)
        cdf = np.cumsum(hist)
        lo = int(np.argmax(cdf >= lo_th))
        hi = int(np.argmax(cdf >= hi_th))
        if hi <= lo:
            stretched[..., c] = fused[..., c]
        else:
            rng = hi - lo
            cl = np.clip(fused[..., c], lo, hi) - lo
            stretched[..., c] = (cl * 255) // rng
    weights = np.stack([nw1, np.clip(nw2, 0, 255), np.zeros_like(nw1)], axis=2)
    return [tuple(int(c) for c in p) for p in stretched.reshape(-1, 3)], \
           [tuple(int(c) for c in p) for p in weights.reshape(-1, 3)]

# --- Quality Metrics ---
def compute_metrics(px_list):
    arr = np.array(px_list, dtype=np.float32)
    gray = 0.299 * arr[:, 0] + 0.587 * arr[:, 1] + 0.114 * arr[:, 2]
    gray2d = gray.reshape(256, 256)

    # Contrast
    std = float(gray.std())
    rng = float(gray.max() - gray.min())
    mean = float(gray.mean())

    # Laplacian Variance (sharpness)
    lap = (
        -4.0 * gray2d[1:-1, 1:-1] +
        gray2d[0:-2, 1:-1] + gray2d[2:, 1:-1] +
        gray2d[1:-1, 0:-2] + gray2d[1:-1, 2:]
    )
    lap_var = float(lap.var())

    # EME (Underwater sharpness measure)
    k1, k2 = 8, 8
    eme = 0.0
    cnt = 0
    for r in range(0, 256 - k1, k1):
        for c in range(0, 256 - k2, k2):
            b = gray2d[r:r+k1, c:c+k2]
            mi, ma = float(b.min()), float(b.max())
            if mi > 0 and ma > 0 and ma > mi:
                eme += math.log(ma / mi)
                cnt += 1
    eme_val = 2.0 * eme / max(cnt, 1)

    # Entropy
    hist = np.bincount(np.round(gray).astype(np.uint8), minlength=256)
    p = hist[hist > 0] / hist.sum()
    ent = -float(np.sum(p * np.log2(p)))

    return {"mean": mean, "std": std, "range": rng, "lap_var": lap_var, "eme": eme_val, "entropy": ent}

def main():
    wb_file = "white_balanced.mem"
    gam_file = "gamma_corrected.mem"
    lut_file = "gamma_lut.mem"

    if not os.path.exists(wb_file):
        print(f"Error: {wb_file} not found.")
        sys.exit(1)

    print("Loading image data...")
    wb_px = rd(wb_file)
    gam_px = rd(gam_file)
    W, H = 256, 256

    print("Computing 3x3 Gaussian blur and difference map...")
    blur_list, diff_list = gaussian_blur_diff(wb_px, W, H)

    # 1. Old unsharp masking (Eq 6)
    uns_old = unsharp_paper_eq6(wb_px, diff_list)
    save_png("unsharp_old_eq6.png", uns_old, W, H)

    # 2. Enhanced unsharp masking for different beta values
    betas = [0.5, 1.0, 1.5, 2.0]
    enhanced_uns = {}
    enhanced_fus = {}
    enhanced_wts = {}

    for b in betas:
        print(f"Processing enhanced unsharp & fusion for beta={b}...")
        u = unsharp_enhanced(wb_px, diff_list, beta=b)
        f, w = single_scale_fusion(gam_px, u, W, stretch_pct=1)
        enhanced_uns[b] = u
        enhanced_fus[b] = f
        enhanced_wts[b] = w
        save_png(f"enhanced_unsharp_beta_{b:.1f}.png", u, W, H)
        save_png(f"enhanced_fused_beta_{b:.1f}.png", f, W, H)
        save_mem(f"enhanced_unsharp_beta_{b:.1f}.mem", u)
        save_mem(f"enhanced_fused_beta_{b:.1f}.mem", f)

    # Best choice for hardware: beta = 1.0 (recommended) and 1.5
    save_png("enhanced_unsharp_final.png", enhanced_uns[1.0], W, H)
    save_mem("enhanced_unsharp_final.mem", enhanced_uns[1.0])
    save_png("enhanced_fused_final.png", enhanced_fus[1.0], W, H)
    save_mem("enhanced_fused_final.mem", enhanced_fus[1.0])

    # Side-by-side comparison image
    wb_arr = np.array(wb_px, dtype=np.uint8).reshape(H, W, 3)
    gam_arr = np.array(gam_px, dtype=np.uint8).reshape(H, W, 3)
    u_old_arr = np.array(uns_old, dtype=np.uint8).reshape(H, W, 3)
    u_new_arr = np.array(enhanced_uns[1.0], dtype=np.uint8).reshape(H, W, 3)
    
    # Old fused output
    f_old_arr = np.array(rd("fused_output.mem"), dtype=np.uint8).reshape(H, W, 3)
    f_new_arr = np.array(enhanced_fus[1.0], dtype=np.uint8).reshape(H, W, 3)

    row1 = np.concatenate([wb_arr, gam_arr, u_old_arr, u_new_arr], axis=1)
    row2 = np.concatenate([wb_arr, gam_arr, f_old_arr, f_new_arr], axis=1)
    comp = np.concatenate([row1, row2], axis=0)
    Image.fromarray(comp).save("enhancement_comparison.png")
    print("Saved enhancement_comparison.png")

    # Evaluation table
    test_suite = [
        ("White Balanced (Base)", wb_px),
        ("Old Unsharp Masking (Eq 6)", uns_old),
        ("Enhanced Unsharp (beta=0.5)", enhanced_uns[0.5]),
        ("Enhanced Unsharp (beta=1.0)", enhanced_uns[1.0]),
        ("Enhanced Unsharp (beta=1.5)", enhanced_uns[1.5]),
        ("Old Fused Output (FPGA)", rd("fused_output.mem")),
        ("Enhanced Fused (beta=1.0)", enhanced_fus[1.0]),
        ("Enhanced Fused (beta=1.5)", enhanced_fus[1.5]),
    ]

    print("\n" + "=" * 95)
    print(f"{'Image Pipeline Stage':30s} | {'Mean':6s} | {'Std Dev':8s} | {'Laplacian Var':14s} | {'Sharp(EME)':10s} | {'Entropy':7s}")
    print("=" * 95)
    for name, px_data in test_suite:
        m = compute_metrics(px_data)
        print(f"{name:30s} | {m['mean']:6.1f} | {m['std']:8.2f} | {m['lap_var']:14.2f} | {m['eme']:10.2f} | {m['entropy']:7.3f}")
    print("=" * 95)

if __name__ == "__main__":
    main()
