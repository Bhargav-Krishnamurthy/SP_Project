"""Bit-exact reference for all five FPGA outputs (white_balanced, gamma_corrected,
unsharp_masking, fused_output, weight_maps).
usage:
  python golden_check_fusion.py image.mem white_balanced.mem gamma_corrected.mem unsharp_masking.mem fused_output.mem [weight_maps.mem] [gamma_lut.mem]
"""
import sys, math
import numpy as np

def rd(path):
    return [(int(l[0:2],16), int(l[2:4],16), int(l[4:6],16)) for l in open(path).read().split()]

def trunc_div(a, b):
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b > 0) else -q

def comp(x, g, mx, mg):
    return max(0, min(255, x + trunc_div((mg - mx) * (255 - x) * g, 65025)))

def gw(p, m, t):
    return 0 if m == 0 else min(255, (p * t) // m)

def white_balance(px):
    D = len(px)
    mr = sum(p[0] for p in px) // D
    mg = sum(p[1] for p in px) // D
    mb = sum(p[2] for p in px) // D
    rc = [comp(p[0], p[1], mr, mg) for p in px]
    bc = [comp(p[2], p[1], mb, mg) for p in px]
    mrc = sum(rc) // D
    mbc = sum(bc) // D
    t = (sum(rc) + sum(p[1] for p in px) + sum(bc)) // (3 * D)
    return [(gw(rc[i], mrc, t), gw(px[i][1], mg, t), gw(bc[i], mbc, t)) for i in range(D)]

def gamma(px, lut):
    return [tuple(lut[c] for c in p) for p in px]

def unsharp(px, W, beta_mode=1):
    H = len(px) // W
    K = ((1,2,1),(2,4,2),(1,2,1))
    def at(y, x):
        return px[min(max(y,0),H-1)*W + min(max(x,0),W-1)]
    out = []
    for y in range(H):
        for x in range(W):
            c = px[y*W + x]
            acc = [0,0,0]
            for dy in (-1,0,1):
                for dx in (-1,0,1):
                    w = K[dy+1][dx+1]
                    n = at(y+dy, x+dx)
                    for ch in range(3):
                        acc[ch] += w * n[ch]
            p = []
            for ch in range(3):
                diff = c[ch] - (acc[ch] >> 4)
                if beta_mode == 0:
                    d_term = diff >> 1
                elif beta_mode == 2:
                    d_term = diff + (diff >> 1)
                elif beta_mode == 3:
                    d_term = diff << 1
                else:
                    d_term = diff
                val = c[ch] + d_term
                p.append(min(max(val, 0), 255))
            out.append(tuple(p))
    return out

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

def single_scale_fusion(in1_px, in2_px, W, stretch_pct=0):
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

def cmp(name, exp, got):
    bad = [i for i in range(len(exp)) if i >= len(got) or got[i] != exp[i]]
    print("%-12s pixels: %d  mismatches: %d" % (name, len(exp), len(bad)))
    for i in bad[:5]:
        print("   ", i, "exp", exp[i], "got", got[i] if i < len(got) else None)
    return not bad

if __name__ == "__main__":
    inp = rd(sys.argv[1])
    W = math.isqrt(len(inp))
    lut_path = sys.argv[7] if len(sys.argv) > 7 else "gamma_lut.mem"
    lut = [int(l, 16) for l in open(lut_path).read().split()]
    wb = white_balance(inp)
    gam = gamma(wb, lut)
    uns = unsharp(wb, W)
    fus, wts = single_scale_fusion(gam, uns, W)

    ok  = cmp("white bal", wb,  rd(sys.argv[2]))
    ok &= cmp("gamma",     gam, rd(sys.argv[3]))
    ok &= cmp("unsharp",   uns, rd(sys.argv[4]))
    ok &= cmp("fused",     fus, rd(sys.argv[5]))
    if len(sys.argv) > 6:
        ok &= cmp("weights", wts, rd(sys.argv[6]))
    print("ALL PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)
