"""Bit-exact reference for all three FPGA outputs.
usage: python golden_check_all.py image.mem white_balanced.mem gamma_corrected.mem unsharp_masking.mem [gamma_lut.mem]
"""
import sys, math

def rd(path):
    return [(int(l[0:2],16), int(l[2:4],16), int(l[4:6],16)) for l in open(path).read().split()]

def trunc_div(a,b):
    q = abs(a)//abs(b)
    return q if (a>=0)==(b>0) else -q
def comp(x,g,mx,mg):
    return max(0,min(255,x+trunc_div((mg-mx)*(255-x)*g, 65025)))
def gw(p,m,t):
    return 0 if m==0 else min(255,(p*t)//m)

def white_balance(px):
    D=len(px)
    mr=sum(p[0] for p in px)//D; mg=sum(p[1] for p in px)//D; mb=sum(p[2] for p in px)//D
    rc=[comp(p[0],p[1],mr,mg) for p in px]
    bc=[comp(p[2],p[1],mb,mg) for p in px]
    mrc=sum(rc)//D; mbc=sum(bc)//D
    t=(sum(rc)+sum(p[1] for p in px)+sum(bc))//(3*D)
    return [(gw(rc[i],mrc,t),gw(px[i][1],mg,t),gw(bc[i],mbc,t)) for i in range(D)]

def gamma(px, lut):
    return [tuple(lut[c] for c in p) for p in px]

def unsharp(px, W):
    H = len(px)//W
    K = ((1,2,1),(2,4,2),(1,2,1))
    def at(y,x):
        return px[min(max(y,0),H-1)*W + min(max(x,0),W-1)]
    D=[]
    for y in range(H):
        for x in range(W):
            c = px[y*W+x]
            acc=[0,0,0]
            for dy in (-1,0,1):
                for dx in (-1,0,1):
                    w=K[dy+1][dx+1]; n=at(y+dy,x+dx)
                    for ch in range(3): acc[ch]+=w*n[ch]
            D.append(tuple(c[ch]-(acc[ch]>>4) for ch in range(3)))
    dmin=min(min(d) for d in D); dmax=max(max(d) for d in D); rng=dmax-dmin
    out=[]
    for p,d in zip(px,D):
        if rng==0: out.append(p)
        else: out.append(tuple((p[ch] + ((d[ch]-dmin)*255)//rng)>>1 for ch in range(3)))
    return out

def cmp(name, exp, got):
    bad=[i for i in range(len(exp)) if i>=len(got) or got[i]!=exp[i]]
    print("%-10s pixels: %d  mismatches: %d" % (name, len(exp), len(bad)))
    for i in bad[:5]: print("   ", i, "exp", exp[i], "got", got[i] if i<len(got) else None)
    return not bad

if __name__=="__main__":
    inp = rd(sys.argv[1]); W = math.isqrt(len(inp))
    lut = [int(l,16) for l in open(sys.argv[5] if len(sys.argv)>5 else "gamma_lut.mem").read().split()]
    wb = white_balance(inp)
    ok  = cmp("white bal", wb, rd(sys.argv[2]))
    ok &= cmp("gamma", gamma(wb,lut), rd(sys.argv[3]))
    ok &= cmp("unsharp", unsharp(wb,W), rd(sys.argv[4]))
    print("ALL PASS" if ok else "FAIL"); sys.exit(0 if ok else 1)
