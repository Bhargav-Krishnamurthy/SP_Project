import sys
def trunc_div(a,b):
    q = abs(a)//abs(b)
    return q if (a>=0)==(b>0) else -q
def comp(x,g,mx,mg):
    corr = trunc_div((mg-mx)*(255-x)*g, 65025)
    return max(0,min(255,x+corr))
def gw(p,m,t):
    return 0 if m==0 else min(255,(p*t)//m)
def run(pixels):
    D=len(pixels)
    mr=sum(p[0] for p in pixels)//D; mg=sum(p[1] for p in pixels)//D; mb=sum(p[2] for p in pixels)//D
    rc=[comp(p[0],p[1],mr,mg) for p in pixels]
    bc=[comp(p[2],p[1],mb,mg) for p in pixels]
    mrc=sum(rc)//D; mgc=mg_c=sum(p[1] for p in pixels)//D; mbc=sum(bc)//D
    t=(sum(rc)+sum(p[1] for p in pixels)+sum(bc))//(3*D)
    return [(gw(rc[i],mrc,t),gw(pixels[i][1],mgc,t),gw(bc[i],mbc,t)) for i in range(D)]
inp=[ (int(l[0:2],16),int(l[2:4],16),int(l[4:6],16)) for l in open(sys.argv[1]).read().split()]
exp=run(inp)
got=[ (int(l[0:2],16),int(l[2:4],16),int(l[4:6],16)) for l in open(sys.argv[2]).read().split()]
bad=[i for i in range(len(exp)) if i>=len(got) or got[i]!=exp[i]]
print("pixels:",len(exp),"mismatches:",len(bad))
for i in bad[:5]: print(i,"in",inp[i],"exp",exp[i],"got",got[i] if i<len(got) else None)
