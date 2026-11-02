"""End-to-end model check on jw's 60fps pairs: renderer-domain images (GPT mono K/D + active ki),
v5b matcher model (model.py), length of the 8-square board span (208 mm), old vs new R/T."""
import sys,json,glob,os,numpy as np,cv2
sys.path.insert(0,'/mnt/project-files/testing60_diag/live_v5b/sim');import model
KD='/mnt/user-data/uploads/ziguanng/stereo_integration/runs/20261029_123023_340b6a9b/raw_calibration_candidate_v1/'
def lk(n):
    c=json.load(open(KD+n+'_1280x720.json'));return np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1.]]),np.array([c['k1'],c['k2'],c['p1'],c['p2'],0.])
g=model.g
def und(img,side):
    K,D=lk(side);P=np.linalg.inv(np.array(g['ki'][side],float).reshape(3,3)/2**30)
    m1,m2=cv2.initUndistortRectifyMap(K,D,np.eye(3),P,(640,720),cv2.CV_32FC1)
    o=cv2.remap(img,m1,m2,cv2.INTER_LINEAR,borderValue=0)
    b,gg,r=[o[...,i].astype(int) for i in range(3)]
    r8=((r>>3)<<3)|(r>>5);g8=((gg>>2)<<2)|(gg>>6);b8=((b>>3)<<3)|(b>>5)
    return ((r8+2*g8+b8)>>2).astype(np.uint8),K,D,P
c=model.Cfg(SW=255,SH=31,TW=21,TH=11,Z=(100.,500.),MX=2,MY=3,U=10,EXCL=2,XONLY=True,BAND=True,CLAMP=True,MINC=12)
new=json.load(open('geometry_v5d.json'))
RTs={'old':(np.array(g['R']),np.array(g['T']),(80.,500.)),'new':(np.array(new['R']),np.array(new['T']),tuple(new['roi_depths_mm']))}
D=sys.argv[1];out={}
for k,(R,T,Z) in RTs.items():
    model.R=R;model.T=T;c.Z=Z;spans=[];errs=[];codes={};n_ok=0;n_try=0
    for p in sorted(glob.glob(D+'/pair_*')):
        gl,KL,DL,PL=und(cv2.imread(p+'/left.png'),'left');gr,_,_,_=und(cv2.imread(p+'/right.png'),'right')
        ok,cl=cv2.findChessboardCornersSB(cv2.cvtColor(cv2.imread(p+'/left.png'),cv2.COLOR_BGR2GRAY),(9,6),cv2.CALIB_CB_EXHAUSTIVE|cv2.CALIB_CB_ACCURACY)
        if not ok: continue
        uv=cv2.undistortPoints(cl,KL,DL,P=PL).reshape(6,9,2)
        # tap points: half a square diagonally outside? use the corner itself (X junction, good texture)
        for row in range(6):
            pts=[]
            for col in (0,8):
                u,v=[int(round(x)) for x in uv[row,col]];n_try+=1
                m,e=model.measure_point(c,gl,gr,u,v)
                codes[e]=codes.get(e,0)+1
                pts.append(model.tri(u,v,*m) if e is None else None)
            if pts[0] is not None and pts[1] is not None:
                errs.append(float(np.linalg.norm(pts[1]-pts[0])-208.0));spans.append((os.path.basename(p),row,round(errs[-1],1),round(float(pts[0][2]))))
    e=np.array(errs)
    out[k]=dict(points=n_try,codes={str(a):b for a,b in codes.items()},spans=len(e),span_err_mm_mean=float(e.mean()) if len(e) else None,
        span_err_mm_median=float(np.median(e)) if len(e) else None,span_err_mm_absmax=float(np.abs(e).max()) if len(e) else None)
    out[k]['per_span']=spans;e=np.abs(e);out[k]['within_2pct']=int((e<=4.16).sum());out[k]['within_5pct']=int((e<=10.4).sum())
    print(k,{a:b for a,b in out[k].items() if a!='per_span'})
json.dump(out,open('e2e_pairs.json','w'),indent=1)
