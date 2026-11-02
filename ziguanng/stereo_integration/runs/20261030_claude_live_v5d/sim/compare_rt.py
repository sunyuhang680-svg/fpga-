"""Compare stereo extrinsics on jw's 60fps pairs 20261030_203705_878327 by triangulating
the chessboard: old R/T (in builds since merged_v1) vs new R/T (leave-one-out).
Intrinsics: GPT 60fps mono (the ones the renderer uses)."""
import cv2,numpy as np,json,glob,os,sys
D,KD,OLD,OUT=sys.argv[1:5]
SQ=26.0;PAT=(9,6)
def load(n):
    c=json.load(open(os.path.join(KD,n+'_1280x720.json')))
    return np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1.]]),np.array([c['k1'],c['k2'],c['p1'],c['p2'],0.])
K0,d0=load('left');K1,d1=load('right')
obj=np.zeros((54,3),np.float32);obj[:,:2]=np.mgrid[0:9,0:6].T.reshape(-1,2)*SQ
O=[];L=[];R=[];N=[]
for p in sorted(glob.glob(D+'/pair_*')):
    g=[cv2.cvtColor(cv2.imread(p+f'/{s}.png'),cv2.COLOR_BGR2GRAY) for s in('left','right')]
    r=[cv2.findChessboardCornersSB(x,PAT,cv2.CALIB_CB_EXHAUSTIVE|cv2.CALIB_CB_ACCURACY) for x in g]
    if r[0][0] and r[1][0]:O.append(obj);L.append(r[0][1]);R.append(r[1][1]);N.append(os.path.basename(p))
crit=(cv2.TERM_CRITERIA_COUNT+cv2.TERM_CRITERIA_EPS,200,1e-9)
def cal(idx):
    r=cv2.stereoCalibrate([O[i] for i in idx],[L[i] for i in idx],[R[i] for i in idx],K0.copy(),d0.copy(),K1.copy(),d1.copy(),(1280,720),flags=cv2.CALIB_FIX_INTRINSIC,criteria=crit)
    return r[5],r[6].ravel()
def tri(i,Rm,T):
    a=cv2.undistortPoints(L[i],K0,d0).reshape(-1,2);b=cv2.undistortPoints(R[i],K1,d1).reshape(-1,2)
    P0=np.hstack([np.eye(3),np.zeros((3,1))]);P1=np.hstack([Rm,T.reshape(3,1)])
    X=cv2.triangulatePoints(P0,P1,a.T,b.T);X=(X[:3]/X[3]).T.reshape(6,9,3)
    h=np.linalg.norm(X[:,1:]-X[:,:-1],axis=2).ravel()-SQ;v=np.linalg.norm(X[1:]-X[:-1],axis=2).ravel()-SQ
    span=np.linalg.norm(X[:,8]-X[:,0],axis=1)-8*SQ
    return dict(z=float(X[...,2].mean()),adj_mean=float(np.concatenate([h,v]).mean()),span_mean=float(span.mean()),span_pct=float(span.mean()/(8*SQ)*100))
od=json.load(open(OLD));Ro=np.array(od['R']);To=np.array(od['T_mm'])
Rn,Tn=cal(range(len(N)))
rows=[]
for i,n in enumerate(N):
    Rl,Tl=cal([j for j in range(len(N)) if j!=i])
    rows.append(dict(pair=n,old=tri(i,Ro,To),new_loo=tri(i,Rl,Tl)))
def summ(k):
    e=np.array([r[k]['span_pct'] for r in rows]);return dict(span_err_pct_mean=float(e.mean()),span_err_pct_absmax=float(np.abs(e).max()))
res=dict(pairs=N,old_RT=dict(R=Ro.tolist(),T=To.tolist()),new_RT=dict(R=Rn.tolist(),T=Tn.tolist(),baseline=float(np.linalg.norm(Tn)),rot_deg=float(np.degrees(np.linalg.norm(cv2.Rodrigues(Rn)[0])))),
  old=summ('old'),new_loo=summ('new_loo'),per_pair=rows)
json.dump(res,open(OUT,'w'),indent=1)
print('old',res['old']);print('new(loo)',res['new_loo']);print('new T',Tn,'baseline',res['new_RT']['baseline'],'rot',res['new_RT']['rot_deg'])
for r in rows:print(r['pair'],'z=%.0f'%r['old']['z'],'old span %+.2f%%'%r['old']['span_pct'],'| new z=%.0f span %+.2f%%'%(r['new_loo']['z'],r['new_loo']['span_pct']))
