"""Classical rectangular opening candidates and depth-based plane fitting.

Appearance alone cannot establish traversability. Missing interior depth is
expected for an opening; sample the frame rim for 3D localization instead.
"""
from dataclasses import dataclass
import numpy as np

@dataclass
class Detection:
    corners: np.ndarray
    confidence: float

def detect_rectangles(image, min_area=500, max_area_ratio=0.9):
    import cv2
    gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY) if image.ndim==3 else image
    edges=cv2.Canny(cv2.GaussianBlur(gray,(5,5),0),50,150)
    contours,_=cv2.findContours(edges,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
    detections=[]
    for contour in contours:
        polygon=cv2.approxPolyDP(contour,0.02*cv2.arcLength(contour,True),True)
        area=abs(cv2.contourArea(polygon))
        if len(polygon)!=4 or not cv2.isContourConvex(polygon) or not min_area<area<gray.size*max_area_ratio:
            continue
        p=polygon[:,0,:].astype(float)
        center=p.mean(axis=0)
        p=p[np.argsort(np.arctan2(p[:,1]-center[1],p[:,0]-center[0]))]
        vectors=np.roll(p,-1,axis=0)-p
        cosines=[abs(np.dot(vectors[i],vectors[(i+1)%4])/(np.linalg.norm(vectors[i])*np.linalg.norm(vectors[(i+1)%4]))) for i in range(4)]
        score=max(0.,1.-float(np.mean(cosines)))
        if score>.6 and not any(np.linalg.norm(d.corners.mean(0)-center)<10 for d in detections):
            detections.append(Detection(p,score))
    return sorted(detections,key=lambda d:d.confidence,reverse=True)

def localize_frame(corners, depth, intrinsics, rim_radius=4):
    """Fit a plane from finite positive frame depths and intersect corner rays.

Camera optical coordinates: x right, y down, z forward. Returns camera frame
center, normal, corners, width/height; the caller must transform to world.
"""
    fx,fy,cx,cy=intrinsics
    if fx<=0 or fy<=0: raise ValueError('Invalid camera intrinsics')
    samples=[]
    h,w=depth.shape
    for i in range(4):
        for t in np.linspace(0,1,20):
            u,v=(1-t)*corners[i]+t*corners[(i+1)%4]
            x,y=int(round(u)),int(round(v))
            patch=depth[max(0,y-rim_radius):min(h,y+rim_radius+1),max(0,x-rim_radius):min(w,x+rim_radius+1)]
            valid=patch[np.isfinite(patch)&(patch>0)&(patch<50)]
            if len(valid):
                z=float(np.percentile(valid,25))
                samples.append([(u-cx)*z/fx,(v-cy)*z/fy,z])
    if len(samples)<12: raise ValueError('Insufficient finite frame depth')
    pts=np.array(samples);center=pts.mean(0)
    _,s,vt=np.linalg.svd(pts-center)
    if s[1]<1e-4: raise ValueError('Degenerate frame plane')
    normal=vt[-1]
    if normal[2]<0: normal=-normal
    residual=np.abs((pts-center)@normal)
    if np.median(residual)>.08: raise ValueError('Frame depth not planar')
    points=[]
    for u,v in corners:
        ray=np.array([(u-cx)/fx,(v-cy)/fy,1.])
        denom=normal@ray
        if abs(denom)<1e-5: raise ValueError('Corner ray parallel to plane')
        scale=float(normal@center/denom)
        if scale<=0: raise ValueError('Frame behind camera')
        points.append(ray*scale)
    points=np.array(points)
    edges=np.linalg.norm(np.roll(points,-1,axis=0)-points,axis=1)
    # Ordered corners normally start at upper-left, but classify edges by image direction.
    image_edges=np.roll(corners,-1,axis=0)-corners
    horizontal=[edges[i] for i,e in enumerate(image_edges) if abs(e[0])>=abs(e[1])]
    vertical=[edges[i] for i,e in enumerate(image_edges) if abs(e[0])<abs(e[1])]
    if len(horizontal)!=2 or len(vertical)!=2: raise ValueError('Unsupported rotated image rectangle')
    return {'center':points.mean(0),'normal':normal,'corners':points,
            'width':min(horizontal),'height':min(vertical),
            'plane_residual':float(np.median(residual))}
