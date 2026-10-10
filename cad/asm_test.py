import parts as Pt, skeleton as S
from kin import *
J, R = Pt.js_by_name()
for n,f in [("pelvis",lambda:Pt.pelvis(J)),("torso",lambda:Pt.torso_shell(J)),("cover",Pt.torso_cover),("fork",lambda:Pt.neck_fork(J)),("head",lambda:Pt.head_shell(J)),("headcover",lambda:Pt.head_cover(J))]:
    s=f(); bb=s.val().BoundingBox() if hasattr(s,'val') else s.BoundingBox()
    print(n, round(s.val().Volume() if hasattr(s,'val') else s.Volume()), [round(v) for v in (bb.xlen,bb.ylen,bb.zlen)])
