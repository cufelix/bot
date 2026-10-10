import robot as Rb
parts,J=Rb.build()
for ang in (5,-5):
    a=Rb.posed(parts["head_shell"],J,{"neck_pitch":ang}); b=Rb.posed(parts["neck_fork"],J,{})
    i=a.intersect(b); bb=i.BoundingBox(); print(ang, round(i.Volume(),1),[round(v,1) for v in (bb.xmin,bb.xmax,bb.ymin,bb.ymax,bb.zmin,bb.zmax)])
print(J["neck_pitch"].pos)
