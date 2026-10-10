import robot as Rb
parts,J=Rb.build()
def bbi(a,b):
    bb=a.intersect(b).BoundingBox(); return [round(v,1) for v in (bb.xmin,bb.xmax,bb.ymin,bb.ymax,bb.zmin,bb.zmax)]
for a,b in [("servo_L_hip_roll","link_L_hip_roll"),("link_L_knee","link_L_ankle_pitch"),("link_L_ankle_pitch","link_L_ankle_roll"),("head_shell","camera_usb")]:
    print(a,b,bbi(parts[a].shape,parts[b].shape))
print(J["L_hip_roll"]); print(J["neck_pitch"])
