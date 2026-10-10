import cadquery as cq
from kin import *
import params as P
Zr=P.ANKLE_ROLL_Z; Zp=Zr+P.OFFSET; Zk=Zp+P.SHANK; Zhp=Zk+P.THIGH; Zhr=Zhp+P.OFFSET; Zhy=Zhr+P.OFFSET
print(Zr,Zp,Zk,Zhp,Zhr,Zhy)
