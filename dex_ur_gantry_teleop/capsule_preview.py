"""Draw exact capsule endpoints supplied by the Rust safety filter."""
import mujoco
import numpy as np


def draw_safety_state(scene,state):
    if state is None:return
    def capsule(a,b,r,color):
        if scene.ngeom>=scene.maxgeom:raise RuntimeError('Insufficient capsule overlay capacity')
        g=scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_CAPSULE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.asarray(color,dtype=np.float32))
        mujoco.mjv_connector(g,mujoco.mjtGeom.mjGEOM_CAPSULE,r,np.asarray(a,dtype=float),np.asarray(b,dtype=float))
        scene.ngeom+=1
    for c in state['capsules']:
        color=[.1,.8,1.,.25] if c['side']=='left' else [1.,.5,.1,.25]
        capsule(c['a'],c['b'],c['radius'],color)
    c=state['column'];capsule(c['a'],c['b'],c['radius'],[.8,.2,1.,.25])
    h=state['table']['height']
    corners=[[-.56,-.4725,h],[.56,-.4725,h],[.56,.2925,h],[-.56,.2925,h]]
    for i in range(4):capsule(corners[i],corners[(i+1)%4],.002,[.2,1.,.3,.8])
