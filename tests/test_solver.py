"""Independent MuJoCo checks for the licensed gantry wheel (requires license)."""
import unittest
from types import SimpleNamespace as NS
import numpy as np
import mujoco
import geo_kin
from dex_ur_gantry_teleop import model_path

ARM = ['shoulder_pan','shoulder_lift','elbow','wrist_1','wrist_2','wrist_3']
R_WORLD_UPPER = np.array([[0.,-1,0],[1,0,0],[0,0,1]])

class SolverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.session=geo_kin.RetargetSession(robot='urgantry',hand='sharpa',collision_avoidance=False)
        cls.model=mujoco.MjModel.from_xml_path(str(model_path()))
        cls.data=mujoco.MjData(cls.model)
        mujoco.mj_forward(cls.model,cls.data)
        shoulders=[]
        for side in ('left','right'):
            a=cls.model.joint(side+'_shoulder_pan_joint').id
            b=cls.model.joint(side+'_shoulder_lift_joint').id
            t=np.linalg.lstsq(np.column_stack((cls.data.xaxis[a],-cls.data.xaxis[b])),cls.data.xanchor[b]-cls.data.xanchor[a],rcond=None)[0]
            shoulders.append(cls.data.xanchor[a]+t[0]*cls.data.xaxis[a])
        cls.origin=np.mean(shoulders,axis=0)

    def setUp(self):
        self.session.reset()

    def test_random_fk_and_ik_against_mujoco(self):
        rng=np.random.default_rng(5)
        for side in ('left','right'):
            ids=[self.model.joint(f'{side}_{n}_joint').id for n in ARM]
            for _ in range(150):
                q=rng.uniform(self.model.jnt_range[ids,0],self.model.jnt_range[ids,1])
                self.data.qpos[:]=0
                self.data.qpos[self.model.jnt_qposadr[ids]]=q
                mujoco.mj_forward(self.model,self.data)
                fk=self.session.eef_fk(side,q)
                body=self.model.body(side+'_hand_hand_C_MC').id
                np.testing.assert_allclose(fk['hand_position'],R_WORLD_UPPER.T@(self.data.xpos[body]-self.origin),atol=1e-9)
                np.testing.assert_allclose(fk['hand_rotation'],R_WORLD_UPPER.T@self.data.xmat[body].reshape(3,3),atol=1e-9)
                index=self.model.joint(side+'_hand_index_MCP_FE').id
                np.testing.assert_allclose(fk['position'],R_WORLD_UPPER.T@(self.data.xanchor[index]-self.origin),atol=1e-9)
                solved=self.session.eef_ik(side,fk['position'],fk['rotation'],self.session.default_q[side])
                self.assertIsNotNone(solved)
                again=self.session.eef_fk(side,solved)
                np.testing.assert_allclose(again['position'],fk['position'],atol=1e-6)
                np.testing.assert_allclose(again['rotation'],fk['rotation'],atol=1e-6)

    def home_frame(self):
        fields={}
        for side in ('left','right'):
            fk=self.session.eef_fk(side,self.session.default_q[side])
            fields[side+'_eef']=NS(position=fk['position'],rotation=fk['rotation'])
        return NS(p_world_upper_body=np.zeros(3),R_world_upper_body=np.eye(3),**fields)

    def test_hold_reset_invalid_and_no_elbow_dependency(self):
        f=self.home_frame();a=self.session.solve(f)
        for side in ('left','right'):np.testing.assert_allclose(getattr(a,'q_goal_'+side),self.session.default_q[side],atol=1e-8)
        f.left_sew=NS(E=np.full(3,np.nan));f.right_sew=NS(E=np.ones(3)*100)
        b=self.session.solve(f)
        np.testing.assert_array_equal(a.q_goal_left,b.q_goal_left)
        f.left_eef.position=np.ones(3)*100
        c=self.session.solve(f);self.assertIsNone(c.q_goal_left);self.assertIsNotNone(c.q_goal_right)
        out=self.session.solve(f,engaged=False);self.assertIsNone(out.q_goal_right)
        f.left_eef=None
        out=self.session.solve(f);self.assertIsNone(out.q_goal_right)

    def test_anchor_modes_and_fixed_offset(self):
        for mode in ('first_frame','shoulder_attached'):
            s=geo_kin.RetargetSession(robot='urgantry',hand='sharpa',base_alignment_mode=mode)
            f=self.home_frame();s.solve(f)
            delta=np.array([0.01,0,0]);f.p_world_upper_body+=delta
            for side in ('left','right'):getattr(f,side+'_eef').position+=delta
            out=s.solve(f)
            for side in ('left','right'):
                target=s.eef_fk(side,getattr(out,'q_goal_'+side))['position']
                expected=self.home_frame().__dict__[side+'_eef'].position
                np.testing.assert_allclose(target,expected+(delta if mode=='first_frame' else 0),atol=1e-6)

    def test_mounted_hand_fk_and_alignment(self):
        rng=np.random.default_rng(9)
        for side in ('left','right'):
            names=self.session.hand_joint_names(side)
            ids=[self.model.joint(n).id for n in names]
            for _ in range(30):
                q=rng.uniform(self.model.jnt_range[ids,0],self.model.jnt_range[ids,1])
                self.data.qpos[:]=0;self.data.qpos[self.model.jnt_qposadr[ids]]=q
                mujoco.mj_forward(self.model,self.data)
                tips=self.session.hand_fingertips_at(side,q)
                b=self.model.body(side+'_hand_hand_C_MC').id
                r=self.data.xmat[b].reshape(3,3);origin=self.data.xpos[b]
                for finger,pos in tips.items():
                    site=self.model.site(f'{side}_hand_{finger}_fingertip').id
                    np.testing.assert_allclose(pos,r.T@(self.data.site_xpos[site]-origin),atol=1e-9)
            # Zero-pose model landmarks are a deterministic complete hand input.
            self.data.qpos[:]=0;mujoco.mj_forward(self.model,self.data)
            b=self.model.body(side+'_hand_hand_C_MC').id
            r=self.data.xmat[b].reshape(3,3);origin=self.data.xpos[b]
            fingers={}
            for finger in ('thumb','index','middle','ring','pinky'):
                joints=[self.model.joint(n).id for n in names if f'_{finger}_' in n]
                slots=[0,2,4] if finger=='thumb' else ([1,3,4] if finger=='pinky' else [0,2,3])
                keys=['thumb_cmc','thumb_mcp','thumb_ip'] if finger=='thumb' else [f'{finger if finger=="pinky" else finger+"_finger"}_{k}' for k in ('mcp','pip','dip')]
                points={key:r.T@(self.data.xanchor[joints[j]]-origin) for key,j in zip(keys,slots)}
                tip=self.model.site(f'{side}_hand_{finger}_fingertip').id
                points[f'{finger if finger in ("thumb","pinky") else finger+"_finger"}_tip']=r.T@(self.data.site_xpos[tip]-origin)
                fingers[finger]=points
            f=self.home_frame();setattr(f,side+'_fingers_wrist',fingers)
            out=self.session.solve(f);q=getattr(out,'q_goal_'+side+'_hand')
            self.assertIsNotNone(q);self.assertTrue(np.isfinite(q).all())
            self.assertTrue(np.all(q>=self.model.jnt_range[ids,0]-1e-8))
            self.assertTrue(np.all(q<=self.model.jnt_range[ids,1]+1e-8))
            del fingers['index']
            held=self.session.solve(f)
            self.assertIsNone(getattr(held,'q_goal_'+side+'_hand'))

    def test_invalid_api(self):
        with self.assertRaises(ValueError):self.session.eef_ik('left',[0,0,0],np.zeros((3,3)))
        with self.assertRaises(ValueError):self.session.reset([0,0],None)
        with self.assertRaises(ValueError):geo_kin.RetargetSession(robot='urgantry',hand='unsupported')

if __name__=='__main__':unittest.main()
