import importlib.util
from pathlib import Path
import numpy as np
import pytest
from geo_kin_core.frames import load_frames

spec=importlib.util.spec_from_file_location('ego_import',Path(__file__).resolve().parents[1]/'scripts/import_ego_retarget.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_irregular_timing_and_dropouts_preserve_landmarks(tmp_path):
    rng=np.random.default_rng(1)
    joints=rng.normal(size=(3,2,21,3))
    pose=np.eye(4);pose[:3,2]=[0,1,0]
    valid=np.ones((3,2),bool);valid[1,1]=False
    src=tmp_path/'input.npz';dst=tmp_path/'frames.npz'
    np.savez(src,skeleton_world=joints,timestamps=[0,.04,.1],valid=valid,head_pose_world=[pose]*3)
    module.convert(src,dst,table_z=0,fps=100)
    stream=load_frames(dst)
    assert len(stream)==11
    np.testing.assert_allclose(stream[3].skeleton['positions'][:21],joints[0,0])
    np.testing.assert_allclose(stream[4].skeleton['positions'][:21],joints[1,0])
    assert stream[4].right_sew is None
    assert np.isnan(stream[4].skeleton['positions'][21:]).all()
    frame=stream[10];sew=frame.left_sew
    wrist=frame.p_world_upper_body+frame.R_world_upper_body@sew.W
    np.testing.assert_allclose(wrist,joints[2,0,0])
    np.testing.assert_allclose(frame.left_fingers['index']['index_finger_tip']+wrist,joints[2,0,8])
    assert 'FullBody_LeftHandMiddleIntermediate' in frame.skeleton['names']
    with pytest.raises(ValueError):module.convert(src,dst,table_z=0,fps=0)


def test_mean_heading_wraparound_fixed_frame_and_unchanged_origin(tmp_path):
    cameras=np.tile(np.eye(4),(3,1,1))
    yaw=np.radians([170.,-170.,180.])
    cameras[:,:3,2]=np.column_stack((np.cos(yaw),np.sin(yaw),np.zeros(3)))
    joints=np.ones((3,2,21,3))
    src=tmp_path/'input.npz'
    np.savez(src,skeleton_world=joints,timestamps=[0.,1.,2.],valid=np.ones((3,2),bool),head_pose_world=cameras)
    module.convert(src,tmp_path/'first.npz',table_z=0)
    module.convert(src,tmp_path/'mean.npz',table_z=0,heading='mean')
    first=load_frames(tmp_path/'first.npz');mean=load_frames(tmp_path/'mean.npz')
    expected=np.average(np.column_stack((np.cos(yaw),np.sin(yaw))),axis=0,weights=[.5,1,.5]);expected/=np.linalg.norm(expected)
    np.testing.assert_allclose(mean[0].R_world_upper_body[:2,0],expected)
    assert mean[0].R_world_upper_body[0,0]<-.99
    np.testing.assert_allclose(mean[0].R_world_upper_body,mean[len(mean)-1].R_world_upper_body)
    np.testing.assert_allclose(mean[0].p_world_upper_body,first[0].p_world_upper_body)
    cameras[:,:3,2]=[[1,0,0],[-1,0,0],[1,0,0]]
    np.savez(src,skeleton_world=joints,timestamps=[0.,1.,2.],valid=np.ones((3,2),bool),head_pose_world=cameras)
    with pytest.raises(ValueError,match='ambiguous'):module.convert(src,tmp_path/'bad.npz',table_z=0,heading='mean')
