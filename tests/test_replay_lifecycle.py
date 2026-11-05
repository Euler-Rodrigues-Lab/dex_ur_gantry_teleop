"""CLI completion and deterministic loop restart regression checks."""
import os
import subprocess
import sys
from unittest.mock import patch
import numpy as np
import pytest
from dex_ur_gantry_teleop import teleop


@pytest.mark.parametrize("branch",["wrist_up","wrist_down"])
def test_loop_resets_robot_and_session(capsys,branch):
    # At 0.1 Hz the 10s clip has two sampled ticks per pass. Record applied
    # commands to verify that the next pass repeats exactly from home.
    commands=[]
    apply=teleop.apply_position_targets
    def record(model,data,**kwargs):
        commands.append(data.ctrl.copy())
        apply(model,data,**kwargs)
    with patch.object(sys,'argv',['gantry-teleop','--headless','--loop','--sample','ipman_roll','--rate','0.1','--steps','5','--wrist-branch',branch]), patch.object(teleop,'apply_position_targets',record):
        teleop.main()
    # Three branch-correct initializations plus five actual updates.
    assert len(commands)==8
    np.testing.assert_allclose(commands[0],commands[3],atol=1e-10)
    np.testing.assert_allclose(commands[0],commands[6],atol=1e-10)
    np.testing.assert_allclose(commands[1],commands[4],atol=1e-10)
    np.testing.assert_allclose(commands[1],commands[7],atol=1e-10)
    assert '5 frames;' in capsys.readouterr().out


def test_one_pass_process_exits():
    result=subprocess.run([sys.executable,'-m','dex_ur_gantry_teleop.teleop',
        '--headless','--sample','ipman_roll','--rate','0.1'],
        capture_output=True,text=True,timeout=15,env=os.environ.copy())
    assert result.returncode==0,result.stderr
    assert '2 frames;' in result.stdout
    assert 'Wrist branch locked: wrist_up' in result.stdout
