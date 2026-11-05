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
    # At 1 Hz the 10s clip has eleven sampled ticks per pass. Record applied
    # commands to verify that the next pass repeats exactly from home.
    commands=[]
    apply=teleop.apply_position_targets
    def record(model,data,**kwargs):
        commands.append(data.ctrl.copy())
        apply(model,data,**kwargs)
    with patch.object(sys,'argv',['gantry-teleop','--headless','--loop','--sample','ipman_roll','--rate','1','--steps','23','--wrist-branch',branch]), patch.object(teleop,'apply_position_targets',record):
        teleop.main()
    # Three branch-correct initializations plus 23 actual updates.
    assert len(commands)==26
    np.testing.assert_allclose(commands[0],commands[12],atol=1e-10)
    np.testing.assert_allclose(commands[0],commands[24],atol=1e-10)
    np.testing.assert_allclose(commands[1],commands[13],atol=1e-10)
    np.testing.assert_allclose(commands[1],commands[25],atol=1e-10)
    assert '23 frames;' in capsys.readouterr().out


@pytest.mark.parametrize("safety",[True,False])
def test_one_pass_process_exits(safety):
    result=subprocess.run([sys.executable,'-m','dex_ur_gantry_teleop.teleop',
        '--headless','--sample','ipman_roll','--rate','1'] + ([] if safety else ['--no-safety-filter']),
        capture_output=True,text=True,timeout=15,env=os.environ.copy())
    assert result.returncode==0,result.stderr
    assert '11 frames;' in result.stdout
    assert 'Wrist branch locked: wrist_up' in result.stdout

    assert ('Rust XPBD enabled:' in result.stdout) == safety
