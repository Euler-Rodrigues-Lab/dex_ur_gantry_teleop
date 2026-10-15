"""CLI completion and deterministic loop restart regression checks."""
import os
import subprocess
import sys
from unittest.mock import patch
import numpy as np
from dex_ur_gantry_teleop import teleop


def test_loop_resets_robot_and_session(capsys):
    # At 0.1 Hz the 10s clip has two sampled ticks per pass. Record applied
    # commands to verify that the next pass repeats exactly from home.
    commands=[]
    apply=teleop.apply_position_targets
    def record(model,data,**kwargs):
        commands.append(data.ctrl.copy())
        apply(model,data,**kwargs)
    with patch.object(sys,'argv',['gantry-teleop','--headless','--loop','--sample','ipman_roll','--rate','0.1','--steps','5']), patch.object(teleop,'apply_position_targets',record):
        teleop.main()
    assert len(commands)==5
    np.testing.assert_allclose(commands[0],commands[2],atol=1e-10)
    np.testing.assert_allclose(commands[0],commands[4],atol=1e-10)
    assert '5 frames;' in capsys.readouterr().out


def test_one_pass_process_exits():
    result=subprocess.run([sys.executable,'-m','dex_ur_gantry_teleop.teleop',
        '--headless','--sample','ipman_roll','--rate','0.1'],
        capture_output=True,text=True,timeout=15,env=os.environ.copy())
    assert result.returncode==0,result.stderr
    assert '2 frames;' in result.stdout
