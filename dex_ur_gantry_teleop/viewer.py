"""Keep MuJoCo's render thread alive until its GLFW cleanup finishes."""
from contextlib import contextmanager
import threading


@contextmanager
def managed_viewer(model, data, **kwargs):
    from mujoco import viewer as mv
    before = set(threading.enumerate())
    handle = mv.launch_passive(model, data, **kwargs)
    # MuJoCo 3.x launches a daemon render thread but Handle.close() only
    # requests exit. Interpreter shutdown can otherwise run glfw.terminate
    # while that thread is still swapping buffers (GLXBadDrawable).
    # Identify only this launch's renderer, never device/network threads.
    renderers = [thread for thread in threading.enumerate()
                 if thread not in before
                 and getattr(thread, '_target', None) is mv._launch_internal]
    try:
        yield handle
    finally:
        handle.close()
        for thread in renderers:
            thread.join(timeout=10)
            if thread.is_alive():
                raise RuntimeError('MuJoCo render thread did not finish shutdown within 10 seconds')


def configure_camera(camera):
    """OpenArm-style three-quarter view, rotated into the gantry's frame."""
    camera.lookat[:] = [0.0, -0.15, 1.2]
    camera.distance = 2.4
    camera.azimuth = 225
    camera.elevation = -18
