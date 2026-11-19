"""Preview a MuJoCo scene, paused at its home keyframe by default."""

import argparse
from pathlib import Path
from queue import Empty, SimpleQueue
import time

import mujoco


def load_model(path: Path, keyframe: str | None = None):
  model = mujoco.MjModel.from_xml_path(str(path.expanduser().resolve()))
  data = mujoco.MjData(model)
  key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, keyframe or "home")
  if keyframe is not None and key_id < 0:
    raise ValueError(f"Keyframe does not exist: {keyframe}")
  reset_model(model, data, key_id)
  return model, data, key_id


def reset_model(model: mujoco.MjModel, data: mujoco.MjData, key_id: int):
  if key_id >= 0:
    mujoco.mj_resetDataKeyframe(model, data, key_id)
  else:
    mujoco.mj_resetData(model, data)
  mujoco.mj_forward(model, data)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("xml", type=Path, help="MuJoCo scene XML")
  parser.add_argument("--keyframe", help="Initial keyframe (uses home if available)")
  parser.add_argument("--play", action="store_true", help="Start physics immediately")
  args = parser.parse_args()
  model, data, key_id = load_model(args.xml, args.keyframe)

  import mujoco.viewer

  events: SimpleQueue[int] = SimpleQueue()
  playing = args.play
  print("Space: play/pause; R or Backspace: reset pose and pause.")
  print("Simulation:", "running" if playing else "paused")
  with mujoco.viewer.launch_passive(model, data, key_callback=events.put) as viewer:
    # Show the floor and visual meshes; group 3 is collision geometry.
    with viewer.lock():
      viewer.opt.geomgroup[0] = 1
      viewer.opt.geomgroup[1] = 1
      viewer.opt.geomgroup[3] = 0
    while viewer.is_running():
      start = time.perf_counter()
      with viewer.lock():
        while True:
          try:
            key = events.get_nowait()
          except Empty:
            break
          if key == 32:
            playing = not playing
            print("Simulation:", "running" if playing else "paused")
          elif key in (82, 114, 259):
            reset_model(model, data, key_id)
            playing = False
            print("Reset to initial pose; simulation paused.")
        if playing:
          mujoco.mj_step(model, data)
        else:
          mujoco.mj_forward(model, data)
      viewer.sync()
      interval = model.opt.timestep if playing else 1.0 / 60.0
      time.sleep(max(0.0, interval - (time.perf_counter() - start)))


if __name__ == "__main__":
  main()
