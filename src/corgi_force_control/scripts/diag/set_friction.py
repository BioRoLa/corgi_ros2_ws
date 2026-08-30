"""Set the ground friction coefficient in the Corgi world, in place.

Rewrites the whole `contactProperties` block so every ground-contacting surface
gets the SAME explicit coulombFriction. That matters for a controlled sweep:

  - `foot_tip` had a ContactProperties entry but no coulombFriction, so it ran
    on the Webots default of 1.0.
  - `wheel_rim` had NO entry at all. The existing `material1 "Floor"` block does
    not catch it -- material2 defaults to "default", so Floor-vs-wheel_rim
    matched nothing and also fell through to 1.0.

Writing 1.0 explicitly therefore reproduces the previous behaviour exactly, so
the mu = 1.0 arm of a sweep stays comparable with everything recorded before,
while mu = 1.5 moves both surfaces together instead of only one.

    python3 set_friction.py 1.5
"""
import re
import sys

WORLD = ("/home/alexc/corgi_ws/corgi_ros2_ws/src/corgi_sim/worlds/"
         "Corgi_ABAD.wbt")

BLOCK = """  contactProperties [
    ContactProperties {{
      material1 "foot_tip"
      material2 "Floor"
      coulombFriction [ {mu} ]
      maxContactJoints 20
    }}
    ContactProperties {{
      material1 "wheel_rim"
      material2 "Floor"
      coulombFriction [ {mu} ]
      maxContactJoints 20
    }}
    ContactProperties {{
      material1 "Floor"
      coulombFriction [ {mu} ]
      maxContactJoints 20
    }}
  ]
"""


def main():
    mu = float(sys.argv[1])
    with open(WORLD) as f:
        text = f.read()
    new, n = re.subn(r"  contactProperties \[.*?\n  \]\n",
                     BLOCK.format(mu=mu), text, count=1, flags=re.S)
    if n != 1:
        raise SystemExit("could not find the contactProperties block")
    with open(WORLD, "w") as f:
        f.write(new)
    print(f"friction set to {mu} on foot_tip, wheel_rim and the generic Floor")


if __name__ == "__main__":
    main()
