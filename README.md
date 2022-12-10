# Darwin Mini



https://github.com/poppy-project/pypot
https://github.com/stephane-caron/pymanoid
https://scaron.info/teaching/how-do-biped-robots-walk.html
https://scaron.info/teaching/prototyping-a-walking-pattern-generator.html
https://osqp.org/docs/examples/mpc.html
http://poppy:8888/tree
https://github.com/petercorke/robotics-toolbox-python
https://github.com/RobotLocomotion/drake


### How to build the V-REP scene (for use with CoppeliaSim simulator)

- Follow the explanations here: https://www.coppeliarobotics.com/helpFiles/en/buildingAModelTutorial.htm
- The Collada import plugin is broken so can only be used once per session (to use it again, need to restart CoppeliaSim)
- Import .dae composed of multiple meshes, but do not check the checkbox to group them, as it would loose their individual material (color) information.  Rather, group them after import through the right click menu.
- Important!  Connect joint to respondable body, and respondable body to joint.  Visual body is connected to the respondable body, but NOT in the dynamic chain.


### Issues

- when starting the notebook to connect to the V-REP simulator, make sure no other client is connected to V-REP or the connection will fail
- need to use `ikpy==2.3.3`, as it pypot is not compatible with the latest version (3.x)
- the wrapper around system time (`from . import pypot_time as time` in `stoppablethread.py`) caused threads not to start properly -> had to get rid of this functionality in pypot

