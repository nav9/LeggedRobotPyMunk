![Alt text](images/leggedRobotOnTerrain.png?raw=true "Legged Robot")

# LeggedRobotPyMunk
A simple legged robot created in PyMunk. This was created to test out some scenarios where the physics appeared to behave strangely.
  
# To run  
Please use Python 3.6.
Install the following packages:  
* `pip install pygame==2.0`
* `pip install pymunk==5.7.0`  
  
and then run using:  
`python3 main.py`
  
# Known issues  
In the newer versions of PyMunk, there's some error in the `SpaceDebugDrawOptions` initialization. The error shows up as:  
`TypeError: <lambda>() missing 1 required positional argument: 'a'
Exception ignored from cffi callback <function SpaceDebugDrawOptions.__init__.<locals>.f6 at 0x7f3ce81114c0>:
Traceback (most recent call last):
  File "/home/navin/.pyenv/versions/3.9.6/lib/python3.9/site-packages/pymunk/space_debug_draw_options.py", line 110, in f6
    return self.color_for_shape(shape)
  File "/home/navin/.pyenv/versions/3.9.6/lib/python3.9/site-packages/pymunk/space_debug_draw_options.py", line 270, in color_for_shape
    return SpaceDebugColor(*shape.color)`  
I'm currently unaware of what causes the problem and I don't have time to fix it. If you figure it out, please do let me know via the Discussions option or the Issues option of this GitHub project.
