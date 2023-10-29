"""define series of possible actions: create a class (base class) 
every possible action needs subclass 
in class Action (etc name): define the code that is the SAME across all actions
in each subclass: class Forward(Action)
define message to execute this movement (move the motor xyz)
DEFINE/BRAINSTORM STRUCTURE/MOTION WE CAN GET (etc. in centimeters, right turn left turn, 180deg, u-turn)
forward
backward
right
left
PARAMETER: HOW TO EXECUTE MOTION (__init___ fn) self,...length away from obstacle forward until 25cm from obstacle
degrees to turn, CW/CCW (+/- #s)
FN THAT SEQUENTIALLY EXECUTES ACTIONS*
APPS/ACTIONS.PY"""

"""
feedback linearizataion
class Action(Fwd,Bwd,Lft,Rt):
    [import button sensing info]
        calls functions in order (ex: Forward, Right, Left, Forward, Forward)
    - use motor rotations? (simpler, #)
    (1) move forward 25 cm(end up in center of box
    subclass Forward(x,y,z):
        move foward 50 cm
    subclass Right (x,y,z):
        turn 90 degrees clockwise [code]
        call Forward
    subclass Left (x,y,z):
        turn 90 degrees counter-clockwise [code]
        call Forward"""
"""
#x, y, z = placeholder variables for any that need to be imported
class Action(x,y,z):
    #move forward/North 25 cm
    def Forward(a,b,c):
        #move forward 50 cm
        test = a + b + c
        print("move 50 cm forward")
    def Right(d,e,f):
        #turn 90 degrees CW
        Forward(2,3,1)
    def Left(g,h,i):
        #turn 90 degrees CCW
        Forward(4,5,6)
"""


class Action:
    def __init__(self):
        pass

    def execute(self):
        pass


class Forward(Action):
    def __init__(self, distance: float):
        super().__init__()

        self.distance = distance

    def execute(self):
        super().execute()

        # control the motor ...
