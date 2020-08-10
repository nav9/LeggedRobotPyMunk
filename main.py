import sys
import pygame
import random
import pymunk
import numpy as np
from pymunk import Vec2d
import pymunk.pygame_util
from pygame.color import THECOLORS
from pygame.locals import USEREVENT, QUIT, KEYDOWN, KEYUP, K_d, K_q, K_n, K_r, K_q, K_ESCAPE, K_UP, K_DOWN, K_RIGHT, K_LEFT

class Mode:
    resettingRobot = 'Resetting robot each time'
    deletingRobot = 'Deleting robot each time'
    reinitializationMode = resettingRobot
    
class Globals:
    screenWidth = 800
    screenHeight = 400
    groundY = 50
    FPS = 50
    MOVEMENT_DURATION = FPS * 2
    highFriction = 20
    motorRates = [] 
    robotColorResettingMode = (0,100,200)
    robotColorDeletingMode = (200,0,0)
    terrainColor = (100,100,100)
    groundColor = (50,50,50)
    robotLegColor = (150, 150, 150)
    robotBodyShapeFilter = pymunk.ShapeFilter(group=1)   
    
class Robot:
    def __init__(self, space):
        self.space = space      
        self.createRobot()        
    
    def deleteRobot(self):
        self.space.remove(self.chassis_b);self.space.remove(self.chassis_shape) 
        self.space.remove(self.leftLeg_1a_body);self.space.remove(self.leftLeg_1a_shape);self.space.remove(self.rightLeg_1a_body);self.space.remove(self.rightLeg_1a_shape) 
        self.space.remove(self.leftLeg_1b_body);self.space.remove(self.leftLeg_1b_shape);self.space.remove(self.rightLeg_1b_body);self.space.remove(self.rightLeg_1b_shape) 
        self.space.remove(self.pj_ba1left);self.space.remove(self.motor_ba1Left);self.space.remove(self.pj_ac1left);self.space.remove(self.motor_ac1Left)  
        self.space.remove(self.pj_ba1Right);self.space.remove(self.motor_ba1Right);self.space.remove(self.pj_ac1Right);self.space.remove(self.motor_ac1Right)     
    
    def createRobot(self):        
        # Create the spider
        chWd = 30; chHt = 20
        heightAboveGround = 50     
        chassisMass = 5
        chassisXY = Vec2d(Globals.screenWidth/2, Globals.groundY + heightAboveGround)
   
        legWd_a = 20; legHt_a = 2
        legWd_b = 20; legHt_b = 2
        legMass = 0.5
        relativeAnguVel = 0
        
        #---chassis
        self.chassis_b = pymunk.Body(chassisMass, pymunk.moment_for_box(chassisMass, (chWd, chHt)))
        self.chassis_b.position = chassisXY
        self.chassis_b.start_position = chassisXY
        self.chassis_b.start_angle = self.chassis_b.angle
        self.chassis_shape = pymunk.Poly.create_box(self.chassis_b, (chWd, chHt))
        if Mode.reinitializationMode == Mode.resettingRobot: self.setRobotToResettingColor()
        if Mode.reinitializationMode == Mode.deletingRobot: self.setRobotToDeletingColor()
        self.chassis_shape.friction = Globals.highFriction
        
        #---first left leg a
        self.leftLeg_1a_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_a, legHt_a)))
        self.leftLeg_1a_body.position = chassisXY - ((chWd/2)+(legWd_a/2), 0)
        self.leftLeg_1a_shape = pymunk.Poly.create_box(self.leftLeg_1a_body, (legWd_a, legHt_a))        
        self.leftLeg_1a_shape.color = Globals.robotLegColor
        self.leftLeg_1a_shape.friction = Globals.highFriction
        self.leftLeg_1a_body.start_position = self.leftLeg_1a_body.position
        self.leftLeg_1a_body.start_angle = self.leftLeg_1a_body.angle  

        #---first left leg b
        self.leftLeg_1b_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_b, legHt_b)))
        self.leftLeg_1b_body.position = self.leftLeg_1a_body.position - ((legWd_a/2)+(legWd_b/2), 0)
        self.leftLeg_1b_shape = pymunk.Poly.create_box(self.leftLeg_1b_body, (legWd_b, legHt_b))        
        self.leftLeg_1b_shape.color = Globals.robotLegColor
        self.leftLeg_1b_shape.friction = Globals.highFriction
        self.leftLeg_1b_body.start_position = self.leftLeg_1b_body.position
        self.leftLeg_1b_body.start_angle = self.leftLeg_1b_body.angle                

        #---first right leg a
        self.rightLeg_1a_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_a, legHt_a)))
        self.rightLeg_1a_body.position = chassisXY + ((chWd/2)+(legWd_a/2), 0)
        self.rightLeg_1a_shape = pymunk.Poly.create_box(self.rightLeg_1a_body, (legWd_a, legHt_a))        
        self.rightLeg_1a_shape.color = Globals.robotLegColor
        self.rightLeg_1a_shape.friction = Globals.highFriction       
        self.rightLeg_1a_body.start_position = self.rightLeg_1a_body.position
        self.rightLeg_1a_body.start_angle = self.rightLeg_1a_body.angle        

        #---first right leg b
        self.rightLeg_1b_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_b, legHt_b)))
        self.rightLeg_1b_body.position = self.rightLeg_1a_body.position + ((legWd_a/2)+(legWd_b/2), 0)
        self.rightLeg_1b_shape = pymunk.Poly.create_box(self.rightLeg_1b_body, (legWd_b, legHt_b))        
        self.rightLeg_1b_shape.color = Globals.robotLegColor
        self.rightLeg_1b_shape.friction = Globals.highFriction     
        self.rightLeg_1b_body.start_position = self.rightLeg_1b_body.position
        self.rightLeg_1b_body.start_angle = self.rightLeg_1b_body.angle        

        #---link left leg b with left leg a       
        self.pj_ba1left = pymunk.PinJoint(self.leftLeg_1b_body, self.leftLeg_1a_body, (legWd_b/2,0), (-legWd_a/2,0))#anchor point coordinates are wrt the body; not the space
        self.motor_ba1Left = pymunk.SimpleMotor(self.leftLeg_1b_body, self.leftLeg_1a_body, relativeAnguVel)
        #---link left leg a with chassis
        self.pj_ac1left = pymunk.PinJoint(self.leftLeg_1a_body, self.chassis_b, (legWd_a/2,0), (-chWd/2, 0))
        self.motor_ac1Left = pymunk.SimpleMotor(self.leftLeg_1a_body, self.chassis_b, relativeAnguVel)
        #---link right leg b with right leg a       
        self.pj_ba1Right = pymunk.PinJoint(self.rightLeg_1b_body, self.rightLeg_1a_body, (-legWd_b/2,0), (legWd_a/2,0))#anchor point coordinates are wrt the body; not the space
        self.motor_ba1Right = pymunk.SimpleMotor(self.rightLeg_1b_body, self.rightLeg_1a_body, relativeAnguVel)
        #---link right leg a with chassis
        self.pj_ac1Right = pymunk.PinJoint(self.rightLeg_1a_body, self.chassis_b, (-legWd_a/2,0), (chWd/2, 0))
        self.motor_ac1Right = pymunk.SimpleMotor(self.rightLeg_1a_body, self.chassis_b, relativeAnguVel)          

        self.space.add(self.chassis_b, self.chassis_shape) 
        self.space.add(self.leftLeg_1a_body, self.leftLeg_1a_shape, self.rightLeg_1a_body, self.rightLeg_1a_shape) 
        self.space.add(self.leftLeg_1b_body, self.leftLeg_1b_shape, self.rightLeg_1b_body, self.rightLeg_1b_shape) 
        self.space.add(self.pj_ba1left, self.motor_ba1Left, self.pj_ac1left, self.motor_ac1Left)  
        self.space.add(self.pj_ba1Right, self.motor_ba1Right, self.pj_ac1Right, self.motor_ac1Right)    
        
        #print('Created at: ', self.getRobotPosition(), self.getRobotAngle())       

        #---prevent collisions among body parts with ShapeFilter        
        self.chassis_shape.filter = Globals.robotBodyShapeFilter
        self.leftLeg_1a_shape.filter = Globals.robotBodyShapeFilter
        self.rightLeg_1a_shape.filter = Globals.robotBodyShapeFilter
        self.leftLeg_1b_shape.filter = Globals.robotBodyShapeFilter
        self.rightLeg_1b_shape.filter = Globals.robotBodyShapeFilter
        
    def setRobotToResettingColor(self):
        self.chassis_shape.color = Globals.robotColorResettingMode

    def setRobotToDeletingColor(self):
        self.chassis_shape.color = Globals.robotColorDeletingMode
                
    def getRobotPosition(self):
        return self.chassis_b.position  
    
    def getRobotAngle(self):
        return self.chassis_b.angle
        
    def assignGlobalMotorRates(self):
        self.motor_ba1Left.rate = Globals.motorRates[0]
        self.motor_ba1Right.rate = Globals.motorRates[1]
        self.motor_ac1Left.rate = Globals.motorRates[2]
        self.motor_ac1Right.rate = Globals.motorRates[3]          
    
class Simulator(object):
    def __init__(self):
        self.display_flags = 0
        self.screenHeight = Globals.screenHeight
        self.display_size = (Globals.screenWidth, Globals.screenHeight)
        self.space = pymunk.Space()
        self.space.gravity = (0.0, -980.0)
        #self.space.damping = 0.999 # to prevent it from blowing up.

        #---Pymunk physics coordinates start from the lower right-hand corner of the screen.
        self.createGround()
        self.screen = None
        self.draw_options = None
        self.countdown = Globals.MOVEMENT_DURATION
        self.textDisplayPos = Vec2d(0, 0)
        self.displayStr = ["Keys:","'r' to reset robot without deletion","'d' to reset robot via deletion","'n' to set new motor rates","'q' to quit"]
        #self.startPosAndAngles = None        
        self.endPos = None
        self.prevEndPos = None
        self.robot = None
        self.createRobot()
        maxMotorRate = 6
        motorRateRangePieces = (maxMotorRate * 2 + 1) * 10
        self.motorRateRange = np.linspace(-maxMotorRate, maxMotorRate, motorRateRangePieces)          
        self.setRandomMotorRates()                
        
    def createRobot(self):
        self.robot = Robot(self.space)
        
    def createGround(self):       
        thickness = 5; firstEndpoint = (0, Globals.groundY); secondEndpoint = (Globals.screenWidth, Globals.groundY)
        ground = pymunk.Segment(self.space.static_body, firstEndpoint, secondEndpoint, thickness)
        ground.friction = Globals.highFriction; ground.color = Globals.groundColor
        self.space.add(ground)   
        self.createTerrainRandomBoxesLowDense()    
        
    def displayStats(self):
        appendedString = ["Reinitialization mode: "+Mode.reinitializationMode]
        for s in self.displayStr[:]:
            appendedString.append(s)
        verticalSeparation = 15
        for i in range(0,len(appendedString),1): 
            self.screen.blit(self.font.render(appendedString[i], 1, THECOLORS["gray"]), self.textDisplayPos + (0, i * verticalSeparation))

    def setRandomMotorRates(self):
        Globals.motorRates = [random.choice(self.motorRateRange), random.choice(self.motorRateRange), random.choice(self.motorRateRange), random.choice(self.motorRateRange)]
        print('.........................\nMotor rates: ', Globals.motorRates)  
        self.prevEndPos = None        
        self.robot.assignGlobalMotorRates()  
        
    def resetBodies(self):
        for body in self.space.bodies:
            if not hasattr(body, 'start_position'):
                continue
            body.position = Vec2d(body.start_position)
            body.force = 0, 0
            body.torque = 0
            body.velocity = 0, 0
            body.angular_velocity = 0
            body.angle = body.start_angle        
    
    def checkEndPositions(self):        
        self.endPos = [self.robot.getRobotPosition(), self.robot.getRobotAngle()]
        if self.prevEndPos != None:
            print("Difference: ", self.prevEndPos[0]-self.endPos[0], self.prevEndPos[1]-self.endPos[1])
        self.prevEndPos = self.endPos[:]#copy by value            
    
    def countdownForReset(self):
        if self.countdown <= 0: 
            self.countdown = Globals.MOVEMENT_DURATION
            self.checkEndPositions()
            if Mode.reinitializationMode == Mode.deletingRobot:
                self.robot.deleteRobot()
                self.createRobot()  
                self.robot.assignGlobalMotorRates()
            if Mode.reinitializationMode == Mode.resettingRobot:
                self.resetBodies()
        self.countdown -= 1   
        
    def createTerrainRandomBoxesLowDense(self):
        numObjects = 30; debrisStartRow = 60; debrisStartCol = 200; debrisEndCol = 500; debrisMaxHt = 10; boxMinSz = 5; boxMaxSz = 10
        for _ in range(numObjects):
            col = random.randint(debrisStartCol, debrisEndCol)
            row = random.randint(debrisStartRow, debrisStartRow + debrisMaxHt)
            wid = random.randint(boxMinSz, boxMaxSz)
            ht = random.randint(boxMinSz, boxMaxSz)
            if random.random() > 0.5:
                self.createBox(col, row, wid, ht, Globals.terrainColor, None)
            else:  
                self.createSphere(col, row, ht/2)     
        
    def createBox(self, x, y, wd, ht, colour, fil):#fil can be passed as None if it is a box that the robot cannot move through
        body = pymunk.Body(body_type = pymunk.Body.KINEMATIC)
        body.position = Vec2d(x, y)
        body.width = wd
        body.height = ht
        try:
            shape = pymunk.Poly.create_box(body, (wd, ht))
            if fil: shape.filter = Globals.robotBodyShapeFilter #to create boxes that the robot can move through
            shape.color = colour
        except:
            pass
        shape.friction = Globals.highFriction
        self.space.add(shape)
        
    def createSphere(self, xPosition, yPosition, radius):
        sphereMass = 5000
        sphereInertia = pymunk.moment_for_circle(sphereMass, 0, radius, (0, 0))
        body = pymunk.Body(sphereMass, sphereInertia, body_type=pymunk.Body.KINEMATIC)
        #x = random.randint(115, 350)
        body.position = xPosition, yPosition
        shape = pymunk.Circle(body, radius, (0, 0))
        #shape.elasticity = 0.95
        shape.friction = Globals.highFriction
        shape.color = Globals.terrainColor
        self.space.add(body, shape)    

    def draw(self):        
        self.screen.fill(THECOLORS["black"])# Clear screen        
        self.space.debug_draw(self.draw_options)# Draw space  
        self.displayStats()      
        pygame.display.flip()# All done, flip the display
    
    def run(self):
        pygame.init()
        self.screen = pygame.display.set_mode(self.display_size, self.display_flags)
#         width, height = self.screen.get_size()
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)
        self.draw_options.constraint_color = 100,100,100 #color of the joints of the robot

#         def to_pygame(p):            
#             return int(p.x), int(-p.y+height) #Small hack to convert pymunk to pygame coordinates
#         def from_pygame(p):
#             return to_pygame(p)

        clock = pygame.time.Clock()
        running = True
        self.font = pygame.font.Font(None, 16)

        while running:
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key in (K_q, K_ESCAPE)):
                    #running = False
                    sys.exit(0)
                elif event.type == KEYDOWN and event.key == K_n:
                    self.setRandomMotorRates()
                elif event.type == KEYDOWN and event.key == K_q:
                    sys.exit()     
                elif event.type == KEYDOWN and event.key == K_r:
                    Mode.reinitializationMode = Mode.resettingRobot
                    self.robot.setRobotToResettingColor()
                    print('Mode: Resetting')
                elif event.type == KEYDOWN and event.key == K_d:
                    Mode.reinitializationMode = Mode.deletingRobot
                    self.robot.setRobotToDeletingColor()
                    print('Mode: Deleting')                                             

            self.draw()
            self.countdownForReset()

            ### Update physics
            iterations = 20
            dt = 1.0/float(Globals.FPS)/float(iterations)
            for _ in range(iterations): #iterations to get a more stable simulation
                self.space.step(dt)

            pygame.display.flip()
            clock.tick(Globals.FPS)

#------------------------------------
#--- Program starts here
#------------------------------------
if __name__ == '__main__':
    sim = Simulator()
    sim.run()
