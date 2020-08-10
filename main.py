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
    
class Globals:
    screenWidth = 800
    screenHeight = 400
    groundY = 50
    FPS = 50
    motorRates = []    
    
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
        heightAboveGround = 20     
        chassisMass = 5; robotPartsFriction = 20.0
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
        self.chassis_shape.color = 0, 100, 200
        self.chassis_shape.friction = robotPartsFriction
        
        #---first left leg a
        self.leftLeg_1a_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_a, legHt_a)))
        self.leftLeg_1a_body.position = chassisXY - ((chWd/2)+(legWd_a/2), 0)
        self.leftLeg_1a_shape = pymunk.Poly.create_box(self.leftLeg_1a_body, (legWd_a, legHt_a))        
        self.leftLeg_1a_shape.color = 150, 150, 150
        self.leftLeg_1a_shape.friction = robotPartsFriction
        self.leftLeg_1a_body.start_position = self.leftLeg_1a_body.position
        self.leftLeg_1a_body.start_angle = self.leftLeg_1a_body.angle  

        #---first left leg b
        self.leftLeg_1b_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_b, legHt_b)))
        self.leftLeg_1b_body.position = self.leftLeg_1a_body.position - ((legWd_a/2)+(legWd_b/2), 0)
        self.leftLeg_1b_shape = pymunk.Poly.create_box(self.leftLeg_1b_body, (legWd_b, legHt_b))        
        self.leftLeg_1b_shape.color = 150, 150, 150
        self.leftLeg_1b_shape.friction = robotPartsFriction
        self.leftLeg_1b_body.start_position = self.leftLeg_1b_body.position
        self.leftLeg_1b_body.start_angle = self.leftLeg_1b_body.angle                

        #---first right leg a
        self.rightLeg_1a_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_a, legHt_a)))
        self.rightLeg_1a_body.position = chassisXY + ((chWd/2)+(legWd_a/2), 0)
        self.rightLeg_1a_shape = pymunk.Poly.create_box(self.rightLeg_1a_body, (legWd_a, legHt_a))        
        self.rightLeg_1a_shape.color = 150, 150, 150
        self.rightLeg_1a_shape.friction = robotPartsFriction       
        self.rightLeg_1a_body.start_position = self.rightLeg_1a_body.position
        self.rightLeg_1a_body.start_angle = self.rightLeg_1a_body.angle        

        #---first right leg b
        self.rightLeg_1b_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_b, legHt_b)))
        self.rightLeg_1b_body.position = self.rightLeg_1a_body.position + ((legWd_a/2)+(legWd_b/2), 0)
        self.rightLeg_1b_shape = pymunk.Poly.create_box(self.rightLeg_1b_body, (legWd_b, legHt_b))        
        self.rightLeg_1b_shape.color = 150, 150, 150
        self.rightLeg_1b_shape.friction = robotPartsFriction     
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
        shape_filter = pymunk.ShapeFilter(group=1)
        self.chassis_shape.filter = shape_filter
        self.leftLeg_1a_shape.filter = shape_filter
        self.rightLeg_1a_shape.filter = shape_filter
        self.leftLeg_1b_shape.filter = shape_filter
        self.rightLeg_1b_shape.filter = shape_filter 
        
    def getRobotPosition(self):
        return self.chassis_b.position  
    
    def getRobotAngle(self):
        return self.chassis_b.angle
        
    def assignRandomMotorRates(self):
        self.motor_ba1Left.rate = Globals.motorRates[0]
        self.motor_ba1Right.rate = Globals.motorRates[1]
        self.motor_ac1Left.rate = Globals.motorRates[2]
        self.motor_ac1Right.rate = Globals.motorRates[3]
        print('.........................')
        print('Motor rates: ', Globals.motorRates)            
    
class Simulator(object):
    def __init__(self):
        self.display_flags = 0
        self.screenHeight = Globals.screenHeight
        self.display_size = (Globals.screenWidth, Globals.screenHeight)
        self.space = pymunk.Space()
        self.space.gravity = (0.0, -1900.0)
        #self.space.damping = 0.999 # to prevent it from blowing up.

        #---Pymunk physics coordinates start from the lower right-hand corner of the screen.
        self.createGround()
        self.screen = None
        self.draw_options = None
        self.countdown = Globals.FPS
        self.statsPos = Vec2d(0, 0)
        self.displayStr = ["Keys:","'r' to reset existingrobot","'d' to delete existing robot and create new one","'n' to set new motor rates","'q' to quit"]
        #self.startPosAndAngles = None        
        self.endPos = None
        self.prevEndPos = None
        self.robot = None
        self.createRobot()
        maxMotorRate = 6
        motorRateRangePieces = (maxMotorRate * 2 + 1) * 10
        self.motorRateRange = np.linspace(-maxMotorRate, maxMotorRate, motorRateRangePieces)          
        self.setRandomMotorRates()        
        self.reinitializationMode = Mode.resettingRobot
        
    def createRobot(self):
        self.robot = Robot(self.space)
        
    def createGround(self):       
        thickness = 5; firstEndpoint = (0, Globals.groundY); secondEndpoint = (Globals.screenWidth, Globals.groundY)
        ground = pymunk.Segment(self.space.static_body, firstEndpoint, secondEndpoint, thickness)
        ground.friction = 20.0; ground.color = 50,50,50
        self.space.add(ground)       
        
    def displayStats(self):
        appendedString = self.displayStr[:] #copying by value instead of reference
        appendedString.append("Reinitialization mode: "+self.reinitializationMode)
        verticalSeparation = 15
        for i in range(0,len(appendedString),1): 
            self.screen.blit(self.font.render(appendedString[i], 1, THECOLORS["gray"]), self.statsPos + (0, i * verticalSeparation))

    def setRandomMotorRates(self):
        Globals.motorRates = [random.choice(self.motorRateRange), random.choice(self.motorRateRange), random.choice(self.motorRateRange), random.choice(self.motorRateRange)]
        self.robot.assignRandomMotorRates()  
        
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
        #print('Reset at: ', self.robot.getRobotPosition(), self.robot.getRobotAngle())            
    
    def checkEndPositions(self):        
        self.endPos = [self.robot.getRobotPosition(), self.robot.getRobotAngle()]
        if self.prevEndPos != None:
            print("Difference: ", self.prevEndPos[0]-self.endPos[0], self.prevEndPos[1]-self.endPos[1])
        self.prevEndPos = self.endPos[:]#copy by value            
    
    def countdownForReset(self):
        if self.countdown <= 0: 
            self.countdown = Globals.FPS
            self.checkEndPositions()
            if self.reinitializationMode == Mode.deletingRobot:
                self.robot.deleteRobot()
                self.createRobot()
                self.setRandomMotorRates()  
            if self.reinitializationMode == Mode.resettingRobot:
                self.resetBodies()
        self.countdown -= 1   

    def draw(self):        
        self.screen.fill(THECOLORS["black"])### Clear the screen        
        self.space.debug_draw(self.draw_options)### Draw space  
        self.displayStats()      
        pygame.display.flip()### All done, lets flip the display
    
    def run(self):
        pygame.init()
        self.screen = pygame.display.set_mode(self.display_size, self.display_flags)
        width, height = self.screen.get_size()
        self.draw_options = pymunk.pygame_util.DrawOptions(self.screen)
        self.draw_options.constraint_color = 100,100,100 #color of the joints of the robot

        def to_pygame(p):            
            return int(p.x), int(-p.y+height) #Small hack to convert pymunk to pygame coordinates
        def from_pygame(p):
            return to_pygame(p)

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
                    self.reinitializationMode = Mode.resettingRobot
                elif event.type == KEYDOWN and event.key == K_d:
                    self.reinitializationMode = Mode.deletingRobot                                             

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
