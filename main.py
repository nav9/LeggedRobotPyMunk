import sys
import pygame
import random
import pymunk
import numpy as np
from pymunk import Vec2d
import pymunk.pygame_util
from pygame.color import THECOLORS
from pygame.locals import USEREVENT, QUIT, KEYDOWN, KEYUP, K_s, K_q, K_n, K_r, K_q, K_ESCAPE, K_UP, K_DOWN, K_RIGHT, K_LEFT

class Simulator(object):
    def __init__(self):
        self.display_flags = 0
        self.screenWidth = 800
        self.screenHeight = 400
        self.display_size = (self.screenWidth, self.screenHeight)
        self.space = pymunk.Space()
        self.space.gravity = (0.0, -1900.0)
        #self.space.damping = 0.999 # to prevent it from blowing up.

        #---Pymunk physics coordinates start from the lower right-hand corner of the screen.
        self.ground_y = 50
        self.createGround()
        self.screen = None
        self.draw_options = None
        maxMotorRate = 6
        motorRateRangePieces = (maxMotorRate * 2 + 1) * 10
        self.motorRateRange = np.linspace(-maxMotorRate, maxMotorRate, motorRateRangePieces)
        self.fps = 50
        self.countdown = self.fps
        self.statsPos = Vec2d(0, 0)
        self.displayStr = "Keys: 'n' to set new motor rates. 'q' to quit"
        self.startPosAndAngles = None        
        self.endPosAndAngles = None
        self.prevEndPosAndAngles = None
        
    def createGround(self):       
        thickness = 5; firstEndpoint = (0, self.ground_y); secondEndpoint = (self.screenWidth, self.ground_y)
        ground = pymunk.Segment(self.space.static_body, firstEndpoint, secondEndpoint, thickness)
        ground.friction = 20.0; ground.color = 50,50,50
        self.space.add(ground)       
        
    def displayStats(self):
        self.screen.blit(self.font.render(self.displayStr, 1, THECOLORS["gray"]), self.statsPos)         
        
    def setRandomMotorRates(self):
        self.motor_ba1Left.rate = random.choice(self.motorRateRange)
        self.motor_ba1Right.rate = random.choice(self.motorRateRange)
        self.motor_ac1Left.rate = random.choice(self.motorRateRange)
        self.motor_ac1Right.rate = random.choice(self.motorRateRange)
        print('.........................')
        print('Motor rates: ', self.motor_ba1Left.rate, self.motor_ba1Left.rate, self.motor_ac1Left.rate, self.motor_ac1Left.rate)

    def resetBodies(self):
        self.startPosAndAngles = []
        for body in self.space.bodies:
            if not hasattr(body, 'start_position'):
                continue
            body.position = Vec2d(body.start_position)
            body.force = 0, 0
            body.torque = 0
            body.velocity = 0, 0
            body.angular_velocity = 0
            body.angle = body.start_angle
            self.startPosAndAngles.append(body.position)
            self.startPosAndAngles.append(body.angle)
        print('Started at: ', self.startPosAndAngles)            
    
    def checkEndPositions(self):
        self.endPosAndAngles = []
        for body in self.space.bodies:
            if not hasattr(body, 'start_position'):
                continue
            self.endPosAndAngles.append(body.position)
            self.endPosAndAngles.append(body.angle)
        print('Ended at: ', self.endPosAndAngles)  
        if self.startPosAndAngles == None: 
            self.startPosAndAngles = self.endPosAndAngles
            self.prevEndPosAndAngles = self.endPosAndAngles
        difference = [] 
        for i in range(len(self.endPosAndAngles)):
            if type(self.endPosAndAngles[i]) is 'pymunk.vec2d.Vec2d':
                difference.append(self.prevEndPosAndAngles[i].get_distance(self.endPosAndAngles[i]))
            else:
                difference.append(self.prevEndPosAndAngles[i]-self.endPosAndAngles[i])
        print("Difference in prev vs. current end: ", difference, "\n")
        self.prevEndPosAndAngles = self.endPosAndAngles
    
    def countdownForReset(self):
        if self.countdown <= 0: 
            self.countdown = self.fps
            self.checkEndPositions()
            self.resetBodies()
        self.countdown -= 1   

    def draw(self):        
        self.screen.fill(THECOLORS["black"])### Clear the screen        
        self.space.debug_draw(self.draw_options)### Draw space  
        self.displayStats()      
        pygame.display.flip()### All done, lets flip the display
    
    def createRobot(self):        
        # Create the spider
        chWd = 30; chHt = 20
        heightAboveGround = 20     
        chassisMass = 5; robotPartsFriction = 20.0
        chassisXY = Vec2d(self.display_size[0]/2, self.ground_y+heightAboveGround)
   
        legWd_a = 20; legHt_a = 2
        legWd_b = 20; legHt_b = 2
        legMass = 0.5
        relativeAnguVel = 0
        
        #---chassis
        chassis_b = pymunk.Body(chassisMass, pymunk.moment_for_box(chassisMass, (chWd, chHt)))
        chassis_b.position = chassisXY
        chassis_b.start_position = chassisXY
        chassis_b.start_angle = chassis_b.angle
        chassis_shape = pymunk.Poly.create_box(chassis_b, (chWd, chHt))
        chassis_shape.color = 0, 100, 200
        chassis_shape.friction = robotPartsFriction
        print("chassis position", chassis_b.position, ", chassis angle",chassis_b.angle)
        
        #---first left leg a
        leftLeg_1a_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_a, legHt_a)))
        leftLeg_1a_body.position = chassisXY - ((chWd/2)+(legWd_a/2), 0)
        leftLeg_1a_shape = pymunk.Poly.create_box(leftLeg_1a_body, (legWd_a, legHt_a))        
        leftLeg_1a_shape.color = 150, 150, 150
        leftLeg_1a_shape.friction = robotPartsFriction
        leftLeg_1a_body.start_position = leftLeg_1a_body.position
        leftLeg_1a_body.start_angle = leftLeg_1a_body.angle  

        #---first left leg b
        leftLeg_1b_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_b, legHt_b)))
        leftLeg_1b_body.position = leftLeg_1a_body.position - ((legWd_a/2)+(legWd_b/2), 0)
        leftLeg_1b_shape = pymunk.Poly.create_box(leftLeg_1b_body, (legWd_b, legHt_b))        
        leftLeg_1b_shape.color = 150, 150, 150
        leftLeg_1b_shape.friction = robotPartsFriction
        leftLeg_1b_body.start_position = leftLeg_1b_body.position
        leftLeg_1b_body.start_angle = leftLeg_1b_body.angle                

        #---first right leg a
        rightLeg_1a_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_a, legHt_a)))
        rightLeg_1a_body.position = chassisXY + ((chWd/2)+(legWd_a/2), 0)
        rightLeg_1a_shape = pymunk.Poly.create_box(rightLeg_1a_body, (legWd_a, legHt_a))        
        rightLeg_1a_shape.color = 150, 150, 150
        rightLeg_1a_shape.friction = robotPartsFriction       
        rightLeg_1a_body.start_position = rightLeg_1a_body.position
        rightLeg_1a_body.start_angle = rightLeg_1a_body.angle        

        #---first right leg b
        rightLeg_1b_body = pymunk.Body(legMass, pymunk.moment_for_box(legMass, (legWd_b, legHt_b)))
        rightLeg_1b_body.position = rightLeg_1a_body.position + ((legWd_a/2)+(legWd_b/2), 0)
        rightLeg_1b_shape = pymunk.Poly.create_box(rightLeg_1b_body, (legWd_b, legHt_b))        
        rightLeg_1b_shape.color = 150, 150, 150
        rightLeg_1b_shape.friction = robotPartsFriction     
        rightLeg_1b_body.start_position = rightLeg_1b_body.position
        rightLeg_1b_body.start_angle = rightLeg_1b_body.angle        

        #---link left leg b with left leg a       
        pj_ba1left = pymunk.PinJoint(leftLeg_1b_body, leftLeg_1a_body, (legWd_b/2,0), (-legWd_a/2,0))#anchor point coordinates are wrt the body; not the space
        self.motor_ba1Left = pymunk.SimpleMotor(leftLeg_1b_body, leftLeg_1a_body, relativeAnguVel)
        #---link left leg a with chassis
        pj_ac1left = pymunk.PinJoint(leftLeg_1a_body, chassis_b, (legWd_a/2,0), (-chWd/2, 0))
        self.motor_ac1Left = pymunk.SimpleMotor(leftLeg_1a_body, chassis_b, relativeAnguVel)
        #---link right leg b with right leg a       
        pj_ba1Right = pymunk.PinJoint(rightLeg_1b_body, rightLeg_1a_body, (-legWd_b/2,0), (legWd_a/2,0))#anchor point coordinates are wrt the body; not the space
        self.motor_ba1Right = pymunk.SimpleMotor(rightLeg_1b_body, rightLeg_1a_body, relativeAnguVel)
        #---link right leg a with chassis
        pj_ac1Right = pymunk.PinJoint(rightLeg_1a_body, chassis_b, (-legWd_a/2,0), (chWd/2, 0))
        self.motor_ac1Right = pymunk.SimpleMotor(rightLeg_1a_body, chassis_b, relativeAnguVel)   
        self.setRandomMotorRates()           

        self.space.add(chassis_b, chassis_shape) 
        self.space.add(leftLeg_1a_body, leftLeg_1a_shape, rightLeg_1a_body, rightLeg_1a_shape) 
        self.space.add(leftLeg_1b_body, leftLeg_1b_shape, rightLeg_1b_body, rightLeg_1b_shape) 
        self.space.add(pj_ba1left, self.motor_ba1Left, pj_ac1left, self.motor_ac1Left)  
        self.space.add(pj_ba1Right, self.motor_ba1Right, pj_ac1Right, self.motor_ac1Right)      

        #---prevent collisions among body parts with ShapeFilter
        shape_filter = pymunk.ShapeFilter(group=1)
        chassis_shape.filter = shape_filter
        leftLeg_1a_shape.filter = shape_filter
        rightLeg_1a_shape.filter = shape_filter
        leftLeg_1b_shape.filter = shape_filter
        rightLeg_1b_shape.filter = shape_filter    

    def main(self):
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

        self.createRobot()

        while running:
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key in (K_q, K_ESCAPE)):
                    #running = False
                    sys.exit(0)
                elif event.type == KEYDOWN and event.key == K_n:
                    self.setRandomMotorRates()               
                elif event.type == KEYDOWN and event.key == K_q:
                    sys.exit()     
#                 elif event.type == KEYDOWN and event.key == K_r:
#                     self.resetBodies()                    

            self.draw()
            self.countdownForReset()

            ### Update physics
            iterations = 20
            dt = 1.0/float(self.fps)/float(iterations)
            for _ in range(iterations): #iterations to get a more stable simulation
                self.space.step(dt)

            pygame.display.flip()
            clock.tick(self.fps)

if __name__ == '__main__':
    sim = Simulator()
    sim.main()
