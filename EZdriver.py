'''
Created on Apr 4, 2012

@author: lanquarden
Edited by General Motors
'''
from enum import Enum
import msgParser
import carState
import carControl
import math
import random
STUCK_TIME = 25

def sigmoid(x):
  return (2 / (1 + math.exp(-x))) - 1

class Direction(Enum):
    FORWARD = 1
    OPPOSITE = -1


class Driver(object):
    '''
    A driver object for the SCRC
    '''

    def __init__(self, stage, lanes_str, seed , lane = 0):
        '''Constructor'''
        self.WARM_UP = 0
        self.QUALIFYING = 1
        self.RACE = 2
        self.UNKNOWN = 3
        self.stage = stage

        self.parser = msgParser.MsgParser()

        self.state = carState.CarState()

        self.control = carControl.CarControl()

#        self.steer_lock = 0.366519
        self.steer_lock = math.pi / 4

        self.max_speed = 200
        self.prev_rpm = None

        self.stuckCounter = 0
        self.bringingCartBack = 0

        self.lanes = [Direction[x] for x in lanes_str.split(',')]
        random.seed(a=seed)  # Set a random seed
        
        if (len (self.lanes) == 1):
            self.pos_target = 0
        else:
            if (lane == 0):
                self.pos_target = - 0.5
            else:
                self.pos_target = 0.5
        
        self.distance_to_change_speed = 150
        self.distance_corner = 70
        
#        OPPUNENTS_LEN = 36 
#        TRACK_LEN = 19
#        
#        ACT_ACCEL = 0
#        ACT_BRAKE = 1
#        ACT_STEER = 2
#        ACT_NUM   = 3
#        
#        STATE_ANGLE = 0
#        STATE_OPONENTS = 1
#        STATE_SPEED_X = STATE_OPONENTS + OPPUNENTS_LEN
#        STATE_SPEED_Y = STATE_SPEED_X + 1
#        STATE_TRACK = STATE_SPEED_Y + 1
#        STATE_TRACK_POS = STATE_TRACK + TRACK_LEN
#        
#        STATE_NUM = STATE_TRACK_POS + 1
#        
#        self.actuators = [0] * ACT_NUM
#        self.sensors = [0] * STATE_NUM
#        self.weights = [[0] * ACT_NUM] * STATE_NUM
#        
#        self.weights[ACT_ACCEL][STATE_SPEED_X] = 0.5
        
    def init(self):
        '''Return init string with rangefinder angles'''
        self.angles = [0 for x in range(19)]
        
        for i in range(5):
            self.angles[i] = -90 + i * 15
            self.angles[18 - i] = 90 - i * 15

        for i in range(5, 9):
            self.angles[i] = -20 + (i-5) * 5
            self.angles[18 - i] = 20 - (i-5) * 5
        
        return self.parser.stringify({'init': self.angles})

    def drive(self, msg):
        self.state.setFromMsg(msg)

        if self.is_stuck():
            self.bringCarBackOnTrack()
        else:
            self.steer()
            self.gear()
            self.speed()

        return self.control.toMsg()

    def is_stuck(self):
        if self.state.trackPos > 1.0 or self.state.trackPos < -1.0:
            # update stuckCounter counter
            self.stuckCounter += 1
            self.bringingCartBack = 150
        else:
            if self.bringingCartBack != 0:
                self.bringingCartBack = self.bringingCartBack - 1
            else:
                # if not stuckCounter reset stuckCounter counter
                self.stuckCounter = 0
        return self.stuckCounter > STUCK_TIME

    def bringCarBackOnTrack(self):
        # Set gear and steering command assuming car is
        # pointing in a direction out of track

        # If car is pointing in direction of the street
        if self.state.angle * self.state.trackPos > 0.0:
            self.control.gear = 1
            self.control.steer = - self.state.angle / 4
        # Back of car is pointing into direction of street
        else:
            # to bring car parallel to track axis
            self.control.steer = self.state.angle / 4  # steerLock;
            self.control.gear = -1  # gear R

        if self.bringingCartBack < 5:

            self.control.accel = 0
            self.control.brake = 1
            self.control.gear = 0
            self.control.steer = 0

        else:

            # Build a CarControl variable and return it

            self.control.accel = 0.3
            self.control.brake = 0
        self.control.clutch = 0
        self.control.focus = 0
        self.control.meta = 0
        return

    def steer(self):
        angle = self.state.angle
#        dist = self.state.trackPos
        dist = self.state.trackPos - self.pos_target
        
        self.control.steer = (angle - dist*0.5)/self.steer_lock
#        self.control.steer = 0.5*(angle - dist)/self.steer_lock

    def gear(self):
        rpm = self.state.rpm
        gear = self.state.gear
        
        if self.prev_rpm is None:
            up = True
        else:
            if (self.prev_rpm - rpm) < 0:
                up = True
            else:
                up = False
        
        if up and rpm > 7000:
            gear += 1
        
        if not up and rpm < 3000:
            gear -= 1

        self.control.gear = gear
        
    def calculateTurnRadius( self ):
        point1_y = math.sin( self.angles[ 8 ] ) * self.state.track[ 8 ]
        point1_x = math.cos( self.angles[ 8 ] ) * self.state.track[ 8 ]
        point2_y = math.sin( self.angles[ 9 ] ) * self.state.track[ 9 ]
        point2_x = math.cos( self.angles[ 9 ] ) * self.state.track[ 9 ]
        point3_y = math.sin( self.angles[ 10 ] ) * self.state.track[ 10 ]
        point3_x = math.cos( self.angles[ 10 ] ) * self.state.track[ 10 ]
        
        slope_1_2 = ( point2_y - point1_y ) / ( point2_x - point1_x )
        slope_2_3 = ( point3_y - point2_y ) / ( point3_x - point2_x )
        
        x_center = ( slope_1_2 * slope_2_3 * ( point1_y - point3_y ) + slope_2_3 * ( point1_x + point2_x ) - slope_1_2 * ( point2_x + point3_x ) ) / ( 2 * ( slope_2_3 - slope_1_2 ) )
        y_center = -1 * ( x_center -  ( ( point1_x + point2_x ) / 2 ) ) / slope_1_2 + ( point1_y + point2_y ) / 2
        
        radius = math.sqrt( math.pow( y_center - point1_y, 2 ) + ( math.pow( x_center - point1_x, 2 ) ) )
        return radius
        
    def speed(self):
        speed = self.state.speedX
        accel = self.control.accel
        
        ct = self.state.track[9]
        if (ct > self.distance_to_change_speed):
            self.desired_speed = self.max_speed
        elif (ct > self.distance_corner):
            self.desired_speed = self.max_speed - self.state.speedX
        else:
            self.desired_speed = 2 * self.steer_lock * self.calculateTurnRadius()
            print "corner"
        print self.desired_speed
        
#        brake = 0     
#        if speed < self.desired_speed:
#            accel += 0.1
#            if accel > 1:
#                accel = 1.0
#        else:
#            accel -= 0.1
#            if accel < 0:
#                brake = - accel
#                accel = 0.0
#                
##
#        self.control.accel = accel
#        self.control.brake = brake


#        accel_brake = sigmoid(self.state.speedX - self.desired_speed)

        accel_brake = sigmoid(self.desired_speed - self.state.speedX)

        print accel_brake
        
        if (accel_brake) > 0:
            self.control.accel = accel_brake
            self.control.brake = 0
        else:
            self.control.brake = -accel_brake
            self.control.accel = 0
#            


    def onShutDown(self):
        pass
    
    def onRestart(self):
        pass
        