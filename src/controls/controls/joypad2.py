#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pygame
pygame.init()
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from mymessages.msg import ActionMesg

class JoyController(Node):
    def __init__(self):
        super().__init__('joycontroller')
        self.axis0 = 0
        self.axis1 = 0
        self.axis2 = 0
        self.axis3 = 0
        self.axis4 = 0
        self.sf = 100
        self.joysticks = {}
        self.done = False
        self.num_buttons = 0
        self.num_hats = 0 # number of hat keys (4) on left side
        self.hat = None
        self.num_axes = 0
        self.joyname = None
        self.joyid = None
        self.powerlevel = None
        self.guid = None
        self.joycount = 0
        self.sf = 100
        self.publisher_lf = self.create_publisher(Float64, '/wamv/thrusters/leftfore/thrust', 10)
        self.publisher_rf = self.create_publisher(Float64, '/wamv/thrusters/rightfore/thrust', 10)
        self.publisher_la = self.create_publisher(Float64, '/wamv/thrusters/leftaft/thrust', 10)
        self.publisher_ra = self.create_publisher(Float64, '/wamv/thrusters/rightaft/thrust', 10)
        self.publisher_action = self.create_publisher(Float32MultiArray, '/action', 10)


    def read_axis0(self,joystick):
        self.axis0 = joystick.get_axis(0)
        return self.axis0
    def read_axis1(self,joystick):
        self.axis1 = joystick.get_axis(1)
        return self.axis1
    def read_axis2(self,joystick):
        self.axis2 = joystick.get_axis(2)
        return self.axis2
    def read_axis3(self,joystick):
        self.axis3 = joystick.get_axis(3)
        return self.axis3      
    def read_axis4(self,joystick):
        self.axis4 = joystick.get_axis(4)
        return self.axis4
    def read_hat(self,joystick):
        # Work on analog only
        self.hat = joystick.get_hat(0)
        return self.hat

def main():

    clock = pygame.time.Clock()

 
    jc = JoyController()

    jc.done = False
    while not jc.done:
        # Event processing step.
        # Possible joystick events: JOYAXISMOTION, JOYBALLMOTION, JOYBUTTONDOWN,
        # JOYBUTTONUP, JOYHATMOTION, JOYDEVICEADDED, JOYDEVICEREMOVED
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                jc.done = True  # Flag that we are done so we exit this loop.

            if event.type == pygame.JOYBUTTONDOWN:
                print("Joystick button pressed.")
                if event.button == 0:
                    joystick = jc.joysticks[event.instance_id]
                    if joystick.rumble(0, 0.7, 500):
                        print(f"Rumble effect played on joystick {event.instance_id}")

            if event.type == pygame.JOYBUTTONUP:
                print("Joystick button released.")

            # Handle hotplugging
            if event.type == pygame.JOYDEVICEADDED:
                # This event will be generated when the program starts for every
                # joystick, filling up the list without needing to create them manually.
                joy = pygame.joystick.Joystick(event.device_index)
                jc.joysticks[joy.get_instance_id()] = joy
                print(f"Joystick {joy.get_instance_id()} connected")

            if event.type == pygame.JOYDEVICEREMOVED:
                del jc.joysticks[event.instance_id]
                print(f"Joystick {event.instance_id} disconnected")

        jc.joycount = pygame.joystick.get_count()
        for joystick in jc.joysticks.values():
            jc.joyid = joystick.get_instance_id()
            jc.joyname = joystick.get_name()
            jc.guid = joystick.get_guid()
            jc.powerlevel = joystick.get_power_level()
            jc.num_axes = joystick.get_numaxes()
            jc.num_buttons = joystick.get_numbuttons()
            jc.num_hats = joystick.get_numhats()
            
            lf = 0
            rf = 0
            la = 0
            ra = 0

            ch1 = -jc.read_axis0(joystick)
            ch2 = -jc.read_axis1(joystick)
            ch3 = -jc.read_axis2(joystick)
            ch4 = -jc.read_axis4(joystick)

            if ch1!=0:
                # revert diagonals lf ra
                lf = 1 - ch1
                rf = ch1
                la = ch1
                ra = 1 - ch1
            elif ch3>=-0.96 and ch3<=0.96 and ch4!=0:
                # surge
                lf = ch4
                rf = ch4
                la = ch4 
                ra = ch4 
            elif ch4>=-0.96 and ch4<=0.96 and ch3!=0:
                # sway rf ra same
                lf = 1 - ch3
                rf = ch3
                la = 1 - ch3
                ra = ch3
            else :
                lf = 0
                rf = 0
                la = 0
                ra = 0
            lf = lf*jc.sf
            rf = rf*jc.sf
            la = la*jc.sf
            ra = ra*jc.sf
            jc.publisher_lf.publish(lf)
            jc.publisher_rf.publish(rf)
            jc.publisher_la.publish(la)
            jc.publisher_ra.publish(ra)
            actionval = [lf,rf,la,ra]
            jc.publisher_action.publish(actionval)



        




        # Limit to 30 frames per second.
        clock.tick(30)


if __name__ == "__main__":
    main()
    # If you forget this line, the program will 'hang'
    # on exit if running from IDLE.
    pygame.quit()