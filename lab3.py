#!/usr/bin/env python3
#!c:/Python35/python3.exe -u
import asyncio
import sys
import cv2
import numpy as np
import cozmo
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes
import time
import os
from glob import glob
from statemachine import StateMachine, State

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')
def nothing(x):
    pass

# Define a decorator as a subclass of Annotator; displays the keypoint
class BoxAnnotator(cozmo.annotate.Annotator):

    cube = None

    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)

        if BoxAnnotator.cube is not None:

            #double size of bounding box to match size of rendered image
            BoxAnnotator.cube = np.multiply(BoxAnnotator.cube,2)

            #define and display bounding box with params:
            #msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
            box = cozmo.util.ImageBox(BoxAnnotator.cube[0]-BoxAnnotator.cube[2]/2,
                                      BoxAnnotator.cube[1]-BoxAnnotator.cube[2]/2,
                                      BoxAnnotator.cube[2], BoxAnnotator.cube[2])
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)

            BoxAnnotator.cube = None


class CubeFSM(StateMachine):
 
    # creating states
    search = State("search", initial=True)
    rotate = State("rotate")
    drive = State("drive")
    idle = State("idle")

    
    # transitions of the state
    align = search.to(rotate)
    go = rotate.to(drive)
    stop = drive.to(idle)
    scan = idle.to(search)
      
    cycle = align | go | stop | scan
    
    arc = 39
    x = 0
    y = 0
    distance = 0
    
    def __init__(self, robot):
        super().__init__()
        self.robot = robot
    
    def on_align(self):
         pass
    
    def on_go(self):
         pass
    
    def on_stop(self):
         pass
    
    def on_scan(self):
         self.robot.turn_in_place(cozmo.util.degrees(arc), in_parallel=True).wait_for_completed()
    
    def before_cycle(self, event: str, source: State, target: State, message: str = ""):
        message = ". " + message if message else ""
        return f"Running {event} from {source.id} to {target.id}{message}"
    
    def on_enter_idle(self):
        print("Don't move.")

    def on_exit_idle(self):
        print("Go ahead!")
    
    def set_cube_pose(self, x, y):
        self.x = x
        self.y = y
    
    def set_cube_dist(self, dist):
        self.distance = dist


async def run(robot: cozmo.robot.Robot):
    robot.world.image_annotator.annotation_enabled = False
    robot.world.image_annotator.add_annotator('box', BoxAnnotator)
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = True
    robot.camera.enable_auto_exposure = True
    gain, exposure, mode = 390,3,1
    
    cube_fsm = CubeFSM(robot)
    
    robot.add_event_handler(cozmo.objects.EvtObjectObserved, object_observed)
    robot.add_event_handler(cozmo.objects.EvtObjectDisappeared, object_disappeared)
    
    cube_obj1 = robot.world.define_custom_cube(CustomObjectTypes.CustomType00,
                                              CustomObjectMarkers.Circles4,
                                              44, 30, 30, True)

    cube_obj2 = robot.world.define_custom_cube(CustomObjectTypes.CustomType01,
                                              CustomObjectMarkers.Diamonds4,
                                              44, 30, 30, True)

    if (cube_obj1 is not None) and (cube_obj2 is not None):
        print("All cubes defined successfully!")
    else:
        print("One or more object definitions failed!")
        return
    
    cube_order = 0
    
    try:
        while True:
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)   #get camera image
            if event.image is not None:

                if mode == 1:
                    robot.camera.enable_auto_exposure = True
                else:
                    robot.camera.set_manual_exposure(exposure,fixed_gain)
                
                BoxAnnotator.cube = cube_obj1
                
                #find pose of cube1 and cube2
                pose1 = cube_obj1.pose
                pose2 = cube_obj2.pose
                cozmo_pose = robot.pose
                #cozmoAngle = cozmo.robot.Angle
                rel_pose1 = pose1.define_pose_relative_to(cozmo_pose)
                rel_pose2 = pose2.define_pose_relative_to(cozmo_pose)
                
                x1, y1, z1 = rel_pose1.position.x_y_z
                x2, y2, z2 = rel_pose2.position.x_y_z
                
                #q0, q1, q2, q3 = rel_pose1.rotation.q0_q1_q2_q3
                
                distance1 = cozmo_pose.position.x - x1
                distance2 = cozmo_pose.position.x - x2
                
                #FSM control
                if(cube_order == 0):
                  cube_fsm.set_cube_pose(x1, y1)
                  cube_fsm.set_cube_dist(distance1)
                  if(cube_obj1.is_visible() == False):
                     cube_fsm.scan()
                  elif(cube_fsm.current_state.id == "search"):
                     if(cube_obj1.is_visible()):
                        cube_fsm.cycle()
                  elif(cube_fsm.current_state.id == "rotate"):
                     #if cube align then cycle
                  elif(cube_fsm.current_state.id == "drive"):
                     #if cube distance is close enough then cycle
                  elif(cube_fsm.current_state.id == "idle"):
                     #idle until cube not visible
                  
                  
                  
    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)
    #cv2.destroyAllWindows()
    
    
def object_observed(evt, **kw):
    if isinstance(evt.obj, CustomObject):
        print("Cozmo started seeing a %s" % str(evt.obj.object_type))
        print("pose: " + evt.pose)


def object_disappeared(evt, **kw):
    if isinstance(evt.obj, CustomObject):
        print("Cozmo stopped seeing a %s" % str(evt.obj.object_type))
    

if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)