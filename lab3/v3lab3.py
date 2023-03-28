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
import math

try:
    from PIL import Image, ImageDraw, ImageFont
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
def make_text_image(text_to_draw, x, y, font=None):

    # make a blank image for the text, initialized to opaque black
    text_image = Image.new('RGBA', cozmo.oled_face.dimensions(), (0, 0, 0, 255))

    # get a drawing context
    dc = ImageDraw.Draw(text_image)

    # draw the text
    dc.text((x, y), text_to_draw, fill=(255, 255, 255, 255), font=font)

    return text_image


# get a font - location depends on OS so try a couple of options
# failing that the default of None will just use a default font
_state_font = None
try:
    _state_font = ImageFont.truetype("arial.ttf", 20)
except IOError:
    try:
        _state_font = ImageFont.truetype("/Library/Fonts/Arial.ttf", 20)
    except IOError:
        pass

def make_text_image(text_to_draw, x, y, font=None):

    # make a blank image for the text, initialized to opaque black
    text_image = Image.new('RGBA', cozmo.oled_face.dimensions(), (0, 0, 0, 255))

    # get a drawing context
    dc = ImageDraw.Draw(text_image)

    # draw the text
    dc.text((x, y), text_to_draw, fill=(255, 255, 255, 255), font=font)

    return text_image

class CubeFSM(StateMachine):
 
    # creating states
    search = State("search", initial=True)
    rotate = State("rotate")
    drive = State("drive")
    #idle = State("idle")

    
    # transitions of the state
    align = search.to(rotate)
    go = rotate.to(drive)
    stop = drive.to(search)
    #scan = idle.to(search)
    
    #self transitions
    search_loop = search.to(search)
    rotate_loop = rotate.to.itself()
    
      
    cycle = align | go | stop
    
    arc = 39
    x = 0
    y = 0
    distance = 0
    rotations = 1
    cube_visible = 0
    theta = 0
    cube_z = 0
    
    
    
    def __init__(self, robot):
        super().__init__()
        self.robot = robot
        self.robot.add_event_handler(cozmo.objects.EvtObjectObserved, self.object_appeared)
        self.robot.add_event_handler(cozmo.objects.EvtObjectDisappeared, self.object_disappeared)
    
    def on_align(self):
         pass
         #self.rotations = -(self.rotations + np.sign(self.rotations))
    
    def on_go(self):
         pass
    
    def on_stop(self):
         self.robot.stop_all_motors()
    
    def on_search(self):
        pass
    
    def before_cycle(self, event: str, source: State, target: State, message: str = ""):
        message = ". " + message if message else ""
        return f"Running {event} from {source.id} to {target.id}{message}"
    
    
    def set_cube_pose(self, x, y):
        self.x = x
        self.y = y
    
    def set_cube_dist(self, dist):
        self.distance = dist
    
    def recal(self):
        self.rotations = 1
    
    def object_appeared(self, evt, **kw):
            if isinstance(evt.obj, CustomObject):
               print("Cozmo started seeing a %s" % str(evt.obj.object_type))
               print("cube pose: " + str(evt.pose.position))
               print("cube angle: " + str(evt.pose.rotation.angle_z))
               self.cube_visible = 1
               self.x = evt.pose.position.x * math.cos(self.robot.pose.rotation.angle_z.radians) - evt.pose.position.y * math.sin(self.robot.pose.rotation.angle_z.radians) + self.robot.pose.position.x
               self.y = evt.pose.position.x * math.sin(self.robot.pose.rotation.angle_z.radians) + evt.pose.position.y * math.cos(self.robot.pose.rotation.angle_z.radians) + self.robot.pose.position.y
               self.distance = math.sqrt((self.x**2) + (self.y**2))
               print("cube w.r.t robot: ", self.x, self.y, self.distance)
               print("robot pose: " + str(self.robot.pose.position))
               print("robot angle", self.robot.pose.rotation.angle_z)
               self.theta = math.atan(self.y/self.x)
               self.cube_z = evt.pose.rotation.angle_z.radians
        
    def object_disappeared(self, evt, **kw):
            if isinstance(evt.obj, CustomObject):
               print("Cozmo stopped seeing a %s" % str(evt.obj.object_type))
               self.cube_visible = 0

async def run(robot: cozmo.robot.Robot):
    robot.world.image_annotator.annotation_enabled = False
    robot.world.image_annotator.add_annotator('box', BoxAnnotator)
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = True
    robot.camera.enable_auto_exposure = True
    fixed_gain, exposure, mode = 3, 27, 0
    
    cube_fsm = CubeFSM(robot)
    
    cube_obj1 = await robot.world.define_custom_cube(CustomObjectTypes.CustomType00,
                                              CustomObjectMarkers.Circles4,
                                              44, 35, 35, True)

    cube_obj2 = await robot.world.define_custom_cube(CustomObjectTypes.CustomType01,
                                              CustomObjectMarkers.Diamonds5,
                                              44, 35, 35, True)

    if (cube_obj1 is not None) and (cube_obj2 is not None):
        print("All cubes defined successfully!")
    else:
        print("One or more object definitions failed!")
    
    cube_order = 0
    leftwheel = 12
    rightwheel = 12
    
    try:
        while True:
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)   #get camera image
            if event.image is not None:

                if mode == 1:
                    robot.camera.enable_auto_exposure = True
                else:
                    robot.camera.set_manual_exposure(exposure,fixed_gain)
                
                
                #find pose of cube1 and cube2

                #rel_pose1 = cube_obj1.pose.define_pose_relative_to(robot.pose)
                #rel_pose2 = cube_obj2.pose.define_pose_relative_to(robot.pose)
                
                #x1, y1, z1 = rel_pose1.position.x_y_z
                #x2, y2, z2 = rel_pose2.position.x_y_z
                
                #q0, q1, q2, q3 = rel_pose1.rotation.q0_q1_q2_q3
                
                #FSM control
                state_image = make_text_image(str(cube_fsm.current_state.id), 16, 16, _state_font)

                oled_face_data = cozmo.oled_face.convert_image_to_screen_data(state_image)

                robot.display_oled_face_image(oled_face_data, 1000 , in_parallel=True)
                
                #print("cube_visible: %d ,", cube_fsm.cube_visible)
                #print("is_visible: %d ", cube_obj1.is_visible)
                #print("robot pose: ", cube_obj1.pose)
                r_sign = np.sign(cube_fsm.theta)
                if(cube_order == 0):
                  if(str(cube_fsm.current_state.id) == "search"):
                     if(cube_fsm.cube_visible):
                        robot.stop_all_motors()
                        robot.play_audio(cozmo.audio.AudioEvents.SfxGameWin)
                        cube_fsm.cycle()
                        
                     #print(str(cube_fsm.current_state.id))
                     if(cube_fsm.y > 0 and cube_fsm.cube_visible == 0):
                        await robot.drive_wheels((-1*leftwheel), rightwheel)
                     elif(cube_fsm.y < 0 and cube_fsm.cube_visible == 0):
                        await robot.drive_wheels(leftwheel, (-1*rightwheel))
                     else:
                        await robot.drive_wheels((-1*leftwheel), rightwheel)
                  elif(str(cube_fsm.current_state.id) == "rotate"):
                    
                    print(str(cube_fsm.current_state.id))
                    if(cube_fsm.y > -80 and cube_fsm.y < 80 ):
                        cube_fsm.cycle()
                        robot.play_audio(cozmo.audio.AudioEvents.SfxGameWin)
                    if(r_sign == 1):
                        await robot.drive_wheels(-5, 5)
                    if(r_sign == -1):
                        await robot.drive_wheels(5, -5)
                    if(cube_fsm.cube_visible == 0):
                        print(str(cube_fsm.current_state.id))
                        cube_fsm.cycle()
                        robot.play_audio(cozmo.audio.AudioEvents.SfxGameWin)
                  elif(str(cube_fsm.current_state.id) == "drive"):
                    await robot.drive_wheels(35, 35)
                    if(cube_fsm.distance < 40):
                        print(str(cube_fsm.current_state.id))
                        cube_fsm.cycle()
                        robot.play_audio(cozmo.audio.AudioEvents.SfxGameWin)
                    
                    if(cube_fsm.cube_visible == 0):
                        print(str(cube_fsm.current_state.id))
                        robot.stop_all_motors()
                        cube_fsm.cycle()
                        robot.play_audio(cozmo.audio.AudioEvents.SfxGameWin)
                  #elif(str(cube_fsm.current_state.id) == "idle"):
                    #if(cube_fsm.cube_visible == 0):
                        #print(str(cube_fsm.current_state.id))
                        #cube_fsm.cycle()
                        #robot.play_audio(cozmo.audio.AudioEvents.SfxGameWin)
                        
                
                  
    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)
    #cv2.destroyAllWindows()
    


if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)