#!/usr/bin/env python3
#!c:/Python35/python3.exe -u
import asyncio
import sys
import cv2
import numpy as np
import cozmo
import time
import os
from glob import glob

from find_cube import *

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')
def nothing(x):
    pass

YELLOW_LOWER = np.array([9, 200, 151])
YELLOW_UPPER = np.array([22, 230, 235])

IMAGE_WIDTH = 320

GREEN_LOWER = np.array([16, 50, 10])
GREEN_UPPER = np.array([100, 200, 80])

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


async def run(robot: cozmo.robot.Robot):
    robot.world.image_annotator.annotation_enabled = False
    robot.world.image_annotator.add_annotator('box', BoxAnnotator)
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = True
    robot.camera.enable_auto_exposure = True
    gain, exposure, mode = 390,3,1

    try:
        while True:
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)   #get camera image
            if event.image is not None:
                image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_BGR2RGB)

                if mode == 1:
                    robot.camera.enable_auto_exposure = True
                else:
                    robot.camera.set_manual_exposure(exposure,fixed_gain)

                #find the cube
                cube = find_cube(image, YELLOW_LOWER, YELLOW_UPPER)
                
                if cube is not None:
                  print(cube)
                BoxAnnotator.cube = cube

                ################################################################
                # Todo: Add Motion Here
                ################################################################
                r_limit = (320 - 40) * (240 - 25)
               
                
                if (cube is None):
                    await robot.turn_in_place(
                        cozmo.util.degrees(39), in_parallel=True).wait_for_completed()
                else:  
                    last_turn = 0
                    freq = 0
                    error = (IMAGE_WIDTH / 2) - cube[0]
                    if last_turn == np.sign(error) * -1:
                        freq += 1
                    else:
                        freq = 0
                    if abs(error) > 30:
                        await robot.turn_in_place(
                            cozmo.util.degrees(np.sign(error) * (10 - freq)), in_parallel=True).wait_for_completed()
                    else:
                       size = np.pi * (cube[2]**2)
                       if size < r_limit:
                           await robot.drive_straight(cozmo.util.distance_mm(102), cozmo.util.speed_mmps(300), in_parallel=True).wait_for_completed()
   
    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)
    #cv2.destroyAllWindows()
    

if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)