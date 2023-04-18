import cozmo
from cozmo.objects import CustomObject, CustomObjectMarkers, CustomObjectTypes
from cmap import *
from gui import *
from utils import *
import asyncio
import random
from time import sleep
import math

MAX_NODES = 20000
cur_path = []
rrt_wait = 1
################################################################################
# NOTE:
# Before you start, please familiarize yourself with class Node in utils.py
# In this project, all nodes are Node object, each of which has its own
# coordinate and parent if necessary. You could access its coordinate by node.x
# or node[0] for the x coordinate, and node.y or node[1] for the y coordinate
################################################################################

def step_from_to(node0, node1, limit=30):
    distance = get_dist(node0, node1)
    if distance < limit:
        return node1

    theta = np.arctan2(node1.y - node0.y, node1.x - node0.x)
    new_x = node0.x + limit * np.cos(theta)
    new_y = node0.y + limit * np.sin(theta)
    new_node = Node([new_x, new_y])

    return new_node
    ############################################################################


def node_generator(cmap):
    rand_node = None

    while True: 
      rand_node = Node([random.uniform(0, cmap.width), random.uniform(0, cmap.height)])

      if (cmap.is_inbound(rand_node) and cmap.is_inside_obstacles(rand_node) == False):
          break
    
    return rand_node


def RRT(cmap, start):
    cmap.add_node(start)
    global stopevent
    map_width, map_height = cmap.get_size()
    
    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        
        rand_node = cmap.get_random_valid_node()
        nearest_node = None
        min_dist = float("inf")
        for node in cmap._nodes:
            dist = get_dist(node, rand_node)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        new_node = step_from_to(nearest_node, rand_node)
        if (cmap.is_inside_obstacles(new_node) == False and cmap.is_collision_with_obstacles([nearest_node, new_node]) == False):
            cmap.add_node(new_node)
            cmap.add_path(nearest_node, new_node)
        ########################################################################
        sleep(0.01)
        
        if cmap.is_solved():
            for goal in cmap._goals:
               cur_path.append(goal)
               i = 0
               while cur_path[i].parent is not None:
                  cur_path.append(cur_path[i].parent)
                  i = i+1
            
            break

    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
        rrt_wait = 0
        #stopevent.set()
    else:
        print("Please try again :-(")


def nothing(x):
    pass

async def CozmoPlanning(robot: cozmo.robot.Robot):
    robot.world.image_annotator.annotation_enabled = False
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = True
    robot.camera.enable_auto_exposure = True
    fixed_gain, exposure, mode = 3, 27, 0
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global cmap, stopevent
    goal_found = 0
    
    cube_obj1 = robot.world.get_light_cube(cozmo.objects.LightCube1Id)

    cube_obj2 = robot.world.get_light_cube(cozmo.objects.LightCube2Id)
    
    cube_obj3 = robot.world.get_light_cube(cozmo.objects.LightCube3Id)
    
    if (cube_obj1 is not None) and (cube_obj2 is not None) and (cube_obj3 is not None):
        print("All cubes defined successfully!")
    else:
        print("One or more object definitions failed!")
       
    try:
        while True:
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)   #get camera image
            if event.image is not None:

                if mode == 1:
                    robot.camera.enable_auto_exposure = True
                else:
                    robot.camera.set_manual_exposure(exposure,fixed_gain)
                    
            target_cube = await robot.world.wait_for_observed_light_cube(timeout=30)
            if(target_cube is not None):
               cube_matrix = np.matrix([[target_cube.pose.position.x],[target_cube.pose.position.y],[1]])
               transform_matrix = np.array([[np.cos(robot.pose.rotation.angle_z.radians), np.sin(robot.pose.rotation.angle_z.radians), -robot.pose.position.x * np.cos(robot.pose.rotation.angle_z.radians) - robot.pose.position.y * np.sin(robot.pose.rotation.angle_z.radians)],
                  [-np.sin(robot.pose.rotation.angle_z.radians), np.cos(robot.pose.rotation.angle_z.radians), robot.pose.position.x * np.sin(robot.pose.rotation.angle_z.radians) - robot.pose.position.y * np.cos(robot.pose.rotation.angle_z.radians)],
                  [0, 0, 1]])
               robot_coord = transform_matrix * cube_matrix
               #print("found target cube: ", target_cube)
               if(target_cube.object_id == 1):
                   # 2. Use RRT to find a path to a specific face of the cube
                   #cmap.set_start(Node([robot.pose.position.x, robot.pose.position.y]))
                   if(goal_found == 0):
                     cmap.add_goal(Node([target_cube.pose.position.x, target_cube.pose.position.y]))
                     goal_found = 1
                   cube1_x = robot_coord[0,0]
                   cube1_y = robot_coord[1,0]
                   #print("x: " + str(cube1_x) + "y: " + str(cube1_y))
                   RRT(cmap, cmap.get_start())
                   
                   path_coord_list = []
                   if cmap.is_solved():
                     print("solved")
                     goal_path = cur_path
                     path_coord_list = [[node.x, node.y] for node in goal_path]
                     path_coord_list.pop()
                     for coord in path_coord_list[::-1]:
                        print("world:", coord)
                        temp_coord = np.matrix([[coord[0]],[coord[1]], [1]])
                        new_coord = transform_matrix * temp_coord
                        new_x = new_coord[0]
                        new_y = new_coord[1]
                        print("robot: ", new_x, new_y)
                        
                        theta = np.arctan2(new_y-robot.pose.position.y, new_x-robot.pose.position.x)
                        distance = np.sqrt((new_x - robot.pose.position.x) ** 2 + (new_y - robot.pose.position.y) ** 2)
                        print("distance: ", distance)
                        print("theta: ", theta)
                        await robot.turn_in_place(
                           cozmo.util.radians(theta), in_parallel=True).wait_for_completed()
                        await robot.drive_straight(cozmo.util.distance_mm(distance),
                            cozmo.util.speed_mmps(150), in_parallel=True).wait_for_completed()
                   
                   
               if(target_cube.object_id == 2):
                  cmap.add_obstacle([Node([target_cube.pose.position.x-44, target_cube.pose.position.y]),
                     Node([target_cube.pose.position.x, target_cube.pose.position.y-44]),
                        Node([target_cube.pose.position.x+44, target_cube.pose.position.y]),
                           Node([target_cube.pose.position.x, target_cube.pose.position.y+44])])
                  cube2_x = robot_coord[0,0]
                  cube2_y = robot_coord[1,0]
               if(target_cube.object_id == 3):
                  cmap.add_obstacle([Node([target_cube.pose.position.x-44, target_cube.pose.position.y]),
                     Node([target_cube.pose.position.x, target_cube.pose.position.y-44]),
                        Node([target_cube.pose.position.x+44, target_cube.pose.position.y]),
                           Node([target_cube.pose.position.x, target_cube.pose.position.y+44])])
                  cube3_x = robot_coord[0,0]
                  cube3_y = robot_coord[1,0]
    
    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)
    print("done")
    
    

      
################################################################################
#                     DO NOT MODIFY CODE BELOW                                 #
################################################################################

class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            sleep(100)
            cmap.reset()
        stopevent.set()


if __name__ == '__main__':
    global cmap, stopevent
    stopevent = threading.Event()
    cmap = CozMap("maps/emptygrid.json", node_generator)
    robot_thread = RobotThread()
    robot_thread.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()
