#!/usr/bin/env python
# TODO(Georg): add license
import rospy
import anytree as at
import PyKDL as kdl
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

shelf_system_key = 'shelf_system'
layer_key = 'shelf_layer'
separator_key = 'separator'
barcode_key = 'barcode'

class Vector3d:
    def __init__(self, x = 0 ,y = 0 ,z = 0):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        return Vector3d(self.x + other.x, self.y + other.y, self.z + other.z)

    def __str__(self):
        return ("x: " + str(self.x) + " y: " + str(self.y) + " z: " + str(self.z))

class TreeNode:
    def __init__(self, name, position):
        self.name = name
        self.position = position
        self.children = []
        #self.allChildren = []
        
    def get_children_flattened(self):
        children_flattened = []
        for c in self.children:
            children_flattened.append(c.name)
            all_children_of_c = c.get_children_flattened()
            for cc in all_children_of_c:
                children_flattened.append(cc)
        return children_flattened



class GroundTruth:

    def __init__(self):
        self.root = TreeNode("root", Vector3d(0,0,0))

    def add_frame(self, parent_frame_name, child_frame_name, position):
        current_frame = self.root
        child_frame = TreeNode(child_frame_name, position)

        if parent_frame_name == self.root.name:
            #self.root.allChildren.append(child_frame_name)
            self.root.children.append(child_frame)
            return

        if parent_frame_name not in self.root.get_children_flattened():
        	print("parent " + parent_frame_name + " does not exist")
        	return

        if child_frame_name in self.root.get_children_flattened():
            print("frame " + child_frame_name + " exists already")
            return

        while(current_frame.name != parent_frame_name):
            for next_frame in current_frame.children:
                if parent_frame_name in next_frame.get_children_flattened():
                    #current_frame.allChildren.append(child_frame_name)
                    current_frame = next_frame
                    break
                if next_frame.name == parent_frame_name:
                    #current_frame.allChildren.append(child_frame_name)
                    current_frame = next_frame
                    break

        #current_frame.allChildren.append(child_frame_name)
        current_frame.children.append(child_frame)


    def get_root_vector(self, frame_name):
        if(frame_name == self.root.name):
            return self.root.position

        if frame_name not in self.root.get_children_flattened():
            print("Frame " + frame_name +  " does not exist")
            return Vector3d()


        root_vector = self.root.position
        v = Vector3d(root_vector.x, root_vector.y, root_vector.z)
        current_frame = self.root
        

        while(current_frame.name != frame_name):
            for next_frame in current_frame.children:
                if frame_name in next_frame.get_children_flattened():
                    v = v + current_frame.position
                    current_frame = next_frame
                    break
                if next_frame.name == frame_name:
                    v = v + current_frame.position
                    current_frame = next_frame
                    break

        v = v + current_frame.position

        return v




    def get_vector_between_frames(self, frame1, frame2):
        t1 = self.get_root_vector(frame1)
        t2 = self.get_root_vector(frame2)
        return Vector3d(t2.x - t1.x, t2.y - t1.y, t2.z - t1.z)




    def print_tree(self, frame = None, depth = 0):
        if(frame == None):
            frame = self.root
        s = ""
        for i in range(0, depth):
            s = s + "     "
        print((s + "frame name: " + frame.name + " position: " + str(frame.position) + "\n").rstrip())
        for child_frame in frame.children:
            self.print_tree(child_frame, depth + 1)


def instantiate_ground_truth():
    layer_to_seperator_y = 0.01
    layer_to_seperator_z = 0.005

    layer_to_barcode_z = -0.035
    layer_to_barcode_y = -0.028


    gt = GroundTruth()

    gt.add_frame("root", "shelf1_root", Vector3d(1,0,0))

    gt.add_frame("shelf1_root", "shelf1_layer1", Vector3d(0.0, -0.028, 0.154))

    gt.add_frame("shelf1_layer1", "shelf_layer1_seperator_1", Vector3d(0.005, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf1_layer1", "shelf1_layer1_seperator_2", Vector3d(0.100, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf1_layer1", "shelf1_layer1_seperator_3", Vector3d(0.240, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf1_layer1", "shelf1_layer1_seperator_4", Vector3d(0.382, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer1", "shelf1_layer1_seperator_5", Vector3d(0.511, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer1", "shelf1_layer1_seperator_6", Vector3d(0.640, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer1", "shelf1_layer1_seperator_7", Vector3d(0.766, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer1", "shelf1_layer1_seperator_8", Vector3d(0.867, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer1", "shelf1_layer1_seperator_9", Vector3d(0.980, layer_to_seperator_z, layer_to_seperator_z))

    gt.add_frame("shelf1_layer1", "027995", Vector3d(0.030, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer1", "544205", Vector3d(0.162, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer1", "384160", Vector3d(0.297, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer1", "457319", Vector3d(0.428, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer1", "534812", Vector3d(0.572, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer1", "402610", Vector3d(0.697, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer1", "433961", Vector3d(0.806, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer1", "507923", Vector3d(0.917, layer_to_barcode_y, layer_to_barcode_z))

    gt.add_frame("shelf1_layer1", "shelf1_layer2", Vector3d(0, 0.102, 0.4))

    gt.add_frame("shelf1_layer2", "shelf1_layer2_seperator_1", Vector3d(0.005, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer2", "shelf1_layer2_seperator_2", Vector3d(0.155, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer2", "shelf1_layer2_seperator_3", Vector3d(0.218, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer2", "shelf1_layer2_seperator_4", Vector3d(0.280, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer2", "shelf1_layer2_seperator_5", Vector3d(0.361, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer2", "shelf1_layer2_seperator_6", Vector3d(0.561, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer2", "shelf1_layer2_seperator_7", Vector3d(0.609, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer2", "shelf1_layer2_seperator_8", Vector3d(0.749, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer2", "shelf1_layer2_seperator_9", Vector3d(0.858, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer2", "shelf1_layer2_seperator_10", Vector3d(0.980, layer_to_seperator_z, layer_to_seperator_z))

    gt.add_frame("shelf1_layer2", "500183", Vector3d(0.065, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer2", "046088", Vector3d(0.197, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer2", "262289", Vector3d(0.302, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer2", "010055", Vector3d(0.414, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer2", "015652", Vector3d(0.538, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer2", "516937", Vector3d(0.664, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer2", "125481", Vector3d(0.906, layer_to_barcode_y, layer_to_barcode_z))

    gt.add_frame("shelf1_layer2", "shelf1_layer3", Vector3d(0 ,0, 0.34))

    gt.add_frame("shelf1_layer3", "shelf1_layer3_seperator_1", Vector3d(0.005, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer3", "shelf1_layer3_seperator_2", Vector3d(0.150, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer3", "shelf1_layer3_seperator_3", Vector3d(0.364, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer3", "shelf1_layer3_seperator_4", Vector3d(0.513, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer3", "shelf1_layer3_seperator_5", Vector3d(0.771, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer3", "shelf1_layer3_seperator_6", Vector3d(0.857, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer3", "shelf1_layer3_seperator_7", Vector3d(0.999, layer_to_seperator_z, layer_to_seperator_z))

    gt.add_frame("shelf1_layer3", "004728", Vector3d(0.056, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer3", "196068", Vector3d(0.240, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer3", "332384", Vector3d(0.426, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer3", "523129", Vector3d(0.584, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer3", "424929", Vector3d(0.772, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer3", "235542", Vector3d(0.902, layer_to_barcode_y, layer_to_barcode_z))

    gt.add_frame("shelf1_layer3", "shelf1_layer4", Vector3d(0 ,0, 0.298))

    gt.add_frame("shelf1_layer4", "shelf1_layer4_seperator_1", Vector3d(0.005, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer4", "shelf1_layer4_seperator_2", Vector3d(0.181, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer4", "shelf1_layer4_seperator_3", Vector3d(0.294, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer4", "shelf1_layer4_seperator_4", Vector3d(0.433, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer4", "shelf1_layer4_seperator_5", Vector3d(0.625, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer4", "shelf1_layer4_seperator_6", Vector3d(0.824, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer4", "shelf1_layer4_seperator_7", Vector3d(0.990, layer_to_seperator_z, layer_to_seperator_z))

    gt.add_frame("shelf1_layer4", "114035", Vector3d(0.068, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer4", "176671", Vector3d(0.217, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer4", "422771", Vector3d(0.328, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer4", "331372", Vector3d(0.498, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer4", "377954", Vector3d(0.701, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer4", "227003", Vector3d(0.874, layer_to_barcode_y, layer_to_barcode_z))

    gt.add_frame("shelf1_layer4", "shelf1_layer5", Vector3d(0 ,0, 0.255))

    gt.add_frame("shelf1_layer5", "shelf1_layer5_seperator_1", Vector3d(0.005, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer5", "shelf1_layer5_seperator_2", Vector3d(0.109, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer5", "shelf1_layer5_seperator_3", Vector3d(0.308, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer5", "shelf1_layer5_seperator_4", Vector3d(0.571, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer5", "shelf1_layer5_seperator_5", Vector3d(0.774, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer5", "shelf1_layer5_seperator_6", Vector3d(0.980, layer_to_seperator_z, layer_to_seperator_z))

    gt.add_frame("shelf1_layer5", "200563", Vector3d(0.030, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer5", "447600", Vector3d(0.186, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer5", "501155", Vector3d(0.412, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer5", "501156", Vector3d(0.662, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer5", "534910", Vector3d(0.861, layer_to_barcode_y, layer_to_barcode_z))

    gt.add_frame("shelf1_layer5", "shelf1_layer6", Vector3d(0 ,0, 0.338))

    gt.add_frame("shelf1_layer6", "shelf1_layer6_seperator_1", Vector3d(0.005, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer6", "shelf1_layer6_seperator_2", Vector3d(0.111, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer6", "shelf1_layer6_seperator_3", Vector3d(0.201, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer6", "shelf1_layer6_seperator_4", Vector3d(0.282, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer6", "shelf1_layer6_seperator_5", Vector3d(0.358, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer6", "shelf1_layer6_seperator_6", Vector3d(0.425, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer6", "shelf1_layer6_seperator_7", Vector3d(0.513, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer6", "shelf1_layer6_seperator_8", Vector3d(0.648, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer6", "shelf1_layer6_seperator_9", Vector3d(0.728, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer6", "shelf1_layer6_seperator_10", Vector3d(0.814, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer6", "shelf1_layer6_seperator_11", Vector3d(0.886, layer_to_seperator_z, layer_to_seperator_z))
    gt.add_frame("shelf1_layer6", "shelf1_layer6_seperator_12", Vector3d(0.982, layer_to_seperator_z, layer_to_seperator_z))

    gt.add_frame("shelf1_layer6", "305553", Vector3d(0.044, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer6", "503131", Vector3d(0.140, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer6", "333192", Vector3d(0.219, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer6", "333140", Vector3d(0.306, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer6", "050081", Vector3d(0.371, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer6", "454259", Vector3d(0.453, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer6", "048180", Vector3d(0.565, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer6", "046445", Vector3d(0.670, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer6", "500124", Vector3d(0.755, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer6", "333154", Vector3d(0.837, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf1_layer6", "023130", Vector3d(0.923, layer_to_barcode_y, layer_to_barcode_z))



    gt.add_frame("root", "shelf2_root", Vector3d(2,0,0))
    gt.add_frame("shelf2_root", "shelf2_layer1", Vector3d(0, -0.028, 0.154))

    gt.add_frame("shelf2_layer1", "shelf2_layer1_seperator_1", Vector3d(-0.005, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer1", "shelf2_layer1_seperator_2", Vector3d(0.051, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer1", "shelf2_layer1_seperator_3", Vector3d(0.086, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer1", "shelf2_layer1_seperator_4", Vector3d(0.129, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer1", "shelf2_layer1_seperator_5", Vector3d(0.181, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer1", "shelf2_layer1_seperator_6", Vector3d(0.389, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer1", "shelf2_layer1_seperator_7", Vector3d(0.589, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer1", "shelf2_layer1_seperator_8", Vector3d(0.785, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer1", "shelf2_layer1_seperator_9", Vector3d(0.991, layer_to_seperator_y, layer_to_seperator_z))

    gt.add_frame("shelf2_layer1", "013462", Vector3d(0.030, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer1", "458823", Vector3d(0.113, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer1", "522273", Vector3d(0.268, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer1", "102696", Vector3d(0.466, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer1", "102694", Vector3d(0.676, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer1", "519151", Vector3d(0.848, layer_to_barcode_y, layer_to_barcode_z))

    gt.add_frame("shelf2_layer1", "shelf2_layer2", Vector3d(0 , 0.102, 0.190))

    gt.add_frame("shelf2_layer2", "shelf2_layer2_seperator_1", Vector3d(0.010, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer2", "shelf2_layer2_seperator_2", Vector3d(0.194, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer2", "shelf2_layer2_seperator_3", Vector3d(0.381, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer2", "shelf2_layer2_seperator_4", Vector3d(0.575, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer2", "shelf2_layer2_seperator_5", Vector3d(0.775, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer2", "shelf2_layer2_seperator_6", Vector3d(0.975, layer_to_seperator_y, layer_to_seperator_z))

    gt.add_frame("shelf2_layer2", "522990", Vector3d(0.106, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer2", "522991", Vector3d(0.274, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer2", "523009", Vector3d(0.446, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer2", "522985", Vector3d(0.653, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer2", "523010", Vector3d(0.841, layer_to_barcode_y, layer_to_barcode_z))

    gt.add_frame("shelf2_layer2", "shelf2_layer3", Vector3d(0 , 0, 0.210))

    gt.add_frame("shelf2_layer3", "shelf2_layer3_seperator_1", Vector3d(0.010, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer3", "shelf2_layer3_seperator_2", Vector3d(0.172, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer3", "shelf2_layer3_seperator_3", Vector3d(0.300, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer3", "shelf2_layer3_seperator_4", Vector3d(0.415, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer3", "shelf2_layer3_seperator_5", Vector3d(0.463, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer3", "shelf2_layer3_seperator_6", Vector3d(0.516, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer3", "shelf2_layer3_seperator_7", Vector3d(0.570, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer3", "shelf2_layer3_seperator_8", Vector3d(0.630, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer3", "shelf2_layer3_seperator_9", Vector3d(0.680, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer3", "shelf2_layer3_seperator_10", Vector3d(0.740, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer3", "shelf2_layer3_seperator_11", Vector3d(0.804, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer3", "shelf2_layer3_seperator_12", Vector3d(0.864, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer3", "shelf2_layer3_seperator_13", Vector3d(0.920, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf2_layer3", "shelf2_layer3_seperator_14", Vector3d(0.976, layer_to_seperator_y, layer_to_seperator_z))

    gt.add_frame("shelf2_layer3", "522989", Vector3d(0.075, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer3", "522988", Vector3d(0.235, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer3", "294763", Vector3d(0.351, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer3", "294760", Vector3d(0.425, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer3", "449181", Vector3d(0.471, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer3", "449182", Vector3d(0.530, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer3", "507237", Vector3d(0.587, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer3", "036942", Vector3d(0.641, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer3", "036946", Vector3d(0.699, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer3", "036945", Vector3d(0.763, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer3", "036988", Vector3d(0.824, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer3", "190755", Vector3d(0.913, layer_to_barcode_y, layer_to_barcode_z))

    gt.add_frame("shelf2_layer3", "shelf2_layer4", Vector3d(0 , 0.145, 0.495))

    gt.add_frame("shelf2_layer4", "463940", Vector3d(0.021, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer4", "155777", Vector3d(0.086, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer4", "539355", Vector3d(0.149, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer4", "356346", Vector3d(0.198, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer4", "513205", Vector3d(0.255, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer4", "544140", Vector3d(0.302, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer4", "325776", Vector3d(0.350, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer4", "045907", Vector3d(0.393, layer_to_barcode_y, layer_to_barcode_z))   
    gt.add_frame("shelf2_layer4", "461819", Vector3d(0.445, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer4", "264913", Vector3d(0.499, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer4", "264912", Vector3d(0.541, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer4", "107184", Vector3d(0.601, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer4", "362767", Vector3d(0.680, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer4", "477847", Vector3d(0.767, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer4", "545202", Vector3d(0.843, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer4", "294993", Vector3d(0.924, layer_to_barcode_y, layer_to_barcode_z))

    gt.add_frame("shelf2_layer4", "shelf2_layer5", Vector3d(0 , 0, 0.298))

    gt.add_frame("shelf2_layer5", "121582", Vector3d(0.020, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer5", "039450", Vector3d(0.074, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer5", "531041", Vector3d(0.128, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer5", "531033", Vector3d(0.177, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer5", "566325", Vector3d(0.230, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer5", "096981", Vector3d(0.293, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer5", "218409", Vector3d(0.350, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer5", "499551", Vector3d(0.418, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer5", "011850", Vector3d(0.471, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer5", "011853", Vector3d(0.525, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer5", "318407", Vector3d(0.586, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer5", "049544", Vector3d(0.655, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer5", "318497", Vector3d(0.705, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer5", "431394", Vector3d(0.755, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer5", "208112", Vector3d(0.809, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer5", "472899", Vector3d(0.870, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer5", "472897", Vector3d(0.924, layer_to_barcode_y, layer_to_barcode_z))

    gt.add_frame("shelf2_layer5", "shelf2_layer6", Vector3d(0 , 0, 0.254))

    gt.add_frame("shelf2_layer6", "349789", Vector3d(0.010, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer6", "349799", Vector3d(0.090, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer6", "349797", Vector3d(0.166, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer6", "387457", Vector3d(0.256, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer6", "515633", Vector3d(0.328, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer6", "447775", Vector3d(0.404, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer6", "535427", Vector3d(0.476, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer6", "046255", Vector3d(0.575, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer6", "479455", Vector3d(0.673, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer6", "467637", Vector3d(0.755, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer6", "317514", Vector3d(0.835, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf2_layer6", "317512", Vector3d(0.916, layer_to_barcode_y, layer_to_barcode_z))







    gt.add_frame("root", "shelf3_root", Vector3d(3,0,0))
    gt.add_frame("shelf3_root", "shelf3_layer1", Vector3d(0, -0.028, 0.154))

    gt.add_frame("shelf3_layer1", "shelf3_layer1_seperator_1", Vector3d(0.005, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer1", "shelf3_layer1_seperator_2", Vector3d(0.090, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer1", "shelf3_layer1_seperator_3", Vector3d(0.285, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer1", "shelf3_layer1_seperator_4", Vector3d(0.380, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer1", "shelf3_layer1_seperator_5", Vector3d(0.470, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer1", "shelf3_layer1_seperator_6", Vector3d(0.559, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer1", "shelf3_layer1_seperator_7", Vector3d(0.645, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer1", "shelf3_layer1_seperator_8", Vector3d(0.735, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer1", "shelf3_layer1_seperator_9", Vector3d(0.821, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer1", "shelf3_layer1_seperator_10", Vector3d(0.907, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer1", "shelf3_layer1_seperator_11", Vector3d(0.995, layer_to_seperator_y, layer_to_seperator_z))

    gt.add_frame("shelf3_layer1", "260725", Vector3d(0.075, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer1", "260707", Vector3d(0.270, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer1", "554301", Vector3d(0.402, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer1", "404491", Vector3d(0.502, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer1", "521873", Vector3d(0.634, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer1", "402589", Vector3d(0.745, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer1", "260667", Vector3d(0.889, layer_to_barcode_y, layer_to_barcode_z))

    gt.add_frame("shelf3_layer1", "shelf3_layer2", Vector3d(0 , 0.102, 0.316))

    gt.add_frame("shelf3_layer2", "shelf3_layer2_seperator_1", Vector3d(0.005, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer2", "shelf3_layer2_seperator_2", Vector3d(0.070, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer2", "shelf3_layer2_seperator_3", Vector3d(0.147, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer2", "shelf3_layer2_seperator_4", Vector3d(0.200, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer2", "shelf3_layer2_seperator_5", Vector3d(0.255, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer2", "shelf3_layer2_seperator_6", Vector3d(0.309, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer2", "shelf3_layer2_seperator_7", Vector3d(0.360, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer2", "shelf3_layer2_seperator_8", Vector3d(0.413, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer2", "shelf3_layer2_seperator_9", Vector3d(0.468, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer2", "shelf3_layer2_seperator_10", Vector3d(0.520, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer2", "shelf3_layer2_seperator_11", Vector3d(0.606, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer2", "shelf3_layer2_seperator_12", Vector3d(0.692, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer2", "shelf3_layer2_seperator_13", Vector3d(0.800, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer2", "shelf3_layer2_seperator_14", Vector3d(0.896, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer2", "shelf3_layer2_seperator_15", Vector3d(0.985, layer_to_seperator_y, layer_to_seperator_z))

    gt.add_frame("shelf3_layer2", "520698", Vector3d(0.026, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer2", "520696", Vector3d(0.096, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer2", "520689", Vector3d(0.188, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer2", "520688", Vector3d(0.295, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer2", "423376", Vector3d(0.431, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer2", "422068", Vector3d(0.550, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer2", "511860", Vector3d(0.625, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer2", "260700", Vector3d(0.736, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer2", "260695", Vector3d(0.832, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer2", "399599", Vector3d(0.923, layer_to_barcode_y, layer_to_barcode_z))

    gt.add_frame("shelf3_layer2", "shelf3_layer3", Vector3d(0 , 0, 0.296))

    gt.add_frame("shelf3_layer3", "shelf3_layer3_seperator_1", Vector3d(0.005, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer3", "shelf3_layer3_seperator_2", Vector3d(0.072, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer3", "shelf3_layer3_seperator_3", Vector3d(0.140, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer3", "shelf3_layer3_seperator_4", Vector3d(0.194, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer3", "shelf3_layer3_seperator_5", Vector3d(0.261, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer3", "shelf3_layer3_seperator_6", Vector3d(0.386, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer3", "shelf3_layer3_seperator_7", Vector3d(0.443, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer3", "shelf3_layer3_seperator_8", Vector3d(0.508, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer3", "shelf3_layer3_seperator_9", Vector3d(0.596, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer3", "shelf3_layer3_seperator_10", Vector3d(0.732, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer3", "shelf3_layer3_seperator_11", Vector3d(0.800, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer3", "shelf3_layer3_seperator_12", Vector3d(0.868, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer3", "shelf3_layer3_seperator_13", Vector3d(0.923, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer3", "shelf3_layer3_seperator_14", Vector3d(0.987, layer_to_seperator_y, layer_to_seperator_z))

    gt.add_frame("shelf3_layer3", "535675", Vector3d(0.020, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer3", "519763", Vector3d(0.093, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer3", "520209", Vector3d(0.151, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer3", "520212", Vector3d(0.213, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer3", "523828", Vector3d(0.399, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer3", "523829", Vector3d(0.467, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer3", "523823", Vector3d(0.542, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer3", "523830", Vector3d(0.644, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer3", "472787", Vector3d(0.750, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer3", "293857", Vector3d(0.820, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer3", "536979", Vector3d(0.886, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer3", "411522", Vector3d(0.951, layer_to_barcode_y, layer_to_barcode_z))

    gt.add_frame("shelf3_layer3", "shelf3_layer4", Vector3d(0 , 0, 0.295))

    gt.add_frame("shelf3_layer4", "shelf3_layer4_seperator_1", Vector3d(0.005, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer4", "shelf3_layer4_seperator_2", Vector3d(0.060, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer4", "shelf3_layer4_seperator_3", Vector3d(0.125, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer4", "shelf3_layer4_seperator_4", Vector3d(0.180, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer4", "shelf3_layer4_seperator_5", Vector3d(0.238, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer4", "shelf3_layer4_seperator_6", Vector3d(0.325, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer4", "shelf3_layer4_seperator_7", Vector3d(0.412, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer4", "shelf3_layer4_seperator_8", Vector3d(0.483, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer4", "shelf3_layer4_seperator_9", Vector3d(0.547, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer4", "shelf3_layer4_seperator_10", Vector3d(0.613, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer4", "shelf3_layer4_seperator_11", Vector3d(0.691, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer4", "shelf3_layer4_seperator_12", Vector3d(0.767, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer4", "shelf3_layer4_seperator_13", Vector3d(0.846, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer4", "shelf3_layer4_seperator_14", Vector3d(0.915, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer4", "shelf3_layer4_seperator_15", Vector3d(0.980, layer_to_seperator_y, layer_to_seperator_z))

    gt.add_frame("shelf3_layer4", "460470", Vector3d(0.016, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer4", "460201", Vector3d(0.080, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer4", "536952", Vector3d(0.132, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer4", "536943", Vector3d(0.192, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer4", "536970", Vector3d(0.265, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer4", "565329", Vector3d(0.351, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer4", "565335", Vector3d(0.435, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer4", "518875", Vector3d(0.503, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer4", "536760", Vector3d(0.560, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer4", "536803", Vector3d(0.645, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer4", "518876", Vector3d(0.725, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer4", "544375", Vector3d(0.794, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer4", "544382", Vector3d(0.860, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer4", "553733", Vector3d(0.940, layer_to_barcode_y, layer_to_barcode_z))

    gt.add_frame("shelf3_layer4", "shelf3_layer5", Vector3d(0 , 0, 0.335))

    gt.add_frame("shelf3_layer5", "shelf3_layer5_seperator_1", Vector3d(0.010, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer5", "shelf3_layer5_seperator_2", Vector3d(0.098, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer5", "shelf3_layer5_seperator_3", Vector3d(0.161, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer5", "shelf3_layer5_seperator_4", Vector3d(0.261, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer5", "shelf3_layer5_seperator_5", Vector3d(0.332, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer5", "shelf3_layer5_seperator_6", Vector3d(0.426, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer5", "shelf3_layer5_seperator_7", Vector3d(0.485, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer5", "shelf3_layer5_seperator_8", Vector3d(0.597, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer5", "shelf3_layer5_seperator_9", Vector3d(0.672, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer5", "shelf3_layer5_seperator_10", Vector3d(0.750, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer5", "shelf3_layer5_seperator_11", Vector3d(0.905, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf3_layer5", "shelf3_layer5_seperator_12", Vector3d(0.983, layer_to_seperator_y, layer_to_seperator_z))

    gt.add_frame("shelf3_layer5", "544799", Vector3d(0.030, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer5", "544760", Vector3d(0.112, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer5", "544788", Vector3d(0.190, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer5", "566217", Vector3d(0.280, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer5", "544782", Vector3d(0.362, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer5", "512018", Vector3d(0.445, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer5", "512036", Vector3d(0.522, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer5", "553735", Vector3d(0.652, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer5", "553736", Vector3d(0.770, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer5", "553726", Vector3d(0.847, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf3_layer5", "553715", Vector3d(0.929, layer_to_barcode_y, layer_to_barcode_z))









    gt.add_frame("root", "shelf4_root", Vector3d(4,0,0))
    gt.add_frame("shelf4_root", "shelf4_layer1", Vector3d(0, -0.028, 0.154))

    gt.add_frame("shelf4_layer1", "shelf4_layer1_seperator_1", Vector3d(0.005, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer1", "shelf4_layer1_seperator_2", Vector3d(0.065, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer1", "shelf4_layer1_seperator_3", Vector3d(0.135, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer1", "shelf4_layer1_seperator_4", Vector3d(0.188, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer1", "shelf4_layer1_seperator_5", Vector3d(0.250, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer1", "shelf4_layer1_seperator_6", Vector3d(0.314, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer1", "shelf4_layer1_seperator_7", Vector3d(0.381, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer1", "shelf4_layer1_seperator_8", Vector3d(0.445, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer1", "shelf4_layer1_seperator_9", Vector3d(0.523, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer1", "shelf4_layer1_seperator_10", Vector3d(0.601, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer1", "shelf4_layer1_seperator_11", Vector3d(0.759, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer1", "shelf4_layer1_seperator_12", Vector3d(0.908, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer1", "shelf4_layer1_seperator_13", Vector3d(0.980, layer_to_seperator_y, layer_to_seperator_z))

    gt.add_frame("shelf4_layer1", "250917", Vector3d(0.051, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer1", "251005", Vector3d(0.149, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer1", "253896", Vector3d(0.232, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer1", "252017", Vector3d(0.367, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer1", "378981", Vector3d(0.468, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer1", "378940", Vector3d(0.545, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer1", "347469", Vector3d(0.664, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer1", "279513", Vector3d(0.815, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer1", "404079", Vector3d(0.934, layer_to_barcode_y, layer_to_barcode_z))

    gt.add_frame("shelf4_layer1", "shelf4_layer2", Vector3d(0 , 0.102, 0.233))

    gt.add_frame("shelf4_layer2", "shelf4_layer2_seperator_1", Vector3d(0.005, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer2", "shelf4_layer2_seperator_2", Vector3d(0.165, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer2", "shelf4_layer2_seperator_3", Vector3d(0.324, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer2", "shelf4_layer2_seperator_4", Vector3d(0.396, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer2", "shelf4_layer2_seperator_5", Vector3d(0.480, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer2", "shelf4_layer2_seperator_6", Vector3d(0.546, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer2", "shelf4_layer2_seperator_7", Vector3d(0.618, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer2", "shelf4_layer2_seperator_8", Vector3d(0.690, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer2", "shelf4_layer2_seperator_9", Vector3d(0.765, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer2", "shelf4_layer2_seperator_10", Vector3d(0.838, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer2", "shelf4_layer2_seperator_11", Vector3d(0.904, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer2", "shelf4_layer2_seperator_12", Vector3d(0.985, layer_to_seperator_y, layer_to_seperator_z))

    gt.add_frame("shelf4_layer2", "470266", Vector3d(0.145, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer2", "274302", Vector3d(0.350, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer2", "278987", Vector3d(0.430, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer2", "519823", Vector3d(0.506, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer2", "278866", Vector3d(0.570, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer2", "278867", Vector3d(0.640, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer2", "497298", Vector3d(0.706, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer2", "250899", Vector3d(0.791, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer2", "522986", Vector3d(0.859, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer2", "279042", Vector3d(0.935, layer_to_barcode_y, layer_to_barcode_z))

    gt.add_frame("shelf4_layer2", "shelf4_layer3", Vector3d(0 , 0, 0.254))

    gt.add_frame("shelf4_layer3", "shelf4_layer3_seperator_1", Vector3d(0.005, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer3", "shelf4_layer3_seperator_2", Vector3d(0.081, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer3", "shelf4_layer3_seperator_3", Vector3d(0.123, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer3", "shelf4_layer3_seperator_4", Vector3d(0.196, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer3", "shelf4_layer3_seperator_5", Vector3d(0.254, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer3", "shelf4_layer3_seperator_6", Vector3d(0.354, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer3", "shelf4_layer3_seperator_7", Vector3d(0.450, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer3", "shelf4_layer3_seperator_8", Vector3d(0.560, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer3", "shelf4_layer3_seperator_9", Vector3d(0.611, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer3", "shelf4_layer3_seperator_10", Vector3d(0.670, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer3", "shelf4_layer3_seperator_11", Vector3d(0.732, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer3", "shelf4_layer3_seperator_12", Vector3d(0.782, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer3", "shelf4_layer3_seperator_13", Vector3d(0.838, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer3", "shelf4_layer3_seperator_14", Vector3d(0.896, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer3", "shelf4_layer3_seperator_15", Vector3d(0.948, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer3", "shelf4_layer3_seperator_16", Vector3d(0.988, layer_to_seperator_y, layer_to_seperator_z))

    gt.add_frame("shelf4_layer3", "469679", Vector3d(0.015, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer3", "252312", Vector3d(0.088, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer3", "251219", Vector3d(0.147, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer3", "251252", Vector3d(0.211, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer3", "309649", Vector3d(0.297, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer3", "518822", Vector3d(0.391, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer3", "256501", Vector3d(0.494, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer3", "346864", Vector3d(0.567, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer3", "251211", Vector3d(0.625, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer3", "453136", Vector3d(0.683, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer3", "476951", Vector3d(0.740, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer3", "476953", Vector3d(0.789, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer3", "250938", Vector3d(0.851, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer3", "522630", Vector3d(0.905, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer3", "522603", Vector3d(0.950, layer_to_barcode_y, layer_to_barcode_z))

    gt.add_frame("shelf4_layer3", "shelf4_layer4", Vector3d(0 , 0, 0.253))

    gt.add_frame("shelf4_layer4", "shelf4_layer4_seperator_1", Vector3d(0.005, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer4", "shelf4_layer4_seperator_2", Vector3d(0.041, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer4", "shelf4_layer4_seperator_3", Vector3d(0.112, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer4", "shelf4_layer4_seperator_4", Vector3d(0.185, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer4", "shelf4_layer4_seperator_5", Vector3d(0.262, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer4", "shelf4_layer4_seperator_6", Vector3d(0.330, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer4", "shelf4_layer4_seperator_7", Vector3d(0.385, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer4", "shelf4_layer4_seperator_8", Vector3d(0.488, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer4", "shelf4_layer4_seperator_9", Vector3d(0.531, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer4", "shelf4_layer4_seperator_10", Vector3d(0.579, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer4", "shelf4_layer4_seperator_11", Vector3d(0.620, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer4", "shelf4_layer4_seperator_12", Vector3d(0.671, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer4", "shelf4_layer4_seperator_13", Vector3d(0.747, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer4", "shelf4_layer4_seperator_14", Vector3d(0.826, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer4", "shelf4_layer4_seperator_15", Vector3d(0.886, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer4", "shelf4_layer4_seperator_16", Vector3d(0.972, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer4", "shelf4_layer4_seperator_17", Vector3d(0.985, layer_to_seperator_y, layer_to_seperator_z))

    gt.add_frame("shelf4_layer4", "251188", Vector3d(0.005, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer4", "471975", Vector3d(0.069, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer4", "250996", Vector3d(0.137, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer4", "251010", Vector3d(0.205, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer4", "453118", Vector3d(0.272, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer4", "251187", Vector3d(0.341, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer4", "253217", Vector3d(0.419, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer4", "446409", Vector3d(0.489, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer4", "457425", Vector3d(0.541, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer4", "418988", Vector3d(0.586, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer4", "535680", Vector3d(0.632, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer4", "432262", Vector3d(0.700, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer4", "515141", Vector3d(0.769, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer4", "250919", Vector3d(0.835, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer4", "250949", Vector3d(0.895, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer4", "254254", Vector3d(0.945, layer_to_barcode_y, layer_to_barcode_z))

    gt.add_frame("shelf4_layer4", "shelf4_layer5", Vector3d(0 , 0, 0.253))

    gt.add_frame("shelf4_layer5", "shelf4_layer5_seperator_1", Vector3d(0.005, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer5", "shelf4_layer5_seperator_2", Vector3d(0.065, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer5", "shelf4_layer5_seperator_3", Vector3d(0.117, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer5", "shelf4_layer5_seperator_4", Vector3d(0.170, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer5", "shelf4_layer5_seperator_5", Vector3d(0.253, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer5", "shelf4_layer5_seperator_6", Vector3d(0.330, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer5", "shelf4_layer5_seperator_7", Vector3d(0.407, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer5", "shelf4_layer5_seperator_8", Vector3d(0.509, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer5", "shelf4_layer5_seperator_9", Vector3d(0.589, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer5", "shelf4_layer5_seperator_10", Vector3d(0.639, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer5", "shelf4_layer5_seperator_11", Vector3d(0.683, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer5", "shelf4_layer5_seperator_12", Vector3d(0.724, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer5", "shelf4_layer5_seperator_13", Vector3d(0.775, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer5", "shelf4_layer5_seperator_14", Vector3d(0.850, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer5", "shelf4_layer5_seperator_15", Vector3d(0.920, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer5", "shelf4_layer5_seperator_16", Vector3d(0.985, layer_to_seperator_y, layer_to_seperator_z))

    gt.add_frame("shelf4_layer5", "253179", Vector3d(0.010, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer5", "250898", Vector3d(0.097, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer5", "534357", Vector3d(0.203, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer5", "415950", Vector3d(0.277, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer5", "415040", Vector3d(0.349, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer5", "518451", Vector3d(0.439, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer5", "366555", Vector3d(0.534, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer5", "515136", Vector3d(0.604, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer5", "256799", Vector3d(0.651, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer5", "295813", Vector3d(0.693, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer5", "415961", Vector3d(0.742, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer5", "308627", Vector3d(0.795, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer5", "256519", Vector3d(0.855, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer5", "403170", Vector3d(0.942, layer_to_barcode_y, layer_to_barcode_z))

    gt.add_frame("shelf4_layer5", "shelf4_layer6", Vector3d(0 , 0, 0.253))

    gt.add_frame("shelf4_layer6", "shelf4_layer6_seperator_1", Vector3d(0.005, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer6", "shelf4_layer6_seperator_2", Vector3d(0.047, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer6", "shelf4_layer6_seperator_3", Vector3d(0.125, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer6", "shelf4_layer6_seperator_4", Vector3d(0.193, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer6", "shelf4_layer6_seperator_5", Vector3d(0.273, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer6", "shelf4_layer6_seperator_6", Vector3d(0.351, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer6", "shelf4_layer6_seperator_7", Vector3d(0.428, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer6", "shelf4_layer6_seperator_8", Vector3d(0.476, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer6", "shelf4_layer6_seperator_9", Vector3d(0.549, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer6", "shelf4_layer6_seperator_10", Vector3d(0.620, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer6", "shelf4_layer6_seperator_11", Vector3d(0.676, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer6", "shelf4_layer6_seperator_12", Vector3d(0.721, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer6", "shelf4_layer6_seperator_13", Vector3d(0.774, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer6", "shelf4_layer6_seperator_14", Vector3d(0.829, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer6", "shelf4_layer6_seperator_15", Vector3d(0.884, layer_to_seperator_y, layer_to_seperator_z))
    gt.add_frame("shelf4_layer6", "shelf4_layer6_seperator_16", Vector3d(0.988, layer_to_seperator_y, layer_to_seperator_z))

    gt.add_frame("shelf4_layer6", "256479", Vector3d(0.005, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer6", "256500", Vector3d(0.060, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer6", "256511", Vector3d(0.138, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer6", "447612", Vector3d(0.215, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer6", "295814", Vector3d(0.296, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer6", "256505", Vector3d(0.370, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer6", "266328", Vector3d(0.433, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer6", "256506", Vector3d(0.492, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer6", "256502", Vector3d(0.563, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer6", "402186", Vector3d(0.633, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer6", "256477", Vector3d(0.683, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer6", "390842", Vector3d(0.735, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer6", "390740", Vector3d(0.804, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer6", "256476", Vector3d(0.850, layer_to_barcode_y, layer_to_barcode_z))
    gt.add_frame("shelf4_layer6", "518581", Vector3d(0.929, layer_to_barcode_y, layer_to_barcode_z))
    
    
    
    

    
    

    #gt.print_tree()
    t1 = gt.get_vector_between_frames("shelf1_root","023130")
    t2 = gt.get_vector_between_frames("305553","500124")
    t3 = gt.get_vector_between_frames("shelf1_layer5_seperator_3","shelf1_layer3")
    t4 = gt.get_vector_between_frames("shelf1_root","390740")
    #t = gt.get_vector_between_frames("root","shelf1_layer1")
    print(str(t1))
    print(str(t2))
    print(str(t3))
    print(str(t4))
    # TODO(Kevin): please complete me; feel free to add as many helper functions as you need
    pass


def to_msg(kdl_vec):
    msg = geometry_msgs.msg.Point()
    msg.x = kdl_vec.x()
    msg.y = kdl_vec.y()
    msg.z = kdl_vec.z()
    return msg


def publish_tf(data):
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    transforms = []
    now = rospy.Time.now()
    for node in at.PreOrderIter(data):
        if node.parent:
            msg = geometry_msgs.msg.TransformStamped()
            msg.header.stamp = now
            msg.header.frame_id = node.parent.name
            msg.child_frame_id = node.name
            msg.transform.rotation.w = 1.0
            msg.transform.translation = to_msg(node.pos)
            transforms.append(msg)
    broadcaster.sendTransform(transforms)


def publish_marker_array(data):
    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    rospy.sleep(rospy.Duration(0.5))
    ma = MarkerArray()
    for id, node in enumerate(at.PreOrderIter(data)):
        # prepare the communalities
        m = Marker()
        m.header.frame_id = node.name
        m.id = id
        m.pose.orientation.w = 1.0
        m.action = Marker.ADD
        m.type = Marker.MESH_RESOURCE
        m.scale = Vector3(1.0, 1.0, 1.0)
        m.color = ColorRGBA(0.0, 0.0, 0.0, 0.0)
        m.mesh_use_embedded_materials = True
        # take care of the object-specific differences in a nested if-structure
        if node.parent: # ignore the root node
            if node.type is shelf_system_key:
                m.ns = 'shelf_body_ns'
                m.mesh_resource = 'package://refills_first_review/meshes/shelves/DMShelfFrameFrontStore.dae'
                m.pose.position.x = 0.5
                m.pose.position.y = 0.313126
                m.pose.position.z = 0.8705
                m.color = ColorRGBA(0.0, 0.8, 0.0, 1.0)
            if node.type is layer_key:
                if 'layer1' not in node.name: # ignore to bottom shelf layers
                    m.ns = 'shelf_layer_ns'
                    m.mesh_resource = 'package://refills_first_review/meshes/shelves/DMShelfLayer4TilesFront.dae'
                    m.pose.position.x = 0.5
                    m.pose.position.y = 0.27
                    m.pose.position.z = -0.067975
                    m.color = ColorRGBA(0.0, 0.8, 0.0, 1.0)
            if node.type is separator_key:
                m.ns = 'separator_ns'
                m.mesh_resource = 'package://refills_first_review/meshes/shelves/DMShelfSeparator4Tiles.dae'
                m.pose.position.x = 0.0
                m.pose.position.y = 0.217
                m.pose.position.z = 0.015
                m.color = ColorRGBA(0.0, 0.0, 0.8, 1.0)
                # m.color = ColorRGBA(0.8, 0.8, 0.8, 0.5)
            if node.type is barcode_key:
                m.type = Marker.CUBE
                m.ns = 'barcode_ns'
                # TODO(Georg): add offset
                m.scale = Vector3(0.04, 0.001, 0.038)
                m.color = ColorRGBA(0.8, 0.0, 0.0, 1.0)
                # m.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
                m.mesh_use_embedded_materials = False
        ma.markers.append(m)
    marker_pub.publish(ma)


def visualize(data):
    for node in at.PreOrderIter(data):
        print node
    publish_tf(data)
    publish_marker_array(data)


def separator_vec(x):
    return kdl.Vector(x, 0.005, 0.005)


def barcode_vec(x, bottom_layer):
    if bottom_layer:
        return kdl.Vector(x, -0.028, -0.04)
    else:
        return kdl.Vector(x, -0.002, -0.04)


def system_node(name, parent, x, y, z):
    return at.Node(name, parent=parent, pos=kdl.Vector(x, y, z), type=shelf_system_key)


def layer_node(name, parent, x, y, z):
    return at.Node(name, parent=parent, pos=kdl.Vector(x, y, z), type=layer_key)


def separator_node(name, parent, x_offset):
    return at.Node(name, parent=parent, pos=separator_vec(x_offset), type=separator_key)


def barcode_node(name, parent, x_offset):
    if "layer1" in parent.name:
        return at.Node(name, parent=parent, pos=barcode_vec(x_offset, True), type=barcode_key)
    else:
        return at.Node(name, parent=parent, pos=barcode_vec(x_offset, False), type=barcode_key)


def complete_layer(name, parent, x, y, z, separators, barcodes):
    layer = layer_node(name, parent, x, y, z)

    for idx, x_offset in enumerate(separators):
        separator_node("{}_sep{}".format(name, idx), layer, x_offset)

    for name, x_offset in barcodes.items():
        barcode_node(name, layer, x_offset)

    return layer


def populate_ground_truth():
    root = at.Node("map")

    # SHELF1
    shelf1 = system_node("shelf1", root, 0.653156, -0.627501, 0.047)

    s1_layer1 = complete_layer("s1_layer1", shelf1, 0.0, -0.028, 0.132,
                               [0.005, 0.100, 0.240, 0.382, 0.511, 0.640, 0.766, 0.867, 0.98],
                               {"027995": 0.030, "544205": 0.162, "384160": 0.297,
                                "457319": 0.428, "534812": 0.572, "402610": 0.697,
                                "433961": 0.806, "507923": 0.917})

    s1_layer2 = complete_layer("s1_layer2", s1_layer1, 0.0, 0.102, 0.4,
                               [0.005, 0.155, 0.218, 0.280, 0.361, 0.51, 0.609, 0.749, 0.858, 0.980],
                               {"500183": 0.065, "046088": 0.197, "262289": 0.302,
                                "010055": 0.414, "015652": 0.538, "516937": 0.664,
                                "125481": 0.906})

    s1_layer3 = complete_layer("s1_layer3", s1_layer2, 0, 0, 0.335,
                               [0.005, 0.150, 0.364, 0.513, 0.771, 0.857, 0.999],
                               {"004728": 0.056, "196068": 0.240, "332384": 0.426,
                                "523129": 0.584, "424929": 0.772, "235542":0.902})

    s1_layer4 = complete_layer("s1_layer4", s1_layer3, 0 , 0, 0.298,
                               [0.005, 0.181, 0.294, 0.433, 0.625, 0.824, 0.990],
                               {"114035" : 0.068, "176671" : 0.217, "422771" : 0.328,
                                "331372" : 0.498, "377954" : 0.701, "227003" : 0.874})

    s1_layer5 = complete_layer("s1_layer5", s1_layer4, 0 , 0, 0.255,
                                [0.005, 0.109, 0.308, 0.005, 0.571, 0.774, 0.980], 
                                {"200563" : 0.030, "447600" : 0.186, "501155" : 0.412,
                                 "501156" : 0.662, "534910" : 0.861})

    s1_layer6 = complete_layer("s1_layer6", s1_layer5, 0 ,0, 0.338, 
                                [0.005, 0.111, 0.201, 0.282, 0.358, 0.425, 0.513, 0.648, 0.728, 0.814, 0.886, 0.982], 
                                {"305553" : 0.044, "503131" : 0.140, "333192" : 0.219, "333140" : 0.306,
                                 "050081" : 0.371, "454259" : 0.453, "048180" : 0.565, "046445" : 0.670,
                                 "500124" : 0.755, "333154" : 0.837, "023130" : 0.923})

    # SHELF2
    shelf2 = system_node("shelf2", root, 1.6521,   -0.624451, 0.05)

    s2_layer1 = complete_layer("s2_layer1", shelf2 , 0, -0.028, 0.134, 
                                [-0.005, 0.051, 0.086, 0.129, 0.181, 0.389, 0.589, 0.785, 0.991], 
                                {"013462" : 0.030, "458823" : 0.113, "522273" : 0.268,
                                 "102696" : 0.466, "102694" : 0.676, "519151" : 0.848}) 

    s2_layer2 = complete_layer("s2_layer2", s2_layer1 , 0 , 0.102, 0.230, 
                                [0.010, 0.194, 0.381, 0.575, 0.775, 0.975], 
                                {"522990" : 0.106, "522991" : 0.274, "523009" : 0.446, "522985" : 0.653, "523010" : 0.841}) 

    s2_layer3 = complete_layer("s2_layer3", s2_layer2 , 0 , 0, 0.210, 
                                [0.010, 0.172, 0.300, 0.415, 0.463, 0.516, 0.570, 0.630, 0.680, 0.740, 0.804, 0.864, 0.920, 0.976], 
                                {"522989" : 0.075, "522988" : 0.235, "294763" : 0.351, "294760" : 0.425,
                                 "449181" : 0.471, "449182" : 0.530, "507237" : 0.587, "036942" : 0.641, 
                                 "036946" : 0.699, "036945" : 0.763, "036988" : 0.824, "190755" : 0.913}) 

    s2_layer4 = complete_layer("s2_layer4", s2_layer3 , 0 , 0.145, 0.54, 
                                [], 
                                {"463940" : 0.021, "155777" : 0.086, "539355" : 0.149, "356346" : 0.198,
                                 "513205" : 0.255, "544140" : 0.302, "325776" : 0.350, "045907" : 0.393, 
                                 "461819" : 0.445, "264913" : 0.499, "264912" : 0.541, "107184" : 0.601, 
                                 "362767" : 0.680, "477847" : 0.767, "545202" : 0.843, "294993" : 0.924})

    s2_layer5 = complete_layer("s2_layer5", s2_layer4 ,0 , 0, 0.298, 
                                [], 
                                {"121582" : 0.020, "039450" : 0.074, "531041" : 0.128, "531033" : 0.177,
                                 "566325" : 0.230, "096981" : 0.293, "218409" : 0.350, "499551" : 0.418,
                                 "011850" : 0.471, "011853" : 0.525, "318407" : 0.586, "049544" : 0.655, 
                                 "318497" : 0.705, "431394" : 0.755, "208112" : 0.809, "472899" : 0.870, 
                                 "472897" : 0.924})

    s2_layer6 = complete_layer("s2_layer6", s2_layer5, 0 , 0, 0.254, 
                                [], 
                                {"349789" : 0.010, "349799" : 0.090, "349797" : 0.166, "387457" : 0.256, 
                                 "515633" : 0.328, "447775" : 0.404, "535427" : 0.476, "046255" : 0.575, 
                                 "479455" : 0.673, "467637" : 0.755, "317514" : 0.835, "317512" : 0.916})

    # SHELF3
    shelf3 = system_node("shelf3", root, 2.650449,   -0.632107, 0.05)

    s3_layer1 = complete_layer("s3_layer1", shelf3 , 0, -0.028, 0.134, 
                                [0.005, 0.090, 0.19, 0.285, 0.380, 0.470, 0.559, 0.645, 0.735, 0.821, 0.907, 0.995], 
                                {"260725" : 0.075, "260707" : 0.270, "554301" : 0.402, "404491" : 0.502, 
                                "521873" : 0.634, "402589" : 0.745, "260667" : 0.889})

    s3_layer2 = complete_layer("s3_layer2", s3_layer1, 0 , 0.102, 0.316, 
                                [0.005, 0.070, 0.147, 0.200, 0.255, 0.309, 0.360, 0.413, 0.468, 0.520, 0.606, 0.692, 0.800, 0.896, 0.985], 
                                {"520698" : 0.026, "520696" : 0.096, "520689" : 0.188, "520688" : 0.295, 
                                 "423376" : 0.431, "422068" : 0.550, "511860" : 0.625, "260700" : 0.736, 
                                 "260695" : 0.832, "399599" : 0.923})

    s3_layer3 = complete_layer("s3_layer3", s3_layer2, 0 , 0, 0.296, 
                                [0.005, 0.072, 0.140, 0.194, 0.261, 0.386, 0.443, 0.508, 0.596, 0.732, 0.800, 0.868, 0.923, 0.987], 
                                {"519763" : 0.093, "535675" : 0.020, "520209" : 0.151, "520212" : 0.213, 
                                 "523828" : 0.399, "523829" : 0.467, "523823" : 0.542, "523830" : 0.644, 
                                 "472787" : 0.750, "293857" : 0.820, "536979" : 0.886, "411522" : 0.951})

    s3_layer4 = complete_layer("s3_layer4", s3_layer3, 0 , 0, 0.295, 
                                [0.005, 0.060, 0.125, 0.180, 0.238, 0.325, 0.412, 0.483, 0.547, 0.613, 0.691, 0.767, 0.846, 0.915, 0.980], 
                                {"460470" : 0.016, "460201" : 0.080, "536952" : 0.132, "536943" : 0.192,
                                 "536970" : 0.265, "565329" : 0.351, "565335" : 0.435, "518875" : 0.503,
                                 "536760" : 0.560, "536803" : 0.645, "518876" : 0.725, "544375" : 0.794, 
                                 "544382" : 0.860, "553733" : 0.940})

    s3_layer5 = complete_layer("s3_layer5", s3_layer4, 0 , 0, 0.335, 
                                [0.010, 0.098, 0.161, 0.261, 0.332, 0.426, 0.485, 0.597, 0.672, 0.750, 0.83, 0.905, 0.983], 
                                {"544799" : 0.030, "544760" : 0.112, "544788" : 0.190, "566217" : 0.280,
                                 "544782" : 0.362, "512018" : 0.445, "512036" : 0.522, "553735" : 0.652,
                                 "553736" : 0.770, "553726" : 0.847, "553715" : 0.929})


    # SHELF4
    shelf4 = system_node("shelf4", root, 3.64369,  -0.630106, 0.05)

    s4_layer1 = complete_layer("s4_layer1", shelf4, 0, -0.028, 0.134, 
                                [0.005, 0.065, 0.135, 0.188, 0.250, 0.314, 0.381, 0.445, 0.523, 0.601, 0.759, 0.908, 0.980], 
                                {"250917" : 0.051, "251005" : 0.149, "253896" : 0.232, "252017" : 0.367, 
                                 "378981" : 0.468, "378940" : 0.545, "347469" : 0.664, "279513" : 0.815, 
                                 "404079" : 0.934})

    s4_layer2 = complete_layer("s4_layer2", s4_layer1, 0 , 0.102, 0.273, 
                                [0.005, 0.165, 0.324, 0.396, 0.480, 0.546, 0.618, 0.690, 0.765, 0.838, 0.904, 0.985], 
                                {"470266" : 0.145, "274302" : 0.350, "278987" : 0.430, "519823" : 0.506,
                                 "278866" : 0.570, "278867" : 0.640, "497298" : 0.706, "250899" : 0.791, 
                                 "522986" : 0.859, "279042" : 0.935})

    s4_layer3 = complete_layer("s4_layer3", s4_layer2, 0 , 0, 0.254, 
                                [0.005, 0.081, 0.123, 0.196, 0.254, 0.354, 0.450, 0.560, 0.611, 0.670, 0.732, 0.782, 0.838, 0.896, 0.948, 0.988], 
                                {"469679" : 0.015, "252312" : 0.088, "251219" : 0.147, "251252" : 0.211,
                                 "309649" : 0.297, "518822" : 0.391, "256501" : 0.494, "346864" : 0.567, 
                                 "251211" : 0.625, "453136" : 0.683, "476951" : 0.740, "476953" : 0.789, 
                                 "250938" : 0.851, "522630" : 0.905, "522603" : 0.950})

    s4_layer4 = complete_layer("s4_layer4", s4_layer3, 0 , 0, 0.253, 
                                [0.005, 0.041, 0.112, 0.185, 0.262, 0.330, 0.385, 0.488, 0.531, 0.579, 0.620, 0.671, 0.747, 0.826, 0.88, 0.93, 0.99], 
                                {"251188" : 0.005, "471975" : 0.069, "250996" : 0.137, "251010" : 0.205, 
                                 "453118" : 0.272, "251187" : 0.341, "253217" : 0.419, "446409" : 0.489, 
                                 "457425" : 0.541, "418988" : 0.586, "535680" : 0.632, "432262" : 0.700, 
                                 "515141" : 0.769, "250919" : 0.835, "250949" : 0.895, "254254" : 0.945})

    s4_layer5 = complete_layer("s4_layer5", s4_layer4, 0 , 0, 0.253, 
                                [0.005, 0.065, 0.117, 0.170, 0.253, 0.330, 0.407, 0.509, 0.589, 0.639, 0.683, 0.724, 0.775, 0.850, 0.920, 0.985], 
                                {"253179" : 0.010, "250898" : 0.097, "534357" : 0.203, "415950" : 0.277, 
                                 "415040" : 0.349, "518451" : 0.439, "366555" : 0.534, "515136" : 0.604, 
                                 "256799" : 0.651, "295813" : 0.693, "415961" : 0.742, "308627" : 0.795, 
                                 "256519" : 0.855, "403170" : 0.942})

    s4_layer6 = complete_layer("s4_layer6", s4_layer5, 0 , 0, 0.253, 
                                [0.005, 0.047, 0.125, 0.193, 0.273, 0.351, 0.428, 0.476, 0.549, 0.620, 0.676, 0.721, 0.774, 0.829, 0.884, 0.988], 
                                {"256479" : 0.005, "256500" : 0.060, "256511" : 0.138, "447612" : 0.215, 
                                 "295814" : 0.296, "256505" : 0.370, "266328" : 0.433, "256506" : 0.492, 
                                 "256502" : 0.563, "402186" : 0.633, "256477" : 0.683, "390842" : 0.735, 
                                 "390740" : 0.804, "256476" : 0.850, "518581" : 0.929})

    return root


if __name__ == '__main__':
    try:
        rospy.init_node('ground_truth_broadcaster')
        visualize(populate_ground_truth())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
