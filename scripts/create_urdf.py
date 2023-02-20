#!/usr/bin/env python3


#OpenCascade
from OCC.Core.gp import  gp_Vec, gp_Quaternion, gp_Pnt , gp_Trsf, gp_Ax1
from OCC.Core.TopoDS import TopoDS_Compound, TopoDS_Solid, TopoDS_Shell

from OCC.Extend.DataExchange import write_stl_file
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform

#ROS
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from urdf_parser_py import urdf

#Python
import numpy as np
import os


#Custom
from import_asembly.Asembly_import import read_step_file_asembly



def changePos2M(segment_location):

    return [segment_location.TranslationPart().X() / 1000, segment_location.TranslationPart().Y() / 1000, segment_location.TranslationPart().Z() / 1000]

def toEuler(segment_location):


    q = np.array([segment_location.GetRotation().X(), segment_location.GetRotation().Y(), segment_location.GetRotation().Z(), segment_location.GetRotation().W()])
    return list(euler_from_quaternion(q))

    
def calculateTfToRoot(joint, joint_list):

    parent_name = joint['parent']

    global_this_tf = joint['location']
    local_tf = gp_Trsf()#copy.deepcopy(global_this_tf)
    found = False
    for other_joint in joint_list:

        if other_joint['child'] == parent_name:
            global_other_tf = other_joint['location']
            local_tf = global_other_tf.Inverted().Multiplied(global_this_tf) 
            found = True
            break

    if found == False: #is to root joint  
            
        local_tf.SetTranslation(gp_Vec(0,0,0)) 
        local_tf.SetRotation(gp_Quaternion(0,0,0,1))

        #local_tf = global_this_tf
    
    return local_tf


def separateRobotPartsFromStep(parts_data):

    robot_joints =[]
    robot_parts = []
    robot_links = []




    avalibel_joint_types = ["fixed","revolute", "prismatic"]
    root_link_name = None

    for part in parts_data:
        part_data = parts_data[part]

        if len(part_data)==4: #type(part) == TopoDS_Solid or type(part) == TopoDS_Compound or type(part) == TopoDS_Shell: #check id solid or compound
            segment_name, segment_color, segment_hierarchy, segment_trans = part_data

            print(segment_hierarchy)
            print(segment_name)

            segment_location = part.Location().Transformation()


            segment_position = changePos2M(segment_location)
            segment_q_orientation = [segment_location.GetRotation().X(), segment_location.GetRotation().Y(), segment_location.GetRotation().Z(), segment_location.GetRotation().W()]
            print(segment_location)
            #Parse joints data
            if segment_hierarchy[1] == "URDF" or segment_hierarchy[1] == "urdf": #check for urdf
                joint_name = segment_hierarchy[2] 
                if joint_name.find("joint_")==0:

                    connection_name = joint_name[6:]
                    connection_id_string = "_to_"
                    ind = connection_name.find(connection_id_string)
                    parent_name = connection_name[0:ind]
                    connection_name = connection_name[len(parent_name)+len(connection_id_string):]
                    ind = connection_name.find("_")
                    child_name = connection_name[0:ind]

                    if child_name == "": #chec if this is root link
                        root_link_name = parent_name
                        child_name = parent_name #to enable moving everithing to this frame
                        parent_name = "" #"root joint doesnt have parent"

    
                    joint_data = {}
                    joint_data["name"] = parent_name + "_" + child_name
                    
                    for test_type in avalibel_joint_types: #iterate trough avalibel type and set correct one
                        if segment_name.find(test_type)==0:
                            joint_data["type"] = test_type
                            break

                    joint_data["parent"] = parent_name
                    joint_data["child"] = child_name
                    joint_data["position"] = segment_position
                    joint_data["rotation"] = segment_q_orientation
                    joint_data["location"] = segment_location
                    robot_joints.append(joint_data)

                    #DEFINE LINKS
                    #Test child links and add
                    if not child_name in robot_links:
                        robot_links.append(child_name)

                    #Create links
                    if parent_name !="": #not epty root mark
                        if not parent_name in robot_links:
                            robot_links.append(parent_name)




                else:

                    print("PROBLEM: Not correct naming of joints in URDF asembly")
                    print(joint_name)


            else:

                if type(part) == TopoDS_Solid:#== TopoDS_Compound : #

                    if False:            
                        if not(segment_hierarchy[1] in robot_links) and not(segment_hierarchy[1] in robot_links_vis_data): #make list of links at top hiarchie
                        #segments_data={segment_hierarchy[-1]:{segment_name:segment_location}}        
                            robot_links_vis_data.append(segment_hierarchy[1])

                            #zloži elemente v dictionary po hiarhiji
                            if segment_hierarchy[-1] in segments_data:
                                segments_data[segment_hierarchy[-1]].update({segment_name: part})
                            else:
                                segments_data.update({segment_hierarchy[-1]:{segment_name:part}})
                    robot_part = {}
                    robot_part["name"] = segment_name
                    robot_part["location"] = segment_location
                    robot_part["hierarchy"] = segment_hierarchy
                    robot_part["part"] = part
                    robot_part["color"] = [segment_color.Red(),segment_color.Green(),segment_color.Blue()]
                    robot_parts.append(robot_part)

                if type(part) == TopoDS_Shell:
                    robot_part = {}
                    robot_part["name"] = segment_name
                    robot_part["location"] = segment_location
                    robot_part["hierarchy"] = segment_hierarchy
                    robot_part["part"] = part
                    robot_part["color"] = [segment_color.Red(),segment_color.Green(),segment_color.Blue()]
                    robot_parts.append(robot_part)


        else:
            segment_name, segment_color = parts_data[part]



    return  robot_parts, robot_joints, robot_links , root_link_name



def createSTLs(robot_parts,package_path):


    #CREATE STLS



    #convert to stls

    stl_output_dir = package_path + "/meshes" 
    output_files = []
    file_names = []

    test_count = 0
    max_parts = 2000
    #root_link_name ="Base"


    for part in robot_parts:

        print(part)
        #file_name = part['hierarchy'][-1]+"-"+part["name"]+".stl"
        if test_count>max_parts:
            break
        test_count = test_count + 1


        made_name = part["name"]
        #for h_name in part['hierarchy']:
            #made_name = h_name + made_name
        name_counter = 0
        file_name = made_name + str(name_counter)
    
        while file_name in file_names:

            name_counter = name_counter + 1
            file_name = made_name + str(name_counter)
            

        file_names.append(file_name)    

        file_name = file_name + ".stl"

        output_file = os.path.join(stl_output_dir,file_name)
        #stl_writer = StlAPI_Writer()
        #stl_writer.SetASCIIMode(True)
        #mesh = BRepMesh_IncrementalMesh( part["part"], 0.1, False, 0.1, True
        #)       


        #mesh = BRepMesh_IncrementalMesh(my_box, 0.1)
        #mesh.Perform()
        trfs = gp_Trsf()
        trfs.SetScale(gp_Pnt(),0.001)
        scaled_part = BRepBuilderAPI_Transform(part['part'], trfs).Shape()
        
        print(output_file)
        write_stl_file(scaled_part, output_file )
        output_files.append(file_name)

        return output_files

def createMaterialsAndColors(robot_parts, robot_links, meshes_paths, root_link_name):

    colors_values = []
    colors_names = []
    color_counter = 0
    materials = []

    link_meshes = {}

    for name in robot_links: #preapre empty matrixes for link meshes

        link_meshes[name] = []

    mesh_i = 0

    for part in robot_parts:

        #PREPARE new color

        if part["color"] in colors_values:
            color_name = colors_names[colors_values.index(part["color"])]

            
        else:
            colors_values.append(part["color"])
            
            color_name = "color"+str(color_counter)
            colors_names.append(color_name)
            materials.append(urdf.Material(name = color_name, color = urdf.Color(part["color"]+[1]) ))
            color_counter = color_counter + 1

        part["material_name"] = color_name

        #FIND_LINK_name:
        current_name = part["name"] 

        if "link_" in current_name: #search for link definition in part name
            current_name = current_name[5:]
            current_name = current_name[0:current_name.find("_")]
        else: #search for link defition in hiearchy
            for parent_name in part["hierarchy"]:
                if "link_" in parent_name:
                    current_name = parent_name[5:]
                    current_name = current_name[0:current_name.find("_")]
                    break
                else:
                    current_name = root_link_name
        print(current_name)
        if current_name in robot_links:
            file_name = meshes_paths[mesh_i]
            link_meshes[current_name].append({"mesh_name":file_name,"mesh_material":color_name,"color_value":part["color"]})
            
        else:
            print("error: no link name")

        mesh_i = mesh_i + 1

    return robot_parts, link_meshes



def generateURDF(robot_joints,robot_links):

    #PREPARE TRANSFORM MATRIX

    for joint in robot_joints:

        tf = calculateTfToRoot(joint, robot_joints)

        joint["local_tf"] = tf

        pass



    # ROBOT META DATA

    robot=urdf.URDF()
    #robot.materials = materials

    robot.name='fifi'

    robot.version='1.0'
    
    robot.gazebos = ['control']

    #CREATE ROBOT JOINTS

    for joint in robot_joints:

        if joint["parent"]!="":#if root link joint dont add it to the joints
            joint_limit=urdf.JointLimit(effort=1000, lower=-1.548, upper=1.548, velocity=0.5)
            or_j = toEuler(joint["local_tf"])
            pos_j = changePos2M(joint["local_tf"])
            
            #pos_j = joint["position"]
            Pos1 = urdf.Pose(xyz=pos_j, rpy=or_j)#'zyx'
            #Pos1 = Pose(xyz=[0,0,0], rpy=[0,0,0,])#'zyx'
            new_joint = urdf.Joint( name= joint["name"], parent=joint["parent"], child=joint["child"], joint_type=joint["type"],
                                axis=[0, 0, 1], origin=Pos1,
                                limit=joint_limit, dynamics=None, safety_controller=None,
                                calibration=None, mimic=None)



            robot.add_joint(new_joint)
        else:
            root_location_in_step = joint["location"]



    #robot.add_material(Mat1)

    #CREATE ROBOT LINKS
    relative_mesh_path = "meshes/"
    stl_urdf_root ="package://test_rospkg/"


    for link_name in robot_links:

        #if link_name != root_link_name: #this is overjuped because we add it leater
            #search for coresponding joint
        urdf_link = urdf.Link(name = link_name, visual = None, inertial = None, collision = None)#, origin = link_pose

        #add visuals

        for mesh in link_meshes[link_name]:
            meshpath = stl_urdf_root + relative_mesh_path + mesh["mesh_name"]

            #ADD MATERIALS
            Mat1 = urdf.Material(name = mesh["mesh_material"], color = urdf.Color(mesh["color_value"]+[1]))
            #if mesh["mesh_material"] in already_defined_colors: #check if you have already somwhere defined this color if not define it
                #Mat1 = Material(name = mesh["mesh_material"] )
            #else:
                #already_defined_colors.append(mesh["mesh_material"])
                #Mat1 = Material(name = mesh["mesh_material"], color = urdf.Color(mesh["color_value"]+[1]))

            #translation = changePos2M(tf)
            if link_name == root_link_name:
                mesh_location = root_location_in_step
            else:
                for joint in robot_joints:
                    if link_name == joint["child"]:
                        mesh_location = joint["location"]
                        break

            translation  = changePos2M(mesh_location.Inverted())# [0,0,0]
            #or_part = toEuler(tf)
            or_part = toEuler(mesh_location.Inverted())#0,0,0]

            Mesh_to_joint_pose = urdf.Pose(xyz=translation,rpy=or_part)#'zyx'


            Vis1 = urdf.Visual(geometry=urdf.Mesh(filename= meshpath), material=Mat1, origin=Mesh_to_joint_pose, name=None)

            #Vis1 = Visual(geometry=Cylinder(radius=0.005, length=0.5), material=None, origin=Pos1, name=None)
            urdf_link.add_aggregate('visual', Vis1)


        robot.add_link(urdf_link)


    return robot



if __name__ == '__main__':

    rospy.init_node('URDF_creator')
    step_file_path = rospy.get_param('~step_file_path')
    rospy.loginfo("Creating URDF from STEP file:")
    rospy.loginfo(step_file_path)

    parts_data = read_step_file_asembly(step_file_path)


    robot_parts, robot_joints, robot_links, root_link_name  = separateRobotPartsFromStep(parts_data)
    print("A")
    for r in  robot_links:
        print(r)
    print("B")

    package_path = "/output_ros_urdf_packages"

    mesh_paths = createSTLs(robot_parts, package_path)

    robot_parts, link_meshes = createMaterialsAndColors(robot_parts, robot_links, mesh_paths, root_link_name)

    step_file_path = rospy.get_param('~step_file_path')

    robot = generateURDF(robot_joints,robot_links)

    if os.path.exists(package_path +"/urdf") == False:
        os.mkdir(package_path +"/urdf")
    urdf_path = package_path +"/urdf/part1.urdf"

    file_handle = open(urdf_path,"w")
    #print(robot)

    file_handle.write(robot.to_xml_string())
    file_handle.close()

    rospy.signal_shutdown("Finish")
    
    
