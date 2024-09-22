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

from catkin_pkg.package_templates import create_package_files, PackageTemplate
import rospkg


#Python
import numpy as np
import os
import xml.etree.ElementTree as ET
from shutil import rmtree

import sys

#Custom
from import_asembly.Asembly_import import read_step_file_asembly



def createXacroPropertyWithParameterizedNamespace(property_name, value, prefix_param_name):
    xacro_string = "<xacro:property name=\"" + property_name + "\" value=\"${"+ prefix_param_name +"}" + value  + "\" />"
    #<xacro:property name="joint_name_1" value="${robot_prefix}joint1" />
    return xacro_string



def createXacroRunMacro():


    xacro_string = """
    <xacro:arg name="robot_namespace"  default=""/>
    <xacro:arg name="standalone"  default="true"/>
    <xacro:property name="test_value" value="$(arg standalone)" />

    <xacro:if value="${test_value}">
    
        <xacro:property name="this_robot_name" value="" />

    </xacro:if>

    <xacro:my_robot robot_name="${this_robot_name}" />
    """ 

    return xacro_string

def changeXacroJointNames(xacro_string, joint_names , property_line = ""):

    link_i = 0
    base_link_string = "<joint name=\""

    for link_name in joint_names:

        new_link_name = "joint_" + str(link_i) + "_name"
        new_property = "<xacro:property name=\""+ new_link_name +"\" value=\"${robot_prefix}" + link_name + "\" />"
        xacro_string = insertToXacroLine(xacro_string, property_line, new_property)
        current_link_string = base_link_string + link_name + "\""
        new_link_string = base_link_string + "${" + new_link_name + "}\""
        xacro_string = xacro_string.replace(current_link_string , new_link_string)
        link_i = link_i + 1 


    return xacro_string


def changeXacroLinkNames(xacro_string, link_names , property_line = ""):

    #<link name="pillar">

    link_i = 0
    base_link_string = "<link name=\""
    link_string_as_parent = "<parent link=\""
    link_string_as_child = "<child link=\""

        

    for link_name in link_names:

        new_link_name = "link_" + str(link_i) + "_name"
        new_property = "<xacro:property name=\""+ new_link_name +"\" value=\"${robot_prefix}" + link_name + "\" />"
        xacro_string = insertToXacroLine(xacro_string, property_line, new_property)


        current_link_string = base_link_string + link_name + "\""
        new_link_string = base_link_string + "${" + new_link_name + "}\""
        xacro_string = xacro_string.replace(current_link_string , new_link_string)

        #replace in joints

        current_link_string = link_string_as_parent + link_name + "\""
        new_link_string = link_string_as_parent + "${" + new_link_name + "}\""
        xacro_string = xacro_string.replace(current_link_string , new_link_string)
    
        current_link_string = link_string_as_child + link_name + "\""
        new_link_string = link_string_as_child + "${" + new_link_name + "}\""
        xacro_string = xacro_string.replace(current_link_string , new_link_string)
    
        link_i = link_i + 1 


    return xacro_string

def substituteInXacro(xacro_string, robot_joints,robot_links_names):

    xacro_head = "<robot xmlns:xacro=\"http://wiki.ros.org/xacro\" name = \"robot\" >"
    xacro_macro_head = "<xacro:macro name=\"my_robot\" params=\"robot_name\">"

    xacro_string = xacro_string.replace("<robot name=\"fifi\" version=\"1.0\">", xacro_head )

    xacro_string = insertToXacroLine(xacro_string, xacro_head,xacro_macro_head)

    xacro_string = insertToXacroLine(xacro_string,"</robot>","</xacro:macro>", before = True)

    prefix_property = "<xacro:property name=\"robot_prefix\" value=\"${robot_name}\" />"
    xacro_string = insertToXacroLine(xacro_string, xacro_macro_head,prefix_property )

    run_string = createXacroRunMacro()

    run_reference = "</xacro:macro>"

    xacro_string = insertToXacroLine(xacro_string,run_reference,run_string)

    xacro_string = changeXacroLinkNames(xacro_string, robot_links_names, prefix_property )
    robot_joints_names = []
    for joint in robot_joints:
        if joint["parent"]!="":
            robot_joints_names.append(joint["name"])

    xacro_string = changeXacroJointNames(xacro_string, robot_joints_names, prefix_property )


    return xacro_string

def insertToXacroLine(xacro_string, reference_text, insert_text , before = False):

    base_lines = xacro_string.splitlines()
    insert_lines = insert_text.splitlines()

    insert_i = 0
    for i, line in enumerate(base_lines):
      
        if reference_text in line:
            insert_i = i
            break
   
    if before == False:
        insert_i = insert_i + 1
       

    new_list = base_lines[:insert_i] + insert_lines + base_lines[insert_i:]

    xacro_string = "\n".join(new_list)

    return xacro_string



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

def findOneVersionOfString(string_word, versions):
    anser = -1

    for version in versions:
        anser = string_word.find(version)

        if anser != -1:
            return anser

    return anser


def separateRobotPartsFromStep(parts_data):
    print("Searching trough step...")
    print("Prepared " +  str(len(parts_data)) + " parts data")

    joints_id_names = ["joint_","JOINT_","Joint_"]
    connection_word_id_names = ["_to_","_TO_" ,"_To_"]
    urdf_id_names = ["urdf","URDF","Urdf"]

    avalibel_joint_types = ["fixed","revolute", "prismatic"]
    joint_types_id_names = {}
    joint_types_id_names["fixed"] =  ["fixed","FIXED","Fixed"]
    joint_types_id_names["revolute"] =  ["revolute","REVOLUTE","Revolute"]
    joint_types_id_names["prismatic"] =  ["prismatic","PRISMATIC","Prismatic"]


    robot_joints =[]
    robot_parts = []
    robot_links = []




    root_link_name = None

    for part in parts_data:
        
        part_data = parts_data[part]
        #print(part_data)

        if len(part_data)==4: #type(part) == TopoDS_Solid or type(part) == TopoDS_Compound or type(part) == TopoDS_Shell: #check id solid or compound
            segment_name, segment_color, segment_hierarchy, segment_trans = part_data
            
            segment_name = segment_name.replace(" ", "-")
            # print("hierarchy:")
            # print(segment_hierarchy)
            # print(segment_name)

            segment_location = part.Location().Transformation()


            segment_position = changePos2M(segment_location)
            segment_q_orientation = [segment_location.GetRotation().X(), segment_location.GetRotation().Y(), segment_location.GetRotation().Z(), segment_location.GetRotation().W()]
            #print(segment_location)
            #Parse joints data
            if len(segment_hierarchy)>-1: #filter out joints, (unlocked hiarchi ? 1 ->  -1
                urdf_detected = False
                h_i = 0
                for hiarchie_names in segment_hierarchy:
                    if findOneVersionOfString(hiarchie_names,urdf_id_names) == 0:
                        print("got to urdf" + str(h_i))
                        urdf_detected = True
                        break
                    
                    h_i = h_i + 1

                if urdf_detected: # findOneVersionOfString(segment_hierarchy[1],urdf_id_names) == 0:  #check for urdf
                   
                    joint_name = segment_hierarchy[h_i+1] 

                    if findOneVersionOfString(joint_name,joints_id_names) ==0:

                        connection_name = joint_name[6:]
                        connection_id_string = "_to_"

                        ind = findOneVersionOfString(connection_name, connection_word_id_names) 

                        if ind == -1: #this is for base joint
                            ind = connection_name.find("_")
                            parent_name = connection_name[0:ind]
                            root_link_name = parent_name
                            child_name = parent_name #to enable moving everithing to this frame
                            parent_name = "" #"root joint doesnt have parent"
                        else:
                            parent_name = connection_name[0:ind]
                            connection_name = connection_name[len(parent_name)+len(connection_id_string):]  #TODO this presumes all len(connection_id_string) is same
                            ind = connection_name.find("_")
                            child_name = connection_name[0:ind]


        
                        joint_data = {}
                        joint_data["name"] = parent_name + "_" + child_name
                        
                        for test_type in avalibel_joint_types: #iterate trough avalibel type and set correct one
                            

                            if findOneVersionOfString(segment_name,joint_types_id_names[test_type])==0:
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



                        continue
                    else:

                        print("PROBLEM: Not correct naming of joints in URDF asembly")
                        print(joint_name)
                        continue

            part_for_saving = False

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

                part_for_saving = True

            if type(part) == TopoDS_Shell:

                part_for_saving = True

            if part_for_saving:
                
                robot_part = {}

                segment_name = segment_name.replace("/", "_") #safty for names includinh / as path

                robot_part["name"] = segment_name
                robot_part["location"] = segment_location
                robot_part["hierarchy"] = segment_hierarchy
                robot_part["part"] = part
                robot_part["color"] = [segment_color.Red(),segment_color.Green(),segment_color.Blue()]
                robot_parts.append(robot_part)




        else:
            segment_name, segment_color = parts_data[part]



    return  robot_parts, robot_joints, robot_links , root_link_name



def createSTLs(robot_parts,meshes_path, mode="binary"):
    print("Preparing meshes...")


    #CREATE STLs

    #convert to stls

    stl_output_dir = meshes_path #package_path + "/meshes" 
    output_files = []
    file_names = []

    test_count = 0
    max_parts = 2000
    #root_link_name ="Base"
    #print("len:"+str(len(robot_parts)))
    #print("robotParts:")

    for part in robot_parts:


        #print(part)
        #file_name = part['hierarchy'][-1]+"-"+part["name"]+".stl"
        if test_count>max_parts:
            print("to many parts, set higher limit!")
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
        
        print("output file: " + output_file)
        write_stl_file(scaled_part, output_file, mode=mode)
        output_files.append(file_name)

    return output_files

def createMaterialsAndColors(robot_parts, robot_links, meshes_paths, root_link_name):
    print("Creating materials...")
    colors_values = []
    colors_names = []
    color_counter = 0
    materials = []

    link_meshes = {}

    for name in robot_links: #preapre empty matrixes for link meshes

        link_meshes[name] = []

    mesh_i = 0

    for part in robot_parts:
        #print(mesh_i)

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
        print("link name: " + current_name)

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
        #print(current_name)
        if current_name in robot_links:
            file_name = meshes_paths[mesh_i]
            link_meshes[current_name].append({"mesh_name":file_name,"mesh_material":color_name,"color_value":part["color"]})
            
        else:
            print("error: no link name: " + current_name)

        mesh_i = mesh_i + 1

    return robot_parts, link_meshes



def generateURDF(robot_joints,robot_links, link_meshes, root_link_name, package_name):

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
                                axis=[1, 0, 0], origin=Pos1,
                                limit=joint_limit, dynamics=None, safety_controller=None,
                                calibration=None, mimic=None)



            robot.add_joint(new_joint)
        else:
            root_location_in_step = joint["location"]



    #robot.add_material(Mat1)

    #CREATE ROBOT LINKS
    relative_mesh_path = "meshes/"
    stl_urdf_root ="package://"+package_name+"/"


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


def createPackageROS(package_name,output_folder_path):

    package_description = "This is automaticly created ROS package with URDF"

    package_template = PackageTemplate._create_package_template(
    package_name=package_name,
    description=package_description,
    licenses=["BSD"] or [],
    maintainer_names=["urfd_from_step_package"],
    author_names=["urfd_from_step_package"],
    version="1.0.0",
    catkin_deps=["rviz"])

    package_path = output_folder_path + "/"+ package_name

    create_package_files(target_path=package_path,
                        package_template=package_template,
                        rosdistro="noetic",
                        #newfiles={"launch/start.launch":"test"}
                        )#,"urdf/test.urdf":"test2"


def setupLaunchXml(new_package_name):



    # importing element tree
    # under the alias of ET

    rospack = rospkg.RosPack()
    template_path = rospack.get_path('urdf_from_step') + "/launch/template.launch"

    print(template_path)

    # Passing the path of the
    # xml document to enable the
    # parsing process
    tree = ET.parse(template_path)
    
    # getting the parent tag of
    # the xml document
    root = tree.getroot()
 
    # go to atributes
    
   
    for child in root:

        try:
            if child.attrib['name'] == "tf_prefix":
                child.attrib['default'] = new_package_name
            if child.attrib['name'] == "urdf_name":
                child.attrib['default'] = new_package_name   
            if  child.attrib['name'] == "urdf_path":
                child.attrib['default'] = "$(find "+ new_package_name +")/urdf/$(arg urdf_name).urdf"
        except:
            pass

    for child in root:

        print(child.attrib)

    xml_string = ET.tostring(root)

    return xml_string

def changeCmake(input_string):


    old_text = "# install(FILES\n#   # myfile1\n#   # myfile2\n#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}\n# )"
    new_text = "install(DIRECTORY\n   launch/\n    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}\n  FILES_MATCHING PATTERN \"*.launch\" \n )"




    output_string = input_string.replace(old_text, new_text)

    return output_string



if __name__ == '__main__':



    rospy.init_node('URDF_creator')
    step_file_path = rospy.get_param('~step_file_path', '/input_step_files/robot_arm.step')
    output_folder_path = rospy.get_param('~output_folder_path', '/output_ros_urdf_packages')
    package_name = rospy.get_param('~urdf_package_name', 'test_package')
    package_path = "/output_ros_urdf_packages/" + package_name

    rospy.loginfo("Creating ROS package:")
    rospy.loginfo(package_name)

    #delete existing package
    if os.path.exists(package_path) == True:
        rospy.loginfo("Package alread exist! Deliting:" + package_name)
        rmtree(package_path)


    createPackageROS(package_name,output_folder_path)


    rospy.loginfo("Creating URDF from STEP file:")
    rospy.loginfo(step_file_path)



    #CHANGE CMAKE

    cmake_path = package_path +"/CMakeLists.txt"

    cmake_file_handle = open(cmake_path,"r+")
    
    text = cmake_file_handle.read()

    changed_text = changeCmake(text) 

    cmake_file_handle.write(changed_text)
    cmake_file_handle.close()
    rospy.loginfo("READ STEP FILE ASEMBLY...")
    parts_data = read_step_file_asembly(step_file_path)

    rospy.loginfo("SEPARATE ROBOT PARTS...")
    robot_parts, robot_joints, robot_links, root_link_name  = separateRobotPartsFromStep(parts_data)
    print("JOINTS:")
    for joint in robot_joints:
        print(joint)

    #sys.exit()

    meshes_path = package_path +"/meshes"
    if os.path.exists(meshes_path) == False:
        os.mkdir(meshes_path)


    mesh_paths = createSTLs(robot_parts, meshes_path)


    robot_parts, link_meshes = createMaterialsAndColors(robot_parts, robot_links, mesh_paths, root_link_name)


    robot = generateURDF(robot_joints,robot_links, link_meshes, root_link_name, package_name)
    #print(robot)

    if os.path.exists(package_path +"/urdf") == False:
        os.mkdir(package_path +"/urdf")

    urdf_path = package_path +"/urdf/"+package_name+".urdf"

    file_handle = open(urdf_path,"w")

    file_handle.write(robot.to_xml_string())
    file_handle.close()

    export_xacro = True
    if export_xacro:
        xacro_string = robot.to_xml_string()

        xacro_string  = substituteInXacro(xacro_string, robot_joints,robot_links)
        #print(xacro_string)

        xacro_path = package_path +"/urdf/"+package_name+".urdf.xacro"
        file_handle = open(xacro_path,"w")
        file_handle.write(xacro_string)
        file_handle.close()

    #COPY LAUNCH TEMPLATE
    print("Prepare launch file..")
    new_package_name = package_name
    xml_string = setupLaunchXml(new_package_name)

    
    if os.path.exists(package_path +"/launch") == False:
        os.mkdir(package_path +"/launch")

    launch_path = package_path +"/launch/load_urdf.launch"

    launch_file_handle = open(launch_path,"wb")
    launch_file_handle.write(xml_string)
    launch_file_handle.close()



    
    print("Conversion finished.")

    rospy.signal_shutdown("Finish")
    
    
