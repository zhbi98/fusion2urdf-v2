# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:46:26 2019

@author: syuntoku
"""

import adsk, os
from xml.etree.ElementTree import Element, SubElement
from . import Link, Joint
from ..utils import utils

def write_link_urdf(joints_dict, repo, links_xyz_dict, file_name, inertial_dict):
    """
    Write links information into urdf "repo/file_name"
    
    
    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: vacant dict
        xyz information of the each link
    file_name: str
        urdf full path
    inertial_dict:
        information of the each inertial
    
    Note
    ----------
    In this function, links_xyz_dict is set for write_joint_tran_urdf.
    The origin of the coordinate of center_of_mass is the coordinate of the link
    """
    with open(file_name, mode='a') as f:
        # for base_link
        center_of_mass = inertial_dict['base_link']['center_of_mass']
        link = Link.Link(name='base_link', xyz=[0,0,0], 
            center_of_mass=center_of_mass, repo=repo,
            mass=inertial_dict['base_link']['mass'],
            inertia_tensor=inertial_dict['base_link']['inertia'])
        links_xyz_dict[link.name] = link.xyz
        link.make_link_xml()
        f.write(link.link_xml)
        f.write('\n')

        # others
        for joint in joints_dict:
            name = joints_dict[joint]['child']
            center_of_mass = \
                [ i-j for i, j in zip(inertial_dict[name]['center_of_mass'], joints_dict[joint]['xyz'])]
            link = Link.Link(name=name, xyz=joints_dict[joint]['xyz'],\
                center_of_mass=center_of_mass,\
                repo=repo, mass=inertial_dict[name]['mass'],\
                inertia_tensor=inertial_dict[name]['inertia'])
            links_xyz_dict[link.name] = link.xyz            
            link.make_link_xml()
            f.write(link.link_xml)
            f.write('\n')


def write_joint_urdf(joints_dict, repo, links_xyz_dict, file_name):
    """
    Write joints and transmission information into urdf "repo/file_name"
    
    
    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: dict
        xyz information of the each link
    file_name: str
        urdf full path
    """
    
    with open(file_name, mode='a') as f:
        for j in joints_dict:
            parent = joints_dict[j]['parent']
            child = joints_dict[j]['child']
            joint_type = joints_dict[j]['type']
            upper_limit = joints_dict[j]['upper_limit']
            lower_limit = joints_dict[j]['lower_limit']
            try:
                xyz = [round(p-c, 6) for p, c in \
                    zip(links_xyz_dict[parent], links_xyz_dict[child])]  # xyz = parent - child
            except KeyError as ke:
                app = adsk.core.Application.get()
                ui = app.userInterface
                ui.messageBox("There seems to be an error with the connection between\n\n%s\nand\n%s\n\nCheck \
whether the connections\nparent=component2=%s\nchild=component1=%s\nare correct or if you need \
to swap component1<=>component2"
                % (parent, child, parent, child), "Error!")
                quit()
                
            joint = Joint.Joint(name=j, joint_type = joint_type, xyz=xyz, \
            axis=joints_dict[j]['axis'], parent=parent, child=child, \
            upper_limit=upper_limit, lower_limit=lower_limit)
            joint.make_joint_xml()
            joint.make_transmission_xml()
            f.write(joint.joint_xml)
            f.write('\n')

def write_gazebo_endtag(file_name):
    """
    Write about gazebo_plugin and the </robot> tag at the end of the urdf
    
    
    Parameters
    ----------
    file_name: str
        urdf full path
    """
    with open(file_name, mode='a') as f:
        f.write('</robot>\n')
        

def write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    try: os.mkdir(save_dir + '/urdf')
    except: pass 

    file_name = save_dir + '/urdf/' + robot_name + '.xacro'  # the name of urdf file
    repo = package_name + '/meshes/'  # the repository of binary stl files
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'.format(robot_name))
        f.write('\n')
        # ROS2: use relative paths so xacro resolves from same directory when launch passes full path
        f.write('<xacro:include filename="materials.xacro" />\n')
        f.write('<xacro:include filename="{}.trans" />\n'.format(robot_name))
        f.write('<xacro:include filename="{}.gazebo" />\n'.format(robot_name))
        f.write('\n')

    write_link_urdf(joints_dict, repo, links_xyz_dict, file_name, inertial_dict)
    write_joint_urdf(joints_dict, repo, links_xyz_dict, file_name)
    write_gazebo_endtag(file_name)

def write_materials_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    try: os.mkdir(save_dir + '/urdf')
    except: pass  

    file_name = save_dir + '/urdf/materials.xacro'  # the name of urdf file
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(robot_name))
        f.write('\n')
        f.write('<material name="silver">\n')
        f.write('  <color rgba="0.700 0.700 0.700 1.000"/>\n')
        f.write('</material>\n')
        f.write('\n')
        f.write('</robot>\n')

def write_transmissions_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    """
    Write joints and transmission information into urdf "repo/file_name"
    
    
    Parameters
    ----------
    joints_dict: dict
        information of the each joint
    repo: str
        the name of the repository to save the xml file
    links_xyz_dict: dict
        xyz information of the each link
    file_name: str
        urdf full path
    """
    
    file_name = save_dir + '/urdf/{}.trans'.format(robot_name)  # the name of urdf file
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(robot_name))
        f.write('\n')

        for j in joints_dict:
            parent = joints_dict[j]['parent']
            child = joints_dict[j]['child']
            joint_type = joints_dict[j]['type']
            upper_limit = joints_dict[j]['upper_limit']
            lower_limit = joints_dict[j]['lower_limit']
            try:
                xyz = [round(p-c, 6) for p, c in \
                    zip(links_xyz_dict[parent], links_xyz_dict[child])]  # xyz = parent - child
            except KeyError as ke:
                app = adsk.core.Application.get()
                ui = app.userInterface
                ui.messageBox("There seems to be an error with the connection between\n\n%s\nand\n%s\n\nCheck \
whether the connections\nparent=component2=%s\nchild=component1=%s\nare correct or if you need \
to swap component1<=>component2"
                % (parent, child, parent, child), "Error!")
                quit()
                
            joint = Joint.Joint(name=j, joint_type = joint_type, xyz=xyz, \
            axis=joints_dict[j]['axis'], parent=parent, child=child, \
            upper_limit=upper_limit, lower_limit=lower_limit)
            if joint_type != 'fixed':
                joint.make_transmission_xml()
                f.write(joint.tran_xml)
                f.write('\n')

        f.write('</robot>\n')

def write_gazebo_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    try: os.mkdir(save_dir + '/urdf')
    except: pass  

    file_name = save_dir + '/urdf/' + robot_name + '.gazebo'  # the name of urdf file
    repo = robot_name + '/meshes/'  # the repository of binary stl files
    #repo = package_name + '/' + robot_name + '/bin_stl/'  # the repository of binary stl files
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="{}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n'.format(robot_name))
        f.write('\n')
        f.write('<xacro:property name="body_color" value="Gazebo/Silver" />\n')
        f.write('\n')

        gazebo = Element('gazebo')
        plugin = SubElement(gazebo, 'plugin')
        plugin.attrib = {'name':'control', 'filename':'libgazebo_ros2_control.so'}
        gazebo_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        f.write(gazebo_xml)

        # for base_link
        f.write('<gazebo reference="base_link">\n')
        f.write('  <material>${body_color}</material>\n')
        f.write('  <mu1>0.2</mu1>\n')
        f.write('  <mu2>0.2</mu2>\n')
        f.write('  <selfCollide>true</selfCollide>\n')
        f.write('  <gravity>true</gravity>\n')
        f.write('</gazebo>\n')
        f.write('\n')

        # others
        for joint in joints_dict:
            name = joints_dict[joint]['child']
            f.write('<gazebo reference="{}">\n'.format(name))
            f.write('  <material>${body_color}</material>\n')
            f.write('  <mu1>0.2</mu1>\n')
            f.write('  <mu2>0.2</mu2>\n')
            f.write('  <selfCollide>true</selfCollide>\n')
            f.write('</gazebo>\n')
            f.write('\n')

        f.write('</robot>\n')

def write_display_launch(package_name, robot_name, save_dir):
    """
    Write ROS2 display launch file "save_dir/launch/display_launch.py"
    """
    try: os.mkdir(save_dir + '/launch')
    except: pass

    file_name = save_dir + '/launch/display_launch.py'
    content = '''# ROS2 launch file for displaying URDF in RViz2
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('{package_name}')
    default_model = os.path.join(pkg_share, 'urdf', '{robot_name}.xacro')
    default_rviz = os.path.join(pkg_share, 'launch', 'urdf.rviz')

    model_arg = DeclareLaunchArgument(name='model', default_value=default_model, description='Path to robot xacro')
    rvizconfig_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz, description='Path to rviz config')

    robot_description = Command(['xacro ', LaunchConfiguration('model')])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{{'robot_description': robot_description}}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    return LaunchDescription([
        model_arg,
        rvizconfig_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])
'''.format(package_name=package_name, robot_name=robot_name)
    with open(file_name, mode='w') as f:
        f.write(content)

def write_gazebo_launch(package_name, robot_name, save_dir):
    """
    Write ROS2 Gazebo launch file "save_dir/launch/gazebo_launch.py"
    """
    try: os.mkdir(save_dir + '/launch')
    except: pass

    file_name = save_dir + '/launch/gazebo_launch.py'
    content = '''# ROS2 launch file for Gazebo simulation
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('{package_name}')
    xacro_path = os.path.join(pkg_share, 'urdf', '{robot_name}.xacro')
    robot_description = Command(['xacro ', xacro_path])

    gazebo_share = get_package_share_directory('gazebo_ros')
    empty_world_launch = os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(empty_world_launch),
        launch_arguments={{'paused': 'true', 'use_sim_time': 'true', 'gui': 'true'}}.items()
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{{'robot_description': robot_description}}],
        output='screen'
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', '{robot_name}'],
        parameters=[{{'robot_description': robot_description}}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
'''.format(package_name=package_name, robot_name=robot_name)
    with open(file_name, mode='w') as f:
        f.write(content)


def write_control_launch(package_name, robot_name, save_dir, joints_dict):
    """
    Write ROS2 control launch file "save_dir/launch/controller_launch.py"
    """
    try: os.mkdir(save_dir + '/launch')
    except: pass

    controller_list = ['joint_state_broadcaster']
    if any(joints_dict[j]['type'] != 'fixed' for j in joints_dict):
        controller_list.append('position_controller')
    controller_args_py = ', '.join(repr(c) for c in controller_list)

    file_name = save_dir + '/launch/controller_launch.py'
    content = '''# ROS2 launch file for ros2_control / gazebo_ros2_control
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[{controller_args}],
            output='screen',
        ),
    ])
'''.format(package_name=package_name, robot_name=robot_name,
           controller_args=controller_args_py)
    with open(file_name, mode='w') as f:
        f.write(content)
        

def write_yaml(package_name, robot_name, save_dir, joints_dict):
    """
    Write ROS2 ros2_control controller config "save_dir/config/controller.yaml"
    """
    try: os.mkdir(save_dir + '/config')
    except: pass

    file_name = save_dir + '/config/controller.yaml'
    joint_names = [j for j in joints_dict if joints_dict[j]['type'] != 'fixed']
    with open(file_name, 'w') as f:
        f.write('# ROS2 ros2_control controller config (used with gazebo_ros2_control)\n')
        f.write('controller_manager:\n')
        f.write('  ros__parameters:\n')
        f.write('    update_rate: 100  # Hz\n')
        f.write('\n')
        f.write('    joint_state_broadcaster:\n')
        f.write('      type: joint_state_broadcaster/JointStateBroadcaster\n')
        f.write('\n')
        if joint_names:
            f.write('    position_controller:\n')
            f.write('      type: position_controllers/JointGroupPositionController\n')
            f.write('      joints:\n')
            for j in joint_names:
                f.write('        - {}\n'.format(j))
        f.write('\n')

