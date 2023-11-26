import random

import numpy as np

import xml.etree.ElementTree as ET

template = '''<?xml version="1.0"?>
<sdf version="1.7">
  <model name="tree">
  </model>
</sdf>
'''

root = ET.fromstring(template)
model = root[0]

first_branch_string = '''<link name="branch_a">
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.06</radius>
            <length>1</length>
          </cylinder>
        </geometry>
    	<material>
    	  <ambient>0.3 0.15 0.15 1</ambient>
    	  <diffuse>0.3 0.15 0.15 1</diffuse>
    	  <specular>0.0 0.0 0.0 1</specular>
    	  <emissive>0 0 0 1</emissive>
    	</material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.06</radius>
            <length>1</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
'''
first_branch = ET.fromstring(first_branch_string)

model.append(first_branch)

stiffnesses = [5000, 1000, 100, 10, 1]
dampings = [100, 20, 5, 3, 0.1]


stiffnesses = [200, 5, 10, 100, 1]
dampings = [10, 5, 1, 0.1]


#stiffnesses = [5000, 1000, 100, 10, 1]
#dampings = [100, 20, 5, 3, 0.1]


stiffnesses = [2000, 300, 10, 1]
dampings = [100, 5, 3, 0.1]

def add_branch(parent_id, branch_id, length, rad, depth):

    half_len = length/2

    branch_string = f'''<link name="branch_{branch_id}">
	  <pose relative_to="joint2_{parent_id}_couple_{branch_id}">0 0 {half_len} 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{rad}</radius>
            <length>{length}</length>
          </cylinder>
        </geometry>
    	<material>
    	  <ambient>0.3 0.15 0.15 1</ambient>
    	  <diffuse>0.3 0.15 0.15 1</diffuse>
    	  <specular>0.0 0.0 0.0 1</specular>
    	  <emissive>0 0 0 1</emissive>
    	</material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{rad}</radius>
            <length>{length*0.6}</length>
          </cylinder>
        </geometry>
      </collision>
     </link>
'''
    branch = ET.fromstring(branch_string)
    model.append(branch)

    r1 = (random.random() - 0.5) * np.pi * 0.5
    r2 = (random.random() - 0.5) * np.pi * 0.5

    if depth == 0:
        r1 = -np.pi/5
        r2 = np.pi/5
    elif depth == 1:
        r1 = (random.random() - 0.5) * np.pi * 0.4
        r2 = (random.random() - 0.5) * np.pi * 0.4

    stiffness = stiffnesses[depth]
    damping = dampings[depth]

    print(f"stiffness: {stiffness} damping: {damping} depth: {depth}")

    branch_z = random.random() * half_len/2 + half_len/2

	# we want a universal joint with stiffness, which Drake doesn't support
    # so use two revolute joints with a coupling in between
    joint1_string = f'''<joint name="joint1_{parent_id}_couple_{branch_id}" type="revolute">
      <pose relative_to="branch_{parent_id}">0 0 {branch_z} {r1} {r2} 0</pose>
      <parent>branch_{parent_id}</parent>
      <child>couple_{parent_id}_{branch_id}</child>
	  <axis>
          <xyz>1 0 0 </xyz>
	      <dynamics>
              <damping>{damping}</damping>
              <friction>0</friction>
              <spring_reference>0</spring_reference>
              <spring_stiffness>{stiffness}</spring_stiffness>
          </dynamics>
          <limit>
            <!-- not actuated -->
            <effort>0</effort>
          </limit>
      </axis>
    </joint>
'''
    couple_string = f'''<link name="couple_{parent_id}_{branch_id}">
      <pose relative_to="joint1_{parent_id}_couple_{branch_id}">0 0 0 0 0 0</pose>
    </link>
'''
    joint2_string = f'''<joint name="joint2_{parent_id}_couple_{branch_id}" type="revolute">
      <pose relative_to="couple_{parent_id}_{branch_id}">0 0 0.01 0 0 0</pose>
      <parent>couple_{parent_id}_{branch_id}</parent>
      <child>branch_{branch_id}</child>
	  <axis>
          <xyz>0 1 0 </xyz>
	      <dynamics>
              <damping>{damping}</damping>
              <friction>0</friction>
              <spring_reference>0</spring_reference>
              <spring_stiffness>{stiffness}</spring_stiffness>
          </dynamics>
          <limit>
            <!-- not actuated -->
            <effort>0</effort>
          </limit>
      </axis>
    </joint>
'''
    joint1 = ET.fromstring(joint1_string)
    joint2 = ET.fromstring(joint2_string)
    couple = ET.fromstring(couple_string)
    model.append(joint1)
    model.append(joint2)
    model.append(couple)

	# Apple origin is at tip of stalk
    apple_rad = 0.03
    stalk = 0.01

    apple_string = f'''<link name="apple_{branch_id}">
      <pose relative_to="joint_{branch_id}_apple">0 0 0 0 0 0</pose>
      <visual name="apple">
        <pose>0 0 {stalk + apple_rad} 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>{apple_rad}</radius>
          </sphere>
        </geometry>
    	<material>
    	  <ambient>0.1 0.5 0.2 1</ambient>
    	  <diffuse>0.1 0.4 0.2 1</diffuse>
    	  <specular>0.2 0.4 0.1 1</specular>
    	  <emissive>0 0 0 1</emissive>
    	</material>
      </visual>
      <visual name="stalk">
        <pose>0 0 {stalk/2} 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.002</radius>
            <length>{stalk}</length>
          </cylinder>
        </geometry>
    	<material>
    	  <ambient>0.4 0.2 0.1 1</ambient>
    	  <diffuse>0.4 0.2 0.1 1</diffuse>
    	  <specular>0.2 0.1 0.1 1</specular>
    	  <emissive>0 0 0 1</emissive>
    	</material>
      </visual>
      <inertial>
        <pose>0 0 {stalk + apple_rad} 0 0 0</pose>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0005</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0005</iyy>
          <iyz>0</iyz>
          <izz>0.0005</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <pose>0 0 {stalk + apple_rad} 0 0 0</pose>
        <geometry>
          <sphere>
            <radius>{apple_rad}</radius>
          </sphere>
        </geometry>
      </collision>
    </link>
'''
    apple_joint_string = f'''<joint name="joint_{branch_id}_apple" type="ball">
      <pose relative_to="branch_{branch_id}">{rad} 0 0 {0} {np.pi/2} 0</pose>
      <parent>branch_{branch_id}</parent>
      <child>apple_{branch_id}</child>
      <axis>
          <dynamics>
              <damping>0.002</damping>
              <friction>0</friction>
              <spring_reference>0</spring_reference>
              <spring_stiffness>0.2</spring_stiffness>
          </dynamics>
          <limit>
            <!-- not actuated -->
            <effort>0</effort>
          </limit>
        </axis>
    </joint>
'''
    apple_z = (np.random.rand() - 0.5) * half_len + 0.4 * half_len
    apple_joint_string = f'''<joint name="joint_{branch_id}_apple" type="universal">
      <pose relative_to="branch_{branch_id}">{rad} 0 {apple_z} 0 {np.pi/2} 0</pose>
      <parent>branch_{branch_id}</parent>
      <child>apple_{branch_id}</child>
	  <axis>
          <xyz>0 1 0 </xyz>
	      <dynamics>
              <damping>0.001</damping>
              <friction>0</friction>
              <spring_reference>0</spring_reference>
              <spring_stiffness>0.2</spring_stiffness>
          </dynamics>
          <limit>
            <!-- not actuated -->
            <effort>0</effort>
          </limit>
      </axis>
	  <axis2>
          <xyz>1 0 0 </xyz>
	      <dynamics>
              <damping>0.001</damping>
              <friction>0</friction>
              <spring_reference>0</spring_reference>
              <spring_stiffness>0.2</spring_stiffness>
          </dynamics>
          <limit>
            <!-- not actuated -->
            <effort>0</effort>
          </limit>
      </axis2>
    </joint>
'''
    apple = ET.fromstring(apple_string)
    apple_joint = ET.fromstring(apple_joint_string)
    model.append(apple)
    model.append(apple_joint)
    return

depth = 3

def recurse(parent_id, n):
    if n == 0:
        return
    # Add 2 kids, and call recurse on each of them
    children =  ['a', 'b', 'c']
    #children =  ['a']
    if n == 3:
        children = ['a']
    for i in children:
        branch_id = parent_id + i
        t = n
        add_branch(parent_id, branch_id, 1/(6-t)* 2, 1/(5-t) * 0.05, depth-n)
        recurse(branch_id, n-1)

recurse('a', depth)

ET.indent(root, space="  ", level=0)
#ET.dump(root)

tree = ET.ElementTree(root)
tree.write("tree.sdf")

