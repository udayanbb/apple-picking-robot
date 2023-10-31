import random
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
          <ambient>0.6 0.2 0.1 1</ambient>
          <diffuse>0.6 0.2 0.1 1</diffuse>
          <specular>0.6 0.2 0.1 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
    </link>
'''
first_branch = ET.fromstring(first_branch_string)

model.append(first_branch)

def add_branch(parent_id, branch_id, length, rad):

    half_len = length/2

    branch_string = f'''<link name="branch_{branch_id}">
	  <pose relative_to="joint_{parent_id}_{branch_id}">0 0 {half_len} 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{rad}</radius>
            <length>{length}</length>
          </cylinder>
        </geometry>
    	<material>
    	  <ambient>0.6 0.2 0.1 1</ambient>
    	  <diffuse>0.6 0.2 0.1 1</diffuse>
    	  <specular>0.6 0.2 0.1 1</specular>
    	  <emissive>0 0 0 1</emissive>
    	</material>
      </visual>
    </link>
'''
    branch = ET.fromstring(branch_string)
    model.append(branch)

    r1 = (random.random() - 0.5) * 1.5
    r2 = (random.random() - 0.5) * 1.5

    joint_string = f'''<joint name="joint_{parent_id}_{branch_id}" type="universal">
      <pose relative_to="branch_{parent_id}">0 0 {half_len} {r1} {r2} 0</pose>
      <parent>branch_{parent_id}</parent>
      <child>branch_{branch_id}</child>
	  <axis>
          <xyz>1 0 0 </xyz>
      </axis>
	  <axis2>
          <xyz>0 1 0 </xyz>
      </axis2>
    </joint>
'''
    joint = ET.fromstring(joint_string)
    model.append(joint)

    return

def recurse(parent_id, n):
    if n == 0:
        return
    # Add 2 kids, and call recurse on each of them
    for i in ['a', 'b', 'c']:
        branch_id = parent_id + i
        t = n
        add_branch(parent_id, branch_id, 1/(6-t)* 1.5, 1/(5-t) * 0.05)
        recurse(branch_id, n-1)

recurse('a', 4)

ET.indent(root, space="  ", level=0)
ET.dump(root)

tree = ET.ElementTree(root)
tree.write("tree.sdf")

