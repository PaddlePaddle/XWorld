#!/usr/bin/python

"""
Convert an .obj model to unit size and set its origin as
((min_x + max_x) / 2, 0, (min_z + max_z) / 2)
"""

import numpy as np
import os
import sys
from xml.etree import ElementTree as et
import glob

XML = """<?xml version="1.0"?>
   <robot name="none">
       <link name="none">
           <visual>
               <origin rpy="1.57 0 0" xyz="0 0 0"/>
               <geometry>
                   <mesh filename="none.obj"/>
               </geometry>
           </visual>
          <collision>
              <geometry>
                  <box size="1 1 1"/>
              </geometry>
          </collision>

          <inertial>
            <mass value="0.0"/>
            <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
          </inertial>
      </link>
  </robot>
"""

def normalize(vertices):
    max_x, max_y, max_z = np.max(np.array(vertices), axis=0)
    min_x, min_y, min_z = np.min(np.array(vertices), axis=0)

    sizes = np.array([max_x - min_x, max_y - min_y, max_z - min_z])
    scale = np.max(sizes)
    nor_sizes = sizes / scale
    return (np.array(vertices) - np.array([(max_x + min_x) / 2, min_y,
                                            (max_z + min_z) / 2])) / float(scale), \
                                            nor_sizes


def normalize_obj_file(obj_file):
    with open(obj_file, "r") as f:
        lines = f.read().splitlines()

    print(len(lines))

    vertices = [map(float, l.split()[1:]) for l in lines if l.startswith("v ")]
    normal_vertices, nor_sizes = normalize(vertices)

    with open(obj_file, "w") as f:
        i = 0
        for l in lines:
            if l.startswith("v "):
                out = "v " + " ".join(map(str, normal_vertices[i]))
                i += 1
            else:
                out = l
            f.write(out + "\n")
    return nor_sizes


def generate_urdf(obj_file, new_name, bb_sizes):
    print bb_sizes
    xml = et.fromstring(XML)
    ## x has 90 rotation
    for n in xml.iter('mesh'):
        n.attrib["filename"] = os.path.basename(obj_file)
    for n in xml.iter('box'):
        n.attrib["size"] = "%f %f %f" % (bb_sizes[0], bb_sizes[1], bb_sizes[2])
    urdf_file = os.path.join(os.path.dirname(obj_file), new_name.split("_")[0] + ".urdf")
    et.ElementTree(xml).write(urdf_file)

"""
usage: ./obj_normalize.py models_3d/goal/furniture
       ./obj_normalize.py models_3d/goal/furniture chair
"""
if __name__ == "__main__":
    path = sys.argv[1]
    if len(sys.argv) > 2:
        dirs = glob.glob(path + "/" + sys.argv[2] + "_*/")
    else:
        dirs = glob.glob(path + "/*/")
    print(dirs)
    for d in dirs:
        obj_files = glob.glob(d + "/*.obj")
        assert len(obj_files) == 1, d

        bb_sizes = normalize_obj_file(obj_files[0])

        generate_urdf(obj_files[0], os.path.basename(os.path.dirname(d)), bb_sizes)
