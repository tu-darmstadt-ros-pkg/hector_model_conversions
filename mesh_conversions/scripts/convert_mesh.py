#!/usr/bin/env python

import rospy
import pymeshlab  # TODO: set dependency to pymeshlab

from mesh_conversions.srv import *


def convert_mesh(input_file_name, output_file_name):

    ms = pymeshlab.MeshSet()

    # load mesh
    try:
        ms.load_new_mesh(input_file_name)
    except pymeshlab.PyMeshLabException as e:
        return ConvertMeshResponse(ConvertMeshResponse.LOAD_FILE_ERROR, str(e))

    # save mesh, save parameters according to
    # https://pymeshlab.readthedocs.io/en/latest/io_format_list.html#save-mesh-parameters
    try:

        if '.ply' in output_file_name:
            ms.save_current_mesh(output_file_name, binary=False, save_face_color=False)

            # Load written file and replace "double" with "float" in header,
            # as otherwise pcl cannot read the ply file with pcl::PointXYZ.
            # Maybe there will be an option for this in the future, see
            # https://github.com/cnr-isti-vclab/PyMeshLab/issues/128
            with open(output_file_name, 'r') as file:
                filedata = file.read()

            filedata = filedata.replace('double', 'float')

            with open(output_file_name, 'w') as file:
                file.write(filedata)

        else:
            # default case without any save parameters
            ms.save_current_mesh(output_file_name)

    except pymeshlab.PyMeshLabException as e:
        return ConvertMeshResponse(ConvertMeshResponse.SAVE_FILE_ERROR, str(e))

    # everything worked, return success
    return ConvertMeshResponse(ConvertMeshResponse.SUCCESS, '')


def convert_mesh_callback(request):
    return convert_mesh(request.input_file_name, request.output_file_name)


def convert_mesh_server():
    service = rospy.Service('~convert_mesh', ConvertMesh, convert_mesh_callback)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('convert_mesh')
    convert_mesh_server()
