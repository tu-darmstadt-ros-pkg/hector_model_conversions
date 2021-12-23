

#include "mesh_to_sdf/mesh_to_sdf.h"
#include <ros/ros.h>


bool convertPlyToTsdfCallback(mesh_to_sdf::ConvertMeshToSdf::Request& request,
                              mesh_to_sdf::ConvertMeshToSdf::Response& response)
{
  std::unique_ptr<mesh_to_sdf::MeshToSdf> mesh_to_sdf;

  try
  {
    // load and convert mesh
    mesh_to_sdf = std::make_unique<mesh_to_sdf::MeshToSdf>(request.input_file, request.transform, request.scale_factor,
                                                           request.fill_inside, request.floodfill_unoccupied,
                                                           request.voxel_size, request.truncation_distance);
  }
  catch (std::invalid_argument& e)
  {
    response.result = mesh_to_sdf::ConvertMeshToSdf::Response::LOAD_FILE_ERROR;
    response.error_msg = e.what();
    return true;
  }
  catch (std::runtime_error& e)
  {
    response.result = mesh_to_sdf::ConvertMeshToSdf::Response::CONVERSION_ERROR;
    response.error_msg = e.what();
    return true;
  }

  if (!mesh_to_sdf)
  {
    return false;
  }

  // save TSDF
  try
  {
    mesh_to_sdf->saveTsdf(request.output_file);
  }
  catch (std::invalid_argument& e)
  {
    response.result = mesh_to_sdf::ConvertMeshToSdf::Response::SAVE_FILE_ERROR;
    response.error_msg = e.what();
    return true;
  }

  return true;
}


bool convertPlyToEsdfCallback(mesh_to_sdf::ConvertMeshToSdf::Request& request,
                              mesh_to_sdf::ConvertMeshToSdf::Response& response)
{
  std::unique_ptr<mesh_to_sdf::MeshToSdf> mesh_to_sdf;

  try
  {
    // load and convert mesh
    mesh_to_sdf = std::make_unique<mesh_to_sdf::MeshToSdf>(request.input_file, request.transform, request.scale_factor,
                                                           request.fill_inside, request.floodfill_unoccupied,
                                                           request.voxel_size, request.truncation_distance);
  }
  catch (std::invalid_argument& e)
  {
    response.result = mesh_to_sdf::ConvertMeshToSdf::Response::LOAD_FILE_ERROR;
    response.error_msg = e.what();
    return true;
  }
  catch (std::runtime_error& e)
  {
    response.result = mesh_to_sdf::ConvertMeshToSdf::Response::CONVERSION_ERROR;
    response.error_msg = e.what();
    return true;
  }

  if (!mesh_to_sdf)
  {
    return false;
  }

  // save ESDF
  try
  {
    mesh_to_sdf->saveEsdf(request.output_file);
  }
  catch (std::invalid_argument& e)
  {
    response.result = mesh_to_sdf::ConvertMeshToSdf::Response::SAVE_FILE_ERROR;
    response.error_msg = e.what();
    return true;
  }

  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, ROS_PACKAGE_NAME);
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // init services
  ros::ServiceServer convert_ply_to_tsdf_service_server = pnh.advertiseService("convert_ply_to_tsdf",
                                                                               convertPlyToTsdfCallback);
  ros::ServiceServer convert_ply_to_esdf_service_server = pnh.advertiseService("convert_ply_to_esdf",
                                                                               convertPlyToEsdfCallback);


  ros::spin();


  return 0;
}
