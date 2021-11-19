#ifndef MESH_TO_SDF_MESH_TO_SDF_H
#define MESH_TO_SDF_MESH_TO_SDF_H


#include "mesh_to_sdf/ConvertMeshToSdf.h"

#include <ros/ros.h>
#include <geometry_msgs/Transform.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>


#include <voxblox/core/esdf_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox_ground_truth/sdf_creator.h>


namespace mesh_to_sdf
{

class MeshToSdf
{
public:
  MeshToSdf(ros::NodeHandle& nh, ros::NodeHandle& pnh);

private:

  bool convertPlyToTsdfCallback(mesh_to_sdf::ConvertMeshToSdf::Request& request,
                                mesh_to_sdf::ConvertMeshToSdf::Response& response);

  bool convertPlyToEsdfCallback(mesh_to_sdf::ConvertMeshToSdf::Request& request,
                                mesh_to_sdf::ConvertMeshToSdf::Response& response);


  /**
   * Load a ply file. The ply mesh needs to be triangular (all faces have 3 vertices),
   * as other meshes cannot be handeled by the voxblox_ground_truth package yet.
   * @param [in] input_file_name Path and name of input ply file
   * @param [out] mesh PolygonMesh in which the ply file is loaded.
   * @param [out] point_cloud Pointcloud from the loaded mesh.
   * @return false if something went wrong, e.g. file does not exist.
   */
  bool loadPlyFile(std::string input_file_name, pcl::PolygonMesh& mesh, pcl::PointCloud<pcl::PointXYZ>& point_cloud);


  /**
   * Converts a geometry_msgs::Transform into a voxblox::Transformation. If all values in the quaternion are zero (as
   * e.g. is default when calling the service), w is set to 1 in order to avoid error when setting the voxblox::Transformation
   * @param transform_msg
   * @return
   */
  voxblox::Transformation convertTransform(geometry_msgs::Transform transform_msg);

  /**
   * Functionality for converting a polygon mesh + pointcloud into a tsdf was mainly taken from here:
   * https://github.com/ethz-asl/voxblox_ground_truth/blob/master/src/user_interfaces/ply_to_sdf_script.cpp
   *
   * Computes a TSDF given a  polygon mesh and a pointcloud containing the vertex coodinates.
   * @param transform Transform to be applied during conversion to rotate and/or translate the tsdf
   * @param scale_factor Scale factor for the tsdf
   * @param fill_inside Whether the mesh inside is free or occupied
   * @param floodfill_unoccupied Floodfill unoccupied space.
   * @param voxel_size Voxel size of the tsdf
   * @param [in] mesh
   * @param [in] vertex_coordinates
   * @return resulting tsdf map
   */
  std::shared_ptr<voxblox::TsdfMap> computeTsdf(voxblox::Transformation transform, float scale_factor, bool fill_inside,
                                                bool floodfill_unoccupied, float voxel_size, pcl::PolygonMesh& mesh,
                                                pcl::PointCloud<pcl::PointXYZ>& vertex_coordinates);

  /**
   * Enforce truncation of tsdf (set all values above truncation distance to truncation_distance considering the signs).
   * @param [in,out] tsdf_map
   * @param [in] truncation_distance If truncation distance is 0, no truncation will be done.
   */
  void truncateTSDF(std::shared_ptr<voxblox::TsdfMap> tsdf_map, float truncation_distance);

  /**
   * Computes an ESDF based on the given tsdf_map.
   * @param tsdf_map
   * @return Computed esdf_map
   */
  std::shared_ptr<voxblox::EsdfMap> computeEsdf(std::shared_ptr<voxblox::TsdfMap> tsdf_map);


  /**
   * Saves the tsdf_map to a file. Can be loaded e.g. using the TsdfServer's loadMap method or service.
   * @param output_file_name File path
   * @param tsdf_map Map that shall be stored.
   * @return false if something went wrong while saving, e.g. file not writable
   */
  bool saveTsdf(const std::string& output_file_name, std::shared_ptr<voxblox::TsdfMap> tsdf_map);

  /**
   * Saves first tsdf_map to the file and appends the esdf map afterwards. Can be loaded e.g. using the EsdfServer's loadMap method or service.
   * When using the loadMap function it is normal, that after the message, that the TSDF was loaded first a message appears that it tries
   * to load a tsdf map into an esdf map as the file is parsed again from the top. But it continues and loads the EsdfMap correctly.
   * @param output_file_name File path
   * @param tsdf_map Tsdf map which is the base of the esdf map, as this also need to be stored in the same file.
   * @param esdf_map Esdf map that shall be stored.
   * @return false if something went wrong while saving, e.g. file not writable.
   */
  bool saveEsdf(const std::string& output_file_name, std::shared_ptr<voxblox::TsdfMap> tsdf_map,
                std::shared_ptr<voxblox::EsdfMap> esdf_map);


  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::ServiceServer convert_ply_to_tsdf_service_server_;
  ros::ServiceServer convert_ply_to_esdf_service_server_;
};
} // end namespace mesh_to_sdf


#endif //MESH_TO_SDF_MESH_TO_SDF_H
