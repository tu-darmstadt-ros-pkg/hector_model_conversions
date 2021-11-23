

#include "mesh_to_sdf/mesh_to_sdf.h"

namespace mesh_to_sdf
{

MeshToSdf::MeshToSdf(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh)
{
  convert_ply_to_tsdf_service_server_ = pnh_.advertiseService("convert_ply_to_tsdf",
                                                              &MeshToSdf::convertPlyToTsdfCallback, this);
  convert_ply_to_esdf_service_server_ = pnh_.advertiseService("convert_ply_to_esdf",
                                                              &MeshToSdf::convertPlyToEsdfCallback, this);
}


bool MeshToSdf::convertPlyToTsdfCallback(mesh_to_sdf::ConvertMeshToSdf::Request& request,
                                         mesh_to_sdf::ConvertMeshToSdf::Response& response)
{
  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZ> point_cloud;

  // load file
  if (!loadPlyFile(request.input_file, mesh, point_cloud))
  {
    response.result = mesh_to_sdf::ConvertMeshToSdf::Response::LOAD_FILE_ERROR;
    response.error_msg = "Error while loading ply file. Please see PCL error for more information.";
  }

  // compute TSDF
  std::shared_ptr<voxblox::TsdfMap> tsdf_map = computeTsdf(convertTransform(request.transform), request.scale_factor,
                                                           request.fill_inside, request.floodfill_unoccupied,
                                                           request.voxel_size, mesh, point_cloud);

  // enforce truncation of tsdf_map (if truncation distance is 0 no truncation will be done)
  truncateTSDF(tsdf_map, request.truncation_distance);

  // save TSDF
  if (!saveTsdf(request.output_file, tsdf_map))
  {
    response.result = mesh_to_sdf::ConvertMeshToSdf::Response::SAVE_FILE_ERROR;
    response.error_msg = "Error while saving tsdf file. Please see voxblox error for more information.";
  }

  // always return true so that caller receives response
  return true;
}


bool MeshToSdf::convertPlyToEsdfCallback(mesh_to_sdf::ConvertMeshToSdf::Request& request,
                                         mesh_to_sdf::ConvertMeshToSdf::Response& response)
{
  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZ> point_cloud;

  // load file
  if (!loadPlyFile(request.input_file, mesh, point_cloud))
  {
    response.result = mesh_to_sdf::ConvertMeshToSdf::Response::LOAD_FILE_ERROR;
    response.error_msg = "Error while loading ply file. Please see PCL error for more information.";
  }

  // compute TSDF
  std::shared_ptr<voxblox::TsdfMap> tsdf_map = computeTsdf(convertTransform(request.transform), request.scale_factor,
                                                           request.fill_inside, request.floodfill_unoccupied,
                                                           request.voxel_size, mesh, point_cloud);

  // enforce truncation of tsdf_map (if truncation distance is 0 no truncation will be done)
  truncateTSDF(tsdf_map, request.truncation_distance);

  // compute ESDF from TSDF
  std::shared_ptr<voxblox::EsdfMap> esdf_map = computeEsdf(tsdf_map);

  // save ESDF
  if (!saveEsdf(request.output_file, tsdf_map, esdf_map))
  {
    response.result = mesh_to_sdf::ConvertMeshToSdf::Response::SAVE_FILE_ERROR;
    response.error_msg = "Error while saving esdf file. Please see voxblox error for more information.";
  }

  // always return true so that caller receives response
  return true;
}


bool MeshToSdf::loadPlyFile(std::string input_file_name, pcl::PolygonMesh& mesh,
                            pcl::PointCloud<pcl::PointXYZ>& point_cloud)
{
  int load_res = pcl::io::loadPLYFile(input_file_name, mesh);
  if (load_res != 0)
  {
    return false;
  }

  pcl::fromPCLPointCloud2(mesh.cloud, point_cloud);

  return true;
}


voxblox::Transformation MeshToSdf::convertTransform(geometry_msgs::Transform transform_msg)
{
  voxblox::Transformation transform;
  transform.getPosition().x() = static_cast<float>(transform_msg.translation.x);
  transform.getPosition().y() = static_cast<float>(transform_msg.translation.y);
  transform.getPosition().z() = static_cast<float>(transform_msg.translation.z);

  // if everything in service request was zero (which is default e.g. when calling using rosservice CLI),
  // set w to 1.0 as otherwise setValues() for rotation results in an error.
  if (transform_msg.rotation.w == 0 && transform_msg.rotation.x == 0 &&
      transform_msg.rotation.y == 0 && transform_msg.rotation.z == 0)
  {
    transform_msg.rotation.w = 1.0;
  }

  transform.getRotation().setValues(static_cast<float>(transform_msg.rotation.w),
                                    static_cast<float>(transform_msg.rotation.x),
                                    static_cast<float>(transform_msg.rotation.y),
                                    static_cast<float>(transform_msg.rotation.z));

  return transform;
}


std::shared_ptr<voxblox::TsdfMap> MeshToSdf::computeTsdf(voxblox::Transformation transform, float scale_factor,
                                                         bool fill_inside, bool floodfill_unoccupied, float voxel_size,
                                                         pcl::PolygonMesh& mesh,
                                                         pcl::PointCloud<pcl::PointXYZ>& vertex_coordinates)
{
  // Initialize the SDF creator
  voxblox::TsdfMap::Config map_config;
  map_config.tsdf_voxel_size = voxel_size;
  voxblox_ground_truth::SdfCreator sdf_creator(map_config);

  // Set whether inside the mesh is free or occupied.
  sdf_creator.setFillInside(fill_inside);

  // Iterate over all triangles
  size_t triangle_i = 0;
  size_t num_triangles = mesh.polygons.size();
  for (const pcl::Vertices& polygon : mesh.polygons)
  {
    // Ensure that the polygon is a triangle (other meshes are not supported)
    CHECK_EQ(polygon.vertices.size(), 3);

    // Indicate progress
    triangle_i++;
    // Only print progress for each promile of completion, to reduce IO wait
    if (triangle_i % (num_triangles / 1000) == 0)
    {
      printf("\rProgress: %3.1f%% - total nr of blocks %lu", triangle_i / static_cast<double>(num_triangles) * 100,
             sdf_creator.getNumberOfAllocatedBlocks());
      std::cout << std::flush;
    }

    // Extract the triangle's vertices from the vertex coordinate pointcloud
    const Point vertex_a = vertex_coordinates[polygon.vertices[0]].getVector3fMap();
    const Point vertex_b = vertex_coordinates[polygon.vertices[1]].getVector3fMap();
    const Point vertex_c = vertex_coordinates[polygon.vertices[2]].getVector3fMap();

    // Transform the vertices from mesh frame into world frame
    TriangularFaceVertexCoordinates triangle_vertices;
    triangle_vertices.vertex_a = transform * (scale_factor * vertex_a);
    triangle_vertices.vertex_b = transform * (scale_factor * vertex_b);
    triangle_vertices.vertex_c = transform * (scale_factor * vertex_c);

    // Update the SDF with the new triangle
    sdf_creator.integrateTriangle(triangle_vertices);
  }

  ROS_INFO("Distance field building complete.");

  // Optionally floodfill unoccupied space.
  if (floodfill_unoccupied)
  {
    sdf_creator.floodfillUnoccupied(4 * voxel_size);
  }

  return std::make_shared<voxblox::TsdfMap>(sdf_creator.getTsdfMap());
}


void MeshToSdf::truncateTSDF(std::shared_ptr<voxblox::TsdfMap> tsdf_map, float truncation_distance)
{
  // do nothing if truncation distance is zero
  if (truncation_distance == 0)
  {
    return;
  }

  // get all allocated blocks
  voxblox::BlockIndexList block_idx_list;
  tsdf_map->getTsdfLayer().getAllAllocatedBlocks(&block_idx_list);

  // iter blocks
  for (Eigen::Matrix<voxblox::IndexElement, 3, 1>& block_idx : block_idx_list)
  {
    voxblox::Layer<voxblox::TsdfVoxel>& layer = *(tsdf_map->getTsdfLayerPtr());
    voxblox::Block<voxblox::TsdfVoxel>& block = layer.getBlockByIndex(block_idx);

    // iter all voxels inside the block
    for (size_t lin_index = 0u; lin_index < block.num_voxels(); ++lin_index)
    {
      voxblox::TsdfVoxel& tsdf_voxel = block.getVoxelByLinearIndex(lin_index);

      // truncate
      if (std::abs(tsdf_voxel.distance) > truncation_distance)
      {
        tsdf_voxel.distance = (tsdf_voxel.distance > 0) ? truncation_distance : -truncation_distance;
      }
    }
  }
}


std::shared_ptr<voxblox::EsdfMap> MeshToSdf::computeEsdf(std::shared_ptr<voxblox::TsdfMap> tsdf_map)
{
  // setup esdf config from tsdf_map
  voxblox::EsdfMap::Config esdf_config;
  esdf_config.esdf_voxel_size = tsdf_map->getTsdfLayerPtr()->voxel_size();
  esdf_config.esdf_voxels_per_side = tsdf_map->getTsdfLayerPtr()->voxels_per_side();

  // setup esdf_integrator config
  voxblox::EsdfIntegrator::Config esdf_integrator_config;
  esdf_integrator_config.min_distance_m = esdf_config.esdf_voxel_size;
  esdf_integrator_config.max_distance_m = static_cast<voxblox::FloatingPoint>(10.0);
  esdf_integrator_config.default_distance_m = esdf_integrator_config.max_distance_m;

  // create ESDF
  std::shared_ptr<voxblox::EsdfMap> esdf_map = std::make_shared<voxblox::EsdfMap>(esdf_config);
  std::shared_ptr<voxblox::EsdfIntegrator> esdf_integrator =
    std::make_shared<voxblox::EsdfIntegrator>(esdf_integrator_config, tsdf_map->getTsdfLayerPtr(),
                                              esdf_map->getEsdfLayerPtr());

  esdf_integrator->updateFromTsdfLayerBatch();

  return esdf_map;
}


bool MeshToSdf::saveTsdf(const std::string& output_file_name, std::shared_ptr<voxblox::TsdfMap> tsdf_map)
{
  return tsdf_map->getTsdfLayer().saveToFile(output_file_name, true);
}


bool MeshToSdf::saveEsdf(const std::string& output_file_name, std::shared_ptr<voxblox::TsdfMap> tsdf_map,
                         std::shared_ptr<voxblox::EsdfMap> esdf_map)
{
  // First save the tsdf layer to the file.
  bool save_tsdf_res = saveTsdf(output_file_name, tsdf_map);

  // Afterwards append the esdf layer.
  return save_tsdf_res &&
         esdf_map->getEsdfLayer().saveToFile(output_file_name, false);
}
} // end namespace mesh_to_sdf