

#include "mesh_to_sdf/mesh_to_sdf.h"

namespace mesh_to_sdf
{

MeshToSdf::MeshToSdf(const std::string& input_file, geometry_msgs::Transform transform, float scale_factor,
                     bool fill_inside, bool floodfill_unoccupied, float voxel_size, float truncation_distance)
{
  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZ> point_cloud;

  // load file
  if (!loadPlyFile(input_file, mesh, point_cloud))
  {
    throw std::invalid_argument("Error while loading ply file. Please see PCL error for more information.");
  }

  // compute TSDF
  computeTsdf(convertTransform(transform), scale_factor, fill_inside, floodfill_unoccupied, voxel_size,
              mesh, point_cloud);

  // enforce truncation of tsdf_map (if truncation distance is 0 no truncation will be done)
  truncateTSDF(truncation_distance);
}


std::shared_ptr<voxblox::TsdfMap> MeshToSdf::getTsdfMap()
{
  return tsdf_map_;
}


std::shared_ptr<voxblox::EsdfMap> MeshToSdf::getEsdfMap()
{
  if (!esdf_map_)
  {
    // compute ESDF from TSDF
    computeEsdf();
  }

  return esdf_map_;
}


void MeshToSdf::saveTsdf(const std::string& output_file_name)
{
  if (!tsdf_map_->getTsdfLayer().saveToFile(output_file_name, true))
  {
    throw std::invalid_argument("Error while saving tsdf file. Please see voxblox error for more information.");
  }
}


void MeshToSdf::saveEsdf(const std::string& output_file_name, bool with_tsdf)
{
  if(with_tsdf)
  {
    // First save the tsdf layer to the file (required for voxblox::EsdfServer::loadMap method).
    if (!tsdf_map_->getTsdfLayer().saveToFile(output_file_name, true))
    {
      throw std::invalid_argument(
        "Error while saving the tsdf part of the esdf file. Please see voxblox error for more information.");
    }
  }

  // Afterwards append the esdf layer.
  if (!esdf_map_)
  {
    // compute ESDF from TSDF
    computeEsdf();
  }

  // clear file if not with tsdf
  if (!esdf_map_->getEsdfLayer().saveToFile(output_file_name, !with_tsdf))
  {
    throw std::invalid_argument(
      "Error while saving the esdf part of the esdf file. Please see voxblox error for more information.");
  }
}


bool MeshToSdf::loadPlyFile(const std::string& input_file_name, pcl::PolygonMesh& mesh,
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


void MeshToSdf::computeTsdf(voxblox::Transformation transform, float scale_factor,
                            bool fill_inside, bool floodfill_unoccupied, float voxel_size,
                            pcl::PolygonMesh& mesh, pcl::PointCloud<pcl::PointXYZ>& vertex_coordinates)
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
    if(polygon.vertices.size() != 3)
    {
      throw std::runtime_error("Error when converting mesh to sdf: Mesh has non-triangular polygons, which is not supported!");
    }

    // Indicate progress
    triangle_i++;
    // Only print progress for each promile of completion, to reduce IO wait
    // !!! ONLY PRINT IF THERE ARE MORE THAN 1000 TRIANGLES!
    // (otherwise num_triangles/1000 is 0 and it results in an Arithmetic exception)
    if(num_triangles / 1000 != 0)
    {
      if (triangle_i % (num_triangles / 1000) == 0)
      {
        printf("\rProgress: %3.1f%% - total nr of blocks %lu", triangle_i / static_cast<double>(num_triangles) * 100,
               sdf_creator.getNumberOfAllocatedBlocks());
        std::cout << std::flush;
      }
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

  tsdf_map_ = std::make_shared<voxblox::TsdfMap>(sdf_creator.getTsdfMap());
}


void MeshToSdf::truncateTSDF(float truncation_distance)
{
  // do nothing if truncation distance is zero
  if (truncation_distance == 0)
  {
    return;
  }

  // get all allocated blocks
  voxblox::BlockIndexList block_idx_list;
  tsdf_map_->getTsdfLayer().getAllAllocatedBlocks(&block_idx_list);

  // iter blocks
  for (Eigen::Matrix<voxblox::IndexElement, 3, 1>& block_idx : block_idx_list)
  {
    voxblox::Layer<voxblox::TsdfVoxel>& layer = *(tsdf_map_->getTsdfLayerPtr());
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


void MeshToSdf::computeEsdf()
{
  // setup esdf config from tsdf_map
  voxblox::EsdfMap::Config esdf_config;
  esdf_config.esdf_voxel_size = tsdf_map_->getTsdfLayerPtr()->voxel_size();
  esdf_config.esdf_voxels_per_side = tsdf_map_->getTsdfLayerPtr()->voxels_per_side();

  // setup esdf_integrator config
  voxblox::EsdfIntegrator::Config esdf_integrator_config;
  esdf_integrator_config.min_distance_m = esdf_config.esdf_voxel_size;
  esdf_integrator_config.max_distance_m = static_cast<voxblox::FloatingPoint>(10.0);
  esdf_integrator_config.default_distance_m = esdf_integrator_config.max_distance_m;

  // create ESDF
  esdf_map_ = std::make_shared<voxblox::EsdfMap>(esdf_config);

  std::shared_ptr<voxblox::EsdfIntegrator> esdf_integrator =
    std::make_shared<voxblox::EsdfIntegrator>(esdf_integrator_config, tsdf_map_->getTsdfLayerPtr(),
                                              esdf_map_->getEsdfLayerPtr());

  esdf_integrator->updateFromTsdfLayerBatch();
}
} // end namespace mesh_to_sdf