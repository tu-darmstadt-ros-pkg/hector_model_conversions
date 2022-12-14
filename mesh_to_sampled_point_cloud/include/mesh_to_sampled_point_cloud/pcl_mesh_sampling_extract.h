
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Katrin Becker
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_TESTS_PCL_MESH_SAMPLING_EXTRACT_H
#define PCL_TESTS_PCL_MESH_SAMPLING_EXTRACT_H


#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>


namespace mesh_to_sampled_point_cloud
{

/**
 * This functionality was mostly taken from here: https://github.com/PointCloudLibrary/pcl/blob/master/tools/mesh_sampling.cpp.
 * The main method is split into several method (e.g. importFile, executeSampling and savePCDFile)
 * and some functionalities were added.
 */
class PCLMeshSamplingExtract
{

public:

  // ===== Constructor and public methods extracted from pcl_mesh_sampling main method ===== //

  /**
   *
   * @param num_sample_points Number points to be sampled during point cloud sampling.
   * @param voxel_filter_leaf_size Leaf size for filtering point cloud using a voxel grid filter.
   * @param transform Translation, rotation and scaling applied.
   * @param write_normals Whether normals should be computed and written when saving the file.
   * @param write_colors Whether colors should be converted and written when saving the file.
   */
  PCLMeshSamplingExtract(int num_sample_points, float voxel_filter_leaf_size, vtkSmartPointer<vtkTransform> transform,
                         bool write_normals, bool write_colors);

  /**
   * Import ply or obj file into polydata_.
   * @param input_file_name
   * @return empty string if no errors occurred, error message otherwise
   */
  std::string importFile(const std::string& input_file_name);

  /**
   * Scale, transform, filter polydata_, sample point_cloud_, filter point_cloud_ into filtered_point_cloud_ using a voxel grid filter.
   * @return false if something went wrong
   */
  bool executeSampling();

  /**
   * Save filtered_point_cloud_ to a pcd file. Uses write_normals_ and write_colors_ in order to select the data that should be written.
   * @param output_file_name
   * @return empty string if no errors occurred, error message otherwise
   */
  std::string savePCDFile(const std::string& output_file_name);


  /**
   * Get the sampled point cloud with data according to write_normals_ and write_colors_, converted to a PCLPointCloud2.
   * @return sampled point cloud
   */
  std::shared_ptr<pcl::PCLPointCloud2> getSampledPointCloud();

  /**
   * Get the sampled point cloud as pcl point cloud with PointXYZ as point type.
   * @return sampled point cloud
   */
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getSampledPointCloudXYZ();

  /**
   * Get the sampled point cloud as pcl point cloud with PointNormal as point type.
   * @throws runtime_error if parameter write_normals_ == false
   * @return sampled point cloud
   */
  std::shared_ptr<pcl::PointCloud<pcl::PointNormal>> getSampledPointCloudXYZN();


private:

  // ===== pcl_mesh_sampling methods ===== //
  double uniform_deviate(int seed);

  void randomPointTriangle(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3,
                           float r1, float r2, Eigen::Vector3f& p);

  void randPSurface(vtkPolyData* polydata, std::vector<double>* cumulativeAreas, double totalArea, Eigen::Vector3f& p,
                    bool calcNormal, Eigen::Vector3f& n, bool calcColor, Eigen::Vector3f& c);

  void uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, std::size_t n_samples, bool calc_normal,
                        bool calc_color, pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_out);


  // ===== new transform method ===== //
  void scaleAndTransform(vtkSmartPointer<vtkTransform> transform);

  // ===== private methods extracted from pcl_mesh_sampling main method ===== //
  void filterTriangles();

  void filterPointCloud();

  // data structures
  vtkSmartPointer<vtkPolyData> polydata_;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr point_cloud_;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filtered_point_cloud_;

  // parameters
  bool write_normals_;
  bool write_colors_;

  int num_sample_points_ = 100000;
  float voxel_filter_leaf_size_ = 0.01;

  vtkSmartPointer<vtkTransform> transform_;
};
} // end namespace mesh_to_sampled_point_cloud

#endif //PCL_TESTS_PCL_MESH_SAMPLING_EXTRACT_H
