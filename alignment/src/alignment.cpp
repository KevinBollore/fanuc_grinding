/************************************************
Alignment file code
This file which check alignment between
CAO and Num represents one node of the entire
demonstrator
************************************************/

// ROS headers
#include <ros/ros.h>
#include <ros/service.h>
#include <alignment/AlignmentService.h> // Description of the Service we will use

// PCL and VTK headers
#include <pcl/io/ply_io.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/impl/sift_keypoint.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/common/centroid.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>

#include <vtkVersion.h>
#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <pcl/filters/voxel_grid.h>

boost::shared_ptr<ros::NodeHandle> node;

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;

inline double
uniform_deviate (int seed)
{
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
}

inline void
randomPointTriangle (float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3,
                     Eigen::Vector4f& p)
{
  float r1 = static_cast<float> (uniform_deviate (rand ()));
  float r2 = static_cast<float> (uniform_deviate (rand ()));
  float r1sqr = sqrtf (r1);
  float OneMinR1Sqr = (1 - r1sqr);
  float OneMinR2 = (1 - r2);
  a1 *= OneMinR1Sqr;
  a2 *= OneMinR1Sqr;
  a3 *= OneMinR1Sqr;
  b1 *= OneMinR2;
  b2 *= OneMinR2;
  b3 *= OneMinR2;
  c1 = r1sqr * (r2 * c1 + b1) + a1;
  c2 = r1sqr * (r2 * c2 + b2) + a2;
  c3 = r1sqr * (r2 * c3 + b3) + a3;
  p[0] = c1;
  p[1] = c2;
  p[2] = c3;
  p[3] = 0;
}

inline void
randPSurface (vtkPolyData * polydata, std::vector<double> * cumulativeAreas, double totalArea, Eigen::Vector4f& p)
{
  float r = static_cast<float> (uniform_deviate (rand ()) * totalArea);

  std::vector<double>::iterator low = std::lower_bound (cumulativeAreas->begin (), cumulativeAreas->end (), r);
  vtkIdType el = vtkIdType (low - cumulativeAreas->begin ());

  double A[3], B[3], C[3];
  vtkIdType npts = 0;
  vtkIdType *ptIds = NULL;
  polydata->GetCellPoints (el, npts, ptIds);
  polydata->GetPoint (ptIds[0], A);
  polydata->GetPoint (ptIds[1], B);
  polydata->GetPoint (ptIds[2], C);
  randomPointTriangle (float (A[0]), float (A[1]), float (A[2]),
                       float (B[0]), float (B[1]), float (B[2]),
                       float (C[0]), float (C[1]), float (C[2]), p);
}

void
uniform_sampling (vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, pcl::PointCloud<pcl::PointXYZ> & cloud_out)
{
  polydata->BuildCells ();
  vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys ();

  double p1[3], p2[3], p3[3], totalArea = 0;
  std::vector<double> cumulativeAreas (cells->GetNumberOfCells (), 0);
  size_t i = 0;
  vtkIdType npts = 0, *ptIds = NULL;
  for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds); i++)
  {
    polydata->GetPoint (ptIds[0], p1);
    polydata->GetPoint (ptIds[1], p2);
    polydata->GetPoint (ptIds[2], p3);
    totalArea += vtkTriangle::TriangleArea (p1, p2, p3);
    cumulativeAreas[i] = totalArea;
  }

  cloud_out.points.resize (n_samples);
  cloud_out.width = static_cast<pcl::uint32_t> (n_samples);
  cloud_out.height = 1;

  for (i = 0; i < n_samples; i++)
  {
    Eigen::Vector4f p;
    randPSurface(polydata, &cumulativeAreas, totalArea, p);
    cloud_out.points[i].x = p[0];
    cloud_out.points[i].y = p[1];
    cloud_out.points[i].z = p[2];
  }
}

/**
 * This is the service function that is called whenever a request is received
 * @param req[int]
 * @param res[out]
 * @return Alway true
 */
bool align(alignment::AlignmentService::Request &req, alignment::AlignmentService::Response &res)
{
  // Get parameters from the message and print them
  ROS_WARN_STREAM(std::endl << req);

  std::string cad_file = req.CADFileName;
  pcl::PolygonMesh::Ptr cad_mesh(new pcl::PolygonMesh());
  if (pcl::io::loadPolygonFileSTL(cad_file, *cad_mesh) == -1) // load the file
  {
    PCL_ERROR("Couldn't read CAD file");
    res.ReturnStatus = false;
    res.ReturnMessage = "Couldn't read CAD file";
    return true;
  }

  std::string scan_file = req.ScanFileName;
  pcl::PolygonMesh::Ptr scan_mesh(new pcl::PolygonMesh());
  if (pcl::io::loadPolygonFileSTL(scan_file, *scan_mesh) == -1) // load the file
  {
    PCL_ERROR("Couldn't read scan file");
    res.ReturnStatus = false;
    res.ReturnMessage = "Couldn't read scan file";
    return true;
  }

  // Uniform sampling of CAD
  vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New ();
  pcl::io::mesh2vtk (*cad_mesh, polydata1);

  //make sure that the polygons are triangles!
  vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New ();
  triangleFilter->SetInputData (polydata1);
  triangleFilter->Update ();

  vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  triangleMapper->SetInputConnection (triangleFilter->GetOutputPort ());
  triangleMapper->Update();
  polydata1 = triangleMapper->GetInput();

  // Process to the uniform sampling and send the result in cloud_cad
  PointCloudXYZ::Ptr cloud_cad(new PointCloudXYZ());
  uniform_sampling (polydata1, req.NumberUniformSampling, *cloud_cad);

  // Apply a voxel grid on our point clouds
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  double leaf_size = req.LeafSize;
  grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  grid.setInputCloud (cloud_cad);
  PCL_INFO("Filtering cloud_cad\n");
  grid.filter(*cloud_cad);
  PCL_INFO("cloud_cad filtered\n");

  PointCloudXYZ::Ptr cloud_scan(new PointCloudXYZ());
  pcl::fromPCLPointCloud2(scan_mesh->cloud, *cloud_scan);
  grid.setInputCloud (cloud_scan);
  PCL_INFO("Filtering cloud_scan\n");
  grid.filter(*cloud_scan);
  PCL_INFO("cloud_scan filtered\n");

  // Process to a centroid alignment ( CAD -> scan )
  // Compute centroid for cloud_cad
  Eigen::Vector4f cloud_cad_centroid;
  pcl::compute3DCentroid(*cloud_cad, cloud_cad_centroid);

  // Compute centroid for cloud_scan
  Eigen::Vector4f cloud_scan_centroid;
  pcl::compute3DCentroid(*cloud_scan, cloud_scan_centroid);

  PCL_INFO("Centroid: transforming cloud_cad\n");
  Eigen::Affine3d align_centroid_matrix(Eigen::Affine3d::Identity());
  align_centroid_matrix.translation() << (cloud_scan_centroid - cloud_cad_centroid).head<3>().cast<double>();
  pcl::transformPointCloud(*cloud_cad, *cloud_cad, align_centroid_matrix);
  PCL_INFO("Centroid: cloud_cad transformed\n");

  // Iterative Closest Point process
  int iterations_count = 0;
  double max_corr_distance = req.MaxCorrDist;
  while(iterations_count < req.MaxIterationNumber)
  {
    if (iterations_count % 100 == 0 && iterations_count != 0)
    {
      max_corr_distance /= 1.2;
    }

    // Use ICP to align point clouds
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations (req.IterationNumber);
    icp.setInputSource (cloud_cad);
    icp.setInputTarget (cloud_scan);
    icp.setMaxCorrespondenceDistance (max_corr_distance);  // 1/5th of the object size

    icp.align (*cloud_cad);

    if (icp.hasConverged ())
    {
      iterations_count += req.IterationNumber;
      pcl::transformPointCloud (*cloud_cad, *cloud_cad, icp.getFinalTransformation ());
    }
    else
    {
      PCL_ERROR("\nICP has not converged.\n");
      res.ReturnStatus = false;
      res.ReturnMessage = "ICP has not converged, please review your parameters!";
      return true;
    }
  }

  // re-convert point clouds to meshes
  pcl::toPCLPointCloud2(*cloud_cad, cad_mesh->cloud);

  // Publish meshes files with new positions or just a simple publish?

  res.ReturnStatus = true;
  res.ReturnMessage = "Alignment succeeded";
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "alignment");
  node.reset(new ros::NodeHandle);

  // Create service server and wait for incoming requests
  ros::ServiceServer service = node->advertiseService("alignment_service", align);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (node->ok())
  {
  }
  return 0;
}
