#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/filters/passthrough.h>
#include <math.h>

using namespace pcl;
using namespace std;
//home/xtong/Documents/workplace/py/exercise-2/
int
  main (int argc, char** argv)
{
   PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
   if(io::loadPLYFile<PointXYZ> ("/home/xtong/Documents/workplace/py/exercise-2/out.ply", *cloud) == -1){
      cout << "fail" << endl;

   } else {

      cout << "loaded" << endl;

      cout << "begin passthrough filter" << endl;
      PointCloud<PointXYZ>::Ptr filtered(new PointCloud<PointXYZ>());
      PassThrough<PointXYZ> filter;
      filter.setInputCloud(cloud);
      filter.filter(*filtered);
      cout << "passthrough filter complete" << endl;

      // cout << "begin moving least squares" << endl;
      // MovingLeastSquares<PointXYZ, PointXYZ> mls;
      // mls.setInputCloud(filtered);
      // mls.setSearchRadius(0.01);
      // mls.setPolynomialFit(true);
      // mls.setPolynomialOrder(2);
      // mls.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
      // mls.setUpsamplingRadius(0.005);
      // mls.setUpsamplingStepSize(0.003);

      // PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ>());
      // mls.process(*cloud_smoothed);
      // cout << "MLS complete" << endl;

      cout << "begin normal estimation" << endl;
      NormalEstimationOMP<PointXYZ, Normal> ne;
      ne.setNumberOfThreads(8);
      ne.setInputCloud(filtered);
      ne.setRadiusSearch(0.01);
      Eigen::Vector4f centroid;
      compute3DCentroid(*filtered, centroid);
      ne.setViewPoint(centroid[0], centroid[1], centroid[2]);

      PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
      ne.compute(*cloud_normals);
      cout << "normal estimation complete" << endl;
      cout << "reverse normals' direction" << endl;

      for(size_t i = 0; i < cloud_normals->size(); ++i){
      	cloud_normals->points[i].normal_x *= -1;
      	cloud_normals->points[i].normal_y *= -1;
      	cloud_normals->points[i].normal_z *= -1;
      }

      cout << "combine points and normals" << endl;
      PointCloud<PointNormal>::Ptr cloud_smoothed_normals(new PointCloud<PointNormal>());
      concatenateFields(*filtered, *cloud_normals, *cloud_smoothed_normals);

      cout << "begin poisson reconstruction" << endl;
      Poisson<PointNormal> poisson;
      poisson.setDepth(8);
      poisson.setSolverDivide(8);
      poisson.setInputCloud(cloud_smoothed_normals);
      poisson.setIsoDivide(8);
      poisson.setSamplesPerNode(15);
      //poisson.setDegree(2);
      PolygonMesh mesh;
      poisson.reconstruct(mesh);
      cout << "OK" << endl;
      //if(cloud->size()>0)
      io::savePLYFile("newcat1.ply", mesh);
      
      /*
      boost::shared_ptr<pcl::PolygonMesh>triangles (new pcl::PolygonMesh);
      pcl::GreedyProjectionTriangulation<pcl::PointNormal>gt;
      gt.setInputCloud(cloud_smoothed_normals);
      gt.setSearchMethod(tree2);
      gt.setSearchRadius(R);
      gt.setMu(mu);
      gt.setMaximumNearestNeighbors(D);
      gt.setMaximumSurfaceAngle(M_PI/4);
      gt.setMinimumAngle(M_PI/18);
      gt.setMaximumAngle(2*M_PI/3);
      gt.setNormalConsistency(false);
      gt.reconstruct(*triangles);
      */
   }
  return (0);
}
