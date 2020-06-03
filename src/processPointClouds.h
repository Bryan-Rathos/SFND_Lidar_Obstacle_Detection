// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include <memory>
#include "render/box.h"
#include "3dkdtree.h"

template<typename PointT>
class ProcessPointClouds 
{
    // these protected member functions are helpers for the public member functions for this class which can be accessed only by this class itself
    protected:
        void insertCloudKDTree(typename pcl::PointCloud<PointT>::Ptr cloud, std::unique_ptr<KdTree>& tree);

        void Proximity(uint index, typename pcl::PointCloud<PointT>::Ptr cloud, typename pcl::PointCloud<PointT>::Ptr newCluster, std::vector<bool>& processed, std::unique_ptr<KdTree>& tree, const float distanceTol);

        std::vector<typename pcl::PointCloud<PointT>::Ptr> EuclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, std::unique_ptr<KdTree>& tree, const float distanceTol);
        
        template <typename U>
        std::vector<U> CrossProduct(std::vector<U>& v1, std::vector<U>& v2);

        
    public:

        //constructor
        ProcessPointClouds();
        //destructor
        ~ProcessPointClouds();

        void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

        typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

        std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);
        
        std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

        std::vector<typename pcl::PointCloud<PointT>::Ptr> CustomClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

        Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

        void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

        typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

        std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};
#endif /* PROCESSPOINTCLOUDS_H_ */