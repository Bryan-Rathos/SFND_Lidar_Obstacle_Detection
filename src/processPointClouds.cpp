// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
// #include "processPointCloudsHelper.cpp"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//destructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::insertCloudKDTree(typename pcl::PointCloud<PointT>::Ptr cloud, std::unique_ptr<KdTree>& tree)
{
	// std::vector<std::vector<float>> tempPoints = points;
	auto tempCloud = cloud->makeShared();

	int cloud_size = cloud->points.size();
	cloud->clear();

    for(int dim=0; dim<cloud_size; dim++)
	{
		// std::vector<float> medianPt;
		PointT medianPt;

		if(dim%3 == 0)
		{
			std::sort(tempCloud->points.begin(), tempCloud->points.end(), [](PointT p1, PointT p2) -> bool{ return p1.x < p2.x; });
			
			// for(const auto& point : tempCloud->points){
			// 	std::cout << point.x << ", "<< point.y << ", " << point.z <<std::endl;
			// }
			
			int size = tempCloud->points.size();
			
			if(size%2 == 0)
			{
				auto a = tempCloud->points[(size-1)/2].x;
				auto b = tempCloud->points[size/2].x;
				medianPt = a < b ? tempCloud->points[(size-1)/2] : tempCloud->points[size/2];
				cloud->push_back(medianPt);
				// std::cout << dim << ": "<< medianPt.x << ", " << medianPt.y << ", " << medianPt.z << std::endl;

				if(a<b)
					tempCloud->erase(tempCloud->points.begin()+((size-1)/2));	
				else
					tempCloud->erase(tempCloud->points.begin()+(size/2));
			}
			else
			{
				medianPt = tempCloud->points[size/2];
				cloud->push_back(medianPt);
				// std::cout << dim << ": "<< medianPt.x << ", " << medianPt.y << ", " << medianPt.z << std::endl;

				tempCloud->erase(tempCloud->points.begin()+(size/2));
			}
		}

		if(dim%3 == 1)
		{
			std::sort(tempCloud->points.begin(), tempCloud->points.end(), [](PointT p1, PointT p2) -> bool{ return p1.y < p2.y; });

			int size = tempCloud->points.size();
			if(size%2 == 0)
			{
				auto a = tempCloud->points[(size-1)/2].y;
				auto b = tempCloud->points[size/2].y;
				medianPt = a < b ? tempCloud->points[(size-1)/2] : tempCloud->points[size/2];
				cloud->push_back(medianPt);
				// std::cout << dim << ": " << medianPt.x << ", " << medianPt.y << ", " << medianPt.z << std::endl;

				if(a<b)
					tempCloud->erase(tempCloud->points.begin()+((size-1)/2));	
				else
					tempCloud->erase(tempCloud->points.begin()+(size/2));
			}
			else
			{
				medianPt = tempCloud->points[size/2];
				cloud->push_back(medianPt);
				// std::cout << dim << ": " << medianPt.x << ", " << medianPt.y << ", " << medianPt.z << std::endl;
				tempCloud->erase(tempCloud->points.begin()+(size/2));
			}
		}

		if(dim%3 == 2)
		{
			std::sort(tempCloud->points.begin(), tempCloud->points.end(), [](PointT p1, PointT p2) -> bool{ return p1.z < p2.z; });
			int size = tempCloud->points.size();
			if(size%2 == 0)
			{
				auto a = tempCloud->points[(size-1)/2].z;
				auto b = tempCloud->points[size/2].z;
				medianPt = a < b ? tempCloud->points[(size-1)/2] : tempCloud->points[size/2];
				cloud->push_back(medianPt);
				// std::cout << dim << ": " << medianPt.x << ", " << medianPt.y << ", " << medianPt.z << std::endl;
				
				if(a<b)
					tempCloud->erase(tempCloud->points.begin()+((size-1)/2));	
				else
					tempCloud->erase(tempCloud->points.begin()+(size/2));
			}
			else
			{
				medianPt = tempCloud->points[size/2];
				cloud->push_back(medianPt);
				// std::cout << dim << ": " << medianPt.x << ", " << medianPt.y << ", " << medianPt.z << std::endl;
				tempCloud->erase(tempCloud->points.begin()+(size/2));
			}
		}

		std::vector<float> pointIn = {medianPt.x, medianPt.y,medianPt.z}; 

		tree->insert(pointIn,dim);
	}
}


template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(uint index, typename pcl::PointCloud<PointT>::Ptr cloud, typename pcl::PointCloud<PointT>::Ptr newCluster,
			   							   std::vector<bool>& processed, std::unique_ptr<KdTree>& tree, const float distanceTol)
{
	processed[index] = true;
	newCluster->push_back(cloud->points[index]);
	std::vector<float> searchPt = {cloud->points[index].x, cloud->points[index].y, cloud->points[index].z};
	std::vector<int> nearbyID = tree->search(searchPt, distanceTol);

	for(auto& id : nearbyID)
	{
		if(processed[id] != true)
		{
			Proximity(id, cloud, newCluster, processed, tree, distanceTol);
		}
	}
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::EuclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, std::unique_ptr<KdTree>& tree, const float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster
	// std::cout << "In EuclideanCluster cloud size: " << cloud->points.size() << std::endl;

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

	std::vector<bool> processed(cloud->points.size(), false);
	
	for(uint idx = 0; idx < cloud->points.size(); idx++)
	{
		if(processed[idx] != true)
		{
			typename pcl::PointCloud<PointT>::Ptr newCluster (new pcl::PointCloud<PointT>);
			Proximity(idx, cloud, newCluster, processed, tree, distanceTol);
			clusters.push_back(newCluster);
		}
	}

	return clusters;
}


template <typename PointT>
template <typename U>
std::vector<U> ProcessPointClouds<PointT>::CrossProduct(std::vector<U>& v1, std::vector<U>& v2)
{
    std::vector<U> res;
    auto x = v1[1]*v2[2] - v1[2]*v2[1];
    auto y = -(v1[0]*v2[2] - v1[2]*v2[0]);
    auto z = v1[0]*v2[1] - v1[1]*v2[0];
    res.push_back(x);
    res.push_back(y);
    res.push_back(z);
    
    return res;
}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr 
ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);

    // Create the filtering object
    pcl::VoxelGrid<PointT> vgf;
    vgf.setInputCloud(cloud);
    vgf.setLeafSize(filterRes, filterRes, filterRes);
    vgf.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr regionOfInterest (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*regionOfInterest);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.5,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
    roof.setInputCloud(regionOfInterest);
    roof.filter(indices);
    
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for(int& point : indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(regionOfInterest);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*regionOfInterest);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return regionOfInterest;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>);

    for(auto index : inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    // TODO:: Fill in this function to find inliers for the cloud.

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());

    seg.setOptimizeCoefficients(true); //optional
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	// Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations
	while(maxIterations--)
	{	
		// Randomly pick 3 points from the 3D point cloud
		// We use unordered set to select three points since this will avoid choosing repaeated points
		// We check the set size in the while loop to check once the 3 point selection is done
		std::unordered_set<int> inliers;
		
		while(inliers.size() < 3)
			inliers.insert(rand()%(cloud->points.size()));		

		// Extract (x,y) coordinates of the two points
		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		std::vector<float> v1 {x2-x1, y2-y1, z2-z1};
		std::vector<float> v2 {x3-x1, y3-y1, z3-z1};

		auto normal = CrossProduct(v1,v2);

		float a = normal[0];
		float b = normal[1];
		float c = normal[2];
		float d = -(a*x1 + b*y1 + c*z1);

		// Randomly sample subset and fit line
		for(int index = 0; index < cloud->points.size(); index++)
		{
			if(inliers.count(index)>0)
				continue;
			
			auto point = cloud->points[index];

			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			// Measure distance between every point and fitted plane
			float distance = fabs(a*x4 + b*y4 + c*z4 + d)/sqrt(a*a + b*b + c*c);

			// If distance is smaller than threshold count it as inlier
			if(distance < distanceTol)
				inliers.insert(index);
		}

		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
	}

	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "ransacPlane took " << elapsedTime.count() << " milliseconds" << std::endl;
	
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new typename pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new typename pcl::PointCloud<PointT>);

    for(int index = 0; index < cloud->points.size(); index++)
	{
		auto point = cloud->points[index];
		if(inliersResult.count(index))
			planeCloud->points.push_back(point);
		else
			obstCloud->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        
        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::CustomClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    auto tree = std::make_unique<KdTree>();

	// std::cout << "CustomClustering CS before insert KD: " << cloud->points.size() << std::endl;

    insertCloudKDTree(cloud, tree);

	// std::cout << "CustomClustering CS after insert KD: " << cloud->points.size() << std::endl;

    //Time clustering process
    auto startTime = std::chrono::steady_clock::now();

	std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters = EuclideanCluster(cloud, tree, clusterTolerance);

	// std::cout << "cloudClusters size: " << cloudClusters.size() << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << cloudClusters.size() << " clusters" << std::endl;

	std::vector<typename pcl::PointCloud<PointT>::Ptr> filteredClusters;

	for(typename pcl::PointCloud<PointT>::Ptr cluster : cloudClusters)
    {
        if( (cluster->points.size() > minSize) && (cluster->points.size() < maxSize) )
		{
			filteredClusters.push_back(cluster);
		}
    }
	std::cout << "Clusters after filtering: " << filteredClusters.size() << std::endl;

    return filteredClusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}