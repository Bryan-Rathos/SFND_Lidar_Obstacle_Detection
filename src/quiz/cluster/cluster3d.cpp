/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include "../../render/box.h"
#include <chrono>
#include <string>
#include "3dkdtree.h"
// #include "3dkd.h"

// Arguments:
// window is the region to draw box around
// increase zoom to see more of the area
pcl::visualization::PCLVisualizer::Ptr initScene(Box window, int zoom)
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, zoom, 0, 1, 0);
  	viewer->addCoordinateSystem(1.0);

  	viewer->addCube(window.x_min, window.x_max, window.y_min, window.y_max, window.z_min, window.z_max, 1, 1, 1, "window");
  	return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZ point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = points[i][2];

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;
}


void render3DTree(Node* node, pcl::visualization::PCLVisualizer::Ptr& viewer, Box window, int& iteration, uint depth=0)
{

	if(node!=NULL)
	{
		Box upperWindow = window;
		Box lowerWindow = window;

		// split on x axis
		if(depth%3 == 0)
		{
			viewer->addLine(pcl::PointXYZ(node->point[0], window.y_min, 0),pcl::PointXYZ(node->point[0], window.y_max, 0),1,0,0,"line"+std::to_string(iteration));
			lowerWindow.x_max = node->point[0];
			upperWindow.x_min = node->point[0];
		}
		// split on y axis
		if(depth%3 == 1)
		{
			viewer->addLine(pcl::PointXYZ(window.x_min, node->point[1], 0),pcl::PointXYZ(window.x_max, node->point[1], 0),0,1,0,"line"+std::to_string(iteration));
			lowerWindow.y_max = node->point[1];
			upperWindow.y_min = node->point[1];
		}
		//split on z axis
		if(depth%3 == 2)
		{
			// viewer->addLine(pcl::PointXYZ(window.x_min, node->point[2], 0),pcl::PointXYZ(window.x_max, node->point[2], 0),0,0,1,"line"+std::to_string(iteration));
			viewer->addLine(pcl::PointXYZ(window.x_min, 0, node->point[2]),pcl::PointXYZ(window.x_max, 0, node->point[2]),0,0,1,"line"+std::to_string(iteration));
			lowerWindow.z_max = node->point[2];
			upperWindow.z_min = node->point[2];
		}
		
		iteration++;

		render3DTree(node->left,viewer, lowerWindow, iteration, depth+1);
		render3DTree(node->right,viewer, upperWindow, iteration, depth+1);
	}
}

// void Proximity(uint index, const std::vector<std::vector<float>>& points, std::vector<int>& newCluster, std::vector<bool>& processed, std::unique_ptr<KdTree>& tree, const float &distanceTol)
// {
// 	processed[index] = true;
// 	newCluster.push_back(index);
// 	std::vector<int> nearbyID = tree->search(points[index], distanceTol);

// 	for(auto& id : nearbyID)
// 	{
// 		if(processed[id] != true)
// 		{
// 			Proximity(id, points, newCluster, processed, tree, distanceTol);
// 		}
// 	}
// }

// std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, std::unique_ptr<KdTree>& tree, const float &distanceTol)
// {
// 	// TODO: Fill out this function to return list of indices for each cluster
// 	std::vector<std::vector<int>> clusters;

// 	std::vector<bool> processed(points.size(), false);
	
// 	for(uint idx = 0; idx < points.size(); idx++) //auto& row : points
// 	{
// 		if(processed[idx] != true)
// 		{
// 			std::vector<int> newCluster;
// 			Proximity(idx, points, newCluster, processed, tree, distanceTol);
// 			clusters.push_back(newCluster);
// 		}
// 	}
	
// 	return clusters;
// }


void Proximity(uint index, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr newCluster,
			   std::vector<bool>& processed, std::unique_ptr<KdTree>& tree, const float &distanceTol)
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


// std::vector<std::vector<int>>
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> euclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
																  std::unique_ptr<KdTree>& tree,
																  const float &distanceTol)
{
	// TODO: Fill out this function to return list of indices for each cluster

	// std::vector<std::vector<int>> clusters;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

	std::vector<bool> processed(cloud->points.size(), false);
	
	for(uint idx = 0; idx < cloud->points.size(); idx++) //auto& row : points
	{
		if(processed[idx] != true)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr newCluster (new pcl::PointCloud<pcl::PointXYZ>);
			Proximity(idx, cloud, newCluster, processed, tree, distanceTol);
			clusters.push_back(newCluster);
		}
	}
	// std::cout << processed.size() <<std::endl;
	for(auto i : processed){
		std::cout << i << ", " <<std::endl;
	}

	return clusters;
}



// Points are inserted by sorting the points on x or y coordinates
// Then the median point is chosen based on x or y coordinates in the previous step
// This median point is inserted into the tree, which helps keeping the KD-tree balanced
// void insertPointsKDTree(std::vector<std::vector<float>> &points, std::unique_ptr<KdTree>& tree)
// {
// 	std::vector<std::vector<float>> tempPoints = points;

// 	points.clear();

// 	int cloud_size = tempPoints.size();

//     for (int dim=0; dim<cloud_size; dim++)
// 	{
// 		std::vector<float> medianPt;

// 		if(dim%3 == 0)
// 		{
// 			std::sort(tempPoints.begin(), tempPoints.end(), [](const std::vector<float>& v1, const std::vector<float>& v2) -> bool{ return v1[0] < v2[0];});
// 			int size = tempPoints.size();
// 			if(size%2 == 0)
// 			{
// 				float a = tempPoints[(size-1)/2][0];
// 				float b = tempPoints[size/2][0];
// 				medianPt = a < b ? tempPoints[(size-1)/2] : tempPoints[size/2];
// 				points.push_back(medianPt);
// 				std::cout << dim << ": "<<medianPt[0] << ", " << medianPt[1] << ", " << medianPt[2] << std::endl;

// 				if(a<b)
// 					tempPoints.erase(tempPoints.begin()+((size-1)/2));	
// 				else
// 					tempPoints.erase(tempPoints.begin()+(size/2));
// 			}
// 			else
// 			{
// 				medianPt = tempPoints[size/2];
// 				points.push_back(medianPt);
// 				std::cout << dim << ": "<<medianPt[0] << ", " << medianPt[1] << ", " << medianPt[2] << std::endl;

// 				tempPoints.erase(tempPoints.begin()+(size/2));
// 			}
// 		}
	
// 		if(dim%3 == 1)
// 		{
// 			std::sort(tempPoints.begin(), tempPoints.end(), [](const std::vector<float>& v1, const std::vector<float>& v2) -> bool{ return v1[1] < v2[1];});
// 			int size = tempPoints.size();
// 			if(size%2 == 0)
// 			{
// 				float a = tempPoints[(size-1)/2][1];
// 				float b = tempPoints[size/2][1];
// 				medianPt = a < b ? tempPoints[(size-1)/2] : tempPoints[size/2];
// 				points.push_back(medianPt);
// 				std::cout << dim << ": "<<medianPt[0] << ", " << medianPt[1] << ", " << medianPt[2] << std::endl;

// 				if(a<b)
// 					tempPoints.erase(tempPoints.begin()+((size-1)/2));	
// 				else
// 					tempPoints.erase(tempPoints.begin()+(size/2));
// 			}
// 			else
// 			{
// 				medianPt = tempPoints[size/2];
// 				points.push_back(medianPt);
// 				std::cout << dim << ": "<<medianPt[0] << ", " << medianPt[1] << ", " << medianPt[2] << std::endl;
// 				tempPoints.erase(tempPoints.begin()+(size/2));
// 			}
// 		}

// 		if(dim%3 == 2)
// 		{
// 			std::sort(tempPoints.begin(), tempPoints.end(), [](const std::vector<float>& v1, const std::vector<float>& v2) -> bool{ return v1[2] < v2[2];});
// 			int size = tempPoints.size();
// 			if(size%2 == 0)
// 			{
// 				float a = tempPoints[(size-1)/2][2];
// 				float b = tempPoints[size/2][2];
// 				medianPt = a < b ? tempPoints[(size-1)/2] : tempPoints[size/2];
// 				points.push_back(medianPt);
// 				std::cout << dim << ": "<<medianPt[0] << ", " << medianPt[1] << ", " << medianPt[2] << std::endl;
// 				if(a<b)
// 					tempPoints.erase(tempPoints.begin()+((size-1)/2));	
// 				else
// 					tempPoints.erase(tempPoints.begin()+(size/2));
// 			}
// 			else
// 			{
// 				medianPt = tempPoints[size/2];
// 				points.push_back(medianPt);
// 				std::cout << dim << ": "<<medianPt[0] << ", " << medianPt[1] << ", " << medianPt[2] << std::endl;
// 				tempPoints.erase(tempPoints.begin()+(size/2));
// 			}
// 		}

// 		tree->insert(medianPt,dim);
// 		// tree->insert(points[dim],dim);
// 	}
// }



void insertCloudKDTree(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::unique_ptr<KdTree>& tree)
{
	// std::vector<std::vector<float>> tempPoints = points;
	auto tempCloud = cloud->makeShared();

	std::cout << "Tempcloud "<< tempCloud->points.size() << std::endl;

	int cloud_size = cloud->points.size();
	cloud->clear();

	std::cout << "Tempcloud after cloud clear "<< tempCloud->points.size() << std::endl;

    for(int dim=0; dim<cloud_size; dim++)
	{
		// std::vector<float> medianPt;
		pcl::PointXYZ medianPt;

		if(dim%3 == 0)
		{
			std::sort(tempCloud->points.begin(), tempCloud->points.end(), [](pcl::PointXYZ p1, pcl::PointXYZ p2) -> bool{ return p1.x < p2.x;});
			
			// for(const auto& point : tempCloud->points){
			// 	std::cout << point.x << ", "<< point.y << ", " << point.z <<std::endl;
			// }
			
			int size = tempCloud->points.size();
			// std::cout << tempCloud->points[(size-1)/2].x << std::endl;
			
			if(size%2 == 0)
			{
				auto a = tempCloud->points[(size-1)/2].x;
				auto b = tempCloud->points[size/2].x;
				medianPt = a < b ? tempCloud->points[(size-1)/2] : tempCloud->points[size/2];
				cloud->push_back(medianPt);
				std::cout << dim << ": "<< medianPt.x << ", " << medianPt.y << ", " << medianPt.z << std::endl;

				if(a<b)
					tempCloud->erase(tempCloud->points.begin()+((size-1)/2));	
				else
					tempCloud->erase(tempCloud->points.begin()+(size/2));
			}
			else
			{
				medianPt = tempCloud->points[size/2];
				cloud->push_back(medianPt);
				std::cout << dim << ": "<< medianPt.x << ", " << medianPt.y << ", " << medianPt.z << std::endl;

				tempCloud->erase(tempCloud->points.begin()+(size/2));
			}
		}

		if(dim%3 == 1)
		{
			std::sort(tempCloud->points.begin(), tempCloud->points.end(), [](pcl::PointXYZ p1, pcl::PointXYZ p2) -> bool{ return p1.y < p2.y;});

			int size = tempCloud->points.size();
			if(size%2 == 0)
			{
				auto a = tempCloud->points[(size-1)/2].y;
				auto b = tempCloud->points[size/2].y;
				medianPt = a < b ? tempCloud->points[(size-1)/2] : tempCloud->points[size/2];
				cloud->push_back(medianPt);
				std::cout << dim << ": " << medianPt.x << ", " << medianPt.y << ", " << medianPt.z << std::endl;

				if(a<b)
					tempCloud->erase(tempCloud->points.begin()+((size-1)/2));	
				else
					tempCloud->erase(tempCloud->points.begin()+(size/2));
			}
			else
			{
				medianPt = tempCloud->points[size/2];
				cloud->push_back(medianPt);
				std::cout << dim << ": " << medianPt.x << ", " << medianPt.y << ", " << medianPt.z << std::endl;
				tempCloud->erase(tempCloud->points.begin()+(size/2));
			}
		}

		if(dim%3 == 2)
		{
			std::sort(tempCloud->points.begin(), tempCloud->points.end(), [](pcl::PointXYZ p1, pcl::PointXYZ p2) -> bool{ return p1.z < p2.z;});
			int size = tempCloud->points.size();
			if(size%2 == 0)
			{
				auto a = tempCloud->points[(size-1)/2].z;
				auto b = tempCloud->points[size/2].z;
				medianPt = a < b ? tempCloud->points[(size-1)/2] : tempCloud->points[size/2];
				cloud->push_back(medianPt);
				std::cout << dim << ": " << medianPt.x << ", " << medianPt.y << ", " << medianPt.z << std::endl;
				
				if(a<b)
					tempCloud->erase(tempCloud->points.begin()+((size-1)/2));	
				else
					tempCloud->erase(tempCloud->points.begin()+(size/2));
			}
			else
			{
				medianPt = tempCloud->points[size/2];
				cloud->push_back(medianPt);
				std::cout << dim << ": " << medianPt.x << ", " << medianPt.y << ", " << medianPt.z << std::endl;
				tempCloud->erase(tempCloud->points.begin()+(size/2));
			}
		}

		std::vector<float> pointIn = {medianPt.x, medianPt.y,medianPt.z}; 

		tree->insert(pointIn,dim);
	}
}


void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main ()
{
	// Create viewer
	Box window;
  	window.x_min =  0;
  	window.x_max = 10;
  	window.y_min =  0;
  	window.y_max = 10;
  	window.z_min =  0;
  	window.z_max = 10;
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene(window, 25);
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

	// Create data
	std::vector<std::vector<float>> points = {{2,3,3}, {5,4,2}, {9,6,7},
											  {4,7,9}, {8,1,5}, {7,2,6},
											  {9,4,1}, {8,4,2}, {9,7,8},
											  {6,3,1}, {3,4,5}, {1,6,8},
											  {9,5,3}, {2,1,3}, {8,7,6}};

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(points);

	std::cout << cloud->points.size() << std::endl;

	// std::vector<std::vector<float>> recover;

	// for(int nIndex = 0; nIndex < cloud->points.size(); nIndex++)
	// {
	// 	recover.push_back({cloud->points[nIndex].x, cloud->points[nIndex].y, cloud->points[nIndex].z});
	// }

	// for(auto& row : recover)
	// {
	// 	for(auto& i : row)
	// 	{
	// 		std::cout << i << ", "; 
	// 	}
	// 	std::cout << std::endl;
	// }

    // renderPointCloud(viewer, cloud, "test");

	// KdTree* tree = new KdTree;
	auto tree = std::make_unique<KdTree>();

	// Time segmentation process
  	// auto startTime = std::chrono::steady_clock::now();

	// insertPointsKDTree(points, tree);
	insertCloudKDTree(cloud, tree);

	std::cout << cloud->points.size() << std::endl;

	// auto endTime = std::chrono::steady_clock::now();
  	// auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	// std::cout << "KDTree created and took " << elapsedTime.count() << " milliseconds" << std::endl;	
    	 
  	// int it = 0;
  	// render3DTree(tree->root,viewer,window, it);
  
  	std::cout << "Test Search" << std::endl;
  	std::vector<int> nearby = tree->search({7,2,1},3.0);
  	for(int index : nearby)
    	std::cout << index << ",";
  	std::cout << std::endl;

  	// Time segmentation process
  	auto startTime = std::chrono::steady_clock::now();
  	//
  	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = euclideanCluster(cloud, tree, 3.0);
  	//
  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << cloudClusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  	// // // Render clusters
  	// int clusterId = 0;
	// std::vector<Color> colors = {Color(0,0,1), Color(0,1,0), Color(0,1,1),
	// 							 Color(1,0,0), Color(1,0,1), Color(1,1,0)};
  	// for(auto& cluster : clusters)
  	// {
  	// 	pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// 	for(int indice: cluster)
  	// 		clusterCloud->points.push_back(pcl::PointXYZ(points[indice][0],points[indice][1],points[indice][2]));
  	// 	renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%6]);
  	// 	++clusterId;
  	// }
  	// if(clusters.size()==0)
  	// 	renderPointCloud(viewer,cloud,"data");

	int clusterId = 0;
    std::vector<Color> colors = {Color(0,0,1), Color(0,1,0), Color(0,1,1),
								 Color(0.7,0.2,0.4), Color(1,0.5,0.6), Color(0.6,0.3,0.8)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        std::cout << cluster->points.size() << std::endl;
        
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        ++clusterId; 

        // Box box = pointProcessor->BoundingBox(cluster);
        // renderBox(viewer,box,clusterId);
    }

	
  	while (!viewer->wasStopped ())
  	{
  		viewer->spinOnce ();
  	}
  	
}
