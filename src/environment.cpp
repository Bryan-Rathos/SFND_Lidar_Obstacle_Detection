/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    bool renderInCloud = false;
    bool renderObstCloud = false;
    bool renderPlaneCloud = false;
    bool renderClusters = false;
    
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    // Lidar* lidar = new Lidar(cars, 0);
    auto lidar = std::make_unique<Lidar>(cars,0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // renderRays(viewer, lidar->position, inputCloud);
    if(renderInCloud)
        renderPointCloud(viewer, inputCloud, "inputCloud");

    // TODO:: Create point processor
    // ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>;
    auto pointProcessor = std::make_unique<ProcessPointClouds<pcl::PointXYZ>>();

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, 100, 0.2);
    // std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->RansacPlane(inputCloud, 100, 0.2);
    
    if(renderObstCloud)
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    if(renderPlaneCloud)
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 3.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0)};//, Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);
        if(renderClusters)
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        ++clusterId; 

        Box box = pointProcessor->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
    }
}

/*
pcl::PointCloud<pcl::PointXYZI>::Ptr CreateData(std::vector<std::vector<float>> points)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  	
  	for(int i = 0; i < points.size(); i++)
  	{
  		pcl::PointXYZI point;
  		point.x = points[i][0];
  		point.y = points[i][1];
  		point.z = points[i][2];
        point.intensity = 0.0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;
}*/


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    bool renderInCloud = false;
    bool renderFiltCloud = false;
    bool renderObstCloud = false;
    bool renderPlaneCloud = true;
    bool renderClusters = true;
    bool renderBBox = true;

    // ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    auto pointProcessorI = std::make_unique<ProcessPointClouds<pcl::PointXYZI>>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000010.pcd");
    
    if(renderInCloud)
        renderPointCloud(viewer,inputCloud,"inputCloud");

    float filterRes = 0.2;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, filterRes , Eigen::Vector4f (-15.0, -6.5, -2.0, 1), Eigen::Vector4f ( 30, 6.5, 3.0, 1));
    
    if(renderFiltCloud)
        renderPointCloud(viewer,filterCloud,"filterCloud");

    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RansacPlane(filterCloud, 40, 0.2);

    if(renderObstCloud)
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    if(renderPlaneCloud)
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));     

    // std::vector<std::vector<float>> points = {{2,3,3}, {5,4,2}, {9,6,7},
	// 										  {4,7,9}, {8,1,5}, {7,2,6},
	// 										  {9,4,1}, {8,4,2}, {9,7,8},
	// 										  {6,3,1}, {3,4,5}, {1,6,8},
	// 										  {9,5,3}, {2,1,3}, {8,7,6}};

	// pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = CreateData(points);

    // std::cout << "Main CS: " << cloud->points.size() << std::endl;

    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 20, 700);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->CustomClustering(segmentCloud.first, 0.3, 25, 400);

    std::cout << "Main cloudClusters size: " << cloudClusters.size() << std::endl;

    int clusterId = 0;
    std::vector<Color> colors = {Color(0,0,1), Color(0,1,0), Color(0,1,1),
								 Color(1,0,0), Color(1,0,1), Color(1,1,0),
								 Color(0.7,0.2,0.4), Color(1,0.5,0.6), Color(0.6,0.3,0.8)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        if(renderClusters)
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        ++clusterId; 

        if(renderBBox)
        {
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
        }
    }
}

// When PCD data is streamed this function is used
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, std::unique_ptr<ProcessPointClouds<pcl::PointXYZI>>& pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    bool renderInCloud = false;
    bool renderFiltCloud = false;
    bool renderObstCloud = false;
    bool renderPlaneCloud = true;
    bool renderClusters = true;
    bool renderBBox = true;

    // ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    // auto pointProcessorI = std::make_unique<ProcessPointClouds<pcl::PointXYZI>>();
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000010.pcd");
    
    if(renderInCloud)
        renderPointCloud(viewer,inputCloud,"inputCloud");

    float filterRes = 0.2;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, filterRes , Eigen::Vector4f (-15.0, -6.0, -2.0, 1), Eigen::Vector4f ( 30, 6.0, 3.0, 1));
    
    if(renderFiltCloud)
        renderPointCloud(viewer,filterCloud,"filterCloud");

    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RansacPlane(filterCloud, 40, 0.2);

    if(renderObstCloud)
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    if(renderPlaneCloud)
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));     

    // std::vector<std::vector<float>> points = {{2,3,3}, {5,4,2}, {9,6,7},
	// 										  {4,7,9}, {8,1,5}, {7,2,6},
	// 										  {9,4,1}, {8,4,2}, {9,7,8},
	// 										  {6,3,1}, {3,4,5}, {1,6,8},
	// 										  {9,5,3}, {2,1,3}, {8,7,6}};

	// pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = CreateData(points);
    // std::cout << "Main CS: " << cloud->points.size() << std::endl;
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 20, 700);

    float clusterTolerance = 0.3;
    int clusterMinSize = 20;
    int clusterMaxSize = 700;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->CustomClustering(segmentCloud.first, clusterTolerance, clusterMinSize, clusterMaxSize);

    std::cout << "Main cloudClusters size: " << cloudClusters.size() << std::endl;

    int clusterId = 0;
    std::vector<Color> colors = {Color(0,0,1), Color(0.8,0.5,0), Color(0,1,1),
								 Color(1,0,0), Color(1,0,1), Color(1,1,0),
								 Color(0.7,0.2,0.4), Color(1,0.5,0.6), Color(0.6,0.3,0.8),
                                 Color(0.4,0.5,0.7), Color(0.6,0.5,1), Color(0.8,0.3,0.4)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        if(renderClusters)
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        ++clusterId; 

        if(renderBBox)
        {
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
        }
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    // SCENE OPTION
    bool generatedHighway = false; // flag for simple Highway visualization
    bool realPCD = true;           // flag for real PCD data 
    bool realPCDFrame = false;     // flag to choose single frame or to stream PCD data. When false PCD will be streamed

    if(generatedHighway)
    {
        simpleHighway(viewer);

        while (!viewer->wasStopped())
        {
            viewer->spinOnce();
        }    
    }

    if(realPCD)
    {
        if(realPCDFrame)
        {
            cityBlock(viewer);

            while (!viewer->wasStopped())
            {
                viewer->spinOnce();
            }   
        }
        else
        {
            // ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
            auto pointProcessorI = std::make_unique<ProcessPointClouds<pcl::PointXYZI>>();

            std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
            auto streamIterator = stream.begin();
            pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

            while (!viewer->wasStopped())
            {
                // Clear viewer
                viewer->removeAllPointClouds();
                viewer->removeAllShapes();

                // Load PCD and run obstacle detetion process
                inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
                cityBlock(viewer, pointProcessorI, inputCloudI);

                streamIterator++;
                if(streamIterator == stream.end())
                    streamIterator = stream.begin();

                viewer->spinOnce();
            }
        }
    }
}