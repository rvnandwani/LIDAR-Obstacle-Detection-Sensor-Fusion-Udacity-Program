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
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
    Lidar* ldr = new Lidar(cars , 0); 
  pcl::PointCloud<pcl::PointXYZ>::Ptr initcloud = ldr->scan();
  	renderPointCloud(viewer ,initcloud, "Lidar" , Color(1,1,1));

    // TODO:: Create point processor
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr , pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane( initcloud, 100, 0.2);
    renderPointCloud(viewer , segmentCloud.first , "Obstacle" , Color(1,0,0));
    renderPointCloud(viewer , segmentCloud.second , "Road" , Color(0,1,0));
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = pointProcessor.Clustering( segmentCloud.first ,1.0, 3, 30);

  int Id=0;
  std::vector<Color> colors = {Color(1,0,0),Color(1,1,0),Color(0,0,1)};
  for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : clusters){
    Box box = pointProcessor.BoundingBox(cluster);
	renderBox(viewer,box,Id);
    renderPointCloud(viewer , cluster , "Obstacle"+std::to_string(Id) , colors[Id % colors.size()]);
    Id++;
  }
  
}


void CityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer , ProcessPointClouds<pcl::PointXYZI>* pointProcessorI , const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){
  
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  // ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
  // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtercloud = pointProcessorI->FilterCloud(inputCloud,0.3,Eigen::Vector4f(-10,-5,-2,1),Eigen::Vector4f(30,8,1,1));
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr , pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_filterCloud = pointProcessorI->SegmentPlane( filtercloud, 25, 0.3);
  renderPointCloud(viewer , segment_filterCloud.first , "Obstacle" , Color(1,0,0));
  renderPointCloud(viewer , segment_filterCloud.second , "Road" , Color(0,1,0));
  // renderPointCloud(viewer,filteredCloud,"filtercloud");

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessorI->Clustering( segment_filterCloud.first ,0.53, 10, 500);

  int Id=0;
  std::vector<Color> colors = {Color(1,0,0),Color(1,1,0),Color(0,0,1)};
  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clusters){
    // renderPointCloud(viewer , cluster , "Obstaclecloud"+std::to_string(Id) , colors[Id % colors.size()]);
    Box box = pointProcessorI->BoundingBox(cluster);
	  renderBox(viewer,box,Id);
    renderPointCloud(viewer , cluster , "Obstacle"+std::to_string(Id) , colors[Id % colors.size()]);
    Id++;
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
    // simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
	std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
	auto streamIterator = stream.begin();
	pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
  
//     CityBlock(viewer);

    while (!viewer->wasStopped ())
    {
         // Clear viewer
  		viewer->removeAllPointClouds();
  		viewer->removeAllShapes();

  		// Load pcd and run obstacle detection process
  		inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
  		CityBlock(viewer, pointProcessorI, inputCloudI);

  		streamIterator++;
  		if(streamIterator == stream.end())
    		streamIterator = stream.begin();

  		viewer->spinOnce ();
    } 
}