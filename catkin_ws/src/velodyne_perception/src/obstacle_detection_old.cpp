
#include <time.h>
#include <stdlib.h>
#include <ros/ros.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

//#include <guidingbot_msgs/VibrationArray.h>

#define GUIDEDOG_DEBUG 1
#define OBSTACLE_PCL_THRES 500

ros::Publisher pub_vb_cmd;
ros::Publisher pub_filtered_point;
ros::Publisher pub_obstacles;
ros::Publisher pub_vis;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
static double threshold_z_min = -0.2;
static double threshold_z_max = 1.8 ; 
static double threshold_x_min = 0.15;
static double threshold_x_max = 2.5; 
static double threshold_y_min = -1.5;
static double threshold_y_max = 1.5;


//guidingbot_msgs::VibrationArray vbArray;

// void vibration_strategy(int left, int center, int right);
 /*void vibration_strategy(int left, int center, int right)
{
    static ros::Time time_begin = ros::Time::now();
    static uint8_t both_side_mode = 0;

    if(center > 1){
        vbArray.intensities[1] = 4;
        vbArray.frequencies[1] = 3;
    }
    else if(center == 1){
        vbArray.intensities[1] = 3;
        vbArray.frequencies[1] = 2;
    }

    if(left > 1){
        vbArray.intensities[2] = 4;
        vbArray.frequencies[2] = 3;
    }
    else if(left == 1){
        vbArray.intensities[2] = 3;
        vbArray.frequencies[2] = 2;
    }

    if(right > 1){
        vbArray.intensities[0] = 4;
        vbArray.frequencies[0] = 3;
    }
    else if(right == 1){
        vbArray.intensities[0] = 3;
        vbArray.frequencies[0] = 2;
    }

    if(ros::Time::now() - time_begin > ros::Duration(0.5))
    {
        if(right != 0 && left != 0)
        switch(both_side_mode)
        {
            case 0:
                both_side_mode = 1;
            case 1:
                vbArray.intensities[0] = 0;
                both_side_mode = 2;
                break;
            case 2:
                vbArray.intensities[2] = 0;
                both_side_mode = 1;
                break;
        }
        else both_side_mode = 0;

        printf("%s\n", "pubed");
        time_begin = ros::Time::now();
        pub_vb_cmd.publish(vbArray);            // Send vibration msg
        vbArray.frequencies.clear();
        vbArray.intensities.clear();
        for(int i = 0; i < 3; i++){
            vbArray.frequencies.push_back(0);
            vbArray.intensities.push_back(0);
        }
        printf("left: %d, front: %d, right: %d, both: %d\n", vbArray.intensities[0], vbArray.intensities[1], vbArray.intensities[2], both_side_mode);

    }
}*/


void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    //guidingbot_msgs::VibrationArray vbArray;
    visualization_msgs::MarkerArray markerArray;
    markerArray.markers.resize(0);

    //for(int i = 0; i < 3; i++){
    //    vbArray.frequencies.push_back(0);
    //    vbArray.intensities.push_back(0);
    //}

    PointCloudXYZ::Ptr cloud(new PointCloudXYZ);
    pcl::fromROSMsg (*input, *cloud); //convert from PointCloud2 to pcl point type
    std::vector<int> indices;

    /******************************** Down Sampling ********************************/
    // pcl::VoxelGrid<pcl::PointXYZ> sor;
    // sor.setInputCloud (cloud);
    // sor.setLeafSize (0.05f, 0.005f, 0.005f);
    // sor.filter (*cloud);	//Downsample point cloud

    /****************** Filter ground and long distance pointcloud ******************/
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> () );
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, threshold_z_min)));
    //range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, threshold_z_max)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::GT, threshold_y_min)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZ> ("y", pcl::ComparisonOps::LT, threshold_y_max)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::GT, threshold_x_min)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZ> ("x", pcl::ComparisonOps::LT, threshold_x_max)));
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem (range_cond);
    condrem.setInputCloud (cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    condrem.filter (*cloud_filtered); // apply filter function

    /* copy a pointcloud fot visualization */
    cloud_filtered->header.frame_id = "velodyne";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud_filtered, *vis_cloud);
    for (int i = 0; i < vis_cloud->points.size(); i++){
      vis_cloud->points[i].g = 255;
    }
    pub_filtered_point.publish(vis_cloud);

#if GUIDEDOG_DEBUG
      printf("-------------------------Cloud information-----------------------------\n");
      printf("Original cloud size: %ld\n", cloud->points.size());
      printf("Filtered cloud size: %ld\n", vis_cloud->points.size());
#endif

    if(cloud_filtered->size() < 50) return;


    /********* Creating the KdTree object for the search method of the extraction *********/
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.2); // 2cm
    ec.setMinClusterSize(100);
    // ec.setMaxClusterSize(3000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_obstacles (new pcl::PointCloud<pcl::PointXYZRGB>);
    vis_obstacles->points.resize(0);


    int left=0, right=0, center=0;

    /****************** Obstacle detection and send vibration messeges ******************/
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        visualization_msgs::Marker marker;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        double xmax = -255.0, xmin = 255.0;
        double ymax = -255.0, ymin = 255.0;
        double zmax = -255.0, zmin = 255.0;

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
            if(cloud_filtered->points[*pit].x > xmax) xmax = cloud_filtered->points[*pit].x;
            else if(cloud_filtered->points[*pit].x < xmin) xmin = cloud_filtered->points[*pit].x;

            if(cloud_filtered->points[*pit].y > ymax) ymax = cloud_filtered->points[*pit].y;
            else if(cloud_filtered->points[*pit].y < ymin) ymin = cloud_filtered->points[*pit].y;

            if(cloud_filtered->points[*pit].z > zmax) zmax = cloud_filtered->points[*pit].z;
            else if(cloud_filtered->points[*pit].z < zmin) zmin = cloud_filtered->points[*pit].z;         
        }

        marker.header.frame_id = "velodyne";
        marker.ns = "basic_shapes";
        marker.id = it - cluster_indices.begin();
        marker.lifetime = ros::Duration(1);             // delete marker every 1 sec
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = (xmax + xmin) / 2;
        marker.pose.position.y = (ymax + ymin) / 2;
        marker.pose.position.z = (zmax + zmin) / 2;
        marker.scale.x = (xmax - xmin);
        marker.scale.y = (ymax - ymin);
        marker.scale.z = (zmax - zmin);
        marker.color.a = 0.5;
        
        // printf("%lf %lf %lf %lf %lf %lf\n", xmin, xmax, ymin, ymax, zmin, zmax);
        markerArray.markers.push_back(marker);
        
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        int tmp_intensity;
        int tmp_freq;
        

        // if(ymax < -0.3 || ymin < -0.35) {
        //     if(xmin > 1.8 && left < 2) left = 1;
        //     else left = 2;       
        // }

        // if(ymin > 0.3 || ymax > 0.35) {
        //     if(xmin > 1.8 && right < 2) right = 1;
        //     else right = 2;
        // }

        // if((ymax+ymin)/2 < 0.3 && (ymax+ymin)/2 > -0.3 ) {
        //     if(xmin > 1.8 && right < 2) center = 1;
        //     else center = 2;
        // }
/*
        if(ymax < -0.3 || ymin < -0.35) {
            if(xmin > 1.5 && vbArray.intensities[2] < 4){
                vbArray.intensities[2] = 2;
                vbArray.frequencies[2] = 2;
            }
            else{
                vbArray.intensities[2] = 3;
                vbArray.frequencies[2] = 4;
            }
        }

        if(ymin > 0.3 || ymax > 0.35) {
            if(xmin > 1.5 && vbArray.intensities[0] < 4){
                vbArray.intensities[0] = 2;
                vbArray.frequencies[0] = 2;
            }
            else{
                vbArray.intensities[0] = 3;
                vbArray.frequencies[0] = 4;
            }
        }

        if((ymax+ymin)/2 < 0.3 && (ymax+ymin)/2 > -0.3 ) {
            if(xmin > 1.5 && vbArray.intensities[1] < 4){
                vbArray.intensities[1] = 2;
                vbArray.frequencies[1] = 2;
            }
            else{
                vbArray.intensities[1] = 3;
                vbArray.frequencies[1] = 4;
            }
        }
*/
        cloud_cluster->header.frame_id = "velodyne";
        for (int i = 0;i < vis_cloud->points.size(); i++){
            vis_cloud->points[i].r = 255.0;
        }
        pcl::copyPointCloud(*cloud_cluster, *vis_cloud);

        int original_size = vis_obstacles->points.size();
        vis_obstacles->points.resize(vis_cloud->points.size() + original_size);
        int color = rand() % 255;
        for(int i = 0; i < vis_cloud->points.size(); i++){
            vis_obstacles->points[i + original_size].x = vis_cloud->points[i].x;
            vis_obstacles->points[i + original_size].y = vis_cloud->points[i].y;
            vis_obstacles->points[i + original_size].z = vis_cloud->points[i].z;
            vis_obstacles->points[i + original_size].r = color;
        }

        #if GUIDEDOG_DEBUG 
            // printf("Obj cloud size: %d, position: %lf %lf %lf\n", cloud_cluster->width, centroid[0], centroid[1], centroid[2]);
        #endif
        
    }
    // vibration_strategy(left, center, right);
    pub_vis.publish(markerArray);
    vis_obstacles->header.frame_id="velodyne";
    pub_obstacles.publish(vis_obstacles);   // Publish obstacle pointcloud
    //pub_vb_cmd.publish(vbArray);            // Send vibration msg

#if GUIDEDOG_DEBUG
    //printf("Obstacles cloud size: %ld\n",vis_obstacles->points.size());
    //printf("left: %d, front: %d, right: %d \n", vbArray.intensities[0], vbArray.intensities[1], vbArray.intensities[2]);
#endif

//----------------------------------------------------------------------
} 





int main (int argc, char** argv){
    srand(time(NULL));

    // Initialize ROS
    ros::init (argc, argv, "obstacle_detection");
    ros::NodeHandle nh;   
    nh.getParam("threshold_z_min", threshold_z_min);
    nh.getParam("threshold_z_max", threshold_z_max);
    nh.getParam("threshold_y_min", threshold_y_min);
    nh.getParam("threshold_y_max", threshold_y_max);
    nh.getParam("threshold_x_min", threshold_x_min);
    nh.getParam("threshold_x_max", threshold_x_max);
    ros::Subscriber model_subscriber = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 1, cloud_cb);
    //pub_vb_cmd = nh.advertise<guidingbot_msgs::VibrationArray> ("vibrate_cmd", 1);
    pub_filtered_point = nh.advertise<sensor_msgs::PointCloud2> ("filtered_point", 1);
    pub_obstacles = nh.advertise<sensor_msgs::PointCloud2> ("obstacles", 1);

    //20180506 SamLiu
    pub_vis = nh.advertise<visualization_msgs::MarkerArray>( "marker_array", 20);

    // ros::Timer timer = nh.createTimer(ros::Duration(0.5), timer_cb);
    /*vbArray.frequencies.clear();
    vbArray.intensities.clear();
    for(int i = 0; i < 3; i++){
        vbArray.frequencies.push_back(0);
        vbArray.intensities.push_back(0);
    }*/

    // Spin
    ros::spin ();
}
