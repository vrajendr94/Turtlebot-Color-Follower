/*****************************************************************************************
 * Name: obstacle_avoidance.cpp
 * Author: Vaishali Rajendren, Surya Tamraparni
 * Date: 02/05/2019
 * HW #6: Non-verbal sensing & Multimodal controllers
 *
 * Description: This program will subscribe to the /blobs topic 
 *              and camera/depth/image_rect topic and use the 
 *              blob information to find and go towards
 *              the blobs in a user-set order and if there's an 
 *              obstacle that is detected from /image_rect topic 
 *              it will move around the obstacle to find the target
 *              and start moving again towards the target. 
 * 
 * Commands Used: roscore
 *                roslaunch turtlebot_bringup minimal.launch
 *                roslaunch astra_launch astra_pro.launch
 *                roslaunch turtlebot_dashboard turltebot_dashboard.launch
 *                rosrun cmvision colorgui image:=/camera/rgb/image_raw
 *                After calibration, copy the YUV and RGB values to color.txt file
 *                Using rosparam set /cmvision/color_file turtlebotws/cmvision/colors.txt
 *                rosrun cmvision cmvision image:=/camera/rgb/image_raw
 *                rosrun obstacle_avoidance obstacle_avoidance 
 *             
 *****************************************************************************************/

#include <kobuki_msgs/BumperEvent.h> 
#include <cmvision/Blob.h>
#include <cmvision/Blobs.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <cmvision/Blobs.h>
#include <stdio.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <time.h>
#include <math.h>
#include <sensor_msgs/Image.h> //added from follower
#include <depth_image_proc/depth_traits.h>  //added from follower

ros::Publisher pub;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


double Center = 320; //mean of the centroid of x from /blobs
std::vector<uint16_t> goal_xs;

//gcounter - counter variable
int gcounter=0;

// fucntional vars
double blob_x;
double obstaclex;
double obstaclez;

//states
int obstate=0; //if obstactle is detected
int blobstate=0; //if target is detected 




/***********************************************************************
 * Function Name: blobsCallBack
 * Parameters: const cmvision::Blobs
 * Returns: void
 *
 * Description: This is the callback function of the /blobs topic and 
 *              calculates centroid of the target
 ************************************************************************/

void blobsCallBack (const cmvision::Blobs& blobsIn) //this gets the centroid of the color blob corresponding to the goal.
{
	if (blobsIn.blob_count > 0){

	blobstate=1;   //sets the target state to 1 if target is available 
	int n=blobsIn.blob_count;
	double goal_sum_x=0;
	double goal_sum_y=0;


	for (int i = 0; i < blobsIn.blob_count; i++){
		goal_sum_x += blobsIn.blobs[i].x;
		goal_sum_y += blobsIn.blobs[i].y;
	}
	goal_sum_x/=n;
	goal_sum_y/=n;   
	blob_x=goal_sum_x; //setting glob var
	}
}

/**************************************************************************************
 * Function Name: imagecb taken from follower.cpp to calculate centroid
 * Parameters: const sensor_msgs::ImageConstPtr& depth_msg
 * Returns: void
 *
 * Description: This is the callback function of the /camera/depth/image_rect topic and 
 *              calculates centroid of the obstacle
 **************************************************************************************/


void imagecb(const sensor_msgs::ImageConstPtr& depth_msg)
  {

	//local variables
  double min_y_ = 0.1; /**< The minimum y position of the points in the box. */
  double max_y_ = 0.5; /**< The maximum y position of the points in the box. */
  double min_x_ = -1; /**< The minimum x position of the points in the box. */
  double max_x_ = 1; /**< The maximum x position of the points in the box. */
  double max_z_ = 0.8; /**< The maximum z position of the points in the box. */
  double goal_z_ = 0.3; /**< The distance away from the robot to hold the centroid */
  double z_scale_ = 2.0; /**< The scaling factor for translational robot speed */
  double x_scale_ = 7.0; /**< The scaling factor for rotational robot speed */
  bool   enabled_ = true; /**< Enable/disable following; just prevents motor commands */



    // Precompute the sin function for each row and column
    uint32_t image_width = depth_msg->width;
    float x_radians_per_pixel = 60.0/57.0/image_width;
    float sin_pixel_x[image_width];
    for (int x = 0; x < image_width; ++x) {
      sin_pixel_x[x] = sin((x - image_width/ 2.0)  * x_radians_per_pixel);
    }

    uint32_t image_height = depth_msg->height;
    float y_radians_per_pixel = 45.0/57.0/image_width;
    float sin_pixel_y[image_height];
    for (int y = 0; y < image_height; ++y) {
      // Sign opposite x for y up values
      sin_pixel_y[y] = sin((image_height/ 2.0 - y)  * y_radians_per_pixel);
    }

    //X,Y,Z of the centroid
    float x = 0.0;
    float y = 0.0;
    float z = 1e6;
    //Number of points observed
    unsigned int n = 0;

    //Iterate through all the points in the region and find the average of the position
    const float* depth_row = reinterpret_cast<const float*>(&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof(float);
    for (int v = 0; v < (int)depth_msg->height; ++v, depth_row += row_step)
    {
     for (int u = 0; u < (int)depth_msg->width; ++u)
     {
       float depth = depth_image_proc::DepthTraits<float>::toMeters(depth_row[u]);
       if (!depth_image_proc::DepthTraits<float>::valid(depth) || depth > max_z_) continue;
       float y_val = sin_pixel_y[v] * depth;
       float x_val = sin_pixel_x[u] * depth;
       if ( y_val > min_y_ && y_val < max_y_ &&   //set min_ values
            x_val > min_x_ && x_val < max_x_)
       {
         x += x_val;
         y += y_val;
         z = std::min(z, depth); //approximate depth as forward.
         n++;
       }
     }
    }

    //If there are points, find the centroid and calculate the command goal.
    if (n>4000)
    {
      x /= n;
      y /= n;        //At this point, x,y,z are the centroid coordinates
      if(z > max_z_){
        ROS_INFO_THROTTLE(1, "Centroid too far away %f, stopping the robot", z);}
      
      if (enabled_)  //enabled
      {
	obstate=1; //setting obstacle state to 1 if an obstacle is detected
	obstaclex= (z - goal_z_) * z_scale_; //setting linear velocity
	obstaclez=-x * x_scale_; //setting angular velocity
      }
    }
  }


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "blob");
  ros::NodeHandle nh;

  //creating publisher for the topic cmd_vel_mux/input/teleop  
  ros::Publisher cmdpub_ = nh.advertise<geometry_msgs::Twist> ("cmd_vel_mux/input/teleop", 10);

  //subscribe to /blobs topic 
  ros::Subscriber blobsSubscriber = nh.subscribe("/blobs", 10, blobsCallBack);
  //subscribe to camera/depth/image_rect topic 
  ros::Subscriber sub_= nh.subscribe<sensor_msgs::Image>("camera/depth/image_rect", 1, imagecb ); // follower.cpp //&BotFollower::imagecb, this

  ros::Rate loop_rate(10);
  geometry_msgs::Twist t;

while(ros::ok()){

 //First state (state 0) 
 if(obstate==0 and blobstate==0)// Cannot see obstacle any more or initial state
{

//For debugging
std::cout << "zero"<< std::endl; 
std::cout << obstate<< std::endl;
std::cout << blobstate<< std::endl;
std::cout << gcounter<< std::endl;

//After an obstacle is detected this value will not be zero 
if(gcounter>0)
{
gcounter--;
geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
std::cout << "stop after obstacle"<< std::endl;
cmd->linear.x = 0.2;
cmd->angular.z = 0.0; //Stop after obstacle
cmdpub_.publish(cmd);
}
else{
geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
cmd->linear.x = 0;
cmd->angular.z = 0.5; //initial state, it spins
cmdpub_.publish(cmd);

}

} 

 else if ((obstate==1 and blobstate==0) ) //State 1 obstacle detection
{
std::cout << gcounter<< std::endl;
if(gcounter<50)
{
gcounter+=1;
}

//For debugging
std::cout << "obstacle"<< std::endl;
std::cout << blobstate<< std::endl;
std::cout << obstate<< std::endl;

geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
cmd->linear.x=0.0;
cmd->angular.z=0.5; //moves away from the obstacle
cmdpub_.publish(cmd);
} 

//Target is detected

else if ((obstate==1 and blobstate ==1) or (obstate==0 and blobstate==1)) //Target state, state = 1
{	
        //For debugging 
	std::cout << "target"<< std::endl;
	std::cout << obstate<< std::endl;
	std::cout << blobstate<< std::endl;
	geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
        cmd->linear.x = 0.2;
        cmd->angular.z = -(blob_x-350.0)*0.005; //Follows the target
        cmdpub_.publish(cmd);
}

    //Reset the state
    obstate=0;
    blobstate=0;
    // Spin
	ros::spinOnce();
	loop_rate.sleep();
  }

}

