#include "ros/ros.h"
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <kobuki_msgs/BumperEvent.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class RoboCtrl
{
	public: 
 		RoboCtrl()
                   : it_(node)
		{
			//ros::NodeHandle node;
                        //購読するtopic名
			odom_sub_ = node.subscribe("odom", 1, &RoboCtrl::odomCallback, this);
                        //bumper_sub_ = node.subscribe("/mobile_base/events/bumper", 1, &RoboCtrl::bumperCallback, this);
                        image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &RoboCtrl::imageCb, this);
                        //Laser_sub_ = node.subscribe<sensor_msgs::LaserScan>("/scan", 100, &RoboCtrl::LaserScan, this);

			//配布するtopic名
			//twist_pub_ = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
                        twist_pub_ = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1); 

                        //内部関数初期化
			frontspeed = 0.5;
                        turnspeed = 0;

                        back_flag = 0;
                        right_flag = 0;
                        left_flag = 0;

                        cv::namedWindow(OPENCV_WINDOW);
		}
                ~RoboCtrl()
                {
                        cv::destroyWindow(OPENCV_WINDOW);
                }
		void moveRobo()
		{
                        //速度データ型宣言
			geometry_msgs::Twist twist;
                        
                        //bumper効果時間計測
                        under_time_ = ros::Time::now() - push_time_;

                        if ( under_time_ < ros::Duration(1.0) )
                        {
                           frontspeed = -0.5;
                           turnspeed = 0.0; 
                        }
                        else if ( under_time_ > ros::Duration(1.0) && under_time_ < ros::Duration(3.0) ) 
                        {
                           frontspeed = 0.0;
                           if (left_flag == 1){turnspeed = 2.0;}
                           else if (back_flag == 1){turnspeed = 2.0;}
                           else if (right_flag == 1){turnspeed = -2.0;}                             
                        }
                        else{
                           //frontspeed = 0.5;
                           //turnspeed = 0.0; 
                           left_flag = 0;
                           right_flag = 0;
                           back_flag = 0; 
                        }
		
                        //ROS速度データに内部関数値を代入
			twist.linear.x = frontspeed;
			twist.angular.z = turnspeed;

                        //速度データ配布 
			twist_pub_.publish(twist);
		}
		void odomCallback(const nav_msgs::Odometry &odom)
		{
			posex = odom.pose.pose.position.x;
                        posey = odom.pose.pose.position.y;
                        
                        //ROS_INFO("x:%lf y:%lf",posex,posey);
                        
                        posex_old = posex;
                        posey_old = posey; 
 
                        return;
		}
                void bumperCallback(const kobuki_msgs::BumperEvent &bumper)
		{ 
                        //ROS_INFO("get data");
  			if( bumper.state == 1)
                        { 
                            //bumper接触時間更新 
                            push_time_ = ros::Time::now();

			    //前進速度									
                            frontspeed = 0.0;

                            if (bumper.bumper == 0)
                            { 
                                ROS_INFO("Left hit");
                                left_flag = 1; 
                            }
                            else if (bumper.bumper == 1)
                            { 
                                ROS_INFO("front hit");
                                //frontspeed = -0.5;
                                back_flag = 1;  
                            }
                            else if (bumper.bumper == 2)
                            { 
                                ROS_INFO("right hit");
                                right_flag = 1; 
                            }
                        }
                        else{
                                ROS_INFO("non hit");
                                //frontspeed = 1.0;
                                //turnspeed = 0.0; 
                             }
		  }
                  void imageCb(const sensor_msgs::ImageConstPtr& msg)
                  {
                       cv_bridge::CvImagePtr cv_ptr;
                       try
                       {
                           cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                       }
                       catch (cv_bridge::Exception& e)
                       {
                           ROS_ERROR("cv_bridge exception: %s", e.what());
                           return;
                       }

                       cv::cvtColor(cv_ptr->image,hsv,CV_BGR2HSV);
                       cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(30, 255, 255), mask); // 色検出でマスク画像の作成
                       //cv::bitwise_and(cv_ptr->image,mask,image);

                       cv::Moments mu = cv::moments( mask, false );
                       cv::Point2f mc = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
       
                       int x = mu.m10/mu.m00;
                       int y = mu.m01/mu.m00;
                       ROS_INFO("obj x=%d y=%d",x,y);

                       if(x>0)
                       { 
                          cv::circle( mask, mc, 4,cv::Scalar(100), 2, 4);
                          frontspeed = 0.5; 
                          if(x > 220 && x < 420){turnspeed = 0.0;}
                          else if(x > 420){turnspeed = -(x - 420)*0.01;}
                          else if(x < 220){turnspeed = -(x - 420)*0.01;}
                       }
                       if(x < 0){
                          frontspeed = 0.0;
                          turnspeed = 2.0;
                       }

                       // Update GUI Window
                       cv::imshow(OPENCV_WINDOW, mask);
                       cv::waitKey(3);
                  }
                  void LaserScan(const sensor_msgs::LaserScanConstPtr& Laser)
	          {
                       int image_width = 640;
                       int image_height = 480;
                       int line_thickness = 5; 
                       int frame_number = (Laser->angle_max - Laser->angle_min)/Laser->angle_increment + 1; 
                       double pixelsize;
                       cv::Mat map_img(cv::Size(image_width, image_height), CV_8UC3, cv::Scalar::all(255));  // cv::Scalar(0,0,255)(B、G、R)
                       pixelsize = Laser->range_max / image_height; 

                       double angle,length,pointX,pointY;

                       //write right data
                       for(int i = 0; i < frame_number; i++)
                       {
                           //angle = 3.14/2 + (Laser->angle_min - MOUNTING_ANGLE_RIGHT/180.0*3.14 + Laser->angle_increment * i);
                           angle = 3.14/2 + (Laser->angle_min + Laser->angle_increment * i); 
                           length = Laser->ranges[i]/pixelsize;
                           pointX = image_width/2 + length*cos(angle);
                           //pointX = IMAGE_WIDTH/2 + (length*cos(angle) + MOUNTING_DISTANCE_RIGHT/pixelsize);
                           pointY = image_height/2 - length*sin(angle);
          
                           if (pointX > 0 && (Laser->ranges[i]>0.0))
                           {
                                cv::rectangle(map_img, cvPoint(int(pointX),int(pointY)), cvPoint(int(pointX)+line_thickness,int(pointY)+line_thickness), CV_RGB(255,0,0), CV_FILLED, 8, 0);
                           }  
       
                       }
                       cv::imshow("Laserscan", map_img);
                       cv::waitKey(3); 
                       
                  }		 

	private:
                ros::NodeHandle node;
		ros::Subscriber odom_sub_;
		ros::Subscriber bumper_sub_;
                ros::Subscriber Laser_sub_;
		ros::Publisher twist_pub_;
                //ROS時間 
                ros::Time push_time_;
                //ROS変化秒 		
 		ros::Duration under_time_;
 
                cv::Mat hsv;
                cv::Mat mask;
                cv::Mat image;

                image_transport::ImageTransport it_;
                image_transport::Subscriber image_sub_;            
                
                double frontspeed;
                double turnspeed;
                double posex;
                double posey;
                double posex_old;
                double posey_old;

                int back_flag;
                int right_flag;
                int left_flag;
};

int main(int argc, char **argv)
{
        //ROSのノード初期化
	ros::init(argc, argv, "robo_ctrl");
	RoboCtrl robo_ctrl;
	ros::Rate r(15);
	while (ros::ok())
	{
		robo_ctrl.moveRobo();
		ros::spinOnce();
		r.sleep();
	}
}
