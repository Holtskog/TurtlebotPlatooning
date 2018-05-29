#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include <time.h>
#include <thread>
#include <cmath>

float max_vel = 0.22; // metres per second
float min_vel = 0.00;
float max_angular_speed = 2.84; // rad/s figure taken from user manual
float min_angular_speed = - 2.84; //  rad/s figure taken from user manual
float curvature_restriction = 5; // maximum curvature allowed, defined as 1/radius, radius in metres. 
float target_distance = 0.5; // reference distance for the distance controller
float kp = 3.14;
float kc = 0.8645;
float kv = 13.9;
float k_theta = 0.249;
float lead_velocity; //velocity of leading car
float d_state;
float v_state;
int iterations = 0;
float t_v;
float t_d;
float distance;
float velocity;
float angle;
float lidar_radius = 0.032;
float theta_ref = 0;
int lengths = 0;
float recorded_distances [150];
float recorded_angles [150];
float recorded_velocity [150];
float recorded_angular [150];

// enable time difference measurements
clock_t t;

// converts degrees to radians
float degrees2radians(float degrees)
	{
		if (degrees > 180)
		{
			degrees = degrees - 360;
		}
		float radians = (degrees/360)*2*M_PI;
		return radians;
	}

void lidarCallback(const sensor_msgs::LaserScan& msg)
{	
	float lowest_value_angle;
	float lowest_value = 3.5;
	float ranges_no_zeroes[360];
	
	//get LiDAR data from the forward facing 91 degrees
	for (int i=315; i<360; i++)
	{
		if (msg.ranges[i] == 0  )	
		{
			ranges_no_zeroes[i] = 3.5;
		}
		else
		{
			ranges_no_zeroes[i] = msg.ranges[i];
		}		
	}

	for (int i = 0; i < 46; i++)
	{
		if ( msg.ranges[i] == 0 )	
		{
			ranges_no_zeroes[i] = 3.5;
		}		
		else
		{	
			ranges_no_zeroes[i] = msg.ranges[i];
		}
	}

	for (int i=315; i<360; i++)
	{
		if ( (ranges_no_zeroes[i] < lowest_value)  )	
		{
			lowest_value = ranges_no_zeroes[i];
			lowest_value_angle = i;
		}		
	}
	for (int i = 0; i < 46; i++)
	{
		if ( (ranges_no_zeroes[i] < lowest_value)  )	
		{
			lowest_value = ranges_no_zeroes[i];
			lowest_value_angle = i;
		}		
	}
	
	//store distance and relative angle to make it available for the main function
	distance = lowest_value + lidar_radius;
	angle = degrees2radians(lowest_value_angle);
	
	if (lengths <= 149)
	{
		recorded_distances[lengths] = distance;
		recorded_angles[lengths] = angle;
		recorded_velocity[lengths] = velocity;
		recorded_angular[lengths] = angle * k_theta;
	}

	if (lengths == 149)
	{
		std::ofstream df;
		df.open("FullSystemTestEmergency.csv");
		for (int i = 0; i < 200; i++)
		{
			df << recorded_distances[i] << "," << recorded_angles[i] << "," << recorded_velocity[i] << "," << recorded_angular[i] << std::endl;
		}
		df.close();
		ROS_INFO_STREAM("HERE?");
	}

	lengths ++;

}

// Receive lead car velocity
void lead_velCallback(const std_msgs::Float32::ConstPtr& msg)
{
	lead_velocity = msg->data;
}

// Receive "self" velocity
void velCallback(const sensor_msgs::JointState& msg)
{
    velocity = ((msg.velocity[0] + msg.velocity[1])/2)*0.033;
}

// Full control system
float control_function(float d_meas, float Vf, float Vl, float time_d, float time_v)
{
	float ref_speed;
	float d_error = d_meas - target_distance;
	float v_error = Vl - Vf;
	
	//calculate the 's' in 1/s
	if(iterations == 0)
	{
		d_state = d_state + (d_error * ((clock() - time_d)/CLOCKS_PER_SEC));
		v_state = v_state + (v_error * ((clock() - time_v)/CLOCKS_PER_SEC));	
	}
	else
	{
		d_state = d_state + (d_error * ((clock() - t_d)/CLOCKS_PER_SEC));
		t_d = clock();
		v_state = v_state + (v_error * ((clock() - t_v)/CLOCKS_PER_SEC));
		t_v = clock();
	}

	//calculate the inner and outer loop equations
	float outer = d_state * kp;
	float inner = v_state * kv;

	//find the reference speed to send to the Dynamixel motors
	ref_speed = outer + inner + (Vl/kc);
	
	iterations = iterations + 1;
    return ref_speed;
}

// steering controller(P)
float steering_control_function(float theta_meas)
{
	float w_ref;
	w_ref = k_theta*(theta_meas - theta_ref);
	return w_ref;
}

int main(int argc, char **argv)
{
    // initialize ros
    ros::init(argc, argv, "main_distance_controller");
	ros::CallbackQueue queue;
    ros::NodeHandle nh;
	nh.setCallbackQueue(&queue);
    ROS_INFO_STREAM("NODE NOTED");

    // declare all subscribers/publishers
    ros::Publisher motor = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Subscriber lidar = nh.subscribe("/scan", 1000, lidarCallback);
    ros::Subscriber lead_vel = nh.subscribe("/lead_speed", 1000, lead_velCallback);
    ros::Subscriber vel = nh.subscribe("/joint_states", 1000, velCallback);    

	ros::Rate loop_rate(50);	


    while (ros::ok())
    {

		float speed;
		float angular_speed;

		//get reference velocity from distance controller
		if (iterations == 0)
		{
			t_d = clock();
			t_v = clock();
			speed = control_function(distance, velocity, lead_velocity, t_d, t_v);
		}
		else
        	{
			speed = control_function(distance, velocity, lead_velocity, t_d, t_v);
		}
		
		//get the reference angular velocity from the steering controller
		angular_speed = steering_control_function(angle);
		
		// Create a variable for the velocity data to be published
		geometry_msgs::Twist vel;
      
        	// Make sure the velocity doesn't go negative or above the maximum
        	if (speed > max_vel)
        	{
        		vel.linear.x = max_vel;
        	}
        	else if (speed >= min_vel)
        	{
        		vel.linear.x = speed;
        	}
        	else if (speed < min_vel)
        	{
            		vel.linear.x = min_vel;
        	}
		
		// limit angular velocities sent to those in hardware specification
		if (angular_speed > max_angular_speed)
        	{
            		vel.angular.z = max_angular_speed;
        	}
        	else if (angular_speed < min_angular_speed )
        	{
            		vel.angular.z = min_angular_speed;
        	}
        	else
        	{
            		vel.angular.z = angular_speed;
        	}

		// restrict curvature
		if ((abs(angular_speed/vel.linear.x)) > (curvature_restriction))
		{
			angular_speed = speed*curvature_restriction;
        		vel.angular.z = angular_speed;
			//ROS_INFO_STREAM("HELLO!!!!");
		}

		//publish linear and angular velocity to the Dynamixel motors
		motor.publish(vel);

		// Make sure all the callbacks are looped by the loop frequency
		queue.callAvailable(ros::WallDuration(0.02));
    	}	

	return 0;
}
