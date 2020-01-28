/*
 * File : matrix_spawn_3d.cpp
 *
 * Author : Jonathan Sanabria
 *
 * Description : 
 *
 *		An application that spawns a 3d matrix of cubes in gazebo. The resulting dimension of 
 *		each axis in ther matrix will be defined by DIMENSION*2 .
 *
 *
 */
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h> 
#include <stdlib.h> 
#include<iostream> 
#include <sstream>      // std::stringstream
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#include <tf/transform_datatypes.h>


/* 
spawn object in gazebo using c++
spawn robot in gazebo using c++
spawn in gazebo using c++

g++ matrix_spawn_3d.cpp -o a_matrix -I/opt/ros/melodic/include -L/opt/ros/melodic/lib -Wl,-rpath,/opt/ros/melodic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization

*/


int DIMENSION = 4 ; // half the dimension of the matrix when using even numbers( i.e. 4 == an 8x8x8 cube matrix

std::stringstream get_box_at(int x , int y , int z, double mass , double length );
std::stringstream get_matrix(int x , int y , int z, double mass , double length ); 


/**
 * Having this "primary-cube-building-logic" which attaches to the world is necessary
 * to avoid having an even longer function. An alternative would be to create a parameter
 * which is changed only after the first cube is made. The only major change would be
 * to modify the joint.
 *	
 * @param int argc 		
 * @param char **argv 			 		 
 * @return int
 */ 
int main(int argc, char **argv) {
    clock_t timer= clock();

    int start = -1*DIMENSION , x = start , y = start , z = start ;
    std::stringstream geometry_template;
    std::stringstream origin_template ;
    std::stringstream cur_link_name ;
    double length = 0.2 ;
    double mass = 10 ;
    double inertia_value = mass / 12.0 * (length*length + length*length) ;

    geometry_template <<"<geometry>\
        			<box size=\""<<length<<" "<<length<<" "<<length<<"\"/>\
     			 </geometry>" ;
    origin_template << "<origin rpy=\"0 0 0\" xyz=\""<<x/2+2*length<<" "<<y/2+2*length<<" "<<z/2+2*length<<"\"/>" ;  
    cur_link_name   << "box_at_"<<x<<"_"<<y<<"_"<<z ;

    ros::init(argc, argv, "a_matrix_A");
    // Node setup
    ros::NodeHandle nh;

    std::stringstream _s ;
    _s.str(std::string());

    _s << "<?xml version=\"1.0\" ?>\
<robot name=\"a_matrix_robotname\"><link name=\"world\"/>\
<link name=\""<<cur_link_name.str()<<"\">\
    <inertial>\
      <mass value=\""<<mass<<"\"/>\
      <origin xyz=\"0 0 0\"/>\
      <inertia ixx=\""<<inertia_value<<"\" ixy=\"0.0\" ixz=\"0.0\"\
               iyy=\""<<inertia_value<<"\" iyz=\"0.0\"\
               izz=\""<<inertia_value<<"\"/>\
    </inertial>\
    <visual>\
	"<< origin_template.str() <<"\
	"<< geometry_template.str() <<"\
    </visual>\
    <collision>\
	"<< origin_template.str() <<"\
	"<< geometry_template.str() <<"\
    </collision>\
  </link>  <joint name=\"link_joint_"<<x<<"_"<<y<<"_"<<z<<"\" type=\"floating\">\
    <parent link=\"world\"/>\
    <origin rpy=\"0 0 0\" xyz=\""<<length<<" "<<length<<" "<<length<<"\"/>\
    <child link=\""<<cur_link_name.str()<<"\"/>\
  </joint>\
  <gazebo reference=\""<<cur_link_name.str()<<"\">\
    <turnGravityOff>true</turnGravityOff>\
    <material>Gazebo/Black</material>\
  </gazebo>";

    _s << get_matrix( x , y , z , mass , length ).str() ;

    _s << "</robot>";

    ROS_INFO("ros::ServiceClient spawn "); 
    ros::ServiceClient spawn = nh.serviceClient< gazebo_msgs::SpawnModel> ("/gazebo/spawn_urdf_model");

    gazebo_msgs::SpawnModel s1;
    geometry_msgs::Pose pose;
    // declare main model name
    s1.request.model_name = "a_matrix"; 

    ROS_INFO("_s >> s1.request.model_xml "); 
    s1.request.model_xml = _s.str();
    s1.request.robot_namespace = "a_matrix";

    pose.position.x=0;
    pose.position.y=0;
    pose.position.z=0;
    /*pose.orientation.x=0;
    pose.orientation.y=0;
    pose.orientation.z=0;
    pose.orientation.w=0;*/
    pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    s1.request.initial_pose = pose;
    s1.request.reference_frame = "world";

    ROS_INFO("spawn.waitForExistence()"); 
    spawn.waitForExistence();
    //std::cout<<s1.request<<std::endl;
    ROS_INFO("spawn.call(s1)"); 
    if (!spawn.call(s1)){
      ROS_ERROR("Failed to call service    time == %f",( clock() - timer ) / (double) CLOCKS_PER_SEC); 
      return 1; 
    } 
 
    ROS_INFO("success    time == %f",( clock() - timer ) / (double) CLOCKS_PER_SEC); 
    return 0;
}





/**
 * Makes up the main matrix forming logic
 *	
 * @param double mass 		
 * @param double length 		
 * @param int x 	 
 * @param int y 	 
 * @param int z 	 		 
 * @return std::stringstream           the return value will make up the main URDF
 */ 
std::stringstream get_matrix(int x , int y , int z, double mass , double length ){

    std::stringstream _s ;
    int loop_count = 0;
    while( x++ < DIMENSION-1 ){
	if( x == -1*DIMENSION+1 ){ 
        // if( first run of the loop )VVVVVVVVVVVVVVVV
        while( z < DIMENSION-1 ){
	    // column directly above the first cube made
	    z++ ;
            _s << get_box_at( x-1 , y , z , mass , length ).str() ;
        }z = -1*DIMENSION ; 
	while( y < DIMENSION-1 ){
	    // row directly adjacent to the first cube made
            y++;
            while( z < DIMENSION ){
              _s << get_box_at( x-1 , y , z , mass , length ).str() ;
	      z++ ;
            }z = -1*DIMENSION ; 
        } y = -1*DIMENSION ;
        } // end if   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        while( y < DIMENSION ){
            while( z < DIMENSION ){
              _s << get_box_at( x , y , z , mass , length ).str() ;
	      z++ ;
            }z = -1*DIMENSION ; 
          y++;
        } y = -1*DIMENSION ; 
    }
    return _s;
}





/**
 * Make a box shape in gazebo. 
 *	ATTN : The position of each box in relative to the box it is attached to (through the joint).
 *		The use of x y z is mainly used for nameing purposes/ to maintain order.
 * @param double mass 		
 * @param double length 		
 * @param int x 	 
 * @param int y 	 
 * @param int z 
 * @return std::stringstream           the return value will make up the main URDF
 */ 
std::stringstream get_box_at(int x , int y , int z, double mass , double length ){
    std::stringstream geometry_template;
    std::stringstream origin_template ;
    std::stringstream cur_link_name ;
    geometry_template <<"<geometry>\
        			<box size=\""<<length<<" "<<length<<" "<<length<<"\"/>\
     			 </geometry>" ;
    origin_template << "<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>" ;
    cur_link_name   << "box_at_"<<x<<"_"<<y<<"_"<<z ;
    double inertia_value = mass / 12.0 * (length*length + length*length) ;
    int scalar_distance = 2 ;

    std::stringstream _s ;
    _s <<"<link name=\""<<cur_link_name.str()<<"\">\
    <inertial>\
      <mass value=\""<<mass<<"\"/>\
      <origin xyz=\"0 0 0\"/>\
      <inertia ixx=\""<<inertia_value<<"\" ixy=\"0.0\" ixz=\"0.0\"\
               iyy=\""<<inertia_value<<"\" iyz=\"0.0\"\
               izz=\""<<inertia_value<<"\"/>\
    </inertial>\
    <visual>\
	"<< origin_template.str() <<"\
	"<< geometry_template.str() <<"\
    </visual>\
    <collision>\
	"<< origin_template.str() <<"\
	"<< geometry_template.str() <<"\
    </collision>\
  </link><joint name=\"link_joint_"<<x<<"_"<<y<<"_"<<z<<"\" type=\"fixed\">\
    <parent link=\"box_at_"<<-1*DIMENSION<<"_"<<-1*DIMENSION<<"_"<<-1*DIMENSION<<"\"/>\
    <origin rpy=\"0 0 0\" xyz=\""<<x*scalar_distance*length<<" "<<y*scalar_distance*length<<" "<<z*scalar_distance*length<<"\"/>\
    <child link=\""<<cur_link_name.str()<<"\"/>\
  </joint>\
    <gazebo reference=\""<<cur_link_name.str()<<"\">\
    <turnGravityOff>true</turnGravityOff>\
	<material>";
		if( x >= 5 && x % 5 == 0 ){
                _s<<"Gazebo/Orange";
                } else if( x >= 4 && x % 4 == 0 ){
                _s<<"Gazebo/Indigo";
                } else if( x >= 3 && x % 3 == 0 ){
                _s<<"Gazebo/Turquoise";
		} else if( x >= 2 && x % 2 == 0 ){
                _s<<"Gazebo/SkyBlue";
		} else if( x >= 1 && x % 1 == 0 ){
                _s<<"Gazebo/Blue";
		} else if( x <= 5 && x % 5 == 0 ){
                _s<<"Gazebo/Orange";
                } else if( x <= 4 && x % 4 == 0 ){
                _s<<"Gazebo/Indigo";
                } else if( x <= 3 && x % 3 == 0 ){
                _s<<"Gazebo/Turquoise";
		} else if( x <= 2 && x % 2 == 0 ){
                _s<<"Gazebo/SkyBlue";
		} else if( x <= 1 && x % 1 == 0 ){
                _s<<"Gazebo/Blue";
		}
       _s<<"</material>\
  </gazebo>";

	return _s;



}







