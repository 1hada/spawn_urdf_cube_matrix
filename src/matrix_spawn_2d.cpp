/*
 * File : matrix_spawn_2d.c
 *
 * Author : Jonathan Sanabria
 *
 * Description : 
 *
 *		An application that spawns a layer of cubes in gazebo. The resulting dimension of 
 *		the matrix layer will be defined by " END_XY_DIMENSION " multiplied by 2.
 *
 *
 */


#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/SpawnModel.h>
#include <tf/transform_datatypes.h>

#include <stdlib.h> 
#include<iostream> 
#include <sstream>      // std::stringstream
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */



/* 

g++ matrix_spawn_2d.cpp -o a_matrix -I/opt/ros/melodic/include -L/opt/ros/melodic/lib -Wl,-rpath,/opt/ros/melodic/lib -lroscpp -lrosconsole -lrostime -lroscpp_serialization

*/

int END_XY_DIMENSION = 50 ; // example : 50 == 100x100 ; 
int offset_scalar  = 2 ; //affect distance between each cube
std::stringstream get_box_at(double mass , double length , int x , int y , int z );
std::stringstream get_matrix(double mass , double length , int x , int y , int z ); 


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
int main(int argc, char **argv ){

    clock_t timer= clock();

    std::stringstream geometry_template;
    std::stringstream origin_template ;
    std::stringstream cur_link_name ;
    std::stringstream main_model_name ;
    main_model_name << "a_matrix" ;
    double length = 0.2 ;
    double mass = 10 ;
    double inertia_value = mass / 12.0 * (length*length + length*length) ;
    int   start = -1*END_XY_DIMENSION , x = start , y = start , z = 0 ;

    geometry_template <<"<geometry>\
        			<box size=\""<<length<<" "<<length<<" "<<length<<"\"/>\
     			 </geometry>" ;
    origin_template << "<origin rpy=\"0 0 0\" xyz=\""<<x*offset_scalar*length<<" "<<y*offset_scalar*length<<" "<<z*offset_scalar*length<<"\"/>" ;  
    cur_link_name   << "box_at_"<<x<<"_"<<y<<"_"<<z ;

    ros::init(argc, argv, "a_matrix");
    // Node setup
    ros::NodeHandle nh;

    std::stringstream ifs ;
    ifs.str(std::string());

    ifs << "<?xml version=\"1.0\" ?>\
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
    <origin rpy=\"0 0 0\" xyz=\""<<length<<" "<<length<<" "<<0<<"\"/>\
    <child link=\""<<cur_link_name.str()<<"\"/>\
  </joint>\
  <gazebo reference=\""<<cur_link_name.str()<<"\">\
    <turnGravityOff>true</turnGravityOff>\
    <material>Gazebo/Black</material>\
  </gazebo>";

    ifs << get_matrix( mass , length , x , y , z ).str() ;

    ifs << "</robot>";

    ROS_INFO("ros::ServiceClient spawn "); 
    ros::ServiceClient spawn = nh.serviceClient< gazebo_msgs::SpawnModel> ("/gazebo/spawn_urdf_model");

    gazebo_msgs::SpawnModel s1;
    geometry_msgs::Pose pose;
    // declare main model name
    s1.request.model_name = main_model_name.str(); 

    ROS_INFO("_s >> s1.request.model_xml "); 
    s1.request.model_xml = ifs.str();
    s1.request.robot_namespace = "a_matrix";
    /*
    pose.position.x=0;
    pose.position.y=0;
    pose.position.z=0;
    pose.orientation.x=0;
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
    if (!spawn.call(s1)) 
    {
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
std::stringstream get_matrix(double mass , double length , int x , int y , int z ){

    std::stringstream _s ;
    int loop_count = 0;
    while( ++x < END_XY_DIMENSION ){
        while( y < END_XY_DIMENSION ){
              _s << get_box_at( mass , length , x , y , z ).str() ;
          y++;
        } y = -1*END_XY_DIMENSION ; 
    } 
    x = -1*END_XY_DIMENSION ;
    while( ++y < END_XY_DIMENSION ){
          _s << get_box_at( mass , length , x , y , z ).str() ;
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
std::stringstream get_box_at(double mass , double length , int x , int y , int z ){
    std::stringstream geometry_template;
    std::stringstream origin_template ;
    std::stringstream cur_link_name ;
    geometry_template <<"<geometry>\
        			<box size=\""<<length<<" "<<length<<" "<<length<<"\"/>\
     			 </geometry>" ;
    origin_template << "<origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>" ;
    cur_link_name   << "box_at_"<<x<<"_"<<y<<"_"<<z ;
    double inertia_value = mass / 12.0 * (length*length + length*length) ;

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
    <parent link=\"box_at_"<<-1*END_XY_DIMENSION<<"_"<<-1*END_XY_DIMENSION<<"_"<<0<<"\"/>\
    <origin rpy=\"0 0 0\" xyz=\""<<x*offset_scalar*length<<" "<<y*offset_scalar*length<<" "<<z*offset_scalar*length<<"\"/>\
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









