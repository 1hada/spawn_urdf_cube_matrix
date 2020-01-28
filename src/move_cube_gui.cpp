/*
 * File : move_cube_gui.cpp
 *
 * Author : Jonathan Sanabria ( 2020 )
 *
 * Description : 
 *
 *		An application that uses GTK ( Gimp Toolkit ) to change the position of a gazebo model
 *		in space. The "movement" from this program comes from a a given models roll pitch and
 *		yaw. 
 *
 *
 */
#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <gazebo_msgs/SetModelState.h>
#include <tf/transform_datatypes.h>

#include <string>
#include <sstream>
#define _USE_MATH_DEFINES
#include <math.h>

#include <iostream>       // std::cout, std::endl
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds

using namespace std; 

#include <gtk/gtk.h>

GtkWidget *window;
GtkWidget *fixed1; 
gint width = 60 ;
gint height = 60 ;
//
//
static void set_rpy( string cb_origin ) ;
static void activate (GtkApplication *app, gpointer user_data);
// INCREMENT
static void pitch_cb_inc (GtkWidget *widget,gpointer   data){ set_rpy ( "+pitch" ) ;}
static void  roll_cb_inc (GtkWidget *widget,gpointer   data){ set_rpy ( "+roll" ) ;}
static void   yaw_cb_inc (GtkWidget *widget,gpointer   data){ set_rpy ( "+yaw" ) ;}
// DECREMENT
static void pitch_cb_dec (GtkWidget *widget,gpointer   data){ set_rpy ( "-pitch" ) ;}
static void  roll_cb_dec (GtkWidget *widget,gpointer   data){ set_rpy ( "-roll" ) ;}
static void   yaw_cb_dec (GtkWidget *widget,gpointer   data){ set_rpy ( "-yaw" ) ;}

static double global_pitch = 0;
static double global_roll  = 0;
static double global_yaw   = 0;
/**

g++ -Wall -Wno-format ~/catkin_ws/src/spawn_urdf_cube_matrix/src/move_cube_gui.cpp -o ~/catkin_ws/src/spawn_urdf_cube_matrix/src/execute_move_cube -I/opt/ros/melodic/include -L/opt/ros/melodic/lib -lroscpp -lrostime -lrosconsole -lroscpp_serialization -Wno-deprecated-declarations -Wno-format-security -lm `pkg-config --cflags --libs gtk+-3.0` -export-dynamic


 */


#define     RADIAN_STEP M_PI/48

/**
 * Initializes the ROS middleware to allow use of the NodeHandle.
 * Creates a new GTK application.
 *	
 * @param int argc 		
 * @param char **argv 			 		 
 * @return int
 */ 
int main(int argc, char **argv)
{

  ros::init(argc, argv, "a_matrix");

  GtkApplication *app;

  app = gtk_application_new ("RollPitchYaw.Controller", G_APPLICATION_FLAGS_NONE);
  g_signal_connect (app, "activate", G_CALLBACK (activate), NULL);
  g_application_run (G_APPLICATION (app), argc, argv);
  g_object_unref (app);

  return 0;
}

/**
 * Modifies the state of the model depending on the GTK callback which invoked the function.
 *	
 * @param string cb_origin			--- Used in order to determine what action to apply on the pose. 		
 * @return void
 */ 
static void set_rpy ( string cb_origin ){

  ros::NodeHandle nh;

  geometry_msgs::Pose the_pose;
  gazebo_msgs::ModelState modelstate;

  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState setmodelstate;

  modelstate.model_name = (string) "a_matrix";
  modelstate.reference_frame = (string) "world";
  if      ( 0==cb_origin.compare("+pitch" )){ global_pitch += RADIAN_STEP ;
  }else if( 0==cb_origin.compare("+roll" )){  global_roll  += RADIAN_STEP ;
  }else if( 0==cb_origin.compare("+yaw" )){   global_yaw   += RADIAN_STEP ;
  }else if( 0==cb_origin.compare("-pitch" )){ global_pitch -= RADIAN_STEP ;
  }else if( 0==cb_origin.compare("-roll" )){  global_roll  -= RADIAN_STEP ;
  }else if( 0==cb_origin.compare("-yaw" )){   global_yaw   -= RADIAN_STEP ;
  }
  the_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw ( global_roll , global_pitch , global_yaw ); // r p y

  modelstate.pose = the_pose;

  setmodelstate.request.model_state = modelstate;
  client.call(setmodelstate);
  cout << "roll : "<< global_roll << " pitch : "<< global_pitch<<" yaw  : "<< global_yaw  << endl;
}

/**
 * Creates the main gui which controls the model.
 *	
 * @param GtkApplication *app           ---
 * @gpointer user_data			--- 
 * @return void
 */ 
static void activate (GtkApplication *app, gpointer user_data){

  GtkWidget *grid;
  GtkWidget *button;

  /* create a new window, and set its title */
  window = gtk_application_window_new (app);
  gtk_window_set_title (GTK_WINDOW (window), "RPY ");
  gtk_container_set_border_width (GTK_CONTAINER (window), 10);

  /* Here we construct the container that is going pack our buttons */
  grid = gtk_grid_new ();

  /* Pack the container in the window */
  gtk_container_add (GTK_CONTAINER (window), grid);

  button = gtk_button_new_with_label ("+ R");
  g_signal_connect (button, "clicked", G_CALLBACK (roll_cb_inc), NULL);
  gtk_grid_attach (GTK_GRID (grid), button, 0, 0, 1, 1);

  button = gtk_button_new_with_label ("- R");
  g_signal_connect (button, "clicked", G_CALLBACK (roll_cb_dec), NULL);
  gtk_grid_attach (GTK_GRID (grid), button, 0, 1, 1, 1);

  button = gtk_button_new_with_label ("+ P");
  g_signal_connect (button, "clicked", G_CALLBACK (pitch_cb_inc), NULL);
  gtk_grid_attach (GTK_GRID (grid), button, 1, 0, 1, 1);

  button = gtk_button_new_with_label ("- P");
  g_signal_connect (button, "clicked", G_CALLBACK (pitch_cb_dec), NULL);
  gtk_grid_attach (GTK_GRID (grid), button, 1, 1, 1, 1);

  button = gtk_button_new_with_label ("+ Y");
  g_signal_connect (button, "clicked", G_CALLBACK (yaw_cb_inc), NULL);
  gtk_grid_attach (GTK_GRID (grid), button, 2, 0, 1, 1);

  button = gtk_button_new_with_label ("- Y");
  g_signal_connect (button, "clicked", G_CALLBACK (yaw_cb_dec), NULL);
  gtk_grid_attach (GTK_GRID (grid), button, 2, 1, 1, 1);

  gtk_widget_show_all (window );
							

}







