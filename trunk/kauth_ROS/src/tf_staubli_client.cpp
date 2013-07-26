// Included to use the ROS messaging system
#include <ros/ros.h>
#include <kautham/tf_server.h>
//#include <tf/transform_datatypes.h>

#include <mt/rotation.h>

// Included to use the shared memory between the Kautham and the publisher
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

// Include the memory share object definition
#include "data_ioc_cell.hpp"

#include <sstream>
#include <iostream>
#include <algorithm>
#include <iterator>

using namespace kautham;
using namespace boost::interprocess;

int main(int argc, char **argv){

//  //Open a shared memory object.
//   shared_memory_object shm(open_only,                      //only open
//                            "KauthamSharedMemory",          //name
//                            read_write                      //read-write mode
//   );

//  //Set size
//  shm.truncate(sizeof(data_ioc_cell));

//  //Map the whole shared memory in this process
//  mapped_region region( shm,         //What to map
//                        read_write   //Map it as read-write
//  );

//  //Get the address of the mapped region
//  void * addr       = region.get_address();

//  //Construct the shared structure in memory
//  data_ioc_cell* data = new (addr) data_ioc_cell;

  //  The amount of parameters should be 6.
  //  If the parameter are 6, it means they are x,y,z,\alpha, \beta, \gama.
  //  If the parameters are 7 it means they are pos and quaternion.
  ros::V_string   _vargv;
  _vargv.clear();
  ros::removeROSArgs(argc, argv, _vargv);
  double x, y, z, alpha, beta, gamma;

//  tf::Quaternion quat;
  mt::Rotation rot;

  if( _vargv.size() == 7 ){
    x = atof( _vargv[1].c_str() );
    y = atof( _vargv[2].c_str() );
    z = atof( _vargv[3].c_str() );
    alpha = atof( _vargv[4].c_str() );
    beta = atof( _vargv[5].c_str() );
    gamma = atof( _vargv[6].c_str() );
    rot.setYpr(gamma, beta, alpha);

  }else
    return -1;

  ros::init(argc, argv, "tf_staubli_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<tf_server>("tf_staubli_soap");
  tf_server srv_message;

  ros::Rate loop_rate(10);

  while( ros::ok() ){
    srv_message.request.tcp.header.stamp = ros::Time::now();
    srv_message.request.tcp.transform.translation.x = x;
    srv_message.request.tcp.transform.translation.y = y;
    srv_message.request.tcp.transform.translation.z = z;
//    tf::quaternionTFToMsg( quat, srv_message.request.tcp.transform.rotation );
    srv_message.request.tcp.transform.rotation.x = rot.at(0);
    srv_message.request.tcp.transform.rotation.y = rot.at(1);
    srv_message.request.tcp.transform.rotation.z = rot.at(2);
    srv_message.request.tcp.transform.rotation.w = rot.at(3);

    if( client.call( srv_message ) ){
      std::stringstream ss;
      std::copy( srv_message.response.joints.position.begin(),
                 srv_message.response.joints.position.end(),
                 std::ostream_iterator<float>(ss, " ")
      );
      ROS_INFO("Joint values: %s", ss.str().c_str() );
    }

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

