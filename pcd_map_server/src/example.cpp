//
// Created by Fanzhe on 5/29/2017.
//
#include "ros/ros.h"
#include "modbus.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "modbus_exm");

    ros::NodeHandle n;

    // create a modbus object
    modbus mb = modbus("192.168.1.174", 502);
     // set slave id
    mb.modbus_set_slave_id(1);
    // connect with the server
    mb.modbus_connect();

    //mb.modbus_write_coil(1, true);
    //mb.modbus_write_coil(1, false);
    
    ros::Rate loop_rate(10);
    while (ros::ok())
    {

    std::cout << "------------------input------------------"<< std::endl; 
    int read_bits;
    mb.modbus_read_input_bits(0, 5, read_bits);
    std::cout << "read_bits:" << read_bits << std::endl; 

    std::cout << "------------------output------------------"<< std::endl; 
    // read coil                        function 0x01
    int read_coil;
    mb.modbus_read_coils(0, 5, read_coil);
    std::cout << "read_coil:" << read_coil << std::endl; 
    
    ros::spinOnce();

    loop_rate.sleep();

    }

    // close connection and free the memory
    mb.modbus_close();
    delete(&mb);
    return 0;
}
