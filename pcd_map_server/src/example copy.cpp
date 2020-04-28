//
// Created by Fanzhe on 5/29/2017.
//
#include "ros/ros.h"
#include "modbus.h"


int main(int argc, char **argv)
{

    ros::init(argc, argv, "modbus_exm");

    ros::NodeHandle n;

    int num = 254;
    int remainder,integer = 0;
    int time = 8;
    int c_num = 0;
    int data[8];
    while (time>0)
    {   
        c_num = 2 << (time-1);
        integer = num / c_num;
        num =  num % c_num;
        data[time-1] = integer;
        time--;
    }
    for (int i = 0; i < 8; i++)
    {
        ROS_INFO("%i : %x",i,data[i]);
    }
    
    /*
    // create a modbus object
    modbus mb = modbus("192.168.1.174", 502);
     // set slave id
    mb.modbus_set_slave_id(1);
    // connect with the server
    mb.modbus_connect();

    //mb.modbus_write_coil(1, true);
    //mb.modbus_write_coil(1, false);
    */
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        /*
    std::cout << "------------------input------------------"<< std::endl; 
    bool read_bits;
    mb.modbus_read_input_bits(0, 5, &read_bits);
    for(int i = 0; i < 5; i++) {
        std::cout << "read_bits:" << read_bits++ << std::endl; 
    }

    std::cout << "------------------output------------------"<< std::endl; 
    // read coil                        function 0x01
    bool read_coil;
    mb.modbus_read_coils(0, 5, &read_coil);
    for(int i = 0; i < 5; i++) {
        std::cout << "read_coil:" << read_coil++ << std::endl; 
    } 
    */
    ros::spinOnce();

    loop_rate.sleep();

    }

    // close connection and free the memory
    // mb.modbus_close();
    // delete(&mb);
    return 0;
}
