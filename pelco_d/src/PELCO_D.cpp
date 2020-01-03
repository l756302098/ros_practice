#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>

#include "serial.h"

using namespace std;

int main(int argc, char const *argv[])
{
    if (argc < 3)
    {
        fprintf(stderr, "Usage: %s <device> <baudrate>\n", argv[0]);
        fprintf(stderr, "Example: %s /dev/ttyACM0 115200\n", argv[0]);
        exit(0);
    }

    Serial device = Serial(argv[1], atoi(argv[2]));

    string input = "";
    char output[512] = "";

    while (true)
    {
        cout << "input> ";
        //test
        char code[7] = {0xff, 0x01, 0x00, 0x08, 0x00, 0xff, 0x08};
        device.Write(code, 7);

        //`cin` tokenizes on whitespace, use getline instead
        getline(cin, input);

        ssize_t w_bytes = device.Write(input.c_str(), input.size());
        // sleep long enough that the Arduino has enough time to respond
        usleep(5000);
        ssize_t r_bytes = device.Read(output, sizeof(output));
        // size_t r_bytes = arduino.ReadBlocking(output, 4);

        cout << "wrote " << w_bytes << "> " << input << endl;
        cout << "read " << r_bytes << "> " << output << endl;
    }

    return 0;
}