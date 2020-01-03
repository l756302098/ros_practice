#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include<iostream>
#include<fstream>
#include<math.h>


using namespace std;

namespace yd_vector3{
class Vector3
{
private:

public:
Vector3():x(0),y(0),z(0){

}
Vector3(float x_,float y_,float z_){
x= x_;
y=y_;
z=z_;
}
 float x;
 float y;
 float z;
 static float TwoPointDistance3D(Vector3 p1, Vector3 p2);

};

float Vector3::TwoPointDistance3D(Vector3 p1, Vector3 p2){
 float i =  sqrt((p1.x-p2.x) * (p1.x-p2.x)
                            + (p1.y - p2.y) * (p1.y - p2.y)
                            + (p1.z - p2.z) * (p1.z - p2.z));

        return i;
}


}