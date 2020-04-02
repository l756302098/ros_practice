#ifndef VECTOR3_UTILS_VALUES_H_
#define VECTOR3_UTILS_VALUES_H_
#include <math.h>

using namespace std;

class Vector3
{
private:
public:
    Vector3() : x(0), y(0), z(0)
    {
    }
    Vector3(float x_, float y_, float z_)
    {
        x = x_;
        y = y_;
        z = z_;
    }
    float x;
    float y;
    float z;

    Vector3 operator-() const { return Vector3(-x, -y, -z); }
    Vector3 operator*(float a /*标量*/) const
    {
        return Vector3(x * a, y * a, z * a);
    }
    Vector3 operator/(float a) const
    {
        float temp = 1.0f / a;
        return Vector3(x * temp, y * temp, z * temp);
    }
    void zero();
    static float distance(Vector3 p1, Vector3 p2){
        float i = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
        return i;
    }
    float magnitude();
    Vector3 normalized();
    Vector3 vertical();
};

#endif