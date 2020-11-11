
#include <math.h>
#include <Vector3.h>

using namespace std;


void Vector3::zero()
{
    x = y = z = 0.0f;
}
float Vector3::magnitude()
{
    return sqrt(x * x + y * y + z * z);
}
Vector3 Vector3::normalized()
{
    float magsq = magnitude();
    float temp = 1 / sqrt(magsq);
    return Vector3(x * temp, y * temp, z * temp);
}
Vector3 Vector3::vertical()
{
    return Vector3(-y / x, 1, 0);
}