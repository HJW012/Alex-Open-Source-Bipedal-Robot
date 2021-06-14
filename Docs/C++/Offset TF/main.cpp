#include <iostream>
#include <string>
#include <map>
#include <math.h>
#include <stdlib.h>
#include <vector>

using namespace std;

typedef double tfScalar;

struct Quaternion {
    double w = 0;
    double x = 0;
    double y = 0;
    double z = 0;
};

struct Vector3 {
    double x = 0;
    double y = 0;
    double z = 0;
};

struct Transform {
    Vector3 translation;
    Quaternion rotation;
};

struct Header {
    int seq = 0;
    int stamp = 0;
    string frame_id = "";
};

struct TransformStamped {
    Header header;
    string child_frame_id = "";
    Transform transform;
};

Quaternion setRPY(const tfScalar& roll, const tfScalar& pitch, const tfScalar& yaw) {
    Quaternion q;

    tfScalar halfYaw = tfScalar(yaw) * tfScalar(0.5);
    tfScalar halfPitch = tfScalar(pitch) * tfScalar(0.5);
    tfScalar halfRoll = tfScalar(roll) * tfScalar(0.5);
    tfScalar cosYaw = cos(halfYaw);
    tfScalar sinYaw = sin(halfYaw);
    tfScalar cosPitch = cos(halfPitch);
    tfScalar sinPitch = sin(halfPitch);
    tfScalar cosRoll = cos(halfRoll);
    tfScalar sinRoll = sin(halfRoll);
    q.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw; //x
    q.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw; //y
    q.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw; //z
    q.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw; //formerly yzx

    return q;
}

void getRPY(Quaternion q, double & roll, double & pitch, double & yaw) {
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (abs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

        // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    yaw = atan2(siny_cosp, cosy_cosp);
}

bool GetRelativeTF(map<string, TransformStamped> transformMap, TransformStamped childTF, TransformStamped parentTF, string baseTF, TransformStamped & relativeTF) {
    bool chainFound = false;
    string currentParent = "";
    TransformStamped currentTF = childTF;
    vector<TransformStamped> chain;
    TransformStamped resultingTF;


    if (transformMap.find(childTF.child_frame_id) == transformMap.end() || transformMap.find(parentTF.header.frame_id) == transformMap.end()) {
        return false;
    }
    while (currentParent != baseTF) {
        currentParent = currentTF.header.frame_id;
        if (currentParent != baseTF) {
            if (transformMap.find(currentParent) == transformMap.end()) {
                return false;
            }
            chain.push_back(currentTF);
            currentTF = transformMap.at(currentParent);
        }
    }
    chainFound = true;

    for (int i = 1; i < chain.size(); i++) {
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
        if (i-1 != 0) {
            getRPY(chain.at(i - 1).transform.rotation, roll, pitch, yaw);
        }
        resultingTF.transform.translation.x = resultingTF.transform.translation.x + chain.at(i).transform.translation.x * cos(roll);
        resultingTF.transform.translation.x = resultingTF.transform.translation.x + chain.at(i).transform.translation.x * cos(roll);
        resultingTF.transform.translation.x = resultingTF.transform.translation.x + chain.at(i).transform.translation.x * cos(roll);
    }

    return chainFound;
}

Quaternion rotAdd(Quaternion q1, Quaternion q2) {
    Quaternion q3;
    q3.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    q3.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    q3.y = q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z;
    q3.z = q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x;

    return q3;
}



int main()
{
    TransformStamped TF1, TF2, TF3, TF4, TF5;
    TF1.header.frame_id = "base_link";
    TF1.child_frame_id = "TF1";
    TF1.transform.translation.x = 0;
    TF1.transform.translation.y = 100;
    TF1.transform.translation.z = -50;
    //TF1.transform.rotation = setRPY()

    TF2.header.frame_id = "TF1";
    TF2.child_frame_id = "TF2";
    TF2.transform.translation.x = 100;
    TF2.transform.translation.y = 0;
    TF2.transform.translation.z = 0;


    TF3.header.frame_id = "TF2";
    TF3.child_frame_id = "TF3";
    TF3.transform.translation.x = 200;
    TF3.transform.translation.y = 0;
    TF3.transform.translation.z = 0;


    TF4.header.frame_id = "TF3";
    TF4.child_frame_id = "TF4";
    TF4.transform.translation.x = 50;
    TF4.transform.translation.y = 0;
    TF4.transform.translation.z = 0;


    TF5.header.frame_id = "TF4";
    TF5.child_frame_id = "TF5";
    TF5.transform.translation.x = 150;
    TF5.transform.translation.y = 0;
    TF5.transform.translation.z = 0;


    map<string, TransformStamped> transformMap;
    transformMap.insert(make_pair(TF1.child_frame_id, TF1));
    transformMap.insert(make_pair(TF2.child_frame_id, TF2));
    transformMap.insert(make_pair(TF3.child_frame_id, TF3));
    transformMap.insert(make_pair(TF4.child_frame_id, TF4));
    transformMap.insert(make_pair(TF5.child_frame_id, TF5));


    Quaternion test = setRPY(M_PI/4, M_PI/4, M_PI/4);


    Quaternion q1;
    q1.w = -0.877;
    q1.x = 0.118;
    q1.y = -0.441;
    q1.z = 0.147;
    Quaternion q2;
    q2.w = 0.893;
    q2.x = 0.11;
    q2.y = -0.413;
    q2.z = 0.138;

    Quaternion q3 = rotAdd(q1, q2);
    q3 = rotAdd(q3, q2);
    cout << "W: " << q3.w << endl;
    cout << "X: " << q3.x << endl;
    cout << "Y: " << q3.y << endl;
    cout << "Z: " << q3.z << endl;

    return 0;
}
