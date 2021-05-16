#include <alex_global/global_definitions.h>

// Kinematic Functions
void fkine();
void legFkine();
void ikine();
void hipikine();
void kneeIkine();
void ankleIkine();

tf2::Quaternion rotAdd(tf2::Quaternion q1a, tf2::Quaternion q2a) {
  geometry_msgs::Quaternion q1 = quatConversion(q1a);
  geometry_msgs::Quaternion q2 = quatConversion(q2a);
  geometry_msgs::Quaternion q3 = rotAdd(q1, q2);
  tf2::Quaternion q3a = quatConversion(q3);;
  return q3a;
}

geometry_msgs::Quaternion rotAdd(geometry_msgs::Quaternion q1, geometry_msgs::Quaternion q2) {
  geometry_msgs::Quaternion q3;
  q3.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
  q3.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
  q3.y = q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z;
  q3.z = q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x;

  return q3;
}

geometry_msgs::TransformStamped getOffsetTF(geometry_msgs::TransformStamped TF1, geometry_msgs::TransformStamped TF2, std::map<std::string, geometry_msgs::TransformStamped> transformMap) {
  //Get rotation matrix
  // First find resulting rotation
  std::cout << "TF1: " << TF1.transform.translation.x << ", " << TF1.transform.translation.y << ", " << TF1.transform.translation.z << std::endl;
  std::cout << "TF2: " << TF2.transform.translation.x << ", " << TF2.transform.translation.y << ", " << TF2.transform.translation.z << std::endl;
  geometry_msgs::Quaternion qR = rotAdd(transformMap[TF1.header.frame_id].transform.rotation, transformMap[TF2.header.frame_id].transform.rotation);
  tf2::Vector3 v1 = tf2::quatRotate(quatConversion(transformMap[TF1.header.frame_id].transform.rotation), vector3Conversion(TF1.transform.translation));
  tf2::Vector3 v2 = tf2::quatRotate(quatConversion(qR), vector3Conversion(TF2.transform.translation));
  std::cout << "v1: " << v1.x() << ", " << v1.y() << ", " << v1.z() << std::endl;
  std::cout << "v2: " << v2.x() << ", " << v2.y() << ", " << v2.z() << std::endl;

  geometry_msgs::TransformStamped TFr;
  TFr.transform.translation.x = v2.x() - v1.x();
  TFr.transform.translation.y = v2.y() - v1.y();
  TFr.transform.translation.z = v2.z() - v1.z();

  return TFr;
}

bool Relative_Distance_In_Tree(std::map<std::string, geometry_msgs::TransformStamped> transformMap, std::string childName1, std::string childName2, std::string baseName, double & result) {
  geometry_msgs::TransformStamped TF1;
  geometry_msgs::TransformStamped TF2;
  if (Relative_TF_In_Chain(transformMap, childName1, baseName, baseName, TF1) && Relative_TF_In_Chain(transformMap, childName2, baseName, baseName, TF2)) {
    result = distance(TF1.transform.translation.x, TF1.transform.translation.y, TF1.transform.translation.z, TF2.transform.translation.x, TF2.transform.translation.y, TF2.transform.translation.z);
    return true;
  } else {
    return false;
  }
}

bool Relative_TF_In_Chain(std::map<std::string, geometry_msgs::TransformStamped> transformMap, std::string childName, std::string parentName, std::string baseName, geometry_msgs::TransformStamped & resultTF) {
  bool chainFound = false;
  bool relativeFromBase = false;
  std::string currentParent = "";
  geometry_msgs::TransformStamped currentTF;
  std::vector<geometry_msgs::TransformStamped> chain;

  // Check if parentName is baseName
  if (parentName == baseName) {
    relativeFromBase = true;
  }

  // Check if TFs exist
  if (transformMap.find(childName) == transformMap.end()){
    // childTF doesn't exist
    return false;
  }

  if (!relativeFromBase && transformMap.find(parentName) == transformMap.end()) {
    // ParentTF doesn't exist
    return false;
  }

  // Check if TFs are in a chain
  currentTF = transformMap[childName];
  while (transformMap.find(currentTF.child_frame_id) != transformMap.end()) {
    chain.insert(chain.begin(), currentTF);

    // If currentTF parent has parentName as its parent, leave loop

    // Check that parent exists
    if (!relativeFromBase) {
      if (currentTF.header.frame_id == parentName) {
        chainFound = true;
        resultTF = currentTF;
        return true;
      }

      if (transformMap.find(currentTF.header.frame_id) == transformMap.end()) {
        // Reached the end of the chain without finding the parentName
        return false;
      }
    } else {
      if (currentTF.header.frame_id == parentName) {
        chainFound = true;
        break;
      }

      if (transformMap.find(currentTF.header.frame_id) == transformMap.end()) {
        // Reached the end of the chain without finding the parentName
        return false;
      }
    }
    currentTF = transformMap.at(currentTF.header.frame_id);
  }

  // Unrotated quaternion
  // tf2::Quaternion runningQuaternion = quatConversion(chain.at(0).transform.rotation);
  // tf2::Vector3 runningVector = quatRotate(runningQuaternion, vector3Conversion(chain.at(1).transform.translation));
  tf2::Quaternion runningQuaternion(0, 0, 0, 1);
  tf2::Vector3 runningVector(0, 0, 0);

  for (int i = 0; i < chain.size() - 1; i++) {
    runningQuaternion = rotAdd(runningQuaternion, quatConversion(chain.at(i).transform.rotation));
    tf2::Vector3 currentVector = tf2::quatRotate(runningQuaternion, vector3Conversion(chain.at(i+1).transform.translation));
    double dotProduct = runningVector.dot(currentVector); // Magnitude of dot product
    runningVector += currentVector;
  }
  resultTF.transform.translation = vector3Conversion(runningVector);
  return true;
}

bool Relative_Quaternion_In_Chain(std::map<std::string, geometry_msgs::TransformStamped> transformMap, std::string childName, std::string parentName, std::string baseName, geometry_msgs::Quaternion & resultQuaternion) {

}


// General Functions
double distance(double x1, double y1, double x2, double y2) {
  double dist = abs(sqrt(pow((x1 - x2), 2) + pow(y1 - y2, 2)));
  return dist;
}

void quadraticFormula(double a, double b, double c, std::vector<double> & x) {
  x.push_back((-b + sqrt(pow(b, 2) - 4 * a * c)) / 2 * a);
  x.push_back((-b - sqrt(pow(b, 2) - 4 * a * c)) / 2 * a);
}


double distance(double x1, double y1, double z1, double x2, double y2, double z2) {
  double dist = abs(sqrt(pow((x1 - x2), 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2)));
  return dist;
}

double distance(geometry_msgs::TransformStamped tf1, geometry_msgs::TransformStamped tf2) {
  double dist = abs(sqrt(pow((tf1.transform.translation.x - tf2.transform.translation.x), 2) + pow(tf1.transform.translation.y - tf2.transform.translation.y, 2) + pow(tf1.transform.translation.z - tf2.transform.translation.z, 2)));
  return dist;
}

double angleCosineRule(double a, double b, double c) {
  double A = acos((pow(b, 2) + pow(c, 2) - pow(a, 2)) / (2 * b * c));
  return A;
}

double sideCosineRule(double b, double c, double A) {
  double a = sqrt(pow(b, 2) + pow(c, 2) - 2 * b * c * cos(A));
  return a;
}

tf2::Quaternion quatConversion(geometry_msgs::Quaternion q) {
  tf2::Quaternion Q(q.x, q.y, q.z, q.w);
  return Q;
}

geometry_msgs::Quaternion quatConversion(tf2::Quaternion q) {
  geometry_msgs::Quaternion Q;
  Q.x = q.x();
  Q.y = q.y();
  Q.z = q.z();
  Q.w = q.w();
  return Q;
}

tf2::Vector3 vector3Conversion(geometry_msgs::Vector3 v) {
  tf2::Vector3 vr(v.x, v.y, v.z);
  return vr;
}

geometry_msgs::Vector3 vector3Conversion(tf2::Vector3 v) {
  geometry_msgs::Vector3 vr;
  vr.x = v.x();
  vr.y = v.y();
  vr.z = v.z();
  return vr;
}

geometry_msgs::Quaternion setRPY(tf2Scalar& roll, tf2Scalar& pitch, tf2Scalar& yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return quatConversion(q);
}

void getRPY(tf2::Quaternion q, double& roll, double& pitch, double& yaw) {
  tf2::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}

void getRPY(geometry_msgs::Quaternion q, double& roll, double& pitch, double& yaw) {
  tf2::Quaternion Q = quatConversion(q);
  getRPY(Q, roll, pitch, yaw);
}

// CAN Functions
float constrain(float in, float min, float max) {
  if (in >= max) {
     in = max;
  }
  if (in <= min)  {
     in = min;
  }

  return in;
}

unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
   float span = x_max - x_min;
   float offset = x_min;
   unsigned int pgg = 0;
   if (bits == 12) {
      pgg = (unsigned int) ((x - offset) * 4095.0 / span);
   }
   if (bits == 16) {
      pgg = (unsigned int) ((x - offset) * 65535.0 / span);
   }

   return pgg;
}

float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
  /// Converts unsigned int to float, given range and number of
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12) {
    pgg = ((float) x_int) * span / 4095.0 + offset;
  }
  if (bits == 16) {
    pgg = ((float) x_int) * span / 65535.0 + offset;
  }
  return pgg;
}

bool getRosParams(ros::NodeHandle nh) {
  bool paramsExist = true;
  // legs
  paramsExist &= nh.getParam("legs/leftLeg/Enabled", leftLeg.Enabled);
  paramsExist &= nh.getParam("legs/leftLeg/M1Enabled", leftLeg.M1.Enabled);
  paramsExist &= nh.getParam("legs/leftLeg/M2Enabled", leftLeg.M2.Enabled);
  paramsExist &= nh.getParam("legs/leftLeg/M3Enabled", leftLeg.M3.Enabled);
  paramsExist &= nh.getParam("legs/leftLeg/M1ID", leftLeg.M1.ID);
  paramsExist &= nh.getParam("legs/leftLeg/M2ID", leftLeg.M2.ID);
  paramsExist &= nh.getParam("legs/leftLeg/M3ID", leftLeg.M3.ID);

  paramsExist &= nh.getParam("legs/rightLeg/Enabled", rightLeg.Enabled);
  paramsExist &= nh.getParam("legs/rightLeg/M1Enabled", rightLeg.M1.Enabled);
  paramsExist &= nh.getParam("legs/rightLeg/M2Enabled", rightLeg.M2.Enabled);
  paramsExist &= nh.getParam("legs/rightLeg/M3Enabled", rightLeg.M3.Enabled);
  paramsExist &= nh.getParam("legs/rightLeg/M1ID", rightLeg.M1.ID);
  paramsExist &= nh.getParam("legs/rightLeg/M2ID", rightLeg.M2.ID);
  paramsExist &= nh.getParam("legs/rightLeg/M3ID", rightLeg.M3.ID);

  // system
  paramsExist &= nh.getParam("system/UseMPU6050", systemSettings.UseMPU6050);
  paramsExist &= nh.getParam("system/UseORIENTUS", systemSettings.UseORIENTUS);
  paramsExist &= nh.getParam("system/AverageIMU", systemSettings.AverageIMU);
  paramsExist &= nh.getParam("system/IMUID1", systemSettings.IMUID1);
  paramsExist &= nh.getParam("system/IMUID2", systemSettings.IMUID2);

  return paramsExist;
}
