#include <alex_global/global_definitions.h>

// Kinematic Functions
void fkine();
void legFkine();
void ikine();
void hipikine();
void kneeIkine();
void ankleIkine();

// General Functions
double distance(double x1, double y1, double x2, double y2) {
  double dist = abs(sqrt(pow((x1 - x2), 2) + pow(y1 - y2, 2)));
  return dist;
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
