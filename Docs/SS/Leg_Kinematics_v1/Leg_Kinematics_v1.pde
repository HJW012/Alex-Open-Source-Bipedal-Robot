int simWidth = 1280;
int simHeight = 720;
int scaleFactor = 1;

// Length values taken from leg concept #2 - for testing purposes
float l1 = 75;
float l2 = 110;
float l3 = 110;
float l4 = 75;
float l5 = 30;
float l6 = 22.563;
float l7 = 110;
float l8 = 105;
float l9 = 20; // end of link to end of link - not on foot
float l10 = 25; // from top corner of foot to middle link
float T1 = 22.563;
float T2 = 22.563;
float T3 = 20;
/* Leg concept 1
float l1 = 18;
float l2 = 80.777;
float l3 = 88.487;
float l4 = 22.78;
float l5 = 57.998;
float l6 = 18;
float l7 = 56.544;
float l8 = 80.777;
float l9 = 18; // end of link to end of link - not on foot
float l10 = 20; // from top corner of foot to middle link
float T1 = 24.233;
float T2 = 24.233;
float T3 = 18;
*/
float degRadConv = PI/180;
float radDegConv = 180/PI;

Point tempPoint = new Point(0, 0);

// All point locations
Point P0 = new Point(0, 0);
Point P1 = new Point(0, 0);
Point P2 = new Point(0, 0);
Point P3 = new Point(0, 0);
Point P4 = new Point(0, 0);
Point P5 = new Point(0, 0);
Point P6 = new Point(0, 0);
Point P7 = new Point(0, 0);
Point P8 = new Point(0, 0);
Point P9 = new Point(0, 0);

// Calculated variables from kinematic equations
Point[] calcFootPoints = {tempPoint, tempPoint, tempPoint};
float calcQ[] = {0, 0, 0};

void setup() {
  size(1280, 720);
  
  // FORWARD KINEMATICS
  println("--- Forward Kinematics ---");
  float[] q = {36.117 * degRadConv, 126.1 * degRadConv, 180 * degRadConv};
  calcFootPoints = fkine(q);
  /*
  print("P8: ");
  calcFootPoints[0].Println();
  
  print("P4: ");
  calcFootPoints[1].Println();
  
  print("P9: ");
  calcFootPoints[2].Println();
  */
  
  // INVERSE KINEMATICS
  println("--- Inverse Kinematics ---");
  Point footPoint1 = new Point(-0.019, -150.751);
  Point footPoint2 = new Point(19.981, -150.751);
  Point footPoint3 = new Point(44.981, -150.751);
  Point[] footPoints = {footPoint1, footPoint2, footPoint3};
  calcQ = ikine1(footPoints);
  print("q1: ");
  println(q[0] * radDegConv);
  
  print("q2: ");
  println(q[1] * radDegConv);
  
  print("q3: ");
  println(q[2] * radDegConv);
}

void draw() {
  background(255); 
  stroke(0);
  fill(0);
  
  int xOffset = simWidth/2;
  int yOffset = 500;
  
  // INVERSE KINEMATICS
  println("--- Inverse Kinematics ---");
  Point footPoint1 = new Point(-3.336, -181.507);
  Point footPoint2 = new Point(14.733, -172.933);
  Point footPoint3 = new Point(36.212, -162.742);
  Point[] footPoints = {footPoint1, footPoint2, footPoint3};
  calcQ = ikine1(footPoints);
  
  circle(P0.x + xOffset, P0.y + yOffset, 10);
  circle(P1.x + xOffset, P1.y + yOffset, 10);
  circle(P2.x + xOffset, P2.y + yOffset, 10);
  circle(P3.x + xOffset, P3.y + yOffset, 10);
  circle(P4.x + xOffset, P4.y + yOffset, 10);
  circle(P5.x + xOffset, P5.y + yOffset, 10);
  circle(P6.x + xOffset, P6.y + yOffset, 10);
  circle(P7.x + xOffset, P7.y + yOffset, 10);
  circle(P8.x + xOffset, P8.y + yOffset, 10);
  circle(P9.x + xOffset, P9.y + yOffset, 10);
  
  line(P0.x + xOffset, P0.y + yOffset, P1.x + xOffset, P1.y + yOffset);
  line(P1.x + xOffset, P1.y + yOffset, P3.x + xOffset, P3.y + yOffset);
  line(P0.x + xOffset, P0.y + yOffset, P2.x + xOffset, P2.y + yOffset);
  line(P2.x + xOffset, P2.y + yOffset, P3.x + xOffset, P3.y + yOffset);
  line(P3.x + xOffset, P3.y + yOffset, P4.x + xOffset, P4.y + yOffset);
  line(P0.x + xOffset, P0.y + yOffset, P5.x + xOffset, P5.y + yOffset);
  line(P5.x + xOffset, P5.y + yOffset, P6.x + xOffset, P6.y + yOffset);
  line(P6.x + xOffset, P6.y + yOffset, P2.x + xOffset, P2.y + yOffset);
  line(P6.x + xOffset, P6.y + yOffset, P7.x + xOffset, P7.y + yOffset);
  line(P7.x + xOffset, P7.y + yOffset, P2.x + xOffset, P2.y + yOffset);
  line(P7.x + xOffset, P7.y + yOffset, P8.x + xOffset, P8.y + yOffset);
  line(P8.x + xOffset, P8.y + yOffset, P4.x + xOffset, P4.y + yOffset);
  line(P4.x + xOffset, P4.y + yOffset, P9.x + xOffset, P9.y + yOffset);
}

Point[] fkine(float q[]) {
  Point[] footPoints = {tempPoint, tempPoint, tempPoint};
  
  P1.x = P0.x + l1 * cos(q[0]);
  P1.y = P0.y - l1 * sin(q[0]);
  P2.x = P0.x + l2 * cos(q[1]);
  P2.y = P0.y - l2 * sin(q[1]);
  
  float lr1 = distance(P1, P2);
  float gamma1 = cosineRule(l1, l2, lr1);
  float alpha = cosineRule(lr1, l1, l2);
  float sigma1 = PI - (alpha + gamma1);
  float gamma2 = cosineRule(l3, lr1, l4);
  float sigma2 = cosineRule(l4, l3, lr1);
  float epsilon = PI - (gamma2 + sigma2);
  float delta = atan((P1.y - P2.y)/(P1.x - P2.x));
  
  P3.x = P1.x - l3 * cos(delta + sigma2);
  P3.y = P1.y - l3 * sin(delta + sigma2); 
  //P3.x = P2.x + l4 * cos(gamma2 - delta); // Alternative to above - use to check calc
  //P3.y = P2.y - l4 * sin(gamma2 - delta);
  
  float mu = gamma2 - delta;
  
  P4.x = P3.x + l5 * cos(mu);
  P4.y = P3.y - l5 * sin(mu);
   
  P5.x = P0.x + l6 * cos(q[2]);
  P5.y = P0.y - l6 * sin(q[2]);
  
  // CHANGED FROM SHEET DUE TO MISTAKE 
  float lr4 = distance(P5, P2);
  float A1 = cosineRule(l6, lr4, l2);
  float A2 = cosineRule(l7, T2, lr4);
  
  P6.x = P2.x + T2 * cos(delta + gamma1 + A1 + A2);
  P6.y = P2.y + T2 * sin(delta + gamma1 + A1 + A2);
  
  float lr2 = distance(P6, P0);
  float phi1 = cosineRule(l7, l6, lr2);
  float omega1 = cosineRule(T3, T1, T2);
  float omega2 = cosineRule(T2, T1, T3);
  float omega3 = PI - (omega1 + omega2);
  float psi = cosineRule(lr4, T2, l7);
  
  P7.x = P2.x + T3 * cos(delta + gamma1 + A1 + A2 + omega3);
  P7.y = P2.y - T3 * sin(delta + gamma1 + A1 + A2 + omega3);
  
  float lr3 = distance(P7, P4);
  float iota1 = cosineRule(l4 + l5, T3, lr3);
  float iota2 = cosineRule(l9, l8, lr3);
  
  P8.x = P7.x + l8 * cos((delta + gamma1 + A1 + A2 + omega3) - (PI - iota1 - iota2));
  P8.y = P7.y - l8 * sin((delta + gamma1 + A1 + A2 + omega3) - (PI - iota1 - iota2));
  
  float tau = atan((P4.y - P8.y) / (P4.x - P8.x));
   
  P9.x = P4.x + l10 * cos(tau);
  P9.y = P4.y + l10 * sin(tau);
  
  footPoints[0] = P8;
  footPoints[1] = P4;
  footPoints[2] = P9;
  
  return footPoints;
}

float[] ikine1(Point footPoints[]) {
  float q[] = {0, 0, 0};
  P8 = footPoints[0];
  P4 = footPoints[1];
  P9 = footPoints[2];
  
  float tau = atan((P4.y - P8.y) / (P4.x - P8.x));
  float lr1 = distance(P4, P0);
  float alpha = cosineRule(lr1, l2, (l4 + l5));
  float omega1 = cosineRule(T3, T1, T2);
  float omega2 = cosineRule(T1, T2, T3);
  float omega3 = PI - (omega1 + omega2);
  float phi1 = cosineRule((l4 + l5), l2, lr1);
  float phi2 = abs(atan(abs((P4.y - P0.y) / (P4.x - P0.x))));
  
  P2.x = P0.x + l2 * cos(phi1 + phi2);
  P2.y = P0.y - l2 * sin(phi1 + phi2);
  
  float phi = phi1 + phi2;
  
  P3.x = P2.x + l4 * cos(phi - alpha);
  P3.y = P2.y - l4 * sin(phi - alpha);
  
  float lr2 = distance(P0, P3);
  float beta = cosineRule(lr2, l1, l3);
  float gamma1 = cosineRule(l3, l1, lr2);
  float gamma = 0;
  if (P3.x > P0.x) {
    gamma = PI - atan((P3.y - P0.y) / (P3.x - P0.x));
  } else {
    gamma = -atan((P3.y - P0.y) / (P3.x - P0.x));
  }
  float gamma2 = gamma - gamma1;
  
  P1.x = P0.x + l1 * cos(gamma2);
  P1.y = P0.y - l1 * sin(gamma2);
  
  float lr3 = distance(P2, P8);
  float omega = cosineRule(lr3, l8, T3);
  float sigma1 = cosineRule(T3, l8, lr3);
  float sigma2 = cosineRule((l4 + l5), lr3, l9);
  float sigma = sigma1 + sigma2;
  
  P7.x = P8.x + l8 * cos(sigma + tau);
  P7.y = P8.y + l8 * sin(sigma + tau);
  
  P6.x = P7.x + T1 * cos(sigma + tau - (PI - (omega3 + omega)));
  P6.y = P7.y + T1 * sin(sigma + tau - (PI - (omega3 + omega)));
  
  float lr4 = distance(P6, P0);
  float iota1 = cosineRule(l7, l6, lr4);
  float iota2 = cosineRule(T2, l2, lr4);
  float iota = iota1 + iota2;
  
  P5.x = P0.x + l6 * cos(phi + iota);
  P5.y = P0.y - l6 * sin(phi + iota);
  
  q[0] = gamma2;
  q[1] = phi;
  q[2] = phi + iota;
  
  return q;
}

float[] ikine2(Point footCentre, float footAngle) {
  float q[] = {0, 0, 0};
  
  return q;
}


class Point {
  float x, y;
  
  Point(float x_, float y_) {
    x = x_;
    y = y_;
  }
  
  Point() {
    x = 0;
    y = 0;
  }
  
  void Print() {
    print(x);
    print(", ");
    print(y);
  }
  
  void Println() {
    print(x);
    print(", ");
    println(y);
  }
}

float distance(Point a, Point b) {
  float dist = sqrt( pow(a.x - b.x, 2) + pow(a.y - b.y, 2) );
  return dist;
}

float cosineRule(float A, float B, float C) {
   float angle = acos( (pow(B, 2) + pow(C, 2) - pow(A, 2)) / (2 * B * C) );
   return angle;
}
