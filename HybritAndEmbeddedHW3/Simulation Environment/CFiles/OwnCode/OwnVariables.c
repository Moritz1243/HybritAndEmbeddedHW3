// Translation velocity uw and rotation velocity up
double uw;
double up;

// Proporcional constant for controllers
double kp;
double kw; 

// Angles. Suffix rad means radians
double tg;
double tgrad;
double thetarad;

// Other temporal values
double temp1;
double temp2;

// Design variables
double p;
int controller = 0; // Hybrid controller
int past = 360;