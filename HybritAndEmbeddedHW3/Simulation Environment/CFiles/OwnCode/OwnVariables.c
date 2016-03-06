// Translation velocity uw and rotation velocity up
double uw = 0;
double up = 0;

// Proporcional constant for controllers
double kp = 0;
double kw = 0; 

// Angles. Suffix rad means radians
double tg = 0;
double tgrad = 0;
double thetarad = 0;

// Other temporal values
double temp1 = 0;
double temp2 = 0;

// Design variables
double p = 0;
int controller = 0; // Hybrid controller
int past = 360;