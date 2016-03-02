// Initialization
uw = 0;
up = 0;
tgrad = atan2((yg - y0),(xg - x0));
tg = tgrad * 180 / PI; // Convert from radians to degrees
thetarad = theta * PI / 180;

/// TASKS 7->11
// uw controller
kw = 13; // Up to 20
temp1 = cos(thetarad) * (x0 - x);
temp2 = sin(thetarad) * (y0 - y);
uw = kw * (temp1 + temp2);

// up controller
kp = 5; //Up to 10
up = kp*(tg - theta)


/// TASKS 12 -> 17
// uw controller
kw = 13; // Up to 20
temp1 = cos(tgrad) * (xg - x);
temp2 = sin(tgrad) * (yg - y);
uw = kw * (temp1 + temp2);

// up controller
p = 1;
kp = 5; //Up to 10 if p=1
temp1 = sin(tgrad) * (x + p*cos(thetarad) - x0);
temp2 = cos(tgrad) * (y + p*sin(thetarad) - y0);
up = kp * (temp1 - temp2);



// Update ur, ul
right = uw + up/2;
left = uw - up/2;