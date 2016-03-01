// Initialization
uw = 0;
up = 0;
tg = atan2((yg - y0),(xg - x0));
tg = tg * 180 / PI; // Convert from radians to degrees

/// TASKS 7->11

// uw controller
kw = 13; // Up to 20
temp1 = cos(theta) * (x0 - x);
temp2 = sin(theta) * (y0 - y);
uw = kw * (temp1 + temp2);

// up controller
kp = 5; //Up to 10
up = kp*(tg - theta);


/// TASKS 12 -> 17
// uw controller
kw = 13; // Up to 20
temp1 = cos(tg) * (xg - x);
temp2 = sin(tg) * (yg - y);
uw = kw * (temp1 + temp2);

// up controller
p = 1;
kp = 5; //Up to 10
temp1 = sin(tg) * (x + p*cos(theta) - x0);
temp2 = cos(tg) * (y + p*sin(theta) - y0);
up = kp * (temp1 - temp2);



// Update ur, ul
right = uw + up/2;
left = uw - up/2;