// Initialization
uw = 0;
up = 0;
thetarad = theta * PI / 180;
tgrad = atan2((yg - y0),(xg - x0));
tg = tgrad * 180 / PI; // Convert from radians to degrees

// If we move from/to -179 to/from 180
if (abs(tg - theta) > (past + 90)) {
	if (tg > 0) {
		tg = tg - 360;
	} else {
		tg = tg + 360;
	}
}
past = (abs(tg - theta));

// Changing controller
if (abs(tg - theta) < 6) { //Six degrees
	controller = 1;
}

if (controller == 0) {

	/// TASKS 7->11
	// uw controller
	kw = 13; // Up to 20
	temp1 = cos(thetarad) * (x0 - x);
	temp2 = sin(thetarad) * (y0 - y);
	uw = kw * (temp1 + temp2);

	// up controller
	kp = 5; //Up to 10
	up = kp*(tg - theta);


} else {

	/// TASKS 12 -> 17
	// uw controller
	kw = 13; // Up to 20
	temp1 = cos(tgrad) * (xg - x);
	temp2 = sin(tgrad) * (yg - y);
	uw = kw * (temp1 + temp2);

	// up controller
	p = 50;
	kp = 0.10 * 180 / PI;
	temp1 = sin(tgrad) * (x + p*cos(thetarad) - x0);
	temp2 = cos(tgrad) * (y + p*sin(thetarad) - y0);
	up = kp * (temp1 - temp2);

}

//Serial.print(controller, DEC);

// Update ur, ul
if (uw > 500) uw = 500;
if (uw < -500) uw = -500;

right = uw + up/2;
left = uw - up/2;