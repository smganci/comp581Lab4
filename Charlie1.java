package lab4;

import java.util.Arrays;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;

/* Collaborators:
 * Tricia Bacon (730011125)
 * Sarah Ganci (720510446)
 * */

public class Charlie1 {
	// components
	private EV3LargeRegulatedMotor motorL;
	private EV3LargeRegulatedMotor motorR;
	private EV3MediumRegulatedMotor motorM;
	private EV3TouchSensor touchSensorL;
	private EV3TouchSensor touchSensorR;
	private EV3UltrasonicSensor sonicSensor;
	private EV3GyroSensor gyroSensor;

	// Robot measurements
	private double radiusL;
	private double radiusR;
	private double L;

	// Sensor modes
	private SensorMode touchL;
	private SensorMode touchR;
	private SensorMode sonic;
	private SensorMode gyro;

	// Current heading
	private double x;
	private double y;
	private double theta; // in radians

	// Last Hitpoint
	private double hitX;
	private double hitY;
	private double goalDist; // last measured distance to goal from m-line

	private double mLineAngle; // angle of m-line

	private double prevt; // for update
	private int hits; // # times we've hit an obstacle

	// gheading should hold the initial read of heading based on gyrosensor after
	// turn right at wall
	// backup if dead reckoning sucks
	private float gheading;

	public Charlie1() {
		this.motorL = new EV3LargeRegulatedMotor(MotorPort.B);
		this.motorR = new EV3LargeRegulatedMotor(MotorPort.C);
		this.motorM = new EV3MediumRegulatedMotor(MotorPort.A);
		this.touchSensorL = new EV3TouchSensor(SensorPort.S4);
		this.touchSensorR = new EV3TouchSensor(SensorPort.S1);
		this.sonicSensor = new EV3UltrasonicSensor(SensorPort.S2);
		this.gyroSensor = new EV3GyroSensor(SensorPort.S3);

		this.touchL = this.touchSensorL.getTouchMode();
		this.touchR = this.touchSensorR.getTouchMode();
		this.sonic = (SensorMode) this.sonicSensor.getDistanceMode();
		this.gyro = (SensorMode) this.gyroSensor.getAngleMode();

		this.radiusL = .028;
		this.radiusR = .028;
		this.L = .12;

		this.x = 1;
		this.y = 0;
		this.theta = Math.PI / 2;

		this.hitX = 1;
		this.hitY = 0;
		this.goalDist = distToGoal(this.x, this.y);

		this.mLineAngle = Math.atan(1.8 / 0.8);

		this.gheading = 0;
		this.hits = 0;
	}

	/*
	 * Name: setLeftSpeed in: angular velocity in degrees (float s) out: nothing
	 * description: sets left motor's speed to that angular velocity
	 */
	public void setLeftSpeed(float s) {
		this.motorL.setSpeed(s);
	}

	/*
	 * Name: setRighttSpeed in: angular velocity in degrees (float s) out: nothing
	 * description: sets right motor's speed to that angular velocity
	 */
	public void setRightSpeed(float s) {
		this.motorR.setSpeed(s);
	}

	/*
	 * Name: setBothSpeed in: angular velocity in degrees (float s) out: nothing
	 * description: sets both motors speed to s
	 */
	public void setBothSpeed(float s) {
		this.setLeftSpeed(s);
		this.setRightSpeed(s);
	}

	/*
	 * Name: moveForwardBoth in: nothing out: nothing description: moves both motors
	 * forward
	 */
	public void moveForwardBoth() {
		this.motorL.forward();
		this.motorR.forward();
	}

	/*
	 * Name: moveBackwardBoth in: nothing out: nothing description: moves both
	 * motors backward
	 */
	public void moveBackwardBoth() {
		this.motorL.backward();
		this.motorR.backward();
	}

	/*
	 * Name: stopBothInstant in: nothing out: nothing description: stops both
	 * instantly
	 */
	public void stopBothInstant() {
		this.motorL.stop(true);
		this.motorR.stop(true);
	}

	/*
	 * Name: moveTillSense in: distance in meters (dist) out: distance traveled
	 * description: moves robot forward till sonic sensor senses an obstacle a
	 * certain distance away
	 */
	public void moveTillSense(double d) {
		this.syncMotors();
		float speed = 270;
		this.setBothSpeed(speed);
		float[] sample_sonic = new float[this.sonic.sampleSize()];
		this.sonic.fetchSample(sample_sonic, 0);
		this.prevt = System.nanoTime();
		while (sample_sonic[0] > d && this.distToGoal(this.x, this.y) > .20 && !Button.ENTER.isDown()) {
			this.moveForwardBoth();
			sonic.fetchSample(sample_sonic, 0);
			this.update(speed, speed);
		}
		this.stopBothInstant();
//		double time = (System.nanoTime() - startTime);
//		this.goaldist = speed * Math.PI / 180 * this.radiusR * time / Math.pow(10, 9);
		// this.getPositionStraight(this.heading, speed, time);
		this.goalDist = distToGoal(this.x, this.y);
		this.stopSync();
	}

	/*
	 * Name: moveTillTouch in: nothing out: nothing description: moves robot forward
	 * till touch sensor encounters an obstacle
	 */
	public void moveTillTouch() {
		this.syncMotors();
		this.setBothSpeed(270);
		// float[] sample_touchL = new float[touchL.sampleSize()];
		float[] sample_touchR = new float[touchR.sampleSize()];
		float[] sample_touchL = new float[touchL.sampleSize()];
		this.prevt = System.nanoTime();
		while (sample_touchR[0] == 0 && sample_touchL[0] == 0) {
			this.moveForwardBoth();
			// touchL.fetchSample(sample_touchL, 0);
			touchR.fetchSample(sample_touchR, 0);
			touchL.fetchSample(sample_touchL, 0);
			this.update(270, 270);
		}
		this.stopBothInstant();
		this.stopSync();
	}

	/*
	 * Name: syncMotors in: nothing out: nothing description: synchronizes left and
	 * right motors
	 */
	public void syncMotors() {
		this.motorL.synchronizeWith(new EV3LargeRegulatedMotor[] { this.motorR });
	}

	public void stopSync() {
		this.motorL.endSynchronization();
	}

	/*
	 * Name: moveForwardTime in: time to move in seconds out: nothing description:
	 * makes robot move forward for a certain amount of time
	 */
	public void moveForwardTime(double sec) {
		this.syncMotors();
		this.moveForwardBoth();
		Delay.msDelay((long) sec);
		this.stopBothInstant();
		this.stopSync();
		this.update(this.motorL.getSpeed(), this.motorR.getSpeed(), sec);
	}

	/*
	 * Name: moveBackwardTime in: time to move in seconds out: nothing description:
	 * makes robot move Backward for a certain amount of time
	 */
	public void moveBackwardTime(double sec) {
		this.syncMotors();
		this.moveBackwardBoth();
		Delay.msDelay((long) sec);
		this.stopBothInstant();
		this.stopSync();
	}

	/*
	 * Name: moveTime in: time to move in seconds out: nothing description: makes
	 * robot move Backward for a certain amount of time
	 */
	private double moveTime(double vr, double vl, double d) {
		return (d / ((vr + vl) / 2) * 1000);
	}

	/*
	 * Name: moveForwardDist in: distance in meters out: nothing description: moves
	 * robot forward a certain distance
	 **/
	public void moveForwardDist(double d) {
		float speed = 270;
		this.setBothSpeed(speed);
		double vl = speed * (Math.PI / 180) * this.radiusL;
		double vr = speed * (Math.PI / 180) * this.radiusR;
		double sec = this.moveTime(vr, vl, d);
		this.moveForwardTime(sec);
	}

	/*
	 * Name: moveBackwardDist in: distance in meters out: nothing description: moves
	 * robot backward a certain distance
	 **/
	public void moveBackwardDist(double d) {
		float speed = 180;
		this.setBothSpeed(speed);
		double vl = speed * (Math.PI / 180) * this.radiusL;
		double vr = speed * (Math.PI / 180) * this.radiusR;
		double sec = this.moveTime(vr, vl, d);
		x = x - d * Math.cos(this.theta);
		y = y - d * Math.sin(this.theta);
		this.moveBackwardTime(sec);
	}

	/*
	 * Name: beep in: nothing out: nothing description: makes robot beep
	 */
	public void beep() {
		Sound.beep();
	}

	/*
	 * Name: buttonWait in: nothing out: nothing description: waits for button press
	 * to continue
	 */
	public void buttonWait() {
		Button.ENTER.waitForPressAndRelease();
	}

	public void setProportionalSpeeds(float son, float angle) {
		double norm = son / 0.3;
		// double addeg= (1.0-norm)*90.0;
		if (norm < 0.5) {
			float addeg = (float) (1.0 - norm) * angle;
			this.setLeftSpeed(180 + addeg);

		} else {
			float addeg = (float) (norm) * angle;
			this.setRightSpeed(180 + addeg);
		}
	}

	public void setDiffSpeeds(float left, float right) {
		this.setLeftSpeed(left);
		this.setRightSpeed(right);
	}

	/*
	 * Name: rotateSonic in: degrees out: nothing description: should rotate sonic
	 * sensor to certain angle relative to robot
	 *
	 */
	public void rotateSonic(int degrees) {
		this.motorM.setSpeed(90);
		this.motorM.rotate(degrees);
	}

	/*
	 * Name: sonicSense in: nothing out: double number representing the median of
	 * the distance sensed from the sonic sensor description: senses 3 times and
	 * returns the median
	 */

	public float sonicSense() {
		float[] dists = new float[3];
		float[] sample_sonic = new float[sonic.sampleSize()];
		for (int i = 0; i < dists.length; i++) {
			// System.out.println("Sensing Sonic");
			sonic.fetchSample(sample_sonic, 0);
			dists[i] = sample_sonic[0];
			int loop = 0;
			while (dists[i] > .8 && loop < 5) {
				// System.out.println("Infinity " + loop);
				sonic.fetchSample(sample_sonic, 0);
				dists[i] = sample_sonic[0];
				loop++;
			}
		}
		Arrays.sort(dists);
		return dists[1];
	}

	/*
	 * Name: rotateRight in: degrees to rotate out: nothing description: should turn
	 * the robot in place towards goal
	 */

	public void rotateRight(double degrees) {
		this.stopSync();
		this.motorL.forward();
		this.setLeftSpeed(180);
		double delay = this.timeToRotate(0, 180, degrees);
		Delay.msDelay((long) delay);
//		this.theta -= (degrees * Math.PI / 180.0);
//		double vl = 180 * (Math.PI / 180) * this.radiusL;
//		this.x += (vl / 2.0) * Math.cos(this.theta) * (delay / 1000.0);
//		this.y += (vl / 2.0) * Math.sin(this.theta) * (delay / 1000.0);
		this.stopBothInstant();
		this.update(180, 0, delay);

	}

	public void rotateRight2(double degrees) {
		this.stopSync();
		this.setLeftSpeed(180);
		this.motorL.forward();
		double nanodelay = System.nanoTime() + (Math.pow(10, 9) * this.timeToRotate(0, 180, degrees));
		while (System.nanoTime() < nanodelay) {
			this.motorL.forward();
			this.update(180, 0);
		}
		this.stopBothInstant();
	}

	/*
	 * Name: rotateLeft in: degrees to rotate out: nothing description: should turn
	 * the robot in place towards goal
	 */

	public void rotateLeft(double degrees) {
//		System.out.println("RotateLeft");
		this.stopSync();
		this.motorR.forward();
		this.setRightSpeed(180);
		double delay = this.timeToRotate(180, 0, degrees);
		Delay.msDelay((long) delay);
		this.stopBothInstant();
		this.update(0, 180, delay);

	}

	// move left forward and right backward to create a spin
	// TODO: Overcorrects. Need to account for that.
	public void turnInPlaceRight(double degrees) {
//		System.out.println("Turning in place right");
//		this.stopSync();
		this.setBothSpeed(180);
		this.motorL.forward();
		this.motorR.backward();
		double delay = this.timeToRotate(-180, 180, -degrees);
//		System.out.println("Delay: " + (delay / 1000));
		Delay.msDelay((long) delay);
		this.stopBothInstant();
		update(180, -180, delay / 1000);
	}

	// move right forward and left backward to create a spin
	// TODO: Overcorrects. Need to account for that.
	public void turnInPlaceLeft(double degrees) {
//		System.out.println("Turning in place left");
//		this.stopSync();
		this.setBothSpeed(180);
		this.motorL.backward();
		this.motorR.forward();
		double delay = this.timeToRotate(180, -180, degrees);
//		System.out.println("Delay: " + (delay / 1000));
		Delay.msDelay((long) delay);
		this.stopBothInstant();
		update(-180, 180, delay / 1000);
	}

	public float gthetha() {
		float[] sample_gyro = new float[gyro.sampleSize()];
		this.gyro.fetchSample(sample_gyro, 0);
		return sample_gyro[0];
	}

	public void setgHeading() {
		this.gheading = this.gthetha();
	}

	public float getHeading() {
		return this.gheading;
	}

//	/*Name: timeToRotate
//	 * in: ul, ur, theta
//	 * out: seconds to move
//	 * */
//
	public double timeToRotate(double ur, double ul, double theta) {
		double vr = ur * Math.PI / 180 * this.radiusR;
		double vl = ul * Math.PI / 180 * this.radiusL;
		double omega = (vr - vl) / this.L;
		double time = (theta * Math.PI / 180) / omega;
		return time * 1000;
	}

///////////////////////////////////////////////// Sensing

	public boolean leftBump() {
		float[] sample_touchL = new float[touchL.sampleSize()];
		touchL.fetchSample(sample_touchL, 0);
		return sample_touchL[0] != 0;
	}

	/*
	 * Name: sonicRotateSense in: none out: float array holding 3 sensed distance
	 * values
	 */

	// problems: sonic now sensing all 0s for some reason? why?
	public float[] sonicRotateSense() {
		float[] senses = new float[4];

		// original angle is -90

		// rotate back -45 degrees to sense behind (angle now at -135)
		this.rotateSonic(-45);
		senses[0] = this.sonicSense();
//		System.out.println(senses[0]);

		// rotate forward 50 degrees to sense where position was but a little past to
		// account for exact reflection
		// angle now at -85
		this.rotateSonic(50);
		senses[1] = this.sonicSense();
//		System.out.println(senses[1]);

		// rotate forward 40 (now angle is -45) so its looking ahead
		this.rotateSonic(40);
		senses[2] = this.sonicSense();
//		System.out.println(senses[2]);

		// rotate forward (angel now at 0) and sense
		this.rotateSonic(45);
		senses[3] = this.sonicSense();
//		System.out.println(senses[3]);

		// returns sonic to original state
		// angle now at -90
		this.rotateSonic(-90);

		return senses;
	}

	public void rotateLeftTillSense(double dist) {
		this.stopSync();
		this.setRightSpeed(180);
		this.motorR.forward();
		float sample_sonic = this.sonicSense();
		this.prevt = System.nanoTime();
		while (sample_sonic > dist) {
			this.motorR.forward();
			sample_sonic = this.sonicSense();
		}
		this.stopBothInstant();
		this.update(0, 180);
	}

	/*
	 * update: in: start time in nano seconds and end time in nano seconds
	 */
	public void update(double ul, double ur) {
		double end = System.nanoTime();
		double start = this.prevt / Math.pow(10, 9);
		// after prevt is converted and stored in start set prevt to end
		this.prevt = end;
		end = end / Math.pow(10, 9);
		double dt = end - start;
//		System.out.println("dt = " + dt);
		// vl/r is equal to the motor speed converted to radians multiplied by radius
		double vr = ur * (Math.PI / 180) * this.radiusR;
		double vl = ul * (Math.PI / 180) * this.radiusL;
		double w = (vr - vl) / this.L;
		// adjust will hold the percent we should multiply by to account for carpet
//		double xadjust = .9748;
		updateTheta(w * dt);
		this.x += /* xadjust */ ((vl + vr) / 2.0) * Math.cos(this.theta) * dt;
		this.y += ((vl + vr) / 2.0) * Math.sin(this.theta) * dt;
		// System.out.println("Pose: (" + this.x + ", " + this.y + ", " + this.theta +
		// ")");
	}

	public void update(double ul, double ur, double dt) {
		// vl/r is equal to the motor speed converted to radians multiplied by radius
//		System.out.println("vl = " + ul); // Not quite, maybe just pass in parameter?
//		System.out.println("vr = " + ur);
		double vr = ur * (Math.PI / 180) * this.radiusR;
		double vl = ul * (Math.PI / 180) * this.radiusL;
		double w = (vr - vl) / this.L;
		updateTheta(w * dt);
		this.x += ((vl + vr) / 2.0) * Math.cos(this.theta) * dt;
		this.y += ((vl + vr) / 2.0) * Math.sin(this.theta) * dt;
		// System.out.println("Pose: (" + this.x + ", " + this.y + ", " + this.theta +
		// ")");
		this.prevt = System.nanoTime();
	}

	// Does not work as of now
	public void updateBackwards() {
		double end = System.nanoTime();
		double start = this.prevt / Math.pow(10, 9);
		// after prevt is converted and stored in start set prevt to end
		this.prevt = end;
		end = end / Math.pow(10, 9);
		double dt = end - start;
		// vl/r is equal to the motor speed converted to radians multiplied by radius
		double vr = -this.motorR.getSpeed() * (Math.PI / 180) * this.radiusR;
		double vl = -this.motorL.getSpeed() * (Math.PI / 180) * this.radiusL;
		double w = (vr - vl) / this.L;

		// adjust will hold the percent we should multiply by to account for carpet
		double xadjust = .9748;
		this.theta += w * dt;
		this.x += xadjust * ((vl + vr) / 2.0) * Math.cos(this.theta) * dt;
		this.y += ((vl + vr) / 2.0) * Math.sin(this.theta) * dt;
		// System.out.println("Pose: (" + this.x + ", " + this.y + ", " + this.theta +
		// ")");
	}

	public void printPos() {
		System.out.println("x: " + this.x + ", y: " + this.y + ", theta: " + this.theta);
	}

//	public boolean withinRange(double x, double y) {
//		return ((this.x < x && this.x >= 0 - x) && (this.y < y && this.y > 0 - y));
//	}

//	public void returnToStart() {
//		// Turn towards goal
////		while (this.theta <= -Math.PI) {
////			this.theta += 2 * Math.PI;
////		}
////		while (this.theta > Math.PI) {
////			this.theta -= 2 * Math.PI;
////		}
////		double angleToTurn = this.theta + Math.PI / 2.0;
////		if (angleToTurn > 0) {
////			this.rotateRight(angleToTurn);
////		} else if (angleToTurn < 0) {
////			this.rotateLeft((-angleToTurn));
////		}
//		this.rotateRight(90);
//		// Move towards goal
//		double dist = this.goaldist;
//		this.setBothSpeed(270);
//		this.moveForwardDist(dist);
//	}

//	public void returnToStartBackup() {
//		this.rotateRight(90);
//		double dist = this.goaldist;
//		this.moveForwardDist(dist);
//	}

	public void thisIsHalloween() {
		// "This is Halloween"
		Sound.playTone(392, 300);
		Sound.playTone(392, 300);
		Sound.playTone(392, 150);
		Sound.playTone(367, 150);
		Sound.playTone(330, 300);
		// "This is Halloween"
		Sound.playTone(392, 300);
		Sound.playTone(392, 300);
		Sound.playTone(392, 150);
		Sound.playTone(367, 150);
		Sound.playTone(330, 300);
		// "Halloween" (higher)
		Sound.playTone(494, 150);
		Sound.playTone(466, 150);
		Sound.playTone(415, 300);
		// "Halloween"
		Sound.playTone(494, 150);
		Sound.playTone(466, 150);
		Sound.playTone(415, 300);
		// "Halloween" (lower)
		Sound.playTone(415, 150);
		Sound.playTone(392, 150);
		Sound.playTone(349, 300);
		// "Halloween"
		Sound.playTone(415, 150);
		Sound.playTone(392, 150);
		Sound.playTone(349, 300);
		// Wow these lyrics really aren't helpful
	}

	// In: x, y
	// Out: distance to goal in m
	public double distToGoal(double x, double y) {
		return Math.sqrt(Math.pow(1.8 - x, 2) + Math.pow(1.8 - y, 2));
	}

	// In: none
	// Out: boolean for whether or not you're in range of m-line
	public boolean inMLineRange() {
		if (this.y < (2.25 * this.x - 2.15) && this.y > (2.25 * this.x - 2.35)) {
			return true;
		}
		return false;
	}

	// In: none
	// Out: none
	// rotate in place to the angle of the m-line
	public void rotateToMLine() {
		System.out.println("Rotating toward the mline");
		System.out.println("Starting at angle:" + this.theta);
		if (this.theta > this.mLineAngle + Math.PI) {
//			System.out.println("Rotate Left");
			turnInPlaceLeft(radiansToDegrees(2 * Math.PI - this.theta + this.mLineAngle));
//			System.out.println("Theta: " + this.theta);
		} else {
//			System.out.println("Rotate Right");
			turnInPlaceRight(radiansToDegrees(this.theta - this.mLineAngle));
//			System.out.println("Theta: " + this.theta);
		}
	}

	// In: degrees
	// Out: radians
	public double degreesToRadians(double degrees) {
		return degrees * Math.PI / 180;
	}

	// In: radians
	// Out: degrees
	public double radiansToDegrees(double radians) {
		return radians * 180 / Math.PI;
	}

	// Getter that returns m-line angle
	public double getMLineTheta() {
		return this.mLineAngle;
	}

	// In: amount to adjust theta (in radians)
	// Out: new theta
	public double updateTheta(double radians) {
		this.theta += radians;
		if (this.theta >= 2 * Math.PI) {
			this.theta -= 2 * Math.PI;
		} else if (this.theta < 0) {
			this.theta += 2 * Math.PI;
		}
		return this.theta;
	}

	// In: radians to set theta value
	// Out: theta
	public double setTheta(double radians) {
		this.theta = radians;
		if (this.theta >= 2 * Math.PI) {
			this.theta -= 2 * Math.PI;
		} else if (this.theta < 0) {
			this.theta += 2 * Math.PI;
		}
		return this.theta;
	}

	// Just making sure this works
	public void spin() {
		this.setBothSpeed(180);
		this.motorL.forward();
		this.motorR.backward();
		Delay.msDelay(5000);
		this.stopBothInstant();
	}

	// I also added turnInPlaceRight and turnInPlaceLeft up by rotateRight and
	// rotateLeft

	// Sarah Stuff below
	public boolean inRange(double x, double y, double range) {
		return this.x < x + range && this.x > x - range && this.y < y + range && this.y < y + range;
	}

	public boolean returnedToHP() {
		return this.inRange(this.hitX, this.hitY, .05);
	}

	public boolean frontBump() {

		float[] sample_touchL = new float[touchL.sampleSize()];
		float[] sample_touchR = new float[touchR.sampleSize()];
		touchL.fetchSample(sample_touchL, 0);
		touchR.fetchSample(sample_touchR, 0);

		return sample_touchR[0] != 0 || sample_touchL[0] != 0;

	}

	public void bug2() {

		// while distance to goal is larger than 15 cm
		while (!Button.ENTER.isDown() && this.inRange(1.8, 1.8, .2)) {

			this.rotateToMLine();
			// CALL move mline till sense
			this.moveTillSense(.10);
			// if move till sense exited due to proximity to goal need to stop so check and
			// break
			if (this.inRange(1.8, 1.8, .25)) {
				break;
			}
			// TURN RIGHT
			this.turnInPlaceRight(90);
			// RESET NEW HIT VALUE
			this.hitX = this.x;
			this.hitY = this.y;
			this.goalDist = this.distToGoal(this.hitX, this.hitY);
			// }

			// CALL trace till mline
			this.tracetmline();
		}
		System.out.println("go to goal");
		this.printPos();
		this.buttonWait();
//		goToGoal();
	}

	public boolean tracetmline() {

		// 1: take a valid reading from sonic
		this.rotateSonic(-70);
		float sonic = this.sonicSense();
		this.syncMotors();
		float bs = 180;
		this.setBothSpeed(bs);
		this.moveForwardBoth();
//		System.out.println(
//				!(this.inMLineRange() && this.distToGoal(this.x, this.y) < this.distToGoal(this.hitX, this.hitY)));
//		System.out.println(!returnedToHP());
//		System.out.println("about to enter loop");
//		//
		double startTime = System.nanoTime();
//		System.out.println("time to wait: " + (System.nanoTime() - startTime > 3 * Math.pow(10, 9)));
		while (!Button.ENTER.isDown()
				&& !(this.inMLineRange() && this.distToGoal(this.x, this.y) < this.distToGoal(this.hitX, this.hitY)
						&& (System.nanoTime() - startTime > 3 * Math.pow(10, 9)))
				&& !(returnedToHP() && (System.nanoTime() - startTime > 3 * Math.pow(10, 9)))) {
//			System.out.println("in loop");
			this.update(this.motorL.getSpeed(), this.motorR.getSpeed());
			if (this.frontBump()) {
				System.out.println("front bump");
				this.update(this.motorL.getSpeed(), this.motorR.getSpeed());
				this.stopBothInstant();
				// 4.1: back up body length
				this.moveBackwardDist(0.15);

				// 4.3: turn
				this.turnInPlaceRight(90);

				// 4.4: move forward
				this.moveForwardBoth();
				this.setBothSpeed(bs);

				// 4.5: continue on to next iteration of big loop
				continue;
			}

			double dbuff = .08;
			double d1 = 0.05 + dbuff;
			double d2 = 0.12 + dbuff;
			double d3 = 0.14 + dbuff;
			double d4 = .16 + dbuff;
			double d5 = .18 + dbuff;
			double d6 = .25 + dbuff;
			if (sonic <= d1) {
				// Option 1: way too close -- move to the right fast
				this.setDiffSpeeds(270, 180);
			} else if (sonic < d2) {
				// Option 2: way too close -- move to the right fast
				this.setDiffSpeeds(240, 180);
			} else if (sonic < d3) {
				// Option 3: a little to close to wall -- adjust right a little
				this.setDiffSpeeds(210, 180);
			} else if (sonic >= d3 && sonic <= d4) {
				// Option 4: just right -- set wheels to same speed and move on forward
				this.setBothSpeed(180);
			} else if (sonic > d4 && sonic <= d5) {
				// Option 5: a little too far from wall -- move left slow
				this.setDiffSpeeds(180, 210);
			} else if (sonic > d5 && sonic <= d6) {
				// Option 6: a little too far from wall -- move left slow
				this.setDiffSpeeds(180, 240);
			} else { // .2
				// Option 7: way too far -- move to the left fast
				this.setDiffSpeeds(180, 270);
			}
			sonic = this.sonicSense();
		}

		// stop motors and sychronization
		this.stopBothInstant();
		this.stopSync();
//		System.out.println("End loop");
		// return sonic to original position
		this.rotateSonic(70);
		// if has returned to hitpoint, return false else true
		boolean error = !returnedToHP();
		this.hitX = this.x;
		this.hitY = this.y;
		this.goalDist = this.distToGoal(this.hitX, this.hitY);
		return error;
	}

	public void goToGoal() {
		this.rotateToMLine();
		moveForwardDist(distToGoal(this.x, this.y));
		this.printPos();
	}

}