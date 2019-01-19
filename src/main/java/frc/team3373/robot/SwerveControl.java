package frc.team3373.robot;

import java.util.NoSuchElementException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveControl {

	enum Side {
		NORTH, SOUTH, EAST, WEST;
	}
	
	enum DriveMode {
		ROBOTCENTRIC, FIELDCENTRIC, OBJECTCENTRIC;
	}

	private SwerveWheel FLWheel;
	private SwerveWheel BLWheel;
	private SwerveWheel FRWheel;
	private SwerveWheel BRWheel;

	private SwerveWheel[] wheelArray;

	private double rotateTarget;
	private double distanceToCenter;

	double robotLength;
	double robotWidth;

	private float orientationOffset = 0;

	private boolean isFieldCentric = true;
	private boolean isObjectCentric = false;

	private double maxTargetSpeed = 0;
	private boolean stoppedRotating = true;
	private float targetRobotAngle = 0;

	SuperAHRS ahrs;

	public SwerveControl(int LFrotateMotorID, int LFdriveMotorID, int LFEncMin, int LFEncMax, int LFEncHome,
			int LBrotateMotorID, int LBdriveMotorID, int LBEncMin, int LBEncMax, int LBEncHome, int RFrotateMotorID,
			int RFdriveMotorID, int RFEncMin, int RFEncMax, int RFEncHome, int RBrotateMotorID, int RBdriveMotorID,
			int RBEncMin, int RBEncMax, int RBEncHome, SuperAHRS AHRS, double width, double length) {

		double rotAngle = Math.toDegrees(Math.atan((width / 2) / (length / 2)));

		robotWidth = width;
		robotLength = length;

		ahrs = AHRS;
		// System.out.println(rotAngle);
		/*
		 * FLWheel=new
		 * SwerveWheel("FrontLeft",LFrotateMotorID,LFdriveMotorID,LFEncMin,LFEncMax,
		 * LFEncHome,rotAngle+270); BLWheel=new
		 * SwerveWheel("BackLeft",LBrotateMotorID,LBdriveMotorID,LBEncMin,LBEncMax,
		 * LBEncHome,rotAngle); FRWheel=new
		 * SwerveWheel("FrontRight",RFrotateMotorID,RFdriveMotorID,RFEncMin,RFEncMax,
		 * RFEncHome,270-rotAngle); BRWheel=new
		 * SwerveWheel("BackRight",RBrotateMotorID,RBdriveMotorID,RBEncMin,RBEncMax,
		 * RBEncHome,rotAngle+90);
		 */

		FLWheel = new SwerveWheel("FrontLeft", LFrotateMotorID, LFdriveMotorID, LFEncMin, LFEncMax, LFEncHome,
				270 - rotAngle);
		BLWheel = new SwerveWheel("BackLeft", LBrotateMotorID, LBdriveMotorID, LBEncMin, LBEncMax, LBEncHome,
				rotAngle + 270);
		FRWheel = new SwerveWheel("FrontRight", RFrotateMotorID, RFdriveMotorID, RFEncMin, RFEncMax, RFEncHome,
				rotAngle + 90);
		BRWheel = new SwerveWheel("BackRight", RBrotateMotorID, RBdriveMotorID, RBEncMin, RBEncMax, RBEncHome,
				rotAngle);

		wheelArray = new SwerveWheel[] { FLWheel, BLWheel, BRWheel, FRWheel };

	}

	// #####################################################
	// #########	   		Autonomus			############
	// #####################################################

	public void drive(float angle, double speed) {
		calculateAutoSwerveControl(angle, speed, 0);
	}

	public void rotateRelative(float angle, double speed) {
		double initalAngle = ahrs.getRotation();
		double targetAngle = initalAngle + angle;
		if (targetAngle >= 360) {
			targetAngle -= 360;
		} else if (targetAngle < 0) {
			targetAngle += 360;
		}

		// calculateAutoSwerveControl(0,0,);
	}

	public void rotateAbsolute(float angle, double speed) {
		double initalAngle = ahrs.getRotation();
		double targetAngle = angle;
		if (targetAngle >= 360) {
			targetAngle -= 360;
		} else if (targetAngle < 0) {
			targetAngle += 360;
		}

		// calculateAutoSwerveControl(0,0,);
	}

	public boolean rotateAroundObject(float angle, float distanceFromCenter, double speed) {
		setDistanceToObject(distanceFromCenter);
		// calculateObjectControl(speed);
		return false;
	}

	public void calculateAutoSwerveControl(double driveAngle, double driveSpeed, double rotateSpeed) {// Driving Angle,
																										// Driving
																										// Speed, rotate
																										// speed
		double translationalXComponent;
		double translationalYComponent;
		double translationalMagnitude;
		double translationalAngle;

		double rotateXComponent;
		double rotateYComponent;

		if (driveAngle > 360) {
			driveAngle = 360;
		} else if (driveAngle < 0) {
			driveAngle = 0;
		}

		driveSpeed = Math.abs(driveSpeed);
		if (driveSpeed > 1)
			driveSpeed = 1;

		if (rotateSpeed > 1) {
			rotateSpeed = 1;
		} else if (rotateSpeed < -1) {
			rotateSpeed = -1;
		}

		double rotationMagnitude = Math.abs(rotateSpeed);

		// We break up the axis to create two vectors for the robot(and each wheel)
		// translational vector
		// rotation vector

		// Same for all wheels so therefore we only do the transitional vector math once
		translationalMagnitude = Math.abs(driveSpeed);
		translationalAngle = driveAngle;

		if (isFieldCentric) {
			// if in field centric mode make offset equal to the current angle of the navX
			orientationOffset = ahrs.getRawRotation();
		}

		// sets the robot front to be at the angle determined by orientationOffset
		translationalAngle += orientationOffset; 

		if (translationalAngle >= 360) {
			translationalAngle -= 360;
		} else if (translationalAngle < 0) {
			translationalAngle += 360;
		}
		// calculates y component of translation vector
		translationalYComponent = Math.sin(Math.toRadians(translationalAngle)) * translationalMagnitude;
		// calculates x component of translation vector
		translationalXComponent = Math.cos(Math.toRadians(translationalAngle)) * translationalMagnitude;

		// math for rotation vector, different for every wheel so we calculate for each
		// one seperately
		for (SwerveWheel wheel : wheelArray) {

			// calculates x component of rotation vector
			rotateXComponent = Math.cos(Math.toRadians(wheel.getRAngle())) * rotationMagnitude;
			// calculates y component of rotation vector
			rotateYComponent = Math.sin(Math.toRadians(wheel.getRAngle())) * rotationMagnitude; 

			if (rotateSpeed > 0) {//
				rotateXComponent = -rotateXComponent;
				rotateYComponent = -rotateYComponent;
			}

			// sets the speed based off translational and rotational vectors
			wheel.setSpeed(Math.sqrt(Math.pow(rotateXComponent + translationalXComponent, 2)
					+ Math.pow((rotateYComponent + translationalYComponent), 2)) * maxTargetSpeed);

			// sets the target angle based off translation and rotational vectors
			wheel.setTargetAngle(Math.toDegrees(Math.atan2((rotateYComponent + translationalYComponent),
					(rotateXComponent + translationalXComponent))));

		}

		// Makes the wheels go to calculated target angle
		FRWheel.goToAngle();
		FLWheel.goToAngle();
		BRWheel.goToAngle();
		BLWheel.goToAngle();
		// Make the wheels drive at their calculated speed
		FRWheel.drive();
		FLWheel.drive();
		BRWheel.drive();
		BLWheel.drive();
	}

	// ######################################################
	// ########### Teleop ############
	// ######################################################
	public void calculateSwerveControl(double LX, double LY, double RX) {
		LY = -LY; // inverts joystick LY to match the Cartesian plane
		double translationalXComponent = LX;
		double translationalYComponent = LY;
		double translationalMagnitude;
		double translationalAngle;

		double rAxis = RX;
		double rotateXComponent;
		double rotateYComponent;

		// Deadband
		if (Math.abs(LX) < 0.1) {
			translationalXComponent = 0;
			LX = 0;
		}

		if (Math.abs(LY) < 0.1) {
			translationalYComponent = 0;
			LY = 0;
		}

		if (Math.abs(RX) < 0.1) {
			rAxis = 0;
			RX = 0;
		}

		/*
		 * if(RX==0){ if(stoppedRotating){ targetRobotAngle=ahrs.getRotation();
		 * stoppedRotating=false; } rAxis=getRotationalCorrection(); RX=rAxis; }else{
		 * stoppedRotating=true; }
		 */

		if (isFieldCentric) {
			// if in field centric mode make offset equal to the current angle of the navX
			orientationOffset = ahrs.getRawRotation();
		}

		double rotationMagnitude = Math.abs(rAxis);

		// We break up the axis to create two vectors for the robot(and each wheel)
		// translational vector
		// rotation vector

		// Same for all wheels so therefore we only do the transitional vector math once.
		// magnitude of joystick
		translationalMagnitude = Math.sqrt(Math.pow(translationalYComponent, 2) + Math.pow(translationalXComponent, 2));
		// angle of joystick
		translationalAngle = Math.toDegrees(Math.atan2(translationalYComponent, translationalXComponent));

		// sets the robot front to be at the angle determined by orientationOffset
		translationalAngle += orientationOffset; 
		if (translationalAngle >= 360) {
			translationalAngle -= 360;
		} else if (translationalAngle < 0) {
			translationalAngle += 360;
		}

		// calculates y component of translation vector
		translationalYComponent = Math.sin(Math.toRadians(translationalAngle)) * translationalMagnitude;
		// calculates x component of translation vector
		translationalXComponent = Math.cos(Math.toRadians(translationalAngle)) * translationalMagnitude; 

		if (LY == 0 && LX == 0 && RX == 0) {
			stopMoving();
		} else {
			// math for rotation vector, different for every wheel so we calculate for each
			// one seperately
			for (SwerveWheel wheel : wheelArray) {

				// calculates x component of rotation vector
				rotateXComponent = Math.cos(Math.toRadians(wheel.getRAngle())) * rotationMagnitude;
				// calculates y component of rotation vector
				rotateYComponent = Math.sin(Math.toRadians(wheel.getRAngle())) * rotationMagnitude; 
				if (rAxis > 0) {// inverts the X and Y to change the direction of the wheels when rotating.
					rotateXComponent = -rotateXComponent;
					rotateYComponent = -rotateYComponent;
				}

				// sets the speed based off translational and rotational vectors
				wheel.setSpeed(Math.sqrt(Math.pow(rotateXComponent + translationalXComponent, 2)
						+ Math.pow((rotateYComponent + translationalYComponent), 2)) * maxTargetSpeed);

				wheel.setTargetAngle(Math.toDegrees(Math.atan2((rotateYComponent + translationalYComponent),
						(rotateXComponent + translationalXComponent))));// sets the target angle based off translation
				// and rotational vectors
			}
		}
		// System.out.println("");
		// Makes the wheels go to calculated target angle
		FRWheel.goToAngle();
		FLWheel.goToAngle();
		BRWheel.goToAngle();
		BLWheel.goToAngle();
		// Make the wheels drive at their calculated speed
		FRWheel.drive();
		FLWheel.drive();
		BRWheel.drive();
		BLWheel.drive();
	}
	/*
	 * private double getRotationalCorrection(){ float currentRotation =
	 * ahrs.getRotation(); float angleError = targetRobotAngle-currentRotation; int
	 * directionMod=-1; int optimalDirection = 1;
	 * 
	 * if(angleError<0) directionMod=1;
	 * 
	 * if(Math.abs(angleError)>180){ SmartDashboard.putBoolean("optimalPath", true);
	 * angleError = (360 - Math.abs(angleError)) % 360; optimalDirection = -1;
	 * //angleError*=-1; }else{ SmartDashboard.putBoolean("optimalPath", false); }
	 * 
	 * SmartDashboard.putNumber("Angle Error", angleError); double
	 * speed=(Math.sqrt(Math.sqrt(Math.abs(angleError))) + 1) * .05 * directionMod *
	 * optimalDirection;//(Math.abs(angleError)*0.01111111)*directionMod*
	 * optimalDirection; if(speed>.7) speed=.7; if(speed<-.7) speed=-.7;
	 * 
	 * SmartDashboard.putNumber("Rotational Speed", speed); return speed; }
	 */

	public void calculateObjectControl(double RX) {
		double distanceToFront = distanceToCenter - robotLength / 2;
		double distanceToBack = distanceToCenter + robotLength / 2;

		FLWheel.setTargetAngle(180 - Math.toDegrees(Math.atan2(robotWidth / 2, distanceToFront)));
		FRWheel.setTargetAngle(180 + Math.toDegrees(Math.atan2(robotWidth / 2, distanceToFront)));
		BLWheel.setTargetAngle(180 - Math.toDegrees(Math.atan2(robotWidth / 2, distanceToBack)));
		BRWheel.setTargetAngle(180 + Math.toDegrees(Math.atan2(robotWidth / 2, distanceToBack)));

		BLWheel.setSpeed(RX);
		BRWheel.setSpeed(RX);

		double speedRatio = Math.sqrt(Math.pow((robotWidth / 2), 2) + Math.pow(distanceToFront, 2))
				/ Math.sqrt(Math.pow((robotWidth / 2), 2) + Math.pow(distanceToBack, 2));

		FLWheel.setSpeed(speedRatio * RX);
		FRWheel.setSpeed(speedRatio * RX);

		FRWheel.goToAngle();
		FLWheel.goToAngle();
		BRWheel.goToAngle();
		BLWheel.goToAngle();

		FRWheel.drive();
		FLWheel.drive();
		BRWheel.drive();
		BLWheel.drive();
	}

	public void setDistanceToObject(double distance) {
		distanceToCenter = distance;
	}

	public void controlMode(DriveMode mode) {
		switch (mode) {
		case ROBOTCENTRIC:
			isFieldCentric = false;
			isObjectCentric = false;
			orientationOffset = 0;
			break;
		case FIELDCENTRIC:
			isFieldCentric = true;
			isObjectCentric = false;
			break;
		case OBJECTCENTRIC:
			isFieldCentric = false;
			isObjectCentric = true;
			break;
		}
	}

	public void changeFront(Side side) {
		// switch out of field centric
		// set the robot front (N,E,S,W)
		switch (side) {
		case NORTH:
			isFieldCentric = false;
			isObjectCentric = false;
			orientationOffset = 0;
			break;
		case EAST:
			isFieldCentric = false;
			isObjectCentric = false;
			orientationOffset = -90;
			break;
		case SOUTH:
			isFieldCentric = false;
			isObjectCentric = false;
			orientationOffset = 180;
			break;
		case WEST:
			isFieldCentric = false;
			isObjectCentric = false;
			orientationOffset = 90;
			break;
		}
	}

	public void resetOrentation() {
		orientationOffset = 0;
		ahrs.reset();
	}
	
	public void setRotationTarget(double distance) {
		rotateTarget = distance;
	}

	public void clearRotationTarge() {
		rotateTarget = 0;
	}

	public void setDriveSpeed(double speed) {
		speed = Math.abs(speed);
		if (speed > 1)
			speed = 1;
		maxTargetSpeed = speed;
	}

	public void printPositions() {
		for (SwerveWheel wheel : wheelArray)
			System.out.print(wheel.name + "'s position: " + wheel.getRawEncoderValue() + ", ");
		System.out.println();
	}

	public void calibrateMinMax() {
		System.out.println("Getting the Mins and Maxs of the Swerve Wheels.");
		for (SwerveWheel wheel : wheelArray) {
			double speed;
			int currentValue = wheel.getRawEncoderValue();
			int previousValue = currentValue;
			int min = currentValue;
			int max = currentValue;
			byte sameNumberCount = 0;

			for (int i = 0; i < 3; i++) {
				speed = 0.01;
				wheel.rawRotate(speed);
				while (previousValue <= currentValue) {
					previousValue = currentValue;
					currentValue = wheel.getRawEncoderValue();
					if (previousValue == currentValue) {
						sameNumberCount++;
						if (sameNumberCount > 20) {
							speed += 0.001;
							wheel.rawRotate(speed);
						}
					} else {
						sameNumberCount = 0;
					}
					if (currentValue > max)
						max = currentValue;
					if (currentValue < min)
						min = currentValue;
				}
				System.out.println(wheel.name + " min: " + min + " max: " + max);

			}
			wheel.rawRotate(0);
		}
	}

	public void calibrateHome() {
		for (SwerveWheel wheel : wheelArray)
			System.out.println(wheel.name + " Home: " + wheel.getRawEncoderValue());
	}

	private void stopMoving() {
		for (SwerveWheel wheel : wheelArray) {
			wheel.setSpeed(0);
			wheel.drive();
		}
	}
}