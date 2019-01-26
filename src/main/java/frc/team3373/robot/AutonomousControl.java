package frc.team3373.robot;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import edu.wpi.first.wpilibj.DigitalInput;

public class AutonomousControl {

	public SuperAHRS ahrs;
	public SwerveControl swerve;
	public Ultrasonic ultra;
	public Object auto;
	public DistanceSensor dist1;
	public DistanceSensor dist2;
	public SuperJoystick shooter;
	public SuperJoystick driver;
	public DigitalInput line;

	// Initializes self and defines AHRS, swerve, and ultrasonic sensor
	public AutonomousControl(SuperAHRS ah, SwerveControl swer, Ultrasonic ult, DistanceSensor d1, DistanceSensor d2,
			SuperJoystick driv, SuperJoystick shoo, DigitalInput line) {
		ahrs = ah;
		swerve = swer;
		ultra = ult;
		dist1 = d1;
		dist2 = d2;
		shooter = shoo;
		driver = driv;
	}

	// Initializes auto by file name and passes in itself as a parameter
	public boolean start(String au) {
		String aut = "frc.team3373.autonomous." + au;
		try {
			auto = Class.forName(aut).getConstructor(AutonomousControl.class).newInstance(this);
		} catch (ClassNotFoundException | IllegalAccessException | InstantiationException | InvocationTargetException
				| NoSuchMethodException ex) {
			System.out.println(ex);
			return false;
		}

		return true;
	}

	// Starts the autonomous running
	public void run() {
		try {
			Method run = auto.getClass().getMethod("run");
			run.invoke(auto);
		} catch (NoSuchMethodException | SecurityException | IllegalArgumentException | InvocationTargetException
				| IllegalAccessException ex) {
			System.out.println(ex);
		}
	}

	// Drives for x milliseconds at y angle and z speed
	public void driveAtAngle(long milliseconds, float angle, double speed) {
		swerve.calculateAutoSwerveControl(angle, speed, 0);
		try {
			Thread.sleep(milliseconds);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		swerve.calculateAutoSwerveControl(angle, 0, 0);
	}

	// Calls rotateRelative(angle, speed) until finished
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

	// Calls rotateAbsolute(angle, speed) until finished
	public void rotateAbsolute(float angle, double speed) {
		double initialAngle = ahrs.getRotation();
		double targetAngle = angle;
		if (targetAngle >= 360) {
			targetAngle -= 360;
		} else if (targetAngle < 0) {
			targetAngle += 360;
		}

		// calculateAutoSwerveControl(0,0,);
	}

	// Calls rotateAroundObject(angle, distance, speed) until finished
	public void rotateAroundObject(float angle, float distance, double speed) {
		swerve.setDistanceToObject(distance);
		// calculateObjectControl(speed);
	}

	public double getUltrasonicDistance() {
		return ultra.getDistance();
	}
}