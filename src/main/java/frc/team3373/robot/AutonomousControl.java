package frc.team3373.robot;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.autonomous.Lineup;
import frc.team3373.robot.SwerveControl.DriveMode;
import frc.team3373.robot.SwerveControl.Side;

public class AutonomousControl {

	private SuperAHRS ahrs;
	private SwerveControl swerve;
	private Ultrasonic ultra;
	private Object auto;
	private DistanceSensor distl;
	private DistanceSensor distr;
	//private SuperJoystick shooter;
	private SuperJoystick driver;
	private DigitalInput line;

	//private PIDController pidAngle;
	private PIDController pidRel;
	private PIDController pidAbs;

	private SuperPIDOutput swerveOut;
	//private DistanceSensorPID distPID;

	private Lineup lineup;

	// Initializes self and defines AHRS, swerve, and ultrasonic sensor
	public AutonomousControl(SuperAHRS ahrs, SwerveControl swerve, DistanceSensor dl, DistanceSensor dr,
			SuperJoystick driver, SuperJoystick shooter, DigitalInput line) {
		this.ahrs = ahrs;
		this.swerve = swerve;
		distl = dl;
		distr = dr;
		//this.shooter = shooter;
		this.driver = driver;
		this.line = line;

		lineup = new Lineup(distl, distr, this.driver, swerve, this.line);

		swerveOut = new SuperPIDOutput(SuperPIDOutput.OutputType.ROTATE, this.swerve);
		//distPID = new DistanceSensorPID(distl, distr);

		//pidAngle = new PIDController(Constants.angleP, Constants.angleI, Constants.angleD, this.ahrs, swerveOut);
		pidAbs = new PIDController(Constants.getNumber("angleP"), Constants.getNumber("angleI"), Constants.getNumber("angleD"), this.ahrs, swerveOut);
		pidRel = new PIDController(Constants.getNumber("angleP"), Constants.getNumber("angleI"), Constants.getNumber("angleD"), this.ahrs, swerveOut);

		pidRel.setOutputRange(-0.2, 0.2);
		pidRel.setInputRange(-180, 180);
		pidRel.setContinuous(true);
		pidRel.setAbsoluteTolerance(0.1);
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
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}

	public void driveForRotations(double distance, float angle, double speed) {
		swerve.resetTravelDistance();
		swerve.calculateAutoSwerveControl(angle, speed, 0);
		while (swerve.getTravelDistance() <= distance && !driver.isXHeld() && !RobotState.isDisabled()) {
			swerve.calculateAutoSwerveControl(angle, speed, 0);
		}
		swerve.calculateAutoSwerveControl(0, 0, 0);
	}

	// Calls rotateRelative(angle, speed) until finished
	public void rotateRelative(float angle, double speed) {
		pidRel.setPID(SmartDashboard.getNumber("P", 0), SmartDashboard.getNumber("I", 0),
				SmartDashboard.getNumber("D", 0));
		double initalAngle = ahrs.getYaw();
		double targetAngle = initalAngle + angle;
		SmartDashboard.putNumber("targetAngle", targetAngle);
		if (targetAngle >= 360) {
			targetAngle -= 360;
		} else if (targetAngle < 0) {
			targetAngle += 360;
		}

		//int count = 0;
		int outCount = 0;

		double outputTolerance = SmartDashboard.getNumber("pOut Tolerance", 0.01); // 0.015
		double outputCount = SmartDashboard.getNumber("outCount", 100); // 7500
		//double countTolerance = SmartDashboard.getNumber("PID Count", 100); // 6000

		pidRel.setOutputRange(-speed, speed);
		ahrs.setTargetAngle(targetAngle);
		pidRel.setAbsoluteTolerance(0.1);
		SmartDashboard.putBoolean("Semi-Auto", true);
		pidRel.enable();
		while (!driver.isXHeld() && !RobotState.isDisabled()) {
			SmartDashboard.putNumber("Yaw", ahrs.getYaw());
			SmartDashboard.putNumber("pOut", swerveOut.getPOutput());
			if (SmartDashboard.getBoolean("Self-Disable", true)) {
				if (swerveOut.getPOutput() == 0) {
				} else if (Math.abs(swerveOut.getPOutput()) < outputTolerance && outCount >= outputCount) {
					pidRel.disable();
					swerve.calculateAutoSwerveControl(0, 0, 0);
					SmartDashboard.putBoolean("Semi-Auto", false);
					return;
				} else if (Math.abs(swerveOut.getPOutput()) < outputTolerance) {
					outCount++;
				} else if (outCount != 0 && !(Math.abs(swerveOut.getPOutput()) < outputTolerance)) {
					outCount = 0;
				}

				/* if (pidRel.onTarget() && count >= countTolerance) { // If 200 or more samples are within the deadband, disable PID and return
					pidRel.disable();
					swerve.calculateAutoSwerveControl(0, 0, 0);
					SmartDashboard.putNumber("onTarget Yaw", ahrs.getYaw());
					SmartDashboard.putBoolean("Semi-Auto", false);
					return;
				} else if (pidRel.onTarget()) {
					count++;
				} else if (count != 0 && !pidRel.onTarget()) {
					count = 0;
				} */
			}
		}
		pidRel.disable();
		swerve.calculateAutoSwerveControl(0, 0, 0);
		SmartDashboard.putBoolean("Semi-Auto", false);
	}

	public void driveSquare() {
		swerve.setControlMode(DriveMode.ROBOTCENTRIC);
		swerve.changeFront(Side.NORTH);
		driveForRotations(20, 90, 0.2);
		rotateAbsolute(90, 0.2);
		driveForRotations(20, 90, 0.2);
		rotateAbsolute(180, 0.2);
		driveForRotations(20, 90, 0.2);
		rotateAbsolute(270, 0.2);
		driveForRotations(20, 90, 0.2);
		rotateAbsolute(0, 0.2);
	}

	public SwerveControl getSwerve() {
		return swerve;
	}

	// Calls rotateAbsolute(angle, speed) until finished
	public void rotateAbsolute(float angle, double speed) {
		System.out.println("Starting turn");
		double targetAngle = angle;
		if (targetAngle >= 360) {
			targetAngle -= 360;
		} else if (targetAngle < 0) {
			targetAngle += 360;
		}

		int count = 0;
		int outCount = 0;

		double outputTolerance = 0.015; // 0.015
		double outputCount = 5000; // 12500 or 7500
		double countTolerance = 6000; // 6000

		pidAbs.setOutputRange(-speed, speed);
		ahrs.setTargetAngle(targetAngle);
		pidAbs.setAbsoluteTolerance(0.1); // 0.05

		SmartDashboard.putBoolean("Semi-Auto", true);
		pidAbs.enable();
		while (!driver.isXHeld() && !RobotState.isDisabled()) {
			SmartDashboard.putNumber("Yaw", ahrs.getYaw());
			if (SmartDashboard.getBoolean("Self-Disable", true)) {
				if (swerveOut.getPOutput() == 0) {
				} else if (Math.abs(swerveOut.getPOutput()) < outputTolerance && outCount >= outputCount) {
					pidAbs.disable();
					swerve.calculateAutoSwerveControl(0, 0, 0);
					SmartDashboard.putBoolean("Semi-Auto", false);
					return;
				} else if (Math.abs(swerveOut.getPOutput()) < outputTolerance) {
					outCount++;
				}

				if (pidAbs.onTarget() && count >= countTolerance) { // If 200 or more samples are within the deadband, disable PID and return
					pidAbs.disable();
					swerve.calculateAutoSwerveControl(0, 0, 0);
					SmartDashboard.putBoolean("Semi-Auto", false);
					return;
				} else if (pidAbs.onTarget()) {
					count++;
				} else if (count != 0 && !pidAbs.onTarget()) {
					count = 0;
				}
			}
		}
		pidAbs.disable();
		swerve.calculateAutoSwerveControl(0, 0, 0);
		SmartDashboard.putBoolean("Semi-Auto", false);
	}

	// Calls rotateAroundObject(angle, distance, speed) until finished
	public void rotateAroundObject(float angle, float distance, double speed) {
		swerve.setDistanceToObject(distance);
		// calculateObjectControl(speed);
	}

	public double getUltrasonicDistance() {
		return ultra.getDistance();
	}

	public void lineup(Lineup.AlignDirection align) {
		lineup.align(align);
	}

	public void rocketLineup(Lineup.AlignDirection align) {
		lineup.rocketAlign(align);
	}

	public void square() {
		lineup.square();
	}
}