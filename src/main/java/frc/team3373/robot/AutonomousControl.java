package frc.team3373.robot;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.autonomous.Lineup;

public class AutonomousControl {

	public SuperAHRS ahrs;
	public SwerveControl swerve;
	public Ultrasonic ultra;
	public Object auto;
	public DistanceSensor distl;
	public DistanceSensor distr;
	public SuperJoystick shooter;
	public SuperJoystick driver;
	public DigitalInput line;

	//private PIDController pidAngle;
	private PIDController pidRel;
	//private PIDController pidAbs;

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
		this.shooter = shooter;
		this.driver = driver;
		this.line = line;

		lineup = new Lineup(distl, distr, driver, swerve, line);

		swerveOut = new SuperPIDOutput(SuperPIDOutput.Type.ROTATE, this.swerve);
		//distPID = new DistanceSensorPID(distl, distr);

		//pidAngle = new PIDController(Constants.angleP, Constants.angleI, Constants.angleD, this.ahrs, swerveOut);
		//pidAbs = new PIDController(Constants.absP, Constants.absI, Constants.absD, this.ahrs, swerveOut);
		pidRel = new PIDController(Constants.relP, Constants.relI, Constants.relD, this.ahrs, swerveOut);

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

		int count = 0;
		int outCount = 0;

		double outputTolerance = SmartDashboard.getNumber("pOut Tolerance", 0.01); // 0.015
		double outputCount = SmartDashboard.getNumber("outCount", 100); // 12500 or 7500
		double countTolerance = SmartDashboard.getNumber("PID Count", 100); // 6000

		pidRel.setOutputRange(-speed, speed);
		ahrs.setTargetAngle(targetAngle);
		pidRel.setAbsoluteTolerance(SmartDashboard.getNumber("PID Tolerance", 0.1)); // 0.05

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
				}

				if (pidRel.onTarget() && count >= countTolerance) { // If 200 or more samples are within the deadband, disable PID and return
					pidRel.disable();
					swerve.calculateAutoSwerveControl(0, 0, 0);
					SmartDashboard.putNumber("onTarget Yaw", ahrs.getYaw());
					SmartDashboard.putBoolean("Semi-Auto", false);
					return;
				} else if (pidRel.onTarget()) {
					count++;
				} else if (count != 0 && !pidRel.onTarget()) {
					count = 0;
				}
			}
		}
		pidRel.disable();
		swerve.calculateAutoSwerveControl(0, 0, 0);
		SmartDashboard.putBoolean("Semi-Auto", false);
	}

	// Calls rotateAbsolute(angle, speed) until finished
	public void rotateAbsolute(float angle, double speed) {
		double targetAngle = angle;
		if (targetAngle >= 360) {
			targetAngle -= 360;
		} else if (targetAngle < 0) {
			targetAngle += 360;
		}

		int count = 0;
		int outCount = 0;

		double outputTolerance = SmartDashboard.getNumber("pOut Tolerance", 0.01); // 0.015
		double outputCount = SmartDashboard.getNumber("outCount", 100); // 12500 or 7500
		double countTolerance = SmartDashboard.getNumber("PID Count", 100); // 6000

		pidRel.setOutputRange(-speed, speed);
		ahrs.setTargetAngle(targetAngle);
		pidRel.setAbsoluteTolerance(SmartDashboard.getNumber("PID Tolerance", 0.1)); // 0.05

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
				}

				if (pidRel.onTarget() && count >= countTolerance) { // If 200 or more samples are within the deadband, disable PID and return
					pidRel.disable();
					swerve.calculateAutoSwerveControl(0, 0, 0);
					SmartDashboard.putNumber("onTarget Yaw", ahrs.getYaw());
					SmartDashboard.putBoolean("Semi-Auto", false);
					return;
				} else if (pidRel.onTarget()) {
					count++;
				} else if (count != 0 && !pidRel.onTarget()) {
					count = 0;
				}
			}
		}
		pidRel.disable();
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

	public void square() {
		lineup.square();
	}
}