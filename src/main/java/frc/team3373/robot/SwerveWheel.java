package frc.team3373.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class SwerveWheel {
	WPI_TalonSRX rotateMotor; // The motor which spins the assembly
	WPI_TalonSRX driveMotor; // The motor powering the wheel

	private double targetAngle = 0;
	private double targetSpeed = 0;
	private double rotAngle;
	public String name; // used for helpful debug
	private int EMin;
	private int EMax;
	private int EHome;
	private boolean reverseDir = false;

	private double stepPerDegree;

	public SwerveWheel(String Name, int rotateMotorID, int driveMotorID, int EncMin, int EncMax, int EncHome,
			double rotationAngle) {
		rotateMotor = new WPI_TalonSRX(rotateMotorID);
		driveMotor = new WPI_TalonSRX(driveMotorID);

		EMin = EncMin;
		EMax = EncMax;
		EHome = EncHome;
		rotAngle = rotationAngle;
		stepPerDegree = (EMax - EMin) / 360.0;

		name = Name;

		System.out.println(stepPerDegree);
		rotateMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
		rotateMotor.setSelectedSensorPosition(rotateMotor.getSensorCollection().getAnalogInRaw(), 0, 0);
		rotateMotor.overrideLimitSwitchesEnable(false);

		rotateMotor.setNeutralMode(NeutralMode.Brake); // Activates brake mode
		driveMotor.setNeutralMode(NeutralMode.Brake);
	}

	public void setTargetAngle(double angle) {
		targetAngle = angle;
	}

	public void setSpeed(double speed) {
		if (speed > 1)
			speed = 1;
		if (speed < -1)
			speed = -1;

		targetSpeed = speed;
	}

	public double getCurrentAngle() {
		double deg = (rotateMotor.getSensorCollection().getAnalogInRaw() - EHome);
		if (deg < 0) {
			deg += EMax;
			deg -= EMin;
		}
		return deg / stepPerDegree;
	}

	public double getRAngle() {
		return rotAngle;
	}

	public int getRawEncoderValue() {
		return rotateMotor.getSensorCollection().getAnalogInRaw();
	}

	public void goToAngle() {
		double current = getCurrentAngle();

		if (targetAngle > 360) {
			targetAngle -= 360;
		} else if (targetAngle < 0) {
			targetAngle += 360;
		}

		double opposite = 180 + targetAngle;

		if (opposite > 360) {
			opposite -= 360;
		} else if (opposite < 0) {
			opposite += 360;
		}

		if (Math.abs(targetAngle - current) > Math.abs(opposite - current)) {
			reverseDir = true;
			targetAngle = opposite;
		} else {
			reverseDir = false;
		}

		double target = targetAngle * stepPerDegree + EHome;

		if (target > EMax) {
			target -= EMax;
			target += EMin;
		}
		// System.out.println(target);

		rotateMotor.set(ControlMode.Position, target);
	}

	public void drive() {
		if (reverseDir) {
			driveMotor.set(-targetSpeed);
		} else {
			driveMotor.set(targetSpeed);
		}
	}

	public void rawRotate(double speed) {
		rotateMotor.set(speed);
	}
}
