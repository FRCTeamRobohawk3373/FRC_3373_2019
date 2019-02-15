package frc.team3373.robot;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {
    private CANSparkMax motor;
    private CANPIDController pid;

    private SuperJoystick joystick;

    private double cargoPosition1;
    private double cargoPosition2;
    private double cargoPosition3;

    private double hatchPosition1;
    private double hatchPosition2;
    private double hatchPosition3;

    private double offset;

    private boolean calibrating;
    private int calStep;

    // private AnalogInput leftLimit;
    // private AnalogInput rightLimit;

    public Elevator(int motorID, SuperJoystick shooter, int leftLimitPort, int rightLimitPort) {
        motor = new CANSparkMax(motorID, MotorType.kBrushless);

        joystick = shooter;
        calibrating = false;

        pid = motor.getPIDController();
        pid.setP(0.2);
        pid.setI(0.001);
        pid.setOutputRange(-0.2, 0.2);

        calStep = 0;

        pid.setReference(0, ControlType.kPosition);
        // leftLimit = new AnalogInput(leftLimitPort);
        // rightLimit = new AnalogInput(rightLimitPort);
    }

    public void move(double speed) {
        if (Math.abs(speed) < 0.05) {
            motor.set(0);
        } else {
            motor.set(speed * 0.15);
        }
    }

    public void calibrate() {
        if (!calibrating) {
            SmartDashboard.putBoolean("Calibrating", true);
            calibrating = true;
            return;
        }
        switch(calStep) {
            case 0:
                hatchPosition1 = motor.getEncoder().getPosition();
                SmartDashboard.putNumber("hatchPosition1", hatchPosition1);
                //cargoPosition1 = motor.getEncoder().getPosition();
                calStep++;
                SmartDashboard.putNumber("CalStep", calStep);
                SmartDashboard.putNumber("Step", 1);
                break;
            case 1:
                hatchPosition2 = motor.getEncoder().getPosition();
                SmartDashboard.putNumber("hatchPosition2", hatchPosition2);
                //cargoPosition2 = motor.getEncoder().getPosition();
                calStep++;
                SmartDashboard.putNumber("CalStep", calStep);
                SmartDashboard.putNumber("Step", 2);
                break;
            case 2:
                hatchPosition3 = motor.getEncoder().getPosition();
                SmartDashboard.putNumber("hatchPosition3", hatchPosition3);
                //cargoPosition3 = motor.getEncoder().getPosition();
                calStep++;
                SmartDashboard.putNumber("CalStep", calStep);
                SmartDashboard.putNumber("Step", 3);
                break;
            case 3:
                SmartDashboard.putBoolean("Calibrating", false);
                calStep = 0;
                break;
        }
    }

    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    public void zero() {
        offset = motor.getEncoder().getPosition();
    }

    public void moveToHeight(int height, Robot.ObjectType obj) {
        switch (height) {
        case 0:
            switch (obj) {
            case CARGO:
                pid.setReference(cargoPosition1 + offset, ControlType.kPosition);
                break;
            case HATCH:
                pid.setReference(hatchPosition1 + offset, ControlType.kPosition);
                break;
            default:
                System.out.println("Object must be defined!");
                return;
            }
            break;
        case 1:
            switch (obj) {
            case CARGO:
                pid.setReference(cargoPosition2 + offset, ControlType.kPosition);
                break;
            case HATCH:
                pid.setReference(hatchPosition2 + offset, ControlType.kPosition);
                break;
            default:
                System.out.println("Object must be defined!");
                return;
            }
            break;
        case 2:
            switch (obj) {
            case CARGO:
                pid.setReference(cargoPosition3 + offset, ControlType.kPosition);
                break;
            case HATCH:
                pid.setReference(hatchPosition3 + offset, ControlType.kPosition);
                break;
            default:
                System.out.println("Object must be defined!");
                return;
            }
            break;
        default:
            System.out.println("Height must be 0, 1, or 2");
        }
    }
}
