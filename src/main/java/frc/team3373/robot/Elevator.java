package frc.team3373.robot;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {
    private CANSparkMax motor;
    private CANPIDController pid;

    private SuperJoystick joystick;

    private double offset;

    private final double slope = 4.883728566344149;

    private boolean calibrating;
    private int calStep;

    private double x1, y1, x2, y2, calSlope = 0;

    // private AnalogInput leftLimit;
    // private AnalogInput rightLimit;

    public Elevator(int motorID, SuperJoystick shooter, int leftLimitPort, int rightLimitPort) {
        motor = new CANSparkMax(motorID, MotorType.kBrushless);

        joystick = shooter;
        calibrating = false;

        motor.setIdleMode(IdleMode.kBrake);

        pid = motor.getPIDController();
        pid.setP(0.2);
        pid.setI(0.001);
        pid.setOutputRange(-0.1, 0.1);
        motor.getEncoder().setPositionConversionFactor(slope);

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
            motor.getEncoder().setPositionConversionFactor(1);
            SmartDashboard.putBoolean("Calibrating", true);
            calibrating = true;
            return;
        }
        switch(calStep) {
            case 0:
                x1 = motor.getEncoder().getPosition();
                y1 = SmartDashboard.getNumber("Inches", 0);
                calStep++;
                break;
            case 1:
                x2 = motor.getEncoder().getPosition();
                y2 = SmartDashboard.getNumber("Inches", 0);
                calSlope = (y2 - y1) / (x2 - x1);
                SmartDashboard.putNumber("Slope", calSlope);
                motor.getEncoder().setPositionConversionFactor(calSlope);
                calStep = 0;
                calibrating = false;
                SmartDashboard.putBoolean("Calibrating", false);
                break;
        }
    }

    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    public void zero() {
        motor.getEncoder().setPosition(0);
    }

    public void zero(int inches) {
        motor.getEncoder().setPosition(inches);
    }

    public void moveToHeight(double inches) {
        pid.setP(SmartDashboard.getNumber("P", 0.2));
        pid.setI(SmartDashboard.getNumber("I", 0.001));
        pid.setD(SmartDashboard.getNumber("D", 0));
        pid.setReference(inches, ControlType.kPosition);
    }

    public void moveToPosition(int height, Robot.ObjectType obj) {
        switch (height) {
        case 0:
            switch (obj) {
            case CARGO:
                
                break;
            case HATCH:
                
                break;
            default:
                System.out.println("Object must be defined!");
                return;
            }
            break;
        case 1:
            switch (obj) {
            case CARGO:
                
                break;
            case HATCH:
                
                break;
            default:
                System.out.println("Object must be defined!");
                return;
            }
            break;
        case 2:
            switch (obj) {
            case CARGO:
                
                break;
            case HATCH:
                
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
