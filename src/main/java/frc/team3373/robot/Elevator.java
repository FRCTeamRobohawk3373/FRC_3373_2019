package frc.team3373.robot;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {
    private CANSparkMax motor;
    private CANPIDController pid;

    private boolean calibrating;
    private int calStep;

    private double[] x;
    private double[] y;

    private double position;

    private double slope;

    private int calLength;
    private int calInches;

    private CANDigitalInput reverseLimit;
    private CANDigitalInput forwardLimit;

    private boolean zeroing;

    public Elevator(int motorID, SuperJoystick shooter) {
        motor = new CANSparkMax(motorID, MotorType.kBrushless);
        calibrating = false;

        motor.setIdleMode(IdleMode.kBrake);

        pid = motor.getPIDController();
        pid.setP(Constants.getNumber("elevatorP"));
        pid.setI(Constants.getNumber("elevatorI"));
        pid.setD(Constants.getNumber("elevatorD"));
        pid.setOutputRange(-0.05, 0.05); // TODO: Change speed for robot
        slope = Constants.getNumber("elevatorSlope");

        reverseLimit = motor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

        calStep = 0;

        zeroing = false;

        motor.set(0);
        absoluteZero();
        reverseLimit.enableLimitSwitch(false);
        System.out.println(reverseLimit.isLimitSwitchEnabled());
        // leftLimit = new AnalogInput(leftLimitPort);
        // rightLimit = new AnalogInput(rightLimitPort);
    }

    public void initPID() {
        position = motor.getEncoder().getPosition();
        pid.setReference(position, ControlType.kPosition);
        SmartDashboard.putNumber("Set Position", position);
    }

    public void rawMovePID(double increment) { // Moves motor by an increments, used for calibration; BE CAREFUL!!!
        pid.setOutputRange(-0.1, 0.1);
        if (Math.abs(increment) > 0.05 && Math.abs(increment) <= 1 && RobotState.isTest()) {
            position += increment * 0.075;
            if (position >= Constants.getNumber("elevatorMaxRotations"))
                position = Constants.getNumber("elevatorMaxRotations");
            pid.setReference(position, ControlType.kPosition);
            SmartDashboard.putNumber("Set Position", position);
        }
    }

    public void refresh() { // Fail-safes and zero checks
        SmartDashboard.putBoolean("reverseLimit", reverseLimit.get());
        if (motor.getEncoder().getPosition() >= Constants.getNumber("elevatorMaxRotations")
                && position > Constants.getNumber("elevatorMaxRotations")) {
            position = Constants.getNumber("elevatorMaxRotations");
            pid.setReference(position, ControlType.kPosition);
        }

        if (motor.getEncoder().getPosition() < 0 && position < 0) {
            position = 0;
            pid.setReference(position, ControlType.kPosition);
        }

        if (motor.get() > 0.2) { // TODO: change to elevator speed
            motor.set(0);
        }

        if (zeroing && !reverseLimit.get()) {
            motor.set(0);
            motor.setIdleMode(IdleMode.kCoast);
        } else if (zeroing && reverseLimit.get()) {
            zeroing = false;
            absoluteZero();
            motor.setIdleMode(IdleMode.kBrake);
        }

        if (reverseLimit.get()) {
            absoluteZero();
            zeroing = false;
        }
    }

    public double getPosition() { // Returns position in inches
        return motor.getEncoder().getPosition() * slope;
    }

    public double getRotations() {
        return motor.getEncoder().getPosition();
    }

    public void absoluteZero() { // Zeroes the elevator
        motor.getEncoder().setPosition(0);
        pid.setReference(0, ControlType.kPosition);
        position = 0;
    }

    public void zero() { // Initializes zeroing for Teleop
        zeroing = true;
    }

    public void cancel() {
        motor.set(0);
        position = 0;
    }

    public void moveToHeight(double inches) { // Moves elevator to a height after checking the position against the min and max heights
        inches -= Constants.getNumber("elevatorMinHeight");
        if (inches < 0)
            inches = 0;
        if (inches >= 0 && inches <= Constants.getNumber("elevatorMaxHeight")) {
            pid.setOutputRange(-0.05, 0.075);
            slope = Constants.getNumber("elevatorSlope", 3.12);
            pid.setP(Constants.getNumber("elevatorP", 0.2)); // TODO: Remove
            pid.setI(Constants.getNumber("elevatorI"));
            pid.setD(Constants.getNumber("elevatorD"));
            pid.setFF(Constants.getNumber("elevatorFF"));
            position = inches / slope;
            pid.setReference(position, ControlType.kPosition);
            SmartDashboard.putNumber("Set Position", position);
        }
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

    public void calibrate() {
        if (!calibrating) {
            SmartDashboard.putBoolean("Calibrating", true);
            calibrating = true;
            calLength = (int) SmartDashboard.getNumber("Calibration Length", 2);
            calInches = (int) SmartDashboard.getNumber("Inches", 10);
            x = new double[calLength];
            y = new double[calLength];
            return;
        }

        if (calStep < calLength) {
            x[calStep] = motor.getEncoder().getPosition();
            y[calStep] = calInches * calStep;
            System.out.println(Integer.toString(calInches * calStep));
            calStep++;
            return;
        }

        calStep = 0;
        slope = linReg(x, y);
        x = null;
        y = null;
        calibrating = false;
        Constants.writeNumber("elevatorSlope", slope);
        SmartDashboard.putBoolean("Calibrating", false);
    }

    public void resetCal() {
        calStep = 0;
        x = null;
        y = null;
        calibrating = false;
        SmartDashboard.putBoolean("Calibrating", false);
        zeroing = false;
    }

    private double linReg(double[] x, double[] y) { // Linear regression algorithm, returns slope
        if (!(x.length == y.length)) {
            System.out.println("x and y must be the same length!");
        }
        int length = x.length;

        // first pass: read in data, compute xbar and ybar
        double sumx = 0.0, sumy = 0.0, sumx2 = 0.0;
        for (int i = 0; i < length; i++) {
            sumx += x[i];
            sumx2 += x[i] * x[i];
            sumy += y[i];
        }
        double xbar = sumx / length;
        double ybar = sumy / length;

        // second pass: compute summary statistics
        double xxbar = 0.0, yybar = 0.0, xybar = 0.0;
        for (int i = 0; i < length; i++) {
            xxbar += (x[i] - xbar) * (x[i] - xbar);
            yybar += (y[i] - ybar) * (y[i] - ybar);
            xybar += (x[i] - xbar) * (y[i] - ybar);
        }
        double beta1 = xybar / xxbar;
        double beta0 = ybar - beta1 * xbar;

        // print results
        System.out.println("y   = " + beta1 + " * x + " + beta0);

        // analyze results
        int df = length - 2;
        double rss = 0.0; // residual sum of squares
        double ssr = 0.0; // regression sum of squares
        for (int i = 0; i < length; i++) {
            double fit = beta1 * x[i] + beta0;
            rss += (fit - y[i]) * (fit - y[i]);
            ssr += (fit - ybar) * (fit - ybar);
        }
        double R2 = ssr / yybar;
        double svar = rss / df;
        double svar1 = svar / xxbar;
        double svar0 = svar / length + xbar * xbar * svar1;
        System.out.println("R^2                 = " + R2);
        System.out.println("std error of beta_1 = " + Math.sqrt(svar1));
        System.out.println("std error of beta_0 = " + Math.sqrt(svar0));
        svar0 = svar * sumx2 / (length * xxbar);
        System.out.println("std error of beta_0 = " + Math.sqrt(svar0));

        System.out.println("SSTO = " + yybar);
        System.out.println("SSE  = " + rss);
        System.out.println("SSR  = " + ssr);
        return beta1;
    }
}
