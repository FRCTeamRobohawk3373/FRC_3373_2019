package frc.team3373.autonomous;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.robot.Constants;
import frc.team3373.robot.DistanceSensor;
import frc.team3373.robot.DistanceSensorPID;
import frc.team3373.robot.SuperJoystick;
import frc.team3373.robot.SuperPIDOutput;
import frc.team3373.robot.SwerveControl;
import frc.team3373.robot.SwerveControl.DriveMode;

public class Lineup {
    private PIDController pid;
    private SwerveControl swerve;
    private SuperJoystick joystick;
    private SuperPIDOutput output;
    private DistanceSensorPID dist;
    private DistanceSensor dright;
    private DistanceSensor dleft;
    private DigitalInput line;

    public static enum AlignDirection { // Stores the direction that the robot should search for the line
        RIGHT, LEFT
    }

    public Lineup(DistanceSensor dl, DistanceSensor dr, SuperJoystick driver, SwerveControl sw, DigitalInput line) {
        swerve = sw;
        joystick = driver;
        dleft = dl;
        dright = dr;
        output = new SuperPIDOutput(SuperPIDOutput.OutputType.ROTATE, sw);
        dist = new DistanceSensorPID(dleft, dright);
        pid = new PIDController(Constants.getNumber("lineupP"), Constants.getNumber("lineupI"), Constants.getNumber("lineupD"), dist, output);
        this.line = line;

        pid.setAbsoluteTolerance(0.1);
        pid.setContinuous(false);
        pid.setOutputRange(-0.175, 0.175);
        pid.setInputRange(3, 32);
    }

    public void square() {
        boolean exit = false;
        pid.setAbsoluteTolerance(Constants.getNumber("lineupTolerance", 0.2)); // Sets deadband for PID

        int count = 0; // Stores how many values have been tested
        pid.setP(Constants.getNumber("lineupP", 0.1)); // TODO: Remove
        pid.setI(Constants.getNumber("lineupI", 0));
        pid.setD(Constants.getNumber("lineupD", 0));

        DriveMode mode = swerve.getControlMode(); // Gets swerve control mode
        swerve.setControlMode(DriveMode.ROBOTCENTRIC);

        pid.enable(); // Enables PID loop

        while (!joystick.isXHeld() && !RobotState.isDisabled() && !exit) {
            if (pid.onTarget() && count >= Constants.getNumber("lineupCount", 10)) { // If 200 or more samples are within the deadband,
                                                  // disable PID and return
                pid.disable();
                swerve.calculateAutoSwerveControl(0, 0, 0);
                swerve.setControlMode(mode);
                return;
            } else if (pid.onTarget()) {
                count++;
                System.out.println(count);
            } else if (count != 0 && !pid.onTarget()) {
                count = 0;
            }
        }
        pid.disable();
        swerve.calculateAutoSwerveControl(0, 0, 0);
        swerve.setControlMode(mode);
    }

    public void rocketAlign(AlignDirection al) {
        SmartDashboard.putBoolean("Aligned", false);

        DriveMode mode = swerve.getControlMode(); // Gets swerve control mode
        swerve.setControlMode(DriveMode.ROBOTCENTRIC);

        boolean fin = false;
        int state = 0; // Stores the step that the lineup is on
        int count = 0;
        double distance;

        AlignDirection align = al; // Holds which way the line is from the robot

        while (!joystick.isXHeld() && !RobotState.isDisabled()) {
            switch (state) {
            case 0:
                distance = (dleft.getDistance() + dright.getDistance()) / 2;
                if (distance < 14 && distance > 13) {
                    if (!fin) {
                        state++;
                    } else {
                        state = 5;
                    }
                } else if (distance > 14) {
                    swerve.calculateAutoSwerveControl(90, 0.1, 0);
                } else if (distance < 13) {
                    swerve.calculateAutoSwerveControl(270, 0.1, 0);
                }
                break;
            case 1:
                if (line.get()) {
                    state = 4;
                }
                switch (align) {
                case RIGHT:
                    swerve.calculateAutoSwerveControl(0, 0.075, 0);
                    if (!fin) {
                        state++;
                    } else {
                        state = 5;
                    }
                    break;
                case LEFT:
                    swerve.calculateAutoSwerveControl(180, 0.075, 0);
                    if (!fin) {
                        state++;
                    } else {
                        state = 5;
                    }
                    break;
                default:
                    System.out.println("AlignDirection must be defined");
                    swerve.setControlMode(mode);
                    SmartDashboard.putBoolean("Aligned", true);
                    return;
                }
                break;
            case 2: // When line is sensed for a certain number of times, go to case 5
                if (line.get() && count == 3) {
                    if (!fin) {
                        state++;
                    } else {
                        state = 5;
                    }
                } else if (line.get()) {
                    count++;
                } else if (count != 0 && !line.get()) {
                    count = 0;
                }
                break;
            case 3: // If 200 or more samples are within the deadband, disable PID and go to case 1
                square();
                state++;
                break;
            case 4:
                fin = true;
                distance = (dleft.getDistance() + dright.getDistance()) / 2;
                if (!line.get()) {
                    if (align == AlignDirection.LEFT) {
                        align = AlignDirection.RIGHT;
                    } else {
                        align = AlignDirection.LEFT;
                    }
                    state = 1;
                }
                if (!(distance < 14 && distance > 13)) {
                    state = 0;
                }
            case 5:
                swerve.setControlMode(mode);
                SmartDashboard.putBoolean("Aligned", true);
                break;
            default:
                System.out.println("State must be 0-3");
                pid.disable();
                swerve.calculateAutoSwerveControl(0, 0, 0);
                swerve.setControlMode(mode);
                SmartDashboard.putBoolean("Aligned", true);
                return;
            }
        }
        SmartDashboard.putBoolean("Aligned", true);
    }

    public void align(AlignDirection al) {
        SmartDashboard.putBoolean("Aligned", false);

        DriveMode mode = swerve.getControlMode(); // Gets swerve control mode
        swerve.setControlMode(DriveMode.ROBOTCENTRIC);

        int state = 0; // Stores the step that the lineup is on
        int count = 0;
        boolean fin = false;

        double distance;

        AlignDirection align = al; // Holds which way the line is from the robot

        while (!joystick.isXHeld() && !RobotState.isDisabled()) {
            switch (state) {
            case 0: // If line is sensed, return. Else, search in the specified direction
                distance = (dleft.getDistance() + dright.getDistance()) / 2;
                if (distance < 12 && distance > 11) {
                    if (!fin) {
                        state++;
                    } else {
                        state = 5;
                    }
                } else if (distance > 12) {
                    swerve.calculateAutoSwerveControl(90, 0.1, 0);
                } else if (distance < 11) {
                    swerve.calculateAutoSwerveControl(270, 0.1, 0);
                }
                break;
            case 1:
                square();
                if (!fin) {
                    state++;
                } else {
                    state = 5;
                }
                break;
            case 2:
                if (line.get()) {
                    state = 4;
                }
                switch (align) {
                case RIGHT:
                    swerve.calculateAutoSwerveControl(0, 0.1, 0);
                    if (!fin) {
                        state++;
                    } else {
                        state = 5;
                    }
                    break;
                case LEFT:
                    swerve.calculateAutoSwerveControl(180, 0.1, 0);
                    if (!fin) {
                        state++;
                    } else {
                        state = 5;
                    }
                    break;
                default:
                    System.out.println("AlignDirection must be defined");
                    swerve.setControlMode(mode);
                    SmartDashboard.putBoolean("Aligned", true);
                    return;
                }
                break;
            case 3: // When line is sensed for a certain number of times, go to case 5
                if (line.get() && count == 3) {
                    if (!fin) {
                        state++;
                    } else {
                        state = 5;
                    }
                } else if (line.get()) {
                    count++;
                } else if (count != 0 && !line.get()) {
                    count = 0;
                }
                break;
            case 4: // If 200 or more samples are within the deadband, disable PID and go to case 1
                square();
                state++;
                break;
            case 5:
                fin = true;
                distance = (dleft.getDistance() + dright.getDistance()) / 2;
                if (!line.get()) {
                    if (align == AlignDirection.LEFT) {
                        align = AlignDirection.RIGHT;
                    } else {
                        align = AlignDirection.LEFT;
                    }
                    state = 1;
                } else if (!(distance < 14 && distance > 13)) {
                    state = 0;
                } else if (Math.abs(dist.pidGet()) > 0.1) {
                    state = 0;
                } else {
                    state++;
                }
            case 6:
                swerve.setControlMode(mode);
                SmartDashboard.putBoolean("Aligned", true);
                break;
            default:
                System.out.println("State must be 0-3");
                pid.disable();
                swerve.calculateAutoSwerveControl(0, 0, 0);
                swerve.setControlMode(mode);
                SmartDashboard.putBoolean("Aligned", true);
                return;
            }
        }
        SmartDashboard.putBoolean("Aligned", true);
    }
}