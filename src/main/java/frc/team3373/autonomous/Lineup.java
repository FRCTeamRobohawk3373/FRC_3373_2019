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
import frc.team3373.robot.SwerveControl.Side;

public class Lineup {
    private PIDController pid;
    private SwerveControl swerve;
    private SuperJoystick joystick;
    private SuperPIDOutput output;
    private DistanceSensorPID dist;
    private DistanceSensor dright;
    private DistanceSensor dleft;
    private DigitalInput line;

    /**
     * Stores the direction that the robot should search for the line: RIGHT, LEFT
     */
    public static enum AlignDirection {
        RIGHT, LEFT
    }

    /**
     * Initializes a class to control lining up the robot.
     * @param dl Left distance sensor
     * @param dr Right distance sensor
     * @param driver Joystick for driver
     * @param sw The SwerveControl for the robot
     * @param line Line sensor input
     */
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

    /**
     * Uses PID to square the robot. Press X on the driver joystick to cancel.
     */
    public void square() {
        boolean exit = false;
        pid.setAbsoluteTolerance(Constants.getNumber("lineupTolerance", 0.2)); // Sets deadband for PID

        int count = 0; // Stores how many values have been tested

        if (RobotState.isTest()) {
            pid.setP(Constants.getNumber("lineupP", 0.1));
            pid.setI(Constants.getNumber("lineupI", 0));
            pid.setD(Constants.getNumber("lineupD", 0));
        }

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

    public void objectSquare() {
        boolean exit = false;
        pid.setAbsoluteTolerance(Constants.getNumber("lineupTolerance", 0.2)); // Sets deadband for PID

        int count = 0; // Stores how many values have been tested

        if (RobotState.isTest()) {
            pid.setP(Constants.getNumber("lineupP", 0.1));
            pid.setI(Constants.getNumber("lineupI", 0));
            pid.setD(Constants.getNumber("lineupD", 0));
        }

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

    /**
     * Aligns the robot to the rocket. Press X on the driver joystick to cancel.
     * @param al The direction to search for the line
     */
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

    /**
     * Aligns the robot to the cargo ship. Press X on the driver joystick to cancel.
     * @param al The direction to search for the line
     */
    public boolean align(AlignDirection al) {
        SmartDashboard.putBoolean("Aligned", false);

        DriveMode mode = swerve.getControlMode(); // Gets swerve control mode
        swerve.setControlMode(DriveMode.ROBOTCENTRIC);
        swerve.changeFront(Side.NORTH);

        int state = 0; // Stores the step that the lineup is on
        int count = 0;
        boolean fin = false;
        boolean lin = false;
        boolean ali = false;
        boolean dis = false;

        double distance;

        AlignDirection align = al; // Holds which way the line is from the robot

        while (!joystick.isXHeld() && !RobotState.isDisabled()) {
            switch (state) {
            case 0:
                distance = (dleft.getDistance() + dright.getDistance()) / 2;
                if (distance < Constants.getNumber("lineupDistance", 12) + 0.5 && distance > Constants.getNumber("lineupDistance", 12) - 0.5) {
                    if (!fin) {
                        state++;
                    } else {
                        state = 5;
                    }
                } else if (distance > Constants.getNumber("lineupDistance", 12) - 0.5) {
                    swerve.calculateAutoSwerveControl(90, 0.1, 0);
                } else if (distance < Constants.getNumber("lineupDistance", 12) + 0.5) {
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
                    return false;
                }
                break;
            case 3: // When line is sensed for a certain number of times, go to case 5
                if (line.get() && count == 3) {
                    swerve.calculateAutoSwerveControl(90, 0, 0);
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
                if (!line.get() && !lin) {
                    lin = true;
                    if (align == AlignDirection.LEFT) {
                        align = AlignDirection.RIGHT;
                    } else {
                        align = AlignDirection.LEFT;
                    }
                    state = 2;
                } else if (!(distance < Constants.getNumber("lineupDistance", 12) + 0.5 && distance > Constants.getNumber("lineupDistance", 12) - 0.5) && !dis) {
                    dis = true;
                    state = 0;
                } else if (Math.abs(dist.pidGet()) > 0.1 && !ali) {
                    ali = true;
                    state = 1;
                } else {
                    state++;
                }
            case 6:
                swerve.setControlMode(mode);
                SmartDashboard.putBoolean("Aligned", true);
                return true;
            default:
                System.out.println("State must be 0-3");
                pid.disable();
                swerve.calculateAutoSwerveControl(0, 0, 0);
                swerve.setControlMode(mode);
                SmartDashboard.putBoolean("Aligned", true);
                return false;
            }
        }
        SmartDashboard.putBoolean("Aligned", true);
        return false;
    }
}