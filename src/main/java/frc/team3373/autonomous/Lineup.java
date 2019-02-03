package frc.team3373.autonomous;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.robot.DistanceSensor;
import frc.team3373.robot.SuperJoystick;
import frc.team3373.robot.SwerveControl;
import frc.team3373.robot.SwerveControl.DriveMode;

public class Lineup extends PIDSubsystem {

    private DistanceSensor dleft; // Right and left distance sensors
    private DistanceSensor dright;

    private SuperJoystick joystick; // Shooter joystick

    private SwerveControl swerve;

    private DigitalInput line;

    public static enum AlignDirection { // Stores the direction that the robot should search for the line
        RIGHT, LEFT
    }

    public Lineup(DistanceSensor dl, DistanceSensor dr, SuperJoystick driver, SwerveControl sw, int linePort) {
        super("Lineup", 0.025, 0.0022, 0); // Creates a PIDController with these values
        line = new DigitalInput(linePort); // Initialises objects
        swerve = sw;
        dleft = dl;
        dright = dr;
        joystick = driver;

        getPIDController().setContinuous(false); // Consider positive and negative values as different directions
        getPIDController().setInputRange(3, 32); // Minimum and maximum inputs from distance sensors
        getPIDController().setOutputRange(-0.2, 0.2); // Minimum and maximum speed for the PID loop
    }

    public void initDefaultCommand() {
    }

    @Override
    protected double returnPIDInput() { // Method for the PIDController to get the input
        double rdist = (double) Math.round(100 * dright.getDistance()) / 100; // Rounds values of distance sensors
        double ldist = (double) Math.round(100 * dleft.getDistance()) / 100;

        if (rdist == -2.0 && ldist == -2.0) { // If the sensors are out of range, do nothing
            rdist = 0;
            ldist = 0;
        } else if (ldist == -2.0) { // If just one of the sensors is out of range set the distance to just above the
                                    // max value
            ldist = 32.0;
        } else if (rdist == -2.0) {
            rdist = 32.0;
        }

        return (rdist - ldist) + 2.8; // Returns the difference of the distance sensors plus an offset
    }

    @Override
    protected void usePIDOutput(double output) { // Method for the PIDController to set the output, rotating the robot
        swerve.calculateAutoSwerveControl(0, 0, output);
    }

    public void square() {
        int count = 0;

        DriveMode mode = swerve.getControlMode(); // Gets swerve control mode
        swerve.setControlMode(DriveMode.ROBOTCENTRIC);

        getPIDController().setAbsoluteTolerance(0.1); // Sets deadband for PID
        getPIDController().enable(); // Enables PID loop

        while (!joystick.isXHeld() && !RobotState.isDisabled()) {
            if (getPIDController().onTarget() && count >= 200) { // If 200 or more samples are within the deadband,
                                                                 // disable PID and return
                getPIDController().disable();
                swerve.calculateAutoSwerveControl(0, 0, 0);
                swerve.setControlMode(mode);
                return;
            } else if (getPIDController().onTarget()) {
                count++;
            } else if (count != 0 && !getPIDController().onTarget()) {
                count = 0;
            }
        }
    }

    public void align(AlignDirection al) {
        SmartDashboard.putBoolean("Aligned", false);

        DriveMode mode = swerve.getControlMode(); // Gets swerve control mode
        swerve.setControlMode(DriveMode.ROBOTCENTRIC);

        int state = 0; // Stores the step that the lineup is on

        AlignDirection align = al; // Holds which way the line is from the robot

        getPIDController().setAbsoluteTolerance(0.1); // Sets deadband for PID
        getPIDController().enable(); // Enables PID loop

        int count = 0; // Stores how many values have been tested

        while (!joystick.isXHeld() && !RobotState.isDisabled()) {
            switch (state) {
            case 0: // If 200 or more samples are within the deadband, disable PID and go to case 1
                if (getPIDController().onTarget() && count >= 200) {
                    getPIDController().disable();
                    swerve.calculateAutoSwerveControl(0, 0, 0);
                    count = 0;
                    state++;
                } else if (getPIDController().onTarget()) {
                    count++;
                } else if (count != 0 && !getPIDController().onTarget()) {
                    count = 0;
                }
                break;
            case 1: // If the average distance is within the correct range, go to line finding.
                    // Else, go towards the right distance
                double dist = (dleft.getDistance() + dright.getDistance()) / 2;
                if (dist < 12 && dist > 11) {
                    state = 3;
                } else if (dist > 12) {
                    System.out.println("Going forward");
                    swerve.calculateAutoSwerveControl(90, 0.1, 0);
                    state++;
                } else if (dist < 11) {
                    System.out.println("Going backward");
                    swerve.calculateAutoSwerveControl(270, 0.1, 0);
                    state++;
                }
                break;
            case 2: // If the average distance is within the correct range, go to line finding
                dist = (dleft.getDistance() + dright.getDistance()) / 2;
                if (dist < 12 && dist > 11) {
                    System.out.println("Stopping");
                    swerve.calculateAutoSwerveControl(0, 0, 0);
                    state++;
                }
                break;
            case 3: // If line is sensed, return. Else, search in the specified direction
                if (line.get()) {
                    swerve.setControlMode(mode);
                    SmartDashboard.putBoolean("Aligned", true);
                    return;
                }
                switch (align) {
                case RIGHT:
                    swerve.calculateAutoSwerveControl(0, 0.1, 0);
                    state++;
                    break;
                case LEFT:
                    swerve.calculateAutoSwerveControl(180, 0.1, 0);
                    state++;
                    break;
                default:
                    System.out.println("AlignDirection must be defined");
                    swerve.setControlMode(mode);
                    SmartDashboard.putBoolean("Aligned", true);
                    return;
                }
                break;
            case 4: // When line is sensed for a certain number of times, go to case 5
                if (line.get() && count == 3) {
                    state++;
                } else if (line.get()) {
                    count++;
                } else if (count != 0 && !line.get()) {
                    count = 0;
                }
                break;
            case 5: // Fine tune squaring with PID
                getPIDController().setAbsoluteTolerance(0.05);
                getPIDController().enable();
                while (!joystick.isXHeld() && !RobotState.isDisabled()) {
                    if (getPIDController().onTarget()) {
                        getPIDController().disable();
                        swerve.calculateAutoSwerveControl(0, 0, 0);
                        swerve.setControlMode(mode);
                        SmartDashboard.putBoolean("Aligned", true);
                        return;
                    }
                }
            default:
                System.out.println("State must be 0-3");
                getPIDController().disable();
                swerve.calculateAutoSwerveControl(0, 0, 0);
                swerve.setControlMode(mode);
                SmartDashboard.putBoolean("Aligned", true);
                return;
            }
        }
    }
}