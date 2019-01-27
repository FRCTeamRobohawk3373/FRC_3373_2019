package frc.team3373.autonomous;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import frc.team3373.robot.DistanceSensor;
import frc.team3373.robot.SuperJoystick;
import frc.team3373.robot.SwerveControl;
import frc.team3373.robot.Ultrasonic;

public class Lineup {

    DistanceSensor dleft; // Right and left distance sensors
    DistanceSensor dright;

    Ultrasonic ultra; // Not used

    SuperJoystick joystick; // Shooter joystick

    SwerveControl swerve;

    DigitalInput line;

    AlignDirection align; // Holds which way the line is from the robot

    SwerveControl.DriveMode mode; // Holds previous swerve DriveMode

    public static enum AlignDirection {
        RIGHT, LEFT, NONE
    }

    public Lineup(DistanceSensor dl, DistanceSensor dr, SuperJoystick driver, SwerveControl swerve, int linePort) {
        align = AlignDirection.NONE; // Initializes align and line sensor
        line = new DigitalInput(linePort);
    }

    public void run(AlignDirection al) {
        mode = swerve.getControlMode(); // Gets swerve control mode
        swerve.setControlMode(SwerveControl.DriveMode.ROBOTCENTRIC);

        int state = 0; // Stores the step that the lineup is on

        align = al;
        while (!joystick.isXHeld() && !RobotState.isDisabled()) {
            switch (state) {
            case 0: // Inital state: Checks which way the robot is rotated according to the distance
                    // sensors and rotates
                if (dleft.getDistance() == dright.getDistance()) {
                    state = 2;
                } else if (dleft.getDistance() > dright.getDistance()) {
                    swerve.calculateAutoSwerveControl(0, 0, 0.3); // Rotate clockwise
                    state++;
                } else if (dleft.getDistance() < dright.getDistance()) {
                    swerve.calculateAutoSwerveControl(0, 0, -0.3); // Rotate counter-clockwise
                    state++;
                }
                break;
            case 1: // Rotating: after distance sensors get the same length, stop swerve
                if (dleft.getDistance() == dright.getDistance()) {
                    swerve.calculateAutoSwerveControl(0, 0, 0);
                    state++;
                }
                break;
            case 2: // Rotation aligned: Drives in the direction the driver specifies
                if (line.get()) {
                    state = 4;
                }
                switch (align) {
                case RIGHT:
                    swerve.calculateAutoSwerveControl(0, 0.3, 0); // Drive right
                    state++;
                    break;
                case LEFT:
                    swerve.calculateAutoSwerveControl(180, 0.3, 0); // Drive left
                    state++;
                    break;
                default:
                    System.out.println("AlignDirection cannot be NONE");
                    break;
                }
                break;
            case 3: // Position aligning: if the line is seen, stop the swerve
                if (line.get()) {
                    swerve.calculateAutoSwerveControl(0, 0, 0);
                    state++;
                }
                break;
            case 4: // Aligned: resets swerve and align direction
                swerve.calculateAutoSwerveControl(0, 0, 0);
                swerve.setControlMode(mode);
                align = AlignDirection.NONE;
                return;
            default:
                System.out.println("Illegal state number");
                return;
            }
        }
        swerve.calculateAutoSwerveControl(0, 0, 0);
    }

}