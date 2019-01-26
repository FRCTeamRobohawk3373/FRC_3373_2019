package frc.team3373.autonomous;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import frc.team3373.robot.DistanceSensor;
import frc.team3373.robot.SuperJoystick;
import frc.team3373.robot.SwerveControl;
import frc.team3373.robot.Ultrasonic;

public class Lineup {

    DistanceSensor dleft;
    DistanceSensor dright;

    Ultrasonic ultra;

    SuperJoystick joystick;

    SwerveControl swerve;

    DigitalInput line;

    AlignDirection align;

    SwerveControl.DriveMode mode;

    public static enum AlignDirection {
        RIGHT, LEFT, NONE
    }

    public Lineup(DistanceSensor dl, DistanceSensor dr, SuperJoystick shooter, SwerveControl swerve, int linePort) {
        align = AlignDirection.NONE;
        line = new DigitalInput(linePort);
    }

    public void run(AlignDirection al) {
        mode = swerve.getControlMode();
        swerve.setControlMode(SwerveControl.DriveMode.ROBOTCENTRIC);

        int state = 0;

        align = al;
        while (!joystick.isXHeld() && !RobotState.isDisabled()) {
            switch (state) {
            case 0: // Inital state
                if (dleft.getDistance() == dright.getDistance()) {
                    state = 2;
                } else if (dleft.getDistance() > dright.getDistance()) {
                    swerve.calculateAutoSwerveControl(0, 0, 0.3);
                    state++;
                } else if (dleft.getDistance() < dright.getDistance()) {
                    swerve.calculateAutoSwerveControl(0, 0, -0.3);
                    state++;
                }
                break;
            case 1: // Rotating
                if (dleft.getDistance() == dright.getDistance()) {
                    state++;
                }
                break;
            case 2: // Rotation aligned
                if (line.get()) {
                    state = 4;
                }
                switch (align) {
                case RIGHT:
                    swerve.calculateAutoSwerveControl(0, 0.3, 0);
                    state++;
                    break;
                case LEFT:
                    swerve.calculateAutoSwerveControl(180, 0.3, 0);
                    state++;
                    break;
                default:
                    System.out.println("Illuminati confirmed");
                    break;
                }
                break;
            case 3: // Position aligning
                if (line.get()) {
                    state++;
                }
                break;
            case 4: // Aligned
                swerve.setControlMode(mode);
                align = AlignDirection.NONE;
                break;
            default:
                System.out.println("AlignDirection cannot be NONE");
            }
        }
    }

}