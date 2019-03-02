package frc.team3373.robot;

import edu.wpi.first.wpilibj.PIDOutput;

public class SuperPIDOutput implements PIDOutput {
    private OutputType type;
    private SwerveControl swerve;
    private double pOut;

    /**
     * The type of output to the swerve control: ROTATE, SWERVEA
     */
    public static enum OutputType {
        ROTATE, DRIVE
    }

    /**
     * Implements a class for interfacing with the swerve controller.
     * @param type If the output should be rotational or forward/backward
     * @param sw the swerve control for the robot
     */
    public SuperPIDOutput(OutputType type, SwerveControl sw) {
        this.type = type;
        swerve = sw;
        pOut = 0;
    }

    public void pidWrite(double output) {
        pOut = output;
        switch (type) {
        case ROTATE:
            swerve.calculateAutoSwerveControl(0, 0, output);
            break;
        case DRIVE:
            swerve.calculateAutoSwerveControl(0, output, 0);
            break;
        default:
            System.out.println("type must be initialized!");
        }
    }

    /**
     * @return The most recent output to the swerve control
     */
    public double getPOutput() {
        return pOut;
    }
}