package frc.team3373.robot;

import edu.wpi.first.wpilibj.PIDOutput;

public class SuperPIDOutput implements PIDOutput {
    private OutputType type;
    private SwerveControl swerve;
    private double pOut;

    public static enum OutputType {
        ROTATE, DRIVE
    }

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

    public double getPOutput() {
        return pOut;
    }
}