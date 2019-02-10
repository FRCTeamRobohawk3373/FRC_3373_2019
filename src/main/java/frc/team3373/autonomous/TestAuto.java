package frc.team3373.autonomous;

import frc.team3373.robot.AutonomousControl;
import frc.team3373.robot.SwerveControl.DriveMode;
import frc.team3373.robot.SwerveControl.Side;

public class TestAuto {
    private AutonomousControl control;

    public TestAuto(AutonomousControl control) {
        this.control = control;
    }

    public void run() {
        control.getSwerve().setControlMode(DriveMode.ROBOTCENTRIC);
        control.getSwerve().changeFront(Side.NORTH);
        control.driveForRotations(20, 90, 0.2);
		control.rotateAbsolute(90, 0.2);
		control.driveForRotations(20, 90, 0.2);
		control.rotateAbsolute(180, 0.2);
		control.driveForRotations(20, 90, 0.2);
		control.rotateAbsolute(270, 0.2);
		control.driveForRotations(20, 90, 0.2);
		control.rotateAbsolute(0, 0.2);
    }
}