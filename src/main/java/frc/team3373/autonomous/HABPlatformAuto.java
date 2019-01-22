/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3373.autonomous;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.team3373.robot.AutonomousControl;
import frc.team3373.robot.Constants;
import frc.team3373.robot.DistanceSensor;
import frc.team3373.robot.SuperJoystick;
import frc.team3373.robot.SwerveControl;

/**
 * 
 */
public class HABPlatformAuto {

    //AutonomousControl controller;
    SuperJoystick joystick;

    byte state = 0;
    int count = 0;

    DigitalInput frontHome;
    DigitalInput backHome;

    DistanceSensor frontSensor;
    DistanceSensor backSensor;

    DoubleSolenoid frontSolenoid;
    DoubleSolenoid backSolenoid;

    SwerveControl swerve;

    WPI_TalonSRX driveMotor;

    public HABPlatformAuto(SuperJoystick joy, SwerveControl sw, int talonid, int frontSolenoidFowardChannel,
            int frontSolenoidReverseChannel, int backSolenoidFowardChannel, int backSolenoidReverseChannel,
            int frontLimitSwitch, int backLimitSwitch, int frontDistanceSensor, int backDistanceSensor) {
        // controller = control;
        joystick = joy;
        frontHome = new DigitalInput(frontLimitSwitch);
        backHome = new DigitalInput(backLimitSwitch);

        frontSensor = new DistanceSensor(frontDistanceSensor, Constants.distanceSensora1, Constants.distanceSensorb1,
                Constants.distanceSensorc1, Constants.distanceSensord1, Constants.distanceSensore1,
                Constants.distanceSensorf1);
        backSensor = new DistanceSensor(backDistanceSensor, Constants.distanceSensora1, Constants.distanceSensorb1,
                Constants.distanceSensorc1, Constants.distanceSensord1, Constants.distanceSensore1,
                Constants.distanceSensorf1);

        frontSolenoid = new DoubleSolenoid(frontSolenoidFowardChannel, frontSolenoidReverseChannel);
        backSolenoid = new DoubleSolenoid(backSolenoidFowardChannel, backSolenoidReverseChannel);

        swerve = sw;

        driveMotor = new WPI_TalonSRX(talonid);
    }

    public boolean climb(double climbHeight) {
        boolean frontDown = false;
        boolean backDown = false;
        state = 0;
        swerve.calculateAutoSwerveControl(0, 0, 0);
        while (!joystick.isBackHeld() && !RobotState.isDisabled()) {
            SmartDashboard.putNumber("frontDistance", frontSensor.getDistance());
            SmartDashboard.putNumber("backDistance", backSensor.getDistance());
            SmartDashboard.putNumber("State", state);
            SmartDashboard.putBoolean("frontLimit", frontHome.get());
            SmartDashboard.putBoolean("backLimit", backHome.get());

            switch (state) {
            case 0:// preclimb
                SmartDashboard.putString("Current Step", "park Arms");
                state++;
                break;
            case 1:// climb
                SmartDashboard.putString("Current Step", "lifting");
                frontSolenoid.set(Value.kForward);
                backSolenoid.set(Value.kForward);

                if (frontSensor.getDistance() > climbHeight) {
                    System.out.print("stoping front");
                    frontSolenoid.set(Value.kOff);
                    frontDown = true;
                }
                if (backSensor.getDistance() > climbHeight) {
                    System.out.print("stoping back");
                    backSolenoid.set(Value.kOff);
                    backDown = true;

                }
                if (frontDown && backDown) {
                    state++;
                }

                break;
            case 2:// Forward 1st stop
                SmartDashboard.putString("Current Step", "driving 1st forward");
                swerve.calculateAutoSwerveControl(90, .3, 0);
                driveMotor.set(0.5);
                if (frontSensor.getDistance() < 6) {
                    swerve.calculateAutoSwerveControl(0, 0, 0);
                    driveMotor.set(0);
                    System.out.println("stoping");
                    state++;
                }
                break;
            case 3:// Lift Font Axle
                SmartDashboard.putString("Current Step", "lift front");
                frontSolenoid.set(Value.kReverse);
                if (!frontHome.get()) {
                    frontSolenoid.set(Value.kOff);
                    System.out.println("front up");
                    state++;
                }
                break;
            case 4:// Forward 2nd stop
                SmartDashboard.putString("Current Step", "driving 2nd forward");
                swerve.calculateAutoSwerveControl(90, .3, 0);
                driveMotor.set(0.5);
                if (backSensor.getDistance() < 6) {
                    swerve.calculateAutoSwerveControl(0, 0, 0);
                    driveMotor.set(0);
                    System.out.println("stoping");
                    state++;
                }
                break;
            case 5:// Lift Back Axle
                SmartDashboard.putString("Current Step", "lift back");
                frontSolenoid.set(Value.kReverse);
                if (!backHome.get()) {
                    frontSolenoid.set(Value.kOff);
                    System.out.println("back up");
                    state++;
                }
                break;
            case 6:// Forward 3rd stop
                SmartDashboard.putString("Current Step", "driving forward a little");
                swerve.calculateAutoSwerveControl(90, .3, 0);
                count++;
                if (count > 40) {
                    swerve.calculateAutoSwerveControl(0, 0, 0);
                    state++;
                }
                break;
            case 7:// completed
                SmartDashboard.putString("Current Step", "Finished");
                return true;
            }
            joystick.clearButtons();
        }
        frontSolenoid.set(Value.kOff);
        backSolenoid.set(Value.kOff);
        driveMotor.set(0);
        swerve.calculateAutoSwerveControl(0, 0, 0);
        return false;
    }
}
