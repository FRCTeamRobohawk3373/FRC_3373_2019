/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3373.autonomous;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.team3373.robot.AutonomousControl;
import frc.team3373.robot.Constants;
import frc.team3373.robot.DistanceSensor;
import frc.team3373.robot.DistanceSensorPID;
import frc.team3373.robot.SuperAHRS;
import frc.team3373.robot.SuperJoystick;
import frc.team3373.robot.SwerveControl;
import frc.team3373.robot.SwerveControl.DriveMode;

/**
 * 
 */
public class HABPlatformAuto {

    //AutonomousControl controller;
    SuperJoystick joystick;
    SuperJoystick joystick2;

    byte state = 0;
    int count = 0;

    DigitalInput rightHome;
    DigitalInput leftHome;

    DistanceSensor rightSensor;
    DistanceSensor leftSensor;
    DistanceSensorPID distDif;

    DoubleSolenoid rightSolenoid;
    DoubleSolenoid leftSolenoid;

    SwerveControl swerve;

    Relay driveMotor;

    SuperAHRS ahrs;

    double driveSpeed;

    public HABPlatformAuto(SuperJoystick joy, SuperJoystick joy2, SwerveControl sw, SuperAHRS AHRS, int relayid, int PCMid,
            int rightSolenoidFowardChannel, int rightSolenoidReverseChannel, int leftSolenoidFowardChannel,
            int leftSolenoidReverseChannel, int rightLimitSwitch, int leftLimitSwitch, int rightDistanceSensor,
            int leftDistanceSensor) {
        // controller = control;
        ahrs = AHRS;

        joystick = joy;
        joystick2 = joy2;

        rightHome = new DigitalInput(rightLimitSwitch);
        leftHome = new DigitalInput(leftLimitSwitch);

        rightSensor = new DistanceSensor(rightDistanceSensor, 1);
        leftSensor = new DistanceSensor(leftDistanceSensor, 0);

        distDif = new DistanceSensorPID(leftSensor, rightSensor);

        SmartDashboard.putNumber("rightDistance", rightSensor.getDistance());
        SmartDashboard.putNumber("leftDistance", leftSensor.getDistance());

        SmartDashboard.putBoolean("leftSolenoid", false);
        SmartDashboard.putBoolean("rightSolenoid", false);
        SmartDashboard.putNumber("Differance", 0);

        rightSolenoid = new DoubleSolenoid(PCMid, rightSolenoidFowardChannel, rightSolenoidReverseChannel);
        leftSolenoid = new DoubleSolenoid(PCMid, leftSolenoidFowardChannel, leftSolenoidReverseChannel);

        swerve = sw;

        driveMotor = new Relay(relayid);

        driveSpeed = .1;
    }

    public void update() {
        SmartDashboard.putNumber("rightDistance", rightSensor.getDistance());
        SmartDashboard.putNumber("leftDistance", leftSensor.getDistance());
        SmartDashboard.putNumber("rightVoltage", rightSensor.getAverage());
        SmartDashboard.putNumber("leftVoltage", leftSensor.getAverage());
    }

    public void liftFront(){
        rightSolenoid.set(Value.kForward);
    }

    public void liftBack(){
        leftSolenoid.set(Value.kForward);
    }

    public void lowerFront(){
        rightSolenoid.set(Value.kReverse);
    }

    public void lowerBack(){
        leftSolenoid.set(Value.kReverse);
    }

    public void stopFront(){
        rightSolenoid.set(Value.kOff);
    }

    public void stopBack(){
        leftSolenoid.set(Value.kOff);
    }

    public void drive(boolean on){
        if(on){
            driveMotor.set(Relay.Value.kForward);
        }else{
            driveMotor.set(Relay.Value.kOff);
        }
    }

    public boolean climb(double climbHeight) {
        boolean frontDown = false;
        boolean backDown = false;

        boolean frontAtHeight = false;
        boolean backAtHeight = false;

        double nextTargetHeight = 0;
        double diff;

        double offset = ahrs.getPitch();

        swerve.setControlMode(DriveMode.ROBOTCENTRIC);
        swerve.calculateAutoSwerveControl(0, 0, 0);

        state = 0;
        while (!joystick.isXHeld() && !joystick2.isXHeld() && !RobotState.isDisabled()) {
            SmartDashboard.putNumber("rightDistance", rightSensor.getDistance());
            SmartDashboard.putNumber("leftDistance", leftSensor.getDistance());
            SmartDashboard.putNumber("State", state);
            SmartDashboard.putBoolean("rightLimit", rightHome.get());
            SmartDashboard.putBoolean("leftLimit", leftHome.get());
            SmartDashboard.putNumber("Pitch", ahrs.getPitch());

            switch (state) {
            case 0:// preclimb
                swerve.calculateAutoSwerveControl(180, driveSpeed, 0);
                count++;

                if (count > 20) {
                    swerve.calculateAutoSwerveControl(0, 0, 0);
                    count=0;
                    state++;
                }

                SmartDashboard.putString("Current Step", "park Arms");
                System.out.println(Constants.getNumber("HABPlatformCenterOffset",0));
                // state++;
                break;
            case 1: // climb
                SmartDashboard.putString("Current Step", "lifting");

                //if (leftSensor.getDistance() <= climbHeight) {//waits for the back sensor to reach the next height and stops the solenoid
                    //leftSolenoid.set(Value.kForward);
                    //backAtHeight = false;
                //} else {
               //    System.out.print("stoping left");
               //     leftSolenoid.set(Value.kOff);
               //     backAtHeight = true;
               // }

                //if (rightSensor.getDistance() <= climbHeight) {//waits for the front sensor to reach the next height and stops the solenoid
                    //rightSolenoid.set(Value.kForward);
                    //frontAtHeight = false;
                //} else {
               //     System.out.print("stoping right");
                //    rightSolenoid.set(Value.kOff);
                //    frontAtHeight = true;
                //}
                
                

                //if (frontAtHeight && backAtHeight) 
                //    state++;


                /*if (frontAtHeight && backAtHeight) { //checks that both the front and the back are at the height
                    //if (joystick.isXPushed()) {
                        if (nextTargetHeight >= climbHeight) {
                            state++;
                        }
                        nextTargetHeight += 2;
                        if (nextTargetHeight >= climbHeight) {
                            nextTargetHeight = climbHeight;
                        }
                        frontAtHeight = false;
                        backAtHeight = false;
                // }
                }
                
                if (rightSensor.getDistance() >= nextTargetHeight) {//waits for the front sensor to reach the next height and stops the solenoid
                    System.out.print("stoping front");
                    rightSolenoid.set(Value.kOff);
                    frontAtHeight = true;
                } else if (!frontAtHeight) {
                    rightSolenoid.set(Value.kForward);
                }
                
                if (leftSensor.getDistance() >= nextTargetHeight) {//waits for the back sensor to reach the next height and stops the solenoid
                    System.out.print("stoping back");
                    leftSolenoid.set(Value.kOff);
                    backAtHeight = true;
                
                } else if (!backAtHeight) {
                    leftSolenoid.set(Value.kForward);
                }*/
                diff = distDif.pidGet();//-(ahrs.getPitch() - offset); //+right -left//
                SmartDashboard.putNumber("Difference", diff);
                if (rightSensor.getDistance() <= climbHeight) {//waits for the front sensor to reach the next height and stops the solenoid
                    if (diff > Constants.getNumber("HABPlatformDeadBand",0.5)+Constants.getNumber("HABPlatformCenterOffset",0)) {
                        SmartDashboard.putBoolean("rightSolenoid", false);
                        rightSolenoid.set(Value.kOff);
                    } else {
                        SmartDashboard.putBoolean("rightSolenoid", true);
                        rightSolenoid.set(Value.kForward);
                    }
                    frontAtHeight = false;
                } else {
                    System.out.print("stoping right");
                    rightSolenoid.set(Value.kOff);
                    frontAtHeight = true;
                }
                
                if (leftSensor.getDistance() <= climbHeight) {//waits for the back sensor to reach the next height and stops the solenoid
                    if (diff < -Constants.getNumber("HABPlatformDeadBand",0.5)+Constants.getNumber("HABPlatformCenterOffset",0)) {
                        SmartDashboard.putBoolean("leftSolenoid", false);
                        leftSolenoid.set(Value.kOff);
                    } else {
                        SmartDashboard.putBoolean("lefttSolenoid", true);
                        leftSolenoid.set(Value.kForward);
                    }
                    backAtHeight = false;
                } else {
                    System.out.print("stoping left");
                    leftSolenoid.set(Value.kOff);
                    backAtHeight = true;
                }

                if (frontAtHeight && backAtHeight) 
                    state++;


                break;
            case 2:// Forward 1st stop
                SmartDashboard.putString("Current Step", "driving 1st forward");
                swerve.calculateAutoSwerveControl(0, driveSpeed, 0);
                driveMotor.set(Relay.Value.kForward);
                if (rightSensor.getDistance() < 8 && rightSensor.getDistance() >=0) {
                    swerve.calculateAutoSwerveControl(0, 0, 0);
                    driveMotor.set(Relay.Value.kOff);
                    System.out.println("stoping");
                    state++;
                }
                break;
            case 3:// Lift right Axle
                SmartDashboard.putString("Current Step", "lift right");
                rightSolenoid.set(Value.kReverse);
                if (!rightHome.get()) {
                    rightSolenoid.set(Value.kOff);
                    System.out.println("right up");
                    state++;
                }
                break;
            case 4:// Forward 2nd stop
                SmartDashboard.putString("Current Step", "driving 2nd forward");
                swerve.calculateAutoSwerveControl(0, driveSpeed, 0);
                driveMotor.set(Relay.Value.kForward);
                if (leftSensor.getDistance() < 8 && leftSensor.getDistance() >=0) {
                    swerve.calculateAutoSwerveControl(0, 0, 0);
                    driveMotor.set(Relay.Value.kOff);
                    System.out.println("stoping");
                    state++;
                }
                break;
            case 5:// Lift left Axle
                SmartDashboard.putString("Current Step", "lift left");
                leftSolenoid.set(Value.kReverse);
                if (!leftHome.get()) {
                    leftSolenoid.set(Value.kOff);
                    System.out.println("left up");
                    state++;
                }
                break;
            case 6:// Forward 3rd stop
                SmartDashboard.putString("Current Step", "driving forward a little");
                swerve.calculateAutoSwerveControl(0, driveSpeed, 0);
                count++;

                if (count > 100) {
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
        rightSolenoid.set(Value.kOff);
        leftSolenoid.set(Value.kOff);
        driveMotor.set(Relay.Value.kOff);
        swerve.calculateAutoSwerveControl(0, 0, 0);
        return false;
    }
}
