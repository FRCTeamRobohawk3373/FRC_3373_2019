/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3373.robot;

import frc.team3373.autonomous.HABPlatformAuto;
import frc.team3373.autonomous.Lineup;
import frc.team3373.autonomous.VisionLineup;
import frc.team3373.robot.SwerveControl.Side;

import java.io.IOException;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public enum ObjectType {
    HATCH, CARGO
  }

  int FRdriveMotorID = 2;
	int FRrotateMotorID = 1;
	int FREncHome = 357; // Zero values (value when wheel is turned to default					// zero- bolt hole facing front.)
	int FREncMin = 10;
	int FREncMax = 897;
	
	int BLdriveMotorID = 4;
	int BLrotateMotorID = 3;
	int BLEncHome = 265;
	int BLEncMin = 11;
	int BLEncMax = 902;
	
	int FLdriveMotorID = 8;
	int FLrotateMotorID = 7;
	int FLEncHome = 426;
	int FLEncMin = 10;
	int FLEncMax = 898;
	
	int BRdriveMotorID = 6;
	int BRrotateMotorID = 5;
	int BREncHome = 99;
	int BREncMin = 10;
	int BREncMax = 899;
	
	double robotWidth = 22.25; // TODO change robot dimensions to match this years robot
  double robotLength = 16.25;
  
  SwerveControl swerve;

  SuperJoystick driver;
  SuperJoystick shooter;
	
  SuperAHRS ahrs;

  DigitalInput line;

  DistanceSensor distl;
  DistanceSensor distr;

  ObjectType object;

  Claw claw;

  Elevator elevator;

  Vision vis;

  HABPlatformAuto HABauto;

  Compressor compressor;
  AutonomousControl control;
  Solenoid armRelease;

  VisionLineup lineup;

  double rotateSpeedMod = .5;

  boolean lockStraight = false;

  boolean liftDirection = true;//1 is down, 0 is up
  boolean startedlift = false;

  //long startTime=0;

  int calInches = 3;
  //private boolean cargoOpen;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    compressor = new Compressor(1);
    compressor.setClosedLoopControl(true);
    compressor.start();

    try {
      Constants.loadConstants();
    } catch (IOException e) {
      e.printStackTrace();
      System.out.println("Failed to load Constants, loading defaults");
      try {
        Constants.loadDefaults();
        Constants.saveConstants();
      } catch (IOException e1) {
        System.out.println("Catastrophic load");
        e1.printStackTrace();
      }
    }

    driver = new SuperJoystick(0);
    shooter = new SuperJoystick(1);
    ahrs = new SuperAHRS(SPI.Port.kMXP);
    
    vis = new Vision();

    swerve = new SwerveControl(FLrotateMotorID, FLdriveMotorID, FLEncMin, FLEncMax, FLEncHome, BLrotateMotorID,
        BLdriveMotorID, BLEncMin, BLEncMax, BLEncHome, FRrotateMotorID, FRdriveMotorID, FREncMin, FREncMax, FREncHome,
        BRrotateMotorID, BRdriveMotorID, BREncMin, BREncMax, BREncHome, ahrs, robotWidth, robotLength);

    //joy1,joy2,swerve,relayid,PCMid,rightSolenoidFowardChannel,rightSolenoidReverseChannel,leftSolenoidFowardChannel,leftSolenoidReverseChannel,rightLimitSwitch,leftLimitSwitch,rightDistanceSensor,leftDistanceSensor
    HABauto = new HABPlatformAuto(driver, shooter, swerve, ahrs, 0, 1, 1, 2, 0, 3, 1, 0, 2, 3);
    claw = new Claw(2, 0, 3, 2, 1);
    
    distl = new DistanceSensor(0, 5);
    distr = new DistanceSensor(1, 6);
    line = new DigitalInput(2);

    control = new AutonomousControl(ahrs, swerve, distl, distr, driver, shooter, line);
    elevator = new Elevator(5,4);

    lineup = new VisionLineup(distl, distr, driver, swerve, vis);

    object = ObjectType.HATCH;

    armRelease = new Solenoid(1, 7);
    armRelease.set(true);
    elevator.absoluteZero();
    SmartDashboard.putString("Object", "HATCH");
    SmartDashboard.putBoolean("Restore Defaults", false);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    if (SmartDashboard.getBoolean("Update Constants", false)) {
      Constants.updateValues();
      SmartDashboard.putBoolean("Update Constants", false);
    } else if (SmartDashboard.getBoolean("Save Constants", false) && RobotState.isTest()) {
      try {
        Constants.saveConstants();
        SmartDashboard.putBoolean("Save Constants", false);
      } catch (IOException e) {
        e.printStackTrace();
      }
    } else if (SmartDashboard.getBoolean("Restore Backup", false) && RobotState.isTest()) {
      try {
        Constants.restoreBackup();
        SmartDashboard.putBoolean("Restore Backup", false);
      } catch (IOException e) {
        e.printStackTrace();
      }
    } else if (SmartDashboard.getBoolean("Restore Defaults", false)) {
      try {
        Constants.loadDefaults();
        SmartDashboard.putBoolean("Restore Defaults", false);
      } catch (IOException e) {
        e.printStackTrace();
      }
    } else {
      SmartDashboard.putBoolean("Save Constants", false);
      SmartDashboard.putBoolean("Restore Backup", false);
      
    }
    SmartDashboard.putNumber("roll", ahrs.getRoll());
    SmartDashboard.putNumber("Pitch", ahrs.getPitch());

    //vis.update();
    //SmartDashboard.putBoolean("isCompressing", compressor.enabled());
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    SmartDashboard.setDefaultBoolean("Update Constants", false);
    elevator.resetCal();
    //elevator.initPID();
    // elevator.zero();

    lockStraight = false;
    compressor.setClosedLoopControl(true);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
    driverControls();
    elevator.refresh();
    SmartDashboard.putNumber("Rotations", elevator.getRotations());
    SmartDashboard.putNumber("Position", elevator.getPosition());
    SmartDashboard.putNumber("FrontLeftDistance", distl.getDistance());
    SmartDashboard.putNumber("FrontRightDistance", distr.getDistance());
  }

  @Override
  public void teleopInit() {
    SmartDashboard.setDefaultBoolean("Update Constants", false);
    elevator.resetCal();
    compressor.setClosedLoopControl(true);
    //elevator.initPID();
    // elevator.zero();

    lockStraight = false;
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //HABauto.update();
    driverControls();
    elevator.refresh();
    SmartDashboard.putNumber("Rotations", elevator.getRotations());
    SmartDashboard.putNumber("Position", elevator.getPosition());
    SmartDashboard.putNumber("FrontLeftDistance", distl.getDistance());
    SmartDashboard.putNumber("FrontRightDistance", distr.getDistance());
    SmartDashboard.putBoolean("Lifting Up", liftDirection);
  }

  /**
   * This function is called at the start of test mode.
   */
  @Override
  public void testInit() {
    SmartDashboard.setDefaultBoolean("Update Constants", false);
    SmartDashboard.setDefaultBoolean("Save Constants", false);
    SmartDashboard.setDefaultBoolean("Restore Backup", false);
    SmartDashboard.setDefaultNumber("Calibration Length", 1);
    SmartDashboard.setDefaultBoolean("Restore Defaults", false);
    SmartDashboard.putNumber("Inches", 10);
    elevator.resetCal();
    elevator.initPID();
    compressor.setClosedLoopControl(true);
    calInches = 3;
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    SmartDashboard.putNumber("Rotations", elevator.getRotations());
    SmartDashboard.putNumber("Position", elevator.getPosition());
    SmartDashboard.putNumber("FrontLeftDistance", distl.getDistance());
    SmartDashboard.putNumber("FrontRightDistance", distr.getDistance());
   
    //swerve.printPositions();
    testControls();
    elevator.refresh();
    elevator.updatePID();
    HABauto.update();
    //VisionObject obj = vis.getObjectClosestToCenter();
    //if(obj!=null){
    //  obj.print();
    //} else {
    //  System.out.println("None");
    //}
  
  }

  public void driverControls() {
    //################################################
    //####          shared Controls               ####
    //################################################
    if (driver.isStartHeld() && shooter.isStartHeld()) {
      System.out.println(HABauto.climb(25.5));
      //auto get on HAB platform
    } else if (driver.isBackHeld() && shooter.isBackHeld()) {
      HABauto.climb(10);
      //auto get on HAB platform
    }

    //################################################
    //####          Driver Controls               ####
    //################################################
    if (driver.isXPushed()) {
      lineup.cancel();
      lockStraight = false;
      liftDirection=!liftDirection;
      startedlift=false;
    }

    if (driver.isAPushed()) {
      //linup.align();
      lockStraight = true;
      control.lineup(Lineup.AlignDirection.LEFT);
    } else if (driver.isBPushed()) {
      lockStraight = true;
      control.lineup(Lineup.AlignDirection.RIGHT);
    }

    if (lineup.isFinished()) {
      if (driver.getRawAxis(2) > .5) {//FieldCentric
        swerve.setControlMode(SwerveControl.DriveMode.FIELDCENTRIC);
      } else if (driver.getRawAxis(3) > .5) {//RobotCentric
        swerve.setControlMode(SwerveControl.DriveMode.ROBOTCENTRIC);
      }

      if (driver.isLBHeld()) {//sniper
        swerve.setDriveSpeed(0.1);
        rotateSpeedMod = 1;
      } else if (driver.isRBHeld() && elevator.getPosition() < 21) {//turbo
        swerve.setDriveSpeed(0.5);
        rotateSpeedMod = .9;
      } else {//regular
        if (elevator.getPosition() < 40) 
          swerve.setDriveSpeed(0.4);
        else {
          swerve.setDriveSpeed(0.2);
        }
        rotateSpeedMod = .5;
      }

      if(driver.isStartHeld()){
        if(liftDirection){
          HABauto.liftFront();
        }else{
          HABauto.lowerFront();
        }
        startedlift=true;
      }else{
        HABauto.stopFront();
      }

      if(driver.isBackHeld()){
        if(liftDirection){
          HABauto.liftBack();
        }else{
          HABauto.lowerBack();
        }
        startedlift=true;
      }else{
        HABauto.stopBack();
      }
      
      if (lockStraight) {
        swerve.changeFront(Side.NORTH);
        swerve.calculateAutoSwerveControl(90*Math.ceil(-driver.getRawAxis(1)), -driver.getRawAxis(1), 0);
        if(-driver.getRawAxis(1)<.5)
          lockStraight = false;
      } else {
        swerve.calculateSwerveControl(driver.getRawAxis(0), driver.getRawAxis(1), driver.getRawAxis(4) * rotateSpeedMod);
        if(driver.getRawAxis(0)>0.05 || driver.getRawAxis(1)>0.05){
          if(startedlift){
            HABauto.drive(true);
          }else{
            HABauto.drive(false);
          }
        }else{
          HABauto.drive(false);
        }

      }
      
      switch (driver.getPOV()) {
      case 0:
        swerve.changeFront(Side.WEST);
        break;
      case 90:
        swerve.changeFront(Side.NORTH);
        break;
      case 180:
        swerve.changeFront(Side.EAST);
        break;
      case 270:
        swerve.changeFront(Side.SOUTH);
        break;
      }

      if (driver.isYPushed())
        swerve.resetOrentation();
    }
    //swerve.controlMode(SwerveControl.DriveMode.FieldCentric);

    //################################################
    //####           Shooter Controls             ####
    //################################################

    if (shooter.isYPushed() && object == ObjectType.HATCH) {
      claw.close();
    } else if (shooter.isAPushed() && object == ObjectType.HATCH) {
      claw.open();
    } else if ((!shooter.isAHeld() && !shooter.isYHeld()) && object == ObjectType.CARGO) {
      claw.close();
    } else if ((shooter.isAHeld() || shooter.isYHeld()) && object == ObjectType.CARGO) {
      claw.open();
    }

    if (shooter.isLBPushed()) {
      object = ObjectType.HATCH;
      SmartDashboard.putString("Object", "HATCH");
      claw.release(object);
    } else if (shooter.isRBPushed()) {
      object = ObjectType.CARGO;
      SmartDashboard.putString("Object", "CARGO");
      claw.grab(object);
    }

    if(shooter.isXHeld()){
      elevator.stop();
    }

    if (shooter.getRawAxis(1) < -0.5) {
      claw.raise();
    } else if (shooter.getRawAxis(1) > 0.5) {
      claw.lower();
    }

    if (shooter.isDPadDownPushed()) {
      elevator.moveToPosition(0, object);
    }else if(shooter.isDPadDownLeftPushed()){
      elevator.moveToPosition(1, object);
    } else if (shooter.isDPadLeftPushed() || shooter.isDPadRightPushed()) {
      elevator.moveToPosition(2, object);
    } else if (shooter.isDPadUpPushed()) {
      elevator.moveToPosition(3, object);
    } else if (shooter.isBPushed()) {
      elevator.moveToHeight(Constants.getNumber("elevatorMinHeight", 19));
      //startTime = System.currentTimeMillis();
    }

    /*if (shooter.isBHeld()) {
      if (System.currentTimeMillis() - startTime > 1000 && startTime!=-2) {
        elevator.zero();
        startTime = -2;
      }
    }*/

    if (Math.abs(shooter.getRawAxis(5)) > 0.05) {
      elevator.rawMovePID(-shooter.getRawAxis(5),0.25);
    }

    driver.clearButtons();
    driver.clearDPad();
    shooter.clearButtons();
    shooter.clearDPad();
  }

  //################################################
  //####         Calibration Controls           ####
  //################################################
  private void testControls() {
    if (RobotState.isTest() && Math.abs(shooter.getRawAxis(5)) > 0.05) {
      elevator.rawMovePID(-shooter.getRawAxis(5),0.25);
    }

    //swerve.printPositions();

    if (shooter.isStartPushed()) {
      elevator.calibrate();
    } else if (shooter.isXPushed()) {
      elevator.resetCal();
    }

    if (shooter.isBPushed()) {
      elevator.moveToHeight(Constants.getNumber("elevatorHeight"));
    }

    if (shooter.isYPushed()) {
      elevator.absoluteZero();
    }

    if (driver.isAPushed()) {
      lineup.align();
      lockStraight = true;
      //control.lineup(Lineup.AlignDirection.LEFT);
    }

    if (shooter.isAPushed()) {
      System.out.println(calInches+" "+distl.getAverage());
      calInches++;
    }

    if (driver.isXPushed()) {
      lineup.cancel();
      lockStraight = false;
    }

    if (driver.isBackHeld()) {
      HABauto.climb(30);
    }

    shooter.clearButtons();
    shooter.clearDPad();
    driver.clearButtons();
    driver.clearDPad();
  }
}
