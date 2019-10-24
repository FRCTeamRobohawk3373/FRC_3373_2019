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
import frc.team3373.robot.SwerveControl.DriveMode;
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
	int FREncHome = 367; // Zero values (value when wheel is turned to default					// zero- bolt hole facing front.)
	int FREncMin = 11;
	int FREncMax = 870;
	
	int BLdriveMotorID = 4;
	int BLrotateMotorID = 3;
	int BLEncHome = 264;
	int BLEncMin = 12;
	int BLEncMax = 902;
	
	int FLdriveMotorID = 8;
	int FLrotateMotorID = 7;
	int FLEncHome = 426;
	int FLEncMin = 11;
	int FLEncMax = 903;
	
	int BRdriveMotorID = 6;
	int BRrotateMotorID = 5;
	int BREncHome = 102;
	int BREncMin = 11;
	int BREncMax = 904;
	
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
  //AutonomousControl control;
  Solenoid armRelease;

  Lineup lineup;
  VisionLineup vlineup;

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


    //distl = new DistanceSensor(2, 9); // !Temp

    try {
      Constants.loadConstants();
    } catch (IOException e) {
      System.out.println("Error loading constants, loading defualts!");
      e.printStackTrace();
      try {
        Constants.loadDefaults();
      } catch (IOException e1) {
        System.out.println("Catastrophic error tyring to load!");
        e1.printStackTrace();
      }
      
    }

    driver = new SuperJoystick(0);
    shooter = new SuperJoystick(1);
    ahrs = new SuperAHRS(SPI.Port.kMXP);
    
    compressor = new Compressor(1);
    compressor.setClosedLoopControl(true);
    compressor.start();

    vis = new Vision();

    swerve = new SwerveControl(FLrotateMotorID, FLdriveMotorID, FLEncMin, FLEncMax, FLEncHome, BLrotateMotorID,
        BLdriveMotorID, BLEncMin, BLEncMax, BLEncHome, FRrotateMotorID, FRdriveMotorID, FREncMin, FREncMax, FREncHome,
        BRrotateMotorID, BRdriveMotorID, BREncMin, BREncMax, BREncHome, ahrs, robotWidth, robotLength);
    swerve.setControlMode(DriveMode.ROBOTCENTRIC);
    //joy1,joy2,swerve,relayid,PCMid,rightSolenoidFowardChannel,rightSolenoidReverseChannel,leftSolenoidFowardChannel,leftSolenoidReverseChannel,rightLimitSwitch,leftLimitSwitch,rightDistanceSensor,leftDistanceSensor
    HABauto = new HABPlatformAuto(driver, shooter, swerve, ahrs, 0, 1, 1, 2, 0, 3, 1, 0, 2, 3);
    claw = new Claw(2, 0, 3, 2, 1);
    claw.open();
    
    distl = new DistanceSensor(0, 5);
    distr = new DistanceSensor(1, 6);
    line = new DigitalInput(2);

    //control = new AutonomousControl(ahrs, swerve, distl, distr, driver, shooter, line);
    elevator = new Elevator(5,4);
    lineup = new Lineup(distl, distr, driver, swerve, line);
    vlineup = new VisionLineup(distl, distr, driver, swerve, vis);

    object = ObjectType.HATCH;

    armRelease = new Solenoid(1, 7);
    armRelease.set(true);
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
    //SmartDashboard.putNumber("roll", ahrs.getRoll());
    //SmartDashboard.putNumber("Pitch", ahrs.getPitch());

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
    //elevator.resetCal();
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
    // SmartDashboard.setDefaultBoolean("Update Constants", false);
    elevator.resetCal();
    compressor.setClosedLoopControl(true);
    // elevator.initPID();
    // elevator.zero();

    lockStraight = false;
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //HABauto.update();
    swerve.printPositions();
    elevator.refresh();
    driverControls();
    SmartDashboard.putNumber("Rotations", elevator.getRotations());
    SmartDashboard.putNumber("Position", elevator.getPosition());
    SmartDashboard.putNumber("FrontLeftDistance", distl.getDistance());
    SmartDashboard.putNumber("FrontRightDistance", distr.getDistance());
    SmartDashboard.putBoolean("Lifting Up", liftDirection);
    SmartDashboard.putNumber("Smart Avg", distl.getSmartAverage());
    SmartDashboard.putNumber("Avg", distl.getAverage());
    SmartDashboard.putNumber("Raw", distl.getVoltage());
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
    swerve.setControlMode(DriveMode.ROBOTCENTRIC);
    swerve.printPositions();

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
    
    swerve.printPositions();
    
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
    /* if (driver.isStartHeld() && shooter.isStartHeld()) {
      HABauto.climb(25.5);
      //auto get on HAB platform
    } else if (driver.isBackHeld() && shooter.isBackHeld()) {
      HABauto.climb(11);
      //auto get on HAB platform
    } */
    
    //################################################
    //####          Driver Controls               ####
    //################################################
    if (driver.isXPushed()) {
      vlineup.cancel();
      lockStraight = false;
    }
    
    /* if (driver.isAPushed()) {
      vlineup.align();
      lockStraight = true;
      //lockStraight = lineup.align(Lineup.AlignDirection.LEFT);
    } else if (driver.isBPushed()) {
      lockStraight = true;
      //lockStraight = lineup.align(Lineup.AlignDirection.RIGHT);
    } */
    
    if (vlineup.isFinished()) {
      if (driver.getRawAxis(2) > .5) {//FieldCentric
        swerve.setControlMode(SwerveControl.DriveMode.FIELDCENTRIC);
      } else if (driver.getRawAxis(3) > .5) {//RobotCentric
        swerve.setControlMode(SwerveControl.DriveMode.ROBOTCENTRIC);
      }
      
      if (driver.isLBHeld()) {//sniper
        swerve.setDriveSpeed(0.175);
        rotateSpeedMod = 1;
      } /* else if (driver.isRBHeld() && elevator.getPosition() < 21) {//turbo
        swerve.setDriveSpeed(0.7);
        rotateSpeedMod = .7;
      }  */else {//regular
        if (elevator.getPosition() < 40) 
          swerve.setDriveSpeed(0.5);
        else {
          swerve.setDriveSpeed(0.2);
        }
        rotateSpeedMod = 0.7;
      }
      
      /*if(driver.isStartHeld()){
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
      }*/
      
      if (lockStraight) {
        swerve.changeFront(Side.NORTH);
        swerve.calculateAutoSwerveControl(180 - 90 * Math.ceil(-driver.getRawAxis(1)),
            Math.max(Math.abs(driver.getRawAxis(1)), 0.05), 0);
        if (driver.getRawAxis(1) > .5) lockStraight = false;
      } else {
        swerve.calculateSwerveControl(driver.getRawAxis(0), driver.getRawAxis(1),
            driver.getRawAxis(4) * rotateSpeedMod);
        /*if(driver.getRawAxis(0)>0.05 || driver.getRawAxis(1)>0.05){
          if(startedlift){
            HABauto.drive(true);
          }else{
            HABauto.drive(false);
          }
        }else{
          HABauto.drive(false);
        }*/
        
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
    
    if (shooter.isXHeld()) {
      elevator.stop();
    }
    
    if (shooter.getRawAxis(1) < -0.5) {
      claw.raise();
    } else if (shooter.getRawAxis(1) > 0.5) {
      claw.lower();
    }
    
    if (shooter.isDPadDownPushed()) {
      elevator.moveToPosition(0, object);
    } else if (shooter.isDPadDownLeftPushed()) {
      elevator.moveToPosition(1, object);
    } else if (shooter.isDPadLeftPushed() || shooter.isDPadRightPushed()) {
      elevator.moveToPosition(2, object);
    } else if (shooter.isDPadUpPushed()) {
      //elevator.moveToPosition(3, object);
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
    
    if (shooter.getRawAxis(2) > .5 && Math.abs(shooter.getRawAxis(5)) > 0.05) {
      elevator.rawMovePID(-shooter.getRawAxis(5), 0.25);
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

    if (shooter.isAPushed()) {
      System.out.println(calInches+" "+distl.getAverage());
      calInches++;
    }

    if (driver.isAPushed()) {
      vlineup.align();
      lockStraight = true;
      //control.lineup(Lineup.AlignDirection.LEFT);
    }

    if (driver.isXPushed()) {
      vlineup.cancel();
      lockStraight = false;
    }

    if (lockStraight) {
      
    } else {
      if (driver.getRawAxis(2) > .5) {//FieldCentric
        swerve.setControlMode(SwerveControl.DriveMode.FIELDCENTRIC);
      } else if (driver.getRawAxis(3) > .5) {//RobotCentric
        swerve.setControlMode(SwerveControl.DriveMode.ROBOTCENTRIC);
      }

      //swerve.calculateSwerveControl(driver.getRawAxis(0), driver.getRawAxis(1),
      //      driver.getRawAxis(4) * rotateSpeedMod);
    }

    /* if(driver.isLBHeld()){
      HABauto.liftBack();
      HABauto.liftFront();
    }else if(driver.isRBHeld()){
      HABauto.lowerBack();
      HABauto.lowerFront();
    }else{
      HABauto.stopBack();
      HABauto.stopFront();
    } */

    if (driver.isBackHeld()) {
      //HABauto.climb(28);
      HABauto.climb(14);
    }

    shooter.clearButtons();
    shooter.clearDPad();
    driver.clearButtons();
    driver.clearDPad();
  }
}
