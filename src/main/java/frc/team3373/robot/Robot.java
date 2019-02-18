/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3373.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.autonomous.HABPlatformAuto;
import frc.team3373.autonomous.Lineup;
import frc.team3373.robot.SwerveControl.Side;

import java.io.IOException;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;

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

  HABPlatformAuto HABauto;

  Compressor compressor;
  AutonomousControl control;
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

    try {
      Constants.loadConstants();
    } catch (IOException e) {
      try {
        Constants.loadDefaults();
      } catch (IOException e1) {
        System.out.println("Catastrophic load");
        e1.printStackTrace();
      }
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    /*try {
      Constants.saveConstants();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }*/

    driver = new SuperJoystick(0);
    shooter = new SuperJoystick(1);
    ahrs = new SuperAHRS(SPI.Port.kMXP);
    
    swerve = new SwerveControl(FLrotateMotorID, FLdriveMotorID, FLEncMin, FLEncMax, FLEncHome, BLrotateMotorID,
        BLdriveMotorID, BLEncMin, BLEncMax, BLEncHome, FRrotateMotorID, FRdriveMotorID, FREncMin, FREncMax, FREncHome,
        BRrotateMotorID, BRdriveMotorID, BREncMin, BREncMax, BREncHome, ahrs, robotWidth, robotLength);
    //joy1,joy2,swerve,relayid,PCMid,rightSolenoidFowardChannel,rightSolenoidReverseChannel,leftSolenoidFowardChannel,leftSolenoidReverseChannel,rightLimitSwitch,leftLimitSwitch,rightDistanceSensor,leftDistanceSensor
    HABauto = new HABPlatformAuto(driver, shooter, swerve, 0, 1, 1, 2, 0, 3, 1, 0, 2, 3);
    claw = new Claw(2, 0, 3, 2, 1);
    
    distl = new DistanceSensor(0, 2);
    distr = new DistanceSensor(1, 3);
    line = new DigitalInput(2);

    control = new AutonomousControl(ahrs, swerve, distl, distr, driver, shooter, line);

    object = ObjectType.HATCH;
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
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    driverControls();
  }

  /**
   * This function is called at the start of test mode.
   */
  @Override
  public void testInit() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    swerve.printPositions();
    
  }

  public void driverControls() {
    //################################################
    //####          shared Controls               ####
    //################################################
    if (driver.isStartHeld() && shooter.isStartHeld()) {
      HABauto.climb(23);
      //auto get on HAB platform
    } else if (driver.isBackHeld() && shooter.isBackHeld()) {
      HABauto.climb(10);
        //auto get on HAB platform
    }

    //################################################
    //####          Driver Controls               ####
    //################################################
    if (driver.isAPushed()) {
      control.lineup(Lineup.AlignDirection.LEFT);
    } else if (driver.isBPushed()) {
      control.lineup(Lineup.AlignDirection.RIGHT);
    }

    if(driver.getRawAxis(2)>.5){//FieldCentric
			swerve.setControlMode(SwerveControl.DriveMode.FIELDCENTRIC);
		}else if(driver.getRawAxis(3)>.5){//RobotCentric
			swerve.setControlMode(SwerveControl.DriveMode.ROBOTCENTRIC);
    } 

    if(driver.isLBHeld()){//sniper
			swerve.setDriveSpeed(0.2);
		}else if(driver.isRBHeld()){//turbo
			swerve.setDriveSpeed(0.7);
    } else {//regular
      swerve.setDriveSpeed(0.4);
    }
    
    swerve.calculateSwerveControl(driver.getRawAxis(0), driver.getRawAxis(1), driver.getRawAxis(4));
    
    switch (driver.getPOV()) {
    case 0:
      swerve.changeFront(Side.NORTH);
      break;
    case 90:
      swerve.changeFront(Side.EAST);
      break;
    case 180:
      swerve.changeFront(Side.SOUTH);
      break;
    case 270:
      swerve.changeFront(Side.WEST);
      break;
    }

    if (driver.isXPushed())
      swerve.resetOrentation();
    //swerve.controlMode(SwerveControl.DriveMode.FieldCentric);

    //################################################
    //####           Shooter Controls             ####
    //################################################

    if(shooter.isYPushed()) {
      claw.grab(object);
    } else if (shooter.isXPushed()) {
      claw.drop(object);
    }

    if (shooter.isLBPushed()) {
      object = ObjectType.HATCH;
    } else if (shooter.isRBPushed()) {
      object = ObjectType.CARGO;
    }

    if (shooter.getRawAxis(5) < -0.5){
      claw.raise();
    } else if (shooter.getRawAxis(5) > 0.5) {
      claw.lower();
    }

    /* if (RobotState.isTest() && Math.abs(shooter.getRawAxis(1)) > 0.05) {
      elevator.move(shooter.getRawAxis(1));
    } */

    /* if (shooter.isDPadDownPushed()) {
      elevator.moveToHeight(0, object);
    } else if (shooter.isDPadLeftPushed() || shooter.isDPadRightPushed()) {
      elevator.moveToHeight(1, object);
    }  else if (shooter.isDPadUpPushed()) {
      elevator.moveToHeight(2, object);
    } */

    driver.clearButtons();
    shooter.clearButtons();
  }
}
