/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3373.robot;

//import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.autonomous.Lineup;
import frc.team3373.robot.SwerveControl.Side;

import java.io.IOException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
//import edu.wpi.first.wpilibj.RobotState;
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

  int LBdriveMotorID = 2;
	int LBrotateMotorID = 1;
	int LBEncHome = 802; // Zero values (value when wheel is turned to default zero- bolt hole facing front.)
	int LBEncMin = 10;
	int LBEncMax = 897;
	
	int LFdriveMotorID = 4;
	int LFrotateMotorID = 3;
	int LFEncHome = 264;
	int LFEncMin = 11;
	int LFEncMax = 902;
	
	int RBdriveMotorID = 8;
	int RBrotateMotorID = 7;
	int RBEncHome = 866;
	int RBEncMin = 10;
	int RBEncMax = 898;
	
	int RFdriveMotorID = 6;
	int RFrotateMotorID = 5;
	int RFEncHome = 102;
	int RFEncMin = 10;
	int RFEncMax = 899;
	
	double robotWidth = 33.25; // TODO change robot dimensions to match this years robot
  double robotLength = 20.4375;

  // SwerveControl swerve;

  //SuperJoystick driver;
  SuperJoystick shooter;

  /* SuperAHRS ahrs;

  DistanceSensor distl;
  DistanceSensor distr;

  AutonomousControl control;
  DigitalInput line;

  Thread ahrsThread;

  double[] voltages;
  int count;
  AnalogInput cal;
  boolean disabled; */

  ObjectType object;
  Elevator elevator;
  Claw claw;


  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //voltages = new double[21];

    shooter = new SuperJoystick(0);
    //shooter = new SuperJoystick(1);
    
    try {
      Constants.loadConstants();
    } catch (IOException e) {
      e.printStackTrace();
    }

    elevator = new Elevator(1, shooter);

    object = ObjectType.HATCH;

    claw = new Claw(2, 0, 3, 2, 1);

    //count = 0;

    //cal = new AnalogInput(0);

    // dial = new Dial(0, 1, 2, 3);

    /* line = new DigitalInput(0);

    ahrs = new SuperAHRS(SPI.Port.kMXP);

    distl = new DistanceSensor(0, 2);
    distr = new DistanceSensor(1, 1);
    
    swerve = new SwerveControl(LFrotateMotorID, LFdriveMotorID, LFEncMin, LFEncMax, LFEncHome, LBrotateMotorID,
        LBdriveMotorID, LBEncMin, LBEncMax, LBEncHome, RFrotateMotorID, RFdriveMotorID, RFEncMin, RFEncMax, RFEncHome,
        RBrotateMotorID, RBdriveMotorID, RBEncMin, RBEncMax, RBEncHome, ahrs, robotWidth, robotLength);

    control = new AutonomousControl(ahrs, swerve, distl, distr, driver, shooter, line); */

    //object = ObjectType.HATCH;

    /* new Thread(()-> {
      while(true){
        SmartDashboard.putNumber("Yaw", ahrs.getYaw());
        SmartDashboard.updateValues();
      }
    }).start(); */
    
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
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
    }
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
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

  @Override
  public void teleopInit() {
    SmartDashboard.setDefaultBoolean("Update Constants", false);
    elevator.resetCal();
    //elevator.initPID();
    elevator.zero();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Position", elevator.getPosition());
    if (shooter.isDPadDownPushed()) {
      elevator.moveToHeight(10);
    } else if(shooter.isDPadLeftPushed()) {
      elevator.moveToHeight(25);
    } else if(shooter.isDPadUpPushed()) {
      elevator.moveToHeight(40);
    } else if(shooter.isDPadRightPushed()) {
      elevator.moveToHeight(Constants.getNumber("elevatorHeight"));
    }

    if (shooter.isAPushed()) {
      elevator.zero();
    }

    SmartDashboard.putNumber("Rotations", elevator.getRotations());
    elevator.refresh();

    shooter.clearButtons();
    shooter.clearDPad();
  }

  /**
   * This function is called at the start of test mode.
   */
  @Override
  public void testInit() {
    SmartDashboard.putBoolean("Calibrating", false);
    SmartDashboard.putNumber("Inches", 0);
    SmartDashboard.setDefaultBoolean("Update Constants", false);
    SmartDashboard.setDefaultBoolean("Save Constants", false);
    SmartDashboard.setDefaultBoolean("Restore Backup", false);
    SmartDashboard.setDefaultNumber("Calibration Length", 1);
    elevator.resetCal();
    elevator.initPID();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    testControls();

    SmartDashboard.putNumber("Rotations", elevator.getRotations());
    SmartDashboard.putNumber("Position", elevator.getPosition());

    /* if (driver.isAPushed() && !disabled) {
      voltages[count] = cal.getAverageVoltage();
      count++;
    }

    if (count == 21 && !disabled) {
      disabled = true;
      for (int i = 0; i < 21; i++) {
        System.out.println(voltages[i]);
      }
    }

    driver.clearA(); */

  }

  public void testControls() {
    elevator.rawMovePID(-shooter.getRawAxis(1));

    if (shooter.isAPushed()) {
      elevator.calibrate();
    }

    if (shooter.isYPushed()) {
      elevator.absoluteZero();
    }

    if (shooter.isBPushed()) {
      elevator.moveToHeight(Constants.getNumber("elevatorHeight"));
    }

    if (shooter.isXPushed()) {
      elevator.resetCal();
    }

    if (!SmartDashboard.getBoolean("Calibrating", false)){
      elevator.refresh();
    }

    shooter.clearButtons();
    shooter.clearDPad();
  }

  public void driverControls() {
    // ################################################
    // #### shared Controls ####
    // ################################################
    /* if (driver.isStartPushed() && shooter.isStartPushed()) {
      //auto get on HAB platform
    } */

    // ################################################
    // #### Driver Controls ####
    // ################################################
    /* if (driver.isAHeld() && driver.isBackPushed()) {
      lineup.align(Lineup.AlignDirection.LEFT);
    } else if (driver.isAHeld() && driver.isStartPushed()) {
      lineup.align(Lineup.AlignDirection.RIGHT);
    } */ 

    /* if (driver.isAHeld() && driver.isBackPushed()) {
      control.lineup(Lineup.AlignDirection.LEFT);
    } else if (driver.isAHeld() && driver.isStartPushed()) {
      control.lineup(Lineup.AlignDirection.RIGHT);
    }

    if (driver.isBPushed()) {
      //control.rotateRelative((float)SmartDashboard.getNumber("Angle", 90), SmartDashboard.getNumber("Speed", 0.2));
      control.driveSquare();
    }

    if (driver.getRawAxis(2) > .5) {// FieldCentric
      swerve.setControlMode(SwerveControl.DriveMode.FIELDCENTRIC);
    } else if (driver.getRawAxis(3) > .5) {// RobotCentric
      swerve.setControlMode(SwerveControl.DriveMode.ROBOTCENTRIC);
    }

    if (driver.isLBHeld()) {// sniper
      swerve.setDriveSpeed(0.2);
    } else if (driver.isRBHeld()) {// turbo
      swerve.setDriveSpeed(0.7);
    } else {// regular
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

    

     if (driver.isYPushed())
      swerve.resetOrentation();  */
    // swerve.controlMode(SwerveControl.DriveMode.FieldCentric);

    // ################################################
    // #### Shooter Controls ####
    // ################################################
    /* if(shooter.isYPushed()) {
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

    if (RobotState.isTest() && Math.abs(shooter.getRawAxis(1)) > 0.05) {
      elevator.move(shooter.getRawAxis(1));
    }

    if (shooter.isDPadDownPushed()) {
      elevator.moveToHeight(0, object);
    } else if (shooter.isDPadLeftPushed() || shooter.isDPadRightPushed()) {
      elevator.moveToHeight(1, object);
    }  else if (shooter.isDPadUpPushed()) {
      elevator.moveToHeight(2, object);
    } */

    //driver.clearButtons();
    //shooter.clearButtons();
  }

}
