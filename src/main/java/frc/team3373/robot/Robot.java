/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3373.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.autonomous.Lineup;
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

  int LBdriveMotorID = 2;
  int LBrotateMotorID = 1;
  int LBEncHome = 590; // Zero values (value when wheel is turned to default // zero- bolt hole facing
                       // front.)
  int LBEncMin = 10;
  int LBEncMax = 879;

  int LFdriveMotorID = 4;
  int LFrotateMotorID = 3;
  int LFEncHome = 602;
  int LFEncMin = 11;
  int LFEncMax = 889;

  int RBdriveMotorID = 8;
  int RBrotateMotorID = 7;
  int RBEncHome = 317;
  int RBEncMin = 12;
  int RBEncMax = 885;

  int RFdriveMotorID = 6;
  int RFrotateMotorID = 5;
  int RFEncHome = 65;
  int RFEncMin = 9;
  int RFEncMax = 891;

  double robotWidth = 22.75; // TODO change robot dimensions to match this years robot
  double robotLength = 27.375;

  SwerveControl swerve;

  SuperJoystick driver;
  SuperJoystick shooter;

  SuperAHRS ahrs;

  // LineFinder linder;

  Lineup lineup;

  DistanceSensor distl;
  DistanceSensor distr;

  Ultrasonic ultra;

  AutonomousControl control;

  DigitalInput line;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    driver = new SuperJoystick(0);
    shooter = new SuperJoystick(1);

    ahrs = new SuperAHRS(SPI.Port.kMXP);

    // linder = new LineFinder(0, 1, swerve);

    line = new DigitalInput(3);

    ultra = new Ultrasonic(1);

    distl = new DistanceSensor(0, Constants.distanceSensora2, Constants.distanceSensorb2, Constants.distanceSensorc2,
        Constants.distanceSensord2, Constants.distanceSensore2, Constants.distanceSensorf2);
    distr = new DistanceSensor(1, Constants.distanceSensora1, Constants.distanceSensorb1, Constants.distanceSensorc1,
        Constants.distanceSensord1, Constants.distanceSensore1, Constants.distanceSensorf1);

    lineup = new Lineup(distl, distr, shooter, swerve, 0);

    swerve = new SwerveControl(LFrotateMotorID, LFdriveMotorID, LFEncMin, LFEncMax, LFEncHome, LBrotateMotorID,
        LBdriveMotorID, LBEncMin, LBEncMax, LBEncHome, RFrotateMotorID, RFdriveMotorID, RFEncMin, RFEncMax, RFEncHome,
        RBrotateMotorID, RBdriveMotorID, RBEncMin, RBEncMax, RBEncHome, ahrs, robotWidth, robotLength);
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

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // joystickControls();
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
    SmartDashboard.putNumber("Distance", ultra.getDistance());
    SmartDashboard.putNumber("Raw Distance", ultra.getRawDistance());
  }

  public void driverControls() {
    /*
     * if (shooter.isBackHeld()) { if (driver.isDPadUpHeld()) {
     * linder.searchLeft(LineFinder.SearchDirection.UP); } else if
     * (driver.isDPadRightPushed()) {
     * linder.searchLeft(LineFinder.SearchDirection.RIGHT); } else if
     * (driver.isDPadLeftPushed()) {
     * linder.searchLeft(LineFinder.SearchDirection.LEFT); } } else if
     * (driver.isStartHeld()) { if (driver.isDPadUpHeld()) {
     * linder.searchRight(LineFinder.SearchDirection.UP); } else if
     * (driver.isDPadRightPushed()) {
     * linder.searchRight(LineFinder.SearchDirection.RIGHT); } else if
     * (driver.isDPadLeftPushed()) {
     * linder.searchRight(LineFinder.SearchDirection.LEFT); } } else if
     * (driver.isXPushed()) { linder.searchCancel(); }
     */

    if (shooter.isAHeld() && shooter.isBackHeld()) {
      lineup.run(Lineup.AlignDirection.LEFT);
    } else if (shooter.isAHeld() && shooter.isStartHeld()) {
      lineup.run(Lineup.AlignDirection.RIGHT);
    }

    driver.clearButtons();
    driver.clearDPad();
    driver.clearStart();
    driver.clearBack();
  }

  public void joystickControls() {
    // ################################################
    // #### shared Controls ####
    // ################################################
    /*
     * if (driver.isStartPushed() && shooter.isStartPushed()) { //auto get on HAB
     * platform }
     */

    // ################################################
    // #### Driver Controls ####
    // ################################################

    if (driver.getRawAxis(2) > .5) {// FieldCentric
      swerve.setControlMode(SwerveControl.DriveMode.FIELDCENTRIC);
    } else if (driver.getRawAxis(3) > .5) {// RobotCentric
      swerve.setControlMode(SwerveControl.DriveMode.ROBOTCENTRIC);
    }

    if (driver.isLBHeld()) {// sniper
      swerve.setDriveSpeed(0.3);
    } else if (driver.isRBHeld()) {// turbo
      swerve.setDriveSpeed(0.7);
    } else {// regular
      swerve.setDriveSpeed(0.5);
    }

    swerve.calculateSwerveControl(driver.getRawAxis(0), driver.getRawAxis(1), driver.getRawAxis(4));

    switch (driver.getPOV()) {
    case 0:
      swerve.changeFront(SwerveControl.Side.NORTH);
      break;
    case 90:
      swerve.changeFront(SwerveControl.Side.EAST);
      break;
    case 180:
      swerve.changeFront(SwerveControl.Side.SOUTH);
      break;
    case 270:
      swerve.changeFront(SwerveControl.Side.WEST);
      break;
    }

    if (driver.isXPushed())
      swerve.resetOrentation();
    // swerve.controlMode(SwerveControl.DriveMode.FieldCentric);

    // ################################################
    // #### Shooter Controls ####
    // ################################################

    driver.clearButtons();
    // shooter.clearButtons();
  }
}
