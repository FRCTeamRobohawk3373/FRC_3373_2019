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

  SwerveControl swerve;

  SuperJoystick driver;
  SuperJoystick shooter;

  SuperAHRS ahrs;

  Lineup lineup;

  DistanceSensor distl;
  DistanceSensor distr;

  ObjectType object;

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

    // voltages = new double[21];

    driver = new SuperJoystick(0);
    //shooter = new SuperJoystick(1);

    // count = 0;

    // cal = new AnalogInput(0);

    ahrs = new SuperAHRS(SPI.Port.kMXP);

    distl = new DistanceSensor(0, Constants.distanceSensora2, Constants.distanceSensorb2, Constants.distanceSensorc2,
        Constants.distanceSensord2, Constants.distanceSensore2, Constants.distanceSensorf2);

    distr = new DistanceSensor(1, Constants.distanceSensora1, Constants.distanceSensorb1, Constants.distanceSensorc1,
        Constants.distanceSensord1, Constants.distanceSensore1, Constants.distanceSensorf1);
        
    swerve = new SwerveControl(LFrotateMotorID, LFdriveMotorID, LFEncMin, LFEncMax, LFEncHome, LBrotateMotorID,
        LBdriveMotorID, LBEncMin, LBEncMax, LBEncHome, RFrotateMotorID, RFdriveMotorID, RFEncMin, RFEncMax, RFEncHome,
        RBrotateMotorID, RBdriveMotorID, RBEncMin, RBEncMax, RBEncHome, ahrs, robotWidth, robotLength);

    lineup = new Lineup(distl, distr, driver, swerve, 0);

    object = ObjectType.HATCH;
    
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
    driverControls();

    SmartDashboard.putNumber("Left", distl.getDistance());
    SmartDashboard.putNumber("Right", distr.getDistance());
    SmartDashboard.putNumber("Left Rounded", (double)Math.round(100 * distl.getDistance())/100);
    SmartDashboard.putNumber("Right Rounded", (double)Math.round(100 * distr.getDistance())/100);
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
    if (driver.isAHeld() && driver.isBackPushed()) {
      lineup.align(Lineup.AlignDirection.LEFT);
    } else if (driver.isAHeld() && driver.isStartPushed()) {
      lineup.align(Lineup.AlignDirection.RIGHT);
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

    

     if (driver.isXPushed())
      swerve.resetOrentation(); 
    // swerve.controlMode(SwerveControl.DriveMode.FieldCentric);

    // ################################################
    // #### Shooter Controls ####
    // ################################################
    /* if(shooter.isYPushed()) {
      claw.grab(object);
    } else if (shooter.isXPushed()) {
      claw.drop(object);
    }

    if (shooter.isAPushed()) {
      if (object == ObjectType.HATCH) {
        object = ObjectType.CARGO;
      } else {
        object = ObjectType.HATCH;
      }
    } */

    driver.clearButtons();
    //shooter.clearButtons();
  }

}
