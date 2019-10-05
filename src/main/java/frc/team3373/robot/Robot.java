/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3373.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotState;
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

  SuperJoystick driver;
  SuperJoystick shooter;

  DistanceSensor distl;
  DistanceSensor distr;

  AnalogInput cal;

  int count;

  public enum ObjectType {
    CARGO, HATCH
  }
  
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

    try {
      Constants.loadConstants();
    } catch (IOException e) {
      e.printStackTrace();
      System.err.println("Catastrophic error tyring to load!");
    }

    shooter = new SuperJoystick(0);
    // cal = new AnalogInput(0); 
    distl = new DistanceSensor(0, 9, true);
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
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Voltage", distl.getAverage());
    SmartDashboard.putNumber("Smart Voltage", distl.getSmartVoltage(8));
    SmartDashboard.putNumber("Smart Average", distl.getSmartAverage(8));
  }

  /**
   * This function is called at the start of test mode.
   */
  @Override
  public void testInit() {
    count = 31;
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    if (shooter.isAPushed()) {
      System.out.println(count+" "+getSmartAverage(cal));
      count--;
    }
    shooter.clearA();
  }

  private double getSmartCal(AnalogInput input) {
    int burst_delay_micros = 1500;
    int numBurstSamples = 16;

    double[] readings = new double[4];

    for (int j = 0; j < 4; j++) {
      double currReading;
      double lowestReading = 5;
      for (int i = 0; i < numBurstSamples; i++) {
          currReading = input.getVoltage();

          if (currReading < lowestReading)
              lowestReading = currReading;

          try {
              Thread.sleep(burst_delay_micros / 1000, (int)(burst_delay_micros % 1000));
          } catch (Exception e) {
              e.printStackTrace();
          }
      }
      readings[j] = lowestReading;
      try {
        Thread.sleep(38);
      } catch (Exception e) {
        e.printStackTrace();
      } 
    }
    return (readings[0] + readings[1] + readings[2] + readings[3]) / 4;
  }

  private double getSmartAverage(AnalogInput input) {
    int burst_delay_micros = 1500;
    int numBurstSamples = 8;

    double currReading;
    double lowestReading = 5;
    for (int i = 0; i < numBurstSamples; i++) {
        currReading = input.getVoltage();

        if (currReading < lowestReading)
            lowestReading = currReading;

        try {
            Thread.sleep(burst_delay_micros / 1000, (int)(burst_delay_micros % 1000));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    return lowestReading;
  }
}
