/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3373.autonomous;

import frc.team3373.robot.SwerveControl;
import frc.team3373.robot.Vision;
import frc.team3373.robot.VisionObject;
import frc.team3373.robot.SwerveControl.*;
import frc.team3373.robot.SuperJoystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3373.robot.Constants;
import frc.team3373.robot.DistanceSensor;
import frc.team3373.robot.DistanceSensorPID;

/**
 * Add your docs here.
 */
public class VisionLineup {

    // private SuperJoystick joystick;

    private DistanceSensor ldist;
    private DistanceSensor rdist;

    private SwerveControl swerve;
    private Vision vis;

    private byte step;
    private boolean finished = true;
    private boolean stop = false;

    private Thread thread;
    
    private DistanceSensorPID disDif;
    public VisionLineup(DistanceSensor dl, DistanceSensor dr, SuperJoystick joy, SwerveControl sw, Vision vs) {
        // joystick = joy;
        ldist = dl;
        rdist = dr;
        swerve = sw;
        vis = vs;

        step = 0;

        disDif = new DistanceSensorPID(ldist, rdist);

        SmartDashboard.putString("lineupInstruction", "None");
        SmartDashboard.putNumber("State", 0);
        SmartDashboard.putNumber("Timeout", 0);
    }

    public void align() {
        if (thread != null) {
            if (!thread.isAlive()) {
                finished = true;
                stop = false;
            }
        }

        if (finished) {
            finished = false;
            stop = false;

            thread = new Thread(() -> {
                DriveMode mode = swerve.getControlMode(); // Gets swerve control mode
                swerve.setControlMode(DriveMode.ROBOTCENTRIC);
                swerve.changeFront(Side.NORTH);
                VisionObject target = vis.getObjectClosestToCenter();
                //target.print();
                int count = 0;
                step = 0;
                if (target != null) {
                    while (!stop && !RobotState.isDisabled()) {
                        SmartDashboard.putNumber("State", step);
                        SmartDashboard.putNumber("Timeout", count);
                        SmartDashboard.putNumber("VisionCount", vis.size());
                        switch (step) {
                        case 0: // drive to target with vision
                            target = vis.getObjectClosestToCenter();
                            if (target != null) {
                                count = 0;
                                if (target.distance > Constants.getNumber("lineUPTargetDistance")) {
                                    if (target.X > Constants.getNumber("lineUPDeadband")) {
                                        SmartDashboard.putString("lineupInstruction",
                                                "driving forward and rotating clockwise");
                                        swerve.calculateAutoSwerveControl(90, Math.min(Math.sqrt(
                                                (target.distance - Constants.getNumber("lineUPTargetDistance")) / 10),
                                                1) * 0.4, target.X * .3);
                                        //rotate clockwise
                                    } else if (target.X < -Constants.getNumber("lineUPDeadband")) {
                                        SmartDashboard.putString("lineupInstruction",
                                                "driving forward and rotating counter clockwise");
                                        swerve.calculateAutoSwerveControl(90, Math.min(Math.sqrt(
                                                (target.distance - Constants.getNumber("lineUPTargetDistance")) / 10),
                                                1) * 0.4, target.X * .3);
                                        //rotate counter clockwise
                                    } else {
                                        SmartDashboard.putString("lineupInstruction", "driving forward");
                                        swerve.calculateAutoSwerveControl(90, Math.min(Math.sqrt(
                                                (target.distance - Constants.getNumber("lineUPTargetDistance")) / 10),
                                                1) * 0.4, 0);
                                        // drive strait
                                    }
                                } else {
                                    count = 0;
                                    step++;
                                }
                            } else {
                                if (count > Constants.getNumber("lineUPTimeOut")) {
                                    stop = true;
                                    break;
                                }
                                //System.out.println("missing target");
                                count++;
                            }
                            break;
                        case 1: // align to target with vision
                            target = vis.getObjectClosestToCenter();
                            if (target != null) {
                                count = 0;
                                if (target.X > Constants.getNumber("lineUPDeadband")) {
                                    //move to the Right
                                    SmartDashboard.putString("lineupInstruction", "move Right");
                                    swerve.calculateAutoSwerveControl(0, target.X * 0.4, 0);
                                } else if (target.X < -Constants.getNumber("lineUPDeadband")) {
                                    //move to the left
                                    SmartDashboard.putString("lineupInstruction", "move Left");
                                    swerve.calculateAutoSwerveControl(180, target.X * 0.4, 0);
                                } else if (ldist.getDistance() == -2) {
                                    SmartDashboard.putString("lineupInstruction", "rotate clockwise around the object");
                                    swerve.setDistanceToObject(target.distance - 8);
                                    swerve.calculateObjectControl(0.1);
                                    
                                    //rotate clockwise around the object
                                } else if (rdist.getDistance() == -2) {
                                    SmartDashboard.putString("lineupInstruction",
                                            "rotate counter clockwise around the object");
                                    swerve.setDistanceToObject(target.distance - 8);
                                    swerve.calculateObjectControl(-0.1);
                                    //rotate counter clockwise around the object
                                    
                                } else {
                                    count = 0;
                                    step++;
                                }
                                
                            } else {
                                if (count > Constants.getNumber("lineUPTimeOut")) {
                                    stop = true;
                                    break;
                                }
                                
                                count++;
                            }
                            break;
                        case 2: // straighten with distance sensors
                            //if(ldist.getDistance()>0)
                            
                            //if(rdist.getDistance()>0)
                            double diff = disDif.pidGet();//right-left
                            if (diff > 0) {
                                if (diff > Constants.getNumber("lineUPDistanceSensorDeadband")) {
                                    //rotate left
                                    //speed = 0.004 * Math.pow(diff, 2);
                                    swerve.calculateAutoSwerveControl(0, 0, -0.004 * Math.pow(diff, 2));
                                    SmartDashboard.putString("lineupInstruction", "rotating left");
                                } else if (diff < -Constants.getNumber("lineUPDistanceSensorDeadband")) {
                                    //rotate right
                                    //speed = -0.004 * Math.pow(diff, 2);
                                    swerve.calculateAutoSwerveControl(0, 0, 0.004 * Math.pow(diff, 2));
                                    SmartDashboard.putString("lineupInstruction", "rotating right");
                                } else {
                                    SmartDashboard.putString("lineupInstruction", "good");
                                    step++;
                                }
                            } else {
                                swerve.calculateAutoSwerveControl(90, .08, 0);
                            }
                            //step++; //temp
                            break;
                        case 3: // center with vision
                            target = vis.getObjectClosestToCenter();
                            if (target != null) {
                                if (target.X > Constants.getNumber("lineUPDeadband")) {
                                    //move to the Right
                                    SmartDashboard.putString("lineupInstruction", "move Right");
                                    swerve.calculateAutoSwerveControl(0, target.X * 0.4, 0);
                                } else if (target.X < -Constants.getNumber("lineUPDeadband")) {
                                    //move to the left
                                    SmartDashboard.putString("lineupInstruction", "move Left");
                                    swerve.calculateAutoSwerveControl(180, target.X * 0.4, 0);
                                } else {
                                    count = 0;
                                    step++;
                                }
                                
                            } else {
                                if (count > Constants.getNumber("lineUPTimeOut")) {
                                    stop = true;
                                    break;
                                }
                                
                                count++;
                            }
                            break;
                        case 4:
                            swerve.calculateAutoSwerveControl(0, 0, 0);
                            SmartDashboard.putString("lineupInstruction", "done");
                            finished = true;
                            break;
                        }
                    }
                    swerve.setControlMode(mode);
                    swerve.calculateAutoSwerveControl(0, 0, 0);
                    SmartDashboard.putString("lineupInstruction", "done");
                    finished = true;
                } else {
                    swerve.setControlMode(mode);
                    swerve.calculateAutoSwerveControl(0, 0, 0);
                    SmartDashboard.putString("lineupInstruction", "None");
                    System.out.println("no Targets");
                    finished = true;
                }
                finished = true;
                swerve.setControlMode(mode);
                swerve.calculateAutoSwerveControl(0, 0, 0);
                SmartDashboard.putString("lineupInstruction", "done");
            });

            thread.start();
        }
    }
    
    public boolean isFinished() {
        return finished;
    }

    public void cancel() {
        stop = true;
    }

}
