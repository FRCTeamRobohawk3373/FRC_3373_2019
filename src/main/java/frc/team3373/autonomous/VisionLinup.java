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

/**
 * Add your docs here.
 */
public class VisionLinup {

    private SuperJoystick joystick;

    private DistanceSensor ldist;
    private DistanceSensor rdist;

    private SwerveControl swerve;
    private Vision vis;

    private byte step;
    private boolean finished = true;
    private boolean stop = false;

    private Thread thread;

    public VisionLinup(DistanceSensor dl, DistanceSensor dr, SuperJoystick joy, SwerveControl sw, Vision vs) {
        joystick = joy;
        ldist = dl;
        rdist = dr;
        swerve = sw;
        vis = vs;

        step = 0;

        SmartDashboard.putString("linupInstruction", "None");
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
                                        SmartDashboard.putString("linupInstruction","driving forward and rotating clockwise");
                                        //rotate clockwise
                                    } else if (target.X < -Constants.getNumber("lineUPDeadband")) {
                                        SmartDashboard.putString("linupInstruction","driving forward and rotating counter clockwise");
                                        //rotate counter clockwise
                                    } else {
                                        SmartDashboard.putString("linupInstruction","driving forward");
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
                                    SmartDashboard.putString("linupInstruction","move Right");
                                }else if(target.X < -Constants.getNumber("lineUPDeadband")){
                                    //move to the left
                                    SmartDashboard.putString("linupInstruction","move Left");
                                } else if (target.rotation > Constants.getNumber("lineUPRotateDeadband")) {
                                    swerve.setDistanceToObject(target.distance);
                                    //rotate clockwise around the object
                                    //-0.5*Math.sqrt(Math.abs(0.03*target.rotation));

                                    SmartDashboard.putString("linupInstruction","rotate around clockwise at: " + (-0.5*Math.sqrt(Math.abs(0.03*target.rotation))));
                                } else if (target.rotation < -Constants.getNumber("lineUPRotateDeadband")) {
                                    swerve.setDistanceToObject(target.distance); 
                                    //rotate counter clockwise around the object
                                    //0.5*Math.sqrt(Math.abs(0.03*target.rotation));
                                    SmartDashboard.putString("linupInstruction","rotate around counter clockwise at: " + 0.5*Math.sqrt(Math.abs(0.03*target.rotation)));

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
                            double diff = ldist.getDistance() - rdist.getDistance();
                            if (diff > Constants.getNumber("lineUPDistanceSensorDeadband")) {
                                //rotate right
                                //speed = -0.004 * Math.pow(diff, 2);
                                SmartDashboard.putString("linupInstruction","rotating right");
                            }else if (diff < -Constants.getNumber("lineUPDistanceSensorDeadband")) {
                                //rotate left
                                //speed = 0.004 * Math.pow(diff, 2);
                                SmartDashboard.putString("linupInstruction","rotating left");
                            } else {
                                SmartDashboard.putString("linupInstruction","good");
                                step++;
                            }
                            //step++; //temp
                            break;
                        case 3: // center with vision
                            target = vis.getObjectClosestToCenter();
                            if (target != null) {
                                if (target.X > Constants.getNumber("lineUPDeadband")) {
                                    //move to the left
                                    SmartDashboard.putString("linupInstruction","move Right");
                                }else if(target.X < -Constants.getNumber("lineUPDeadband")){
                                    //move to the right
                                    SmartDashboard.putString("linupInstruction","move Left");
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
                            finished = true;
                            break;
                        }
                    }
                    finished = true;
                } else {
                    SmartDashboard.putString("linupInstruction", "None");
                    System.out.println("no Targets");
                    finished = true;
                }
                finished = true;
                SmartDashboard.putString("linupInstruction", "None");
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
