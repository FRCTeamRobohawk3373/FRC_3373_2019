/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3373.autonomus;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.team3373.robot.AutonomousControl;
import frc.team3373.robot.SuperJoystick;

/**
 * 
 */
public class HABPlatformAuto {

    AutonomousControl controller;
    SuperJoystick joystick;

    byte state = 0;
    int count = 0;

    DigitalInput frontHome;
    DigitalInput backHome;

    public HABPlatformAuto(SuperJoystick joy) {
        // controller = control;
        joystick = joy;
        frontHome = new DigitalInput(0);
        backHome = new DigitalInput(1);
    }

    public boolean run() {

        return true;
    }

    public boolean climb(double climbHeight) {
        boolean frontDown = false;
        boolean backDown = true;
        while (!joystick.isBackPushed()) {
            switch (state) {
            case 0:// preclimb
                System.out.println("park Arms");
                state++;
                break;
            case 1:// climb
                System.out.println("lifting");
                if (getDistance() > climbHeight) {
                    System.out.print("stoping front");
                    frontDown = true;
                }
                if (getDistance() > climbHeight) {
                    System.out.print("stoping back");
                    backDown = true;

                }
                if (frontDown && backDown) {
                    state++;
                }

                break;
            case 2:// Forward 1st stop
                System.out.println("driving forward");
                if (getDistance(true) < 6) {
                    System.out.println("stoping");
                    state++;
                }
                break;
            case 3:// Lift Font Axle
                System.out.println("lift front");
                if (frontHome.get()) {
                    System.out.println("front up");
                    state++;
                }
                break;
            case 4:// Forward 2nd stop
                System.out.println("driving forward");
                if (getDistance(true) < 6) {
                    System.out.println("stoping");
                    state++;
                }
                break;
            case 5:// Lift Back Axle
                System.out.println("lift back");
                if (backHome.get()) {
                    System.out.println("back up");
                    state++;
                }
                break;
            case 6:// Forward 3rd stop
                System.out.println("driving forward a little");
                count++;
                if (count > 40) {
                    state++;
                }
                break;
            case 7:// completed
                System.out.println("Finished");
                return true;
            }
            joystick.clearBack();
        }
        return false;
    }

    private double getDistance() {
        count++;
        return count;
    }

    private double getDistance(boolean rev) {
        if (rev) {
            count--;
        } else {
            count++;
        }
        return count;
    }
}
