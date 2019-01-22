package frc.team3373.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;;

public class LineFinder {
    private DigitalInput right; // back of robot
    private DigitalInput left; // front of robot

    private boolean leftFirst;
    private boolean rightFirst;
    private boolean clockwise;
    private boolean missed;
    private boolean rotating;

    private SwerveControl swerve;

    private int step;

    private float init;

    private SearchMode mode;
    private SearchDirection direction;

    private SwerveControl.DriveMode drivemode; //Stores previous swerve drivemode

    private AHRS ahrs;

    private enum SearchMode {
        SEARCH_LEFT, SEARCH_RIGHT, NONE; //Defines if search is running or not
    }

    public enum SearchDirection {
        UP, RIGHT, LEFT, NONE
    }

    public LineFinder(int lPort, int rPort, SwerveControl swerv) {
        left = new DigitalInput(lPort);  //Initializes left sensor and right sensor as DigitalInputs
        right = new DigitalInput(rPort);

        ahrs = new AHRS(SerialPort.Port.kMXP);

        swerve = swerv;

        leftFirst = false;  //Flags for algorithm
        rightFirst = false;
        clockwise = false;
        missed = false;
        rotating = false;
        
        step = 1; //Counts what step the algorithm is on

        mode = SearchMode.NONE; //Keeps track of the search mode

        direction = SearchDirection.NONE; //Keeps track of translation direction
    }

    public void searchLeft(SearchDirection direct) {
        mode = SearchMode.SEARCH_LEFT; //Public call to search left, sets mode to SEARCH_LEFT
        direction = direct;

        drivemode = swerve.getControlMode();

        swerve.setControlMode(SwerveControl.DriveMode.FIELDCENTRIC);
    }

    private void searchLeft() {
        mode = SearchMode.SEARCH_LEFT; //Resets mode

        step = 1; //Resets algorithm step to 1

        leftFirst = false; //Resets flags to false
        rightFirst = false;
        clockwise = false;
        missed = false;
        rotating = false;

        swerve.calculateAutoSwerveControl(0, 0, 0); //Stops swerve

        swerve.setControlMode(SwerveControl.DriveMode.FIELDCENTRIC);
    }

    public void searchRight(SearchDirection direct) {
        mode = SearchMode.SEARCH_RIGHT; //Public call to search right, sets mode to SEARCH_RIGHT
        direction = direct;

        drivemode = swerve.getControlMode();
    }

    private void searchRight() {
        mode = SearchMode.SEARCH_RIGHT; //Resets mode

        step = 1; //Resets algorithm step to 1

        leftFirst = false; //Resets flags to false
        rightFirst = false;
        clockwise = false;
        missed = false;
        rotating = false;

        swerve.calculateAutoSwerveControl(0, 0, 0);
    }

    public void searchCancel() { //Public call to cancel the search
        mode = SearchMode.NONE; //Resets mode

        direction = SearchDirection.NONE;

        step = 1; //Resets algorithm step to 1

        leftFirst = false; //Resets flags to false
        rightFirst = false;
        clockwise = false;
        rotating = false;

        swerve.calculateAutoSwerveControl(0, 0, 0);
        swerve.setControlMode(drivemode);
    }
    
    private void rotateInit() {
        init = ahrs.getYaw();
        if (clockwise) {
            swerve.calculateAutoSwerveControl(0, 0, 0.3);
        } else if (!clockwise) {
            swerve.calculateAutoSwerveControl(0, 0, -0.3);
        }

        rotating = true;
    }

    private void rotate() {
        if (Math.abs(ahrs.getYaw() - init) == 45 && rotating) {
            swerve.calculateAutoSwerveControl(0, 0, 0);
            rotating = false;
        }
    }

    private void translateLeft() {
        switch(direction) {
            case UP:
                swerve.calculateAutoSwerveControl(180, 0.3, 0);
                break;
            case RIGHT:
                swerve.calculateAutoSwerveControl(270, 0.3, 0);
                break;
            case LEFT:
                swerve.calculateAutoSwerveControl(90, 0.3, 0);
                break;
            default:
                System.out.println("Undefined direction");
                break;
        }
    }

    private void translateRight() {
        switch(direction) {
            case UP:
                swerve.calculateAutoSwerveControl(0, 0.3, 0);
                break;
            case RIGHT:
                swerve.calculateAutoSwerveControl(90, 0.3, 0);
                break;
            case LEFT:
                swerve.calculateAutoSwerveControl(270, 0.3, 0);
                break;
            default:
                System.out.println("Undefined direction");
                break;
        }
    }

    public boolean lineUpdate() {
        rotate();

        //Put SearchMode mode on SmartDashboard
        SmartDashboard.putBoolean("Front", left.get());
        SmartDashboard.putBoolean("Back", right.get());
        if (mode == SearchMode.SEARCH_LEFT) {
            SmartDashboard.putString("Mode", "SEARCH_LEFT");
        } else if (mode == SearchMode.SEARCH_RIGHT) {
            SmartDashboard.putString("Mode", "SEARCH_RIGHT");
        } else if (mode == SearchMode.NONE) {
            SmartDashboard.putString("Mode", "NONE");
        }

        //Switch checks what SearchMode mode is in and updates it
        switch (mode) {
        case SEARCH_LEFT:
            if (!left.get() && !right.get() && step == 1) {
                translateLeft();
                step = 2;
                return false;
            } else if (left.get() && !right.get() && step == 2) {
                step = 3;
                leftFirst = true;
                return false;
            } else if (!left.get() && step == 3 && leftFirst) {
                if (right.get()) {
                    clockwise = false;
                    rotateInit();
                } else {
                    clockwise = true;
                    rotateInit();
                }
                step = 4;
                return false;
            } else if (!right.get() && step == 4 && !clockwise && leftFirst) {
                translateRight();
                step = 5;
                return false;
            } else if (!left.get() && step == 5 && !clockwise && leftFirst) {
                searchCancel();
            }
            
            else if (right.get() && step == 4 && clockwise && leftFirst) {
                translateLeft();
                step = 5;
                return false;
            } else if (left.get() && step == 5 && clockwise && leftFirst && !missed) {
                searchCancel();
            }
            
            else if (!right.get() && step == 4 && clockwise && leftFirst && !rotating) {
                SmartDashboard.putString("Command", "(If swerve not moving) Move forward 10 inches, move right 2 in"); //TODO: measure
                missed = true;
                step = 5;
            } else if (step == 5 && clockwise && leftFirst && missed && !rotating) {
                SmartDashboard.putString("Command", "(If swerve not moving) Call searchLeft()");
                searchLeft();
            }
            
            else if (!left.get() && right.get() && step == 2) {
                clockwise = true;
                rotateInit();
                step = 3;
                rightFirst = true;
                return false;
            } else if (left.get() && step == 3 && rightFirst) {
                translateLeft();
                step = 4;
                return false;
            } else if (right.get() && step == 4 && rightFirst) {
                searchCancel();
            }
            
            else {return false;}
        case SEARCH_RIGHT:
        if (!left.get() && !right.get() && step == 1) {
            translateRight();
            step = 2;
            return false;
        } else if (right.get() && !left.get() && step == 2) {
            translateRight();
            step = 3;
            rightFirst = true;
            return false;
        } else if (!right.get() && step == 3 && rightFirst) {
            if (left.get()) {
                rotateInit();
                clockwise = true;
            } else {
                rotateInit();
                clockwise = false;
            }
            step = 4;
            return false;
        } else if (!left.get() && step == 4 && clockwise && rightFirst) {
            translateLeft();
            step = 5;
            return false;
        } else if (right.get() && step == 5 && clockwise && rightFirst) {
            searchCancel();
        }
        
        else if (left.get() && step == 4 && !clockwise && rightFirst) {
            translateRight();
            step = 5;
            return false;
        } else if (right.get() && step == 5 && !clockwise && rightFirst && !missed) {
            searchCancel();
        } 
        
        else if (!right.get() && left.get() && step == 2) {
            clockwise = false;
            rotateInit();
            step = 3;
            leftFirst = true;
            return false;
        }
        
        else if (right.get() && step == 3 && leftFirst) {
            translateRight();
            step = 4;
            return false;
        } else if (left.get() && step == 4 && leftFirst && !missed) {
            searchCancel();
        }
            
        else if (!right.get() && step == 3 && leftFirst && !rotating) {
            SmartDashboard.putString("Command", "(If swerve not moving) Move forward 10 inches, move left 2 in"); //TODO: measure
            missed = true;
            step = 5;
        } else if (step == 4 && leftFirst && missed && !rotating) {
            SmartDashboard.putString("Command", "(If swerve not moving) Call searchRight()");
            searchRight();
        }
        
        else {return false;}
        default:
            return true;
        }
    }
}