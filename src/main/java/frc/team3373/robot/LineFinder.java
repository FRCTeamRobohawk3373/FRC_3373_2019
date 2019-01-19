package frc.team3373.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LineFinder {
    private DigitalInput right; // back of robot
    private DigitalInput left; // front of robot

    private boolean leftFirst;
    private boolean rightFirst;
    private boolean clockwise;
    private boolean missed;

    private int step;

    private SearchMode mode;

    private SearchDirection direction;

    private enum SearchMode {
        SEARCH_LEFT, SEARCH_RIGHT, SEARCH_NONE; //Defines if search is running or not
    }

    public enum SearchDirection {
        UP, RIGHT, DOWN, LEFT, NONE
    }

    public LineFinder(int lPort, int rPort) {
        left = new DigitalInput(lPort);  //Initializes left sensor and right sensor as DigitalInputs
        right = new DigitalInput(rPort);

        leftFirst = false;  //Flags for algorithm
        rightFirst = false;
        clockwise = false;
        missed = false;
        
        step = 1; //Counts what step the algorithm is on

        mode = SearchMode.SEARCH_NONE; //Keeps track of the search mode

        direction = SearchDirection.NONE; //Keeps track of translation direction
    }

    public void searchLeft(SearchDirection direct) {
        mode = SearchMode.SEARCH_LEFT; //Public call to search left, sets mode to SEARCH_LEFT
        direction = direct;
    }

    private void searchLeft() {
        mode = SearchMode.SEARCH_LEFT; //Resets mode

        step = 1; //Resets algorithm step to 1

        leftFirst = false; //Resets flags to false
        rightFirst = false;
        clockwise = false;
        missed = false;

        SmartDashboard.putString("Command", "Stop");
    }

    public void searchRight(SearchDirection direct) {
        mode = SearchMode.SEARCH_RIGHT; //Public call to search right, sets mode to SEARCH_RIGHT
        direction = direct;
    }

    private void searchRight() {
        mode = SearchMode.SEARCH_RIGHT; //Resets mode

        step = 1; //Resets algorithm step to 1

        leftFirst = false; //Resets flags to false
        rightFirst = false;
        clockwise = false;
        missed = false;

        SmartDashboard.putString("Command", "Stop");
    }

    public void searchCancel() { //Public call to cancel the search
        mode = SearchMode.SEARCH_NONE; //Resets mode

        direction = SearchDirection.NONE;

        step = 1; //Resets algorithm step to 1

        leftFirst = false; //Resets flags to false
        rightFirst = false;
        clockwise = false;

        SmartDashboard.putString("Command", "Stop");
    }

    public boolean lineUpdate() {
        //Put SearchMode mode on SmartDashboard
        SmartDashboard.putBoolean("Front", left.get());
        SmartDashboard.putBoolean("Back", right.get());
        if (mode == SearchMode.SEARCH_LEFT) {
            SmartDashboard.putString("Mode", "SEARCH_LEFT");
        } else if (mode == SearchMode.SEARCH_RIGHT) {
            SmartDashboard.putString("Mode", "SEARCH_RIGHT");
        }else if (mode == SearchMode.SEARCH_NONE) {
            SmartDashboard.putString("Mode", "SEARCH_NONE");
        }
        //Switch checks what SearchMode mode is in and updates it
        switch (mode) {
        case SEARCH_LEFT:
            if (!left.get() && !right.get() && step == 1) {
                SmartDashboard.putString("Command", "Translate left");
                step = 2;
                return false;
            } else if (left.get() && !right.get() && step == 2) {
                step = 3;
                leftFirst = true;
                return false;
            } else if (!left.get() && step == 3 && leftFirst) {
                if (right.get()) {
                    SmartDashboard.putString("Command", "Rotate counter-clockwise");
                    clockwise = false;
                } else {
                    SmartDashboard.putString("Command", "Rotate clockwise until 45 degrees");
                    clockwise = true;
                }
                step = 4;
                return false;
            } else if (!right.get() && step == 4 && !clockwise && leftFirst) {
                SmartDashboard.putString("Command", "Translate right");
                step = 5;
                return false;
            } else if (!left.get() && step == 5 && !clockwise && leftFirst) {
                searchCancel();
            }
            
            else if (right.get() && step == 4 && clockwise && leftFirst) {
                SmartDashboard.putString("Command", "Translate left");
                step = 5;
                return false;
            } else if (left.get() && step == 5 && clockwise && leftFirst && !missed) {
                searchCancel();
            }
            
            else if (!right.get() && step == 4 && clockwise && leftFirst /*&& swerve not moving*/) {
                SmartDashboard.putString("Command", "(If swerve not moving) Move forward 10 inches, move right 2 in"); //TODO: measure
                missed = true;
                step = 5;
            } else if (step == 5 && clockwise && leftFirst && missed /*&& swerve not moving*/) {
                SmartDashboard.putString("Command", "(If swerve not moving) Call searchLeft()");
                searchLeft();
            }
            
            else if (!left.get() && right.get() && step == 2) {
                SmartDashboard.putString("Command", "Rotate counter-clockwise");
                step = 3;
                rightFirst = true;
                return false;
            } else if (left.get() && step == 3 && rightFirst) {
                SmartDashboard.putString("Command", "Translate left");
                step = 4;
                return false;
            } else if (right.get() && step == 4 && rightFirst) {
                searchCancel();
            }
            
            else {return false;}
        case SEARCH_RIGHT:
        if (!left.get() && !right.get() && step == 1) {
            SmartDashboard.putString("Command", "Translate right");
            step = 2;
            return false;
        } else if (right.get() && !left.get() && step == 2) {
            SmartDashboard.putString("Command", "Translate right");
            step = 3;
            rightFirst = true;
            return false;
        } else if (!right.get() && step == 3 && rightFirst) {
            if (left.get()) {
                SmartDashboard.putString("Command", "Rotate counter-clockwise");
                clockwise = true;
            } else {
                SmartDashboard.putString("Command", "Rotate clockwise");
                clockwise = false;
            }
            step = 4;
            return false;
        } else if (!left.get() && step == 4 && clockwise && rightFirst) {
            SmartDashboard.putString("Command", "Translate left");
            step = 5;
            return false;
        } else if (right.get() && step == 5 && clockwise && rightFirst) {
            searchCancel();
        }
        
        else if (left.get() && step == 4 && !clockwise && rightFirst) {
            SmartDashboard.putString("Command", "Translate right");
            step = 5;
            return false;
        } else if (right.get() && step == 5 && !clockwise && rightFirst && !missed) {
            searchCancel();
        } 
        
        else if (!right.get() && left.get() && step == 2) {
            SmartDashboard.putString("Command", "Rotate counter-clockwise until 45 degrees");
            step = 3;
            leftFirst = true;
            return false;
        }
        
        else if (right.get() && step == 3 && leftFirst) {
            SmartDashboard.putString("Command", "Translate right");
            step = 4;
            return false;
        } else if (left.get() && step == 4 && leftFirst && !missed) {
            searchCancel();
        }
            
        else if (!right.get() && step == 3 && leftFirst /*&& swerve not moving*/) {
            SmartDashboard.putString("Command", "(If swerve not moving) Move forward 10 inches, move left 2 in"); //TODO: measure
            missed = true;
            step = 5;
        } else if (step == 4 && leftFirst && missed /*&& swerve not moving*/) {
            SmartDashboard.putString("Command", "(If swerve not moving) Call searchRight()");
            searchRight();
        }
        
        else {return false;}
        default:
            return true;
        }
    }
}