package frc.team3373.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LineFinder {
    private DigitalInput right; // back of robot
    private DigitalInput left; // front of robot

    private boolean leftFirst;
    private boolean rightFirst;
    private boolean clockwise;

    private int step;

    private SearchMode mode;

    private enum SearchMode {
        SEARCH_LEFT, SEARCH_RIGHT, SEARCH_NONE;
    }

    public LineFinder(int lPort, int rPort) {
        left = new DigitalInput(lPort);
        right = new DigitalInput(rPort);

        leftFirst = false;
        rightFirst = false;
        clockwise = false;
        step = 1;

        mode = SearchMode.SEARCH_NONE;
    }

    public void searchLeft() {
        mode = SearchMode.SEARCH_LEFT;
    }

    public void searchRight() {
        mode = SearchMode.SEARCH_RIGHT;
    }

    public void searchCancel() {
        mode = SearchMode.SEARCH_NONE;
        // leftOver = false;
        // rightOver = false;
        step = 1;
        leftFirst = false; // can take out one
        rightFirst = false;
        clockwise = false;

        SmartDashboard.putString("Command", "Stop");
    }

    public boolean lineUpdate() {
        SmartDashboard.putBoolean("Front", left.get());
        SmartDashboard.putBoolean("Back", right.get());
        if (mode == SearchMode.SEARCH_LEFT) {
            SmartDashboard.putString("Mode", "SEARCH_LEFT");
        } else if (mode == SearchMode.SEARCH_RIGHT) {
            SmartDashboard.putString("Mode", "SEARCH_RIGHT");
        }else if (mode == SearchMode.SEARCH_NONE) {
            SmartDashboard.putString("Mode", "SEARCH_NONE");
        }
        switch (mode) {
        case SEARCH_LEFT:
            if (!left.get() && !right.get() && step == 1) {
                SmartDashboard.putString("Command", "Translate left");
                step = 2;
                return false;
            } else if (left.get() && !right.get() && step == 2) {
                //SmartDashboard.putString("Command", "Translate left");
                step = 3;
                leftFirst = true;
                return false;
            } else if (!left.get() && step == 3 && leftFirst) {
                if (right.get()) {
                    SmartDashboard.putString("Command", "Rotate counter-clockwise");
                    clockwise = false;
                } else {
                    SmartDashboard.putString("Command", "Rotate clockwise");
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
            } else if (left.get() && step == 5 && clockwise && leftFirst) {
                searchCancel();
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
        } else if (right.get() && step == 5 && !clockwise && rightFirst) {
            searchCancel();
        } 
        
        else if (!right.get() && left.get() && step == 2) {
            SmartDashboard.putString("Command", "Rotate counter-clockwise");
            step = 3;
            leftFirst = true;
            return false;
        } else if (right.get() && step == 3 && leftFirst) {
            SmartDashboard.putString("Command", "Translate right");
            step = 4;
            return false;
        } else if (left.get() && step == 4 && leftFirst) {
            searchCancel();
        }
        
        else {return false;}
        default:
            return true;
        }
    }
}