package frc.team3373.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class LineFinder {
    private DigitalInput right; // back of robot
    private DigitalInput left; // front of robot

    private boolean leftCorrect;
    private boolean rightCorrect;
    private boolean leftOver;
    private boolean rightOver;
    private boolean moveRight;
    private boolean moveLeft;

    private SearchMode mode;

    private enum SearchMode {
        SEARCH_LEFT, SEARCH_RIGHT, SEARCH_NONE;
    }

    public LineFinder(int rPort, int lPort) {
        right = new DigitalInput(rPort);
        left = new DigitalInput(lPort);

        leftCorrect = false;
        rightCorrect = false;
        leftOver = false;
        rightOver = false;
        moveRight = false;
        moveLeft = false;
    }

    public void searchLeft() {
        mode = SearchMode.SEARCH_LEFT;
    }

    public void searchRight() {
        mode = SearchMode.SEARCH_RIGHT;
    }

    public void searchCancel() {
        mode = SearchMode.SEARCH_NONE;
        leftCorrect = false;
        rightCorrect = false;
        leftOver = false;
        rightOver = false;
    }

    public boolean Update() {
        switch (mode) {
        case SEARCH_LEFT:
            if (leftCorrect && rightCorrect) {
                mode = SearchMode.SEARCH_NONE;
                leftCorrect = false;
                rightCorrect = false;
                leftOver = false;
                rightOver = false;
                return true;
            } else if (!left.get() && !right.get() && !leftOver && !rightOver) {
                System.out.println("Translate left");
                return false;
            } else if (left.get() && !right.get() && !leftOver && !rightOver) {
                System.out.println("Translate left");
                leftOver = true;
                return false;
            } else if (!left.get() && leftOver && !rightOver) {
                if (right.get()) {
                    System.out.println("Rotate counter-clockwise");
                    rightOver = true;
                } else {
                    System.out.println("Rotate clockwise");
                }
                leftCorrect = true;
                return false;
            } else if (!left.get() && !right.get()  && leftOver && rightOver) {
                System.out.println("Translate right");
                moveRight = true;
                return false;
            } else if (!left.get() && !right.get() && leftOver  && !rightOver) {
                System.out.println("Translate left");
                moveLeft = true;
            } else if (!left.get() && right.get() && leftOver && rightOver && moveRight) {
                mode = SearchMode.SEARCH_NONE;
                leftCorrect = false;
                rightCorrect = false;
                leftOver = false;
                rightOver = false;
                return true;
            } else if (!left.get() && right.get() && !leftCorrect && !rightCorrect && !leftOver) {
                System.out.println("Rotate front left");
                rightCorrect = true;
                return false;
            } else if (!left.get() && right.get() && !leftCorrect && rightCorrect && !leftOver) {
                System.out.println("Rotate front left");
                return false;
            } else if (left.get() && right.get() && !leftCorrect && rightCorrect && !leftOver) {
                System.out.println("Rotate front left");
                leftOver = true;
                return false;
            } else if (!left.get() && right.get() && !leftCorrect && rightCorrect && leftOver) {
                mode = SearchMode.SEARCH_NONE;
                leftCorrect = false;
                rightCorrect = false;
                leftOver = false;
                rightOver = false;
                return true;
            } else {System.out.println("ERROR!"); return false;}
        case SEARCH_RIGHT:
            if (leftCorrect && rightCorrect) {
                mode = SearchMode.SEARCH_NONE;
                leftCorrect = false;
                rightCorrect = false;
                leftOver = false;
                rightOver = false;
                return true;
            } else if (!left.get() && !right.get() && !leftCorrect && !rightCorrect && !rightOver) {
                System.out.println("Translate right");
                return false;
            } else if (left.get() && !right.get() && !leftCorrect && !rightCorrect && !rightOver) {
                System.out.println("Rotate back right");
                leftCorrect = true;
                return false;
            } else if (left.get() && !right.get() && leftCorrect && !rightCorrect && !rightOver) {
                System.out.println("Rotate back right");
                return false;
            } else if (left.get() && right.get() && leftCorrect && !rightCorrect && !rightOver) {
                System.out.println("Rotate back right");
                rightOver = true;
                return false;
            } else if (!left.get() && !right.get() && leftCorrect && !rightCorrect && rightOver) {
                mode = SearchMode.SEARCH_NONE;
                leftCorrect = false;
                rightCorrect = false;
                leftOver = false;
                rightOver = false;
                return true;
            } else if (!left.get() && right.get() && !leftCorrect && !rightCorrect && !rightOver) {
                System.out.println("Translate right");
                rightOver = true;
                return false;
            } else if (!left.get() && !right.get() && !leftCorrect && !rightCorrect && rightOver) {
                System.out.println("Rotate front right");
                rightCorrect = true;
                return false;
            } else if (left.get() && right.get() && !leftCorrect && rightCorrect && !rightOver) {
                System.out.println("Rotate front right");
                rightCorrect = true;
                return false;
            } else if (!left.get() && right.get() && !leftCorrect && rightCorrect && !rightOver) {
                mode = SearchMode.SEARCH_NONE;
                leftCorrect = false;
                rightCorrect = false;
                leftOver = false;
                rightOver = false;
                return true;
            } else {System.out.println("ERROR!"); return false;}
        default:
            return true;
        }
    }
}