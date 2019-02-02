package frc.team3373.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.team3373.robot.Robot.ObjectType;

public class Claw {
    private DoubleSolenoid lift;
    private DoubleSolenoid grab;

    private boolean grabbed;

    public Claw(int liftForwardChannel, int liftReverseChannel, int grabForwardChannel, int grabReverseChannel) {
        lift = new DoubleSolenoid(liftForwardChannel, liftReverseChannel);
        grab = new DoubleSolenoid(grabForwardChannel, grabReverseChannel);
    }

    public void grab(Robot.ObjectType obj) {
        if (obj == Robot.ObjectType.CARGO && grabbed) {
            grabbed = false;
            // Open arms
        } else if (obj == ObjectType.CARGO && !grabbed) {
            grabbed = true;
            // Close arms
        } else if (obj == Robot.ObjectType.HATCH && grabbed) {
            grabbed = false;
            // Close arms
        } else if (obj == ObjectType.HATCH && !grabbed) {
            grabbed = true;
            // Open arms
        }
    }

    public void drop(Robot.ObjectType obj) {
        if (obj == Robot.ObjectType.CARGO) {
            // Close arms
        } else if (obj == ObjectType.HATCH) {
            // Open arms
        }
    }
}
