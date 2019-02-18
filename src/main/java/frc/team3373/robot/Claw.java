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
            grab.set(Value.kForward);
        } else if (obj == ObjectType.CARGO && !grabbed) {
            grabbed = true;
            grab.set(Value.kReverse);
        } else if (obj == Robot.ObjectType.HATCH && grabbed) {
            grabbed = false;
            grab.set(Value.kReverse);
        } else if (obj == ObjectType.HATCH && !grabbed) {
            grabbed = true;
            grab.set(Value.kForward);
        }
    }

    public void drop(Robot.ObjectType obj) {
        if (obj == Robot.ObjectType.CARGO) {
            grab.set(Value.kReverse);
        } else if (obj == ObjectType.HATCH) {
            grab.set(Value.kForward);
        }
    }

    public void raise() {
        lift.set(Value.kForward);
    }

    public void lower() {
        lift.set(Value.kReverse);
    }
}
