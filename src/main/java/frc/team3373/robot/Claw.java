package frc.team3373.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.team3373.robot.Robot.ObjectType;

public class Claw {
    private DoubleSolenoid lift;
    private DoubleSolenoid grab;

    private boolean grabbed;

    public Claw(int PCM, int liftForwardChannel, int liftReverseChannel, int grabForwardChannel, int grabReverseChannel) {
        lift = new DoubleSolenoid(PCM, liftForwardChannel, liftReverseChannel);
        grab = new DoubleSolenoid(PCM, grabForwardChannel, grabReverseChannel);
    }

    public void grab(Robot.ObjectType obj) {
        if (obj == Robot.ObjectType.CARGO) {
            grab.set(Value.kForward);
        } else if (obj == Robot.ObjectType.HATCH) {
            grab.set(Value.kReverse);
        }
    }

    public void drop(Robot.ObjectType obj) {
        if (obj == Robot.ObjectType.CARGO) {
            grab.set(Value.kReverse);
            threadDisable(grab, Value.kForward);
        } else if (obj == ObjectType.HATCH) {
            grab.set(Value.kForward);
            threadDisable(grab, Value.kReverse);
        }
    }

    public void raise() {
        lift.set(Value.kForward);
    }

    public void lower() {
        lift.set(Value.kReverse);
    }

    private void threadDisable(DoubleSolenoid sol, Value value) {
        new Thread(()-> {
            try {
                Thread.sleep(500);
                sol.set(value);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        });
    }
}
