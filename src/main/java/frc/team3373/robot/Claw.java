package frc.team3373.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.team3373.robot.Robot.ObjectType;

public class Claw {
    private DoubleSolenoid lift;
    private DoubleSolenoid grab;

    private Thread solThread;

    // private boolean grabbed;

    public Claw(int PCM, int liftForwardChannel, int liftReverseChannel, int grabForwardChannel, int grabReverseChannel) {
        lift = new DoubleSolenoid(PCM, liftForwardChannel, liftReverseChannel);
        grab = new DoubleSolenoid(PCM, grabForwardChannel, grabReverseChannel);
    }

    public void grab(ObjectType obj) {
        if (obj == ObjectType.CARGO) {
            if (solThread != null)
                solThread.interrupt();
            grab.set(Value.kForward);
        } else if (obj == ObjectType.HATCH) {
            if (solThread != null)
                solThread.interrupt();
            grab.set(Value.kReverse);
        }
    }

    public void open() {
        grab.set(Value.kReverse);
    }

    public void close() {
        grab.set(Value.kForward);
    }

    public void release(ObjectType obj) {
        if (obj == ObjectType.CARGO) {
            grab.set(Value.kReverse);
        } else if (obj == ObjectType.HATCH) {
            grab.set(Value.kForward);
            disableSolenoid(grab, Value.kOff);
        }
    }

    public void raise() {
        lift.set(Value.kForward);
    }

    public void lower() {
        lift.set(Value.kReverse);
    }

    private void disableSolenoid(DoubleSolenoid sol, Value value) {
        solThread = new Thread(() -> {
            try {
                Thread.sleep(500);
                sol.set(value);
            } catch (InterruptedException e) {
            }
        });
        solThread.run();
    }
}
