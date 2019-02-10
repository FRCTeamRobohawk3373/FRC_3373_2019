package frc.team3373.robot;

import edu.wpi.first.wpilibj.PIDController;

public class SuperPID extends PIDController {
    SuperAHRS ahrs;

    public SuperPID(double p, double i, double d, DistanceSensorPID dist, SuperPIDOutput output) {
        super(p, i, d, dist, output);
    }

    public SuperPID(double p, double i, double d, SuperAHRS ahrs, SuperPIDOutput output) {
        super(p, i, d, ahrs, output);
    }

    protected void initDefaultCommand() {
    }

    public void start() {
        super.enable();
    }

    public void end() {
        super.disable();
    }
}