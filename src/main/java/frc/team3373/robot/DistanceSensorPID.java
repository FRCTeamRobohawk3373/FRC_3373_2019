package frc.team3373.robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DistanceSensorPID implements PIDSource {
    DistanceSensor dleft;
    DistanceSensor dright;

    PIDSourceType sourceType;

    public DistanceSensorPID(DistanceSensor dl, DistanceSensor dr) {
        dleft = dl;
        dright = dr;

        sourceType = PIDSourceType.kDisplacement;
    }

    public double pidGet() {
        double rdist = (double) Math.round(100 * dright.getDistance()) / 100; // Rounds values of distance sensors
        double ldist = (double) Math.round(100 * dleft.getDistance()) / 100;

        SmartDashboard.putNumber("Left Distance", dleft.getDistance());// TODO: Remove debug
        SmartDashboard.putNumber("Right Distance", dright.getDistance());
        SmartDashboard.putNumber("Left Avg Dist", ldist);
        SmartDashboard.putNumber("Right Avg Dist", rdist);

        if (rdist == -2.0 && ldist == -2.0) { // If the sensors are out of range, do nothing
            rdist = 0;
            ldist = 0;
        } else if (ldist == -2.0) { // If just one of the sensors is out of range set the distance to just above the
                                    // max value
            ldist = 32.0;
        } else if (rdist == -2.0) {
            rdist = 32.0;
        }

        return (rdist - ldist) + 2.8; // Returns the difference of the distance sensors plus an offset
    }

    public PIDSourceType getPIDSourceType() {
        return sourceType;
    }

    public void setPIDSourceType(PIDSourceType type) {
        sourceType = type;
    }
}