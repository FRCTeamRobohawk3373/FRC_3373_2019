package frc.team3373.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotState;

public class Elevator {
    private WPI_TalonSRX rMotor;
    private WPI_TalonSRX lMotor;

    private SuperJoystick joystick;

    private AnalogInput leftLimit;
    private AnalogInput rightLimit;

    public Elevator(int leftMotor, int rightMotor, SuperJoystick shooter, int leftLimitPort, int rightLimitPort) {
        rMotor = new WPI_TalonSRX(rightMotor);
        lMotor = new WPI_TalonSRX(leftMotor);

        leftLimit = new AnalogInput(leftLimitPort);
        rightLimit = new AnalogInput(rightLimitPort);

        joystick = shooter;
    }

    public void raise() {
        // Raise elevator
    }

    public void lower() {
        // Lower elevator
    }

    public void moveToHeight(int height, Robot.ObjectType obj) {
        while(!joystick.isXHeld() && !RobotState.isDisabled()) {
            switch(height) {
                case 0:
                    switch(obj) {
                        case CARGO:
                            //
                            break;
                        case HATCH:
                            //
                            break;
                        default:
                            System.out.println("Object must be defined!");
                            return;
                    }
                    break;
                case 1:
                    switch(obj) {
                        case CARGO:
                            //
                            break;
                        case HATCH:
                            //
                            break;
                        default:
                            System.out.println("Object must be defined!");
                            return;
                    }
                    break;
                case 2:
                    switch(obj) {
                        case CARGO:
                            //
                            break;
                        case HATCH:
                            //
                            break;
                        default:
                            System.out.println("Object must be defined!");
                            return;
                    }
                    break;
                default:
                    System.out.println("Height must be 0, 1, or 2");
            }
        }
    }
}
