package frc.team3373.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class Dial {
    DigitalInput positionOnes;
    DigitalInput positionTwos;
    DigitalInput positionFours;
    DigitalInput positionEights;

    public Dial(int onesPort, int twosPort, int foursPort, int eightsPort) {
        positionOnes = new DigitalInput(onesPort);
        positionTwos = new DigitalInput(twosPort);
        positionFours = new DigitalInput(foursPort);
        positionEights = new DigitalInput(eightsPort);
    }

    public int getPosition() {
        int position = 15;
        if (positionOnes.get()) {
            position -= 1;
        }
        if (positionTwos.get()) {
            position -= 2;
        }
        if (positionFours.get()) {
            position -= 4;
        }
        if (positionEights.get()) {
            position -= 8;
        }
        return position;
    }
}