package frc.team3373.robot;

import edu.wpi.first.wpilibj.AnalogInput;;

public class DistanceSensor {

    double[][] table;

    private AnalogInput sensor; // Analog sensor

    private boolean useLookup = false;

    private double a1;
    private double b1;
    private double c1;
    private double d1;
    private double e1;
    private double f1;

    public DistanceSensor(int port, double[][] lookupTable) {
        sensor = new AnalogInput(port);

        table = lookupTable;

        useLookup = true;

        sensor.setAverageBits(8); // Sets how many readings should be averaged
    }

    public DistanceSensor(int port, double a, double b, double c, double d, double e, double f) {
        sensor = new AnalogInput(port);

        a1 = a;
        b1 = b;
        c1 = c;
        d1 = d;
        e1 = e;
        f1 = f;

        useLookup = false;

        sensor.setAverageBits(8); // Sets how many readings should be averaged
    }

    public double getDistance() {
        double x = sensor.getAverageVoltage();

        if (x < 0.42) {
            return -2;
        }

        if (!useLookup) {
            return ((a1 * Math.pow(x, b1)) + c1) / ((d1 * Math.pow(x, e1)) + f1); // Curve fit equation:
            // y_1\sim\frac{ax_1^b+c}{dx_1^f+g} (Desmos)
        } else {
            return lookupTable(sensor.getAverageVoltage());
        }
    }

    private double lookupTable(double input) { // Uses a lookup table to find distance
        double[] pdata = new double[2];
        double m = -1; // Slope
        double distance = -1;

        if (input < 0.42) {
            return -2;
        }

        for (double[] data : table) {
            if (input < data[0]) {
                m = (data[1] - pdata[1]) / (data[0] - pdata[0]);
                distance = (m * (input - data[0])) + data[1];
                return distance;
            }
            pdata[0] = data[0];
            pdata[1] = data[1];
        }
        return -1;
    }

}