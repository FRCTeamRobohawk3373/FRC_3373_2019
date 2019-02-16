package frc.team3373.robot;

import org.json.JSONArray;

import edu.wpi.first.wpilibj.AnalogInput;

public class DistanceSensor {

    double[][] table;

    private AnalogInput sensor; // Analog sensor

    private boolean useLookup = false;

    private double a;
    private double b;
    private double c;
    private double d;
    private double e;
    private double f;

    Constants con = new Constants();

    public DistanceSensor(int port, double[][] lookupTable) {
        sensor = new AnalogInput(port);

        table = lookupTable;

        useLookup = true;

        sensor.setAverageBits(8); // Sets how many readings should be averaged, 2^bits
    }

    public DistanceSensor(int port, int serial) {
        sensor = new AnalogInput(port);

        JSONArray array = Constants.getArray("distanceSensorValues").getJSONArray(serial);

        a = array.getDouble(0);
        b = array.getDouble(1);
        c = array.getDouble(2);
        d = array.getDouble(3);
        e = array.getDouble(4);
        f = array.getDouble(5);

        useLookup = false;

        sensor.setAverageBits(8); // Sets how many readings should be averaged
    }

    public double getDistance() {
        double x = sensor.getAverageVoltage();

        if (x < 0.42) { // Voltage is out of range
            return -2;
        }

        if (!useLookup) {
            return ((a * Math.pow(x, b)) + c) / ((d * Math.pow(x, e)) + f); // Curve fit equation:
                                                                                  // y_1\sim\frac{ax_1^b+c}{dx_1^f+g}
                                                                                  // (Desmos)
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