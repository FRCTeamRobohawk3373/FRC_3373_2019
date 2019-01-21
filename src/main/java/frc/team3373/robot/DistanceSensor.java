package frc.team3373.robot;

import edu.wpi.first.wpilibj.AnalogInput;;

public class DistanceSensor {
    private double[][] table1 = { //Data for sensor 1
        {0.42, 31},
        {0.44, 29},
        {0.48, 27},
        {0.5, 25},
        {0.56, 23},
        {0.60, 21},
        {0.64, 19},
        {0.72, 17},
        {0.78, 15},
        {0.83, 14},
        {0.87, 13},
        {0.95, 12},
        {1.03, 11},
        {1.12, 10},
        {1.22, 9},
        {1.35, 8},
        {1.51, 7},
        {1.76, 6},
        {2.0, 5},
        {2.4, 4},
        {3.1, 3}
    };

    private double a1 = 31.0509;
    private double b1 = 1.89643;
    private double c1 = 2.40458;
    private double d1 = 2.89964;
    private double e1 = 3.00817;
    private double f1 = 0.0597594;

    private double[][] table2 = { //Data for sensor 2
        {0.43, 31},
        {0.45, 29},
        {0.47, 27},
        {0.5, 25},
        {0.54, 23},
        {0.58, 21},
        {0.62, 19},
        {0.68, 17},
        {0.76, 15},
        {0.79, 14},
        {0.85, 13},
        {0.91, 12},
        {1.0, 11},
        {1.08, 10},
        {1.18, 9},
        {1.32, 8},
        {1.50, 7},
        {1.75, 6},
        {2.07, 5},
        {2.51, 4},
        {3.14, 3}
    };

    private double a2 = 31.1277;
    private double b2 = 2.03942;
    private double c2 = 1.285;
    private double d2 = 2.97952;
    private double e2 = 3.08719;
    private double f2 = 0.00183152;

    private AnalogInput sensor;

    private int number;

    public DistanceSensor(int port, int sensorNumber) throws SensorException {
        if (sensorNumber != 1 && sensorNumber != 2) {
            throw new SensorException("sensorNumber must be 1 or 2");
        }
        sensor = new AnalogInput(port);
        number = sensorNumber;

        sensor.setAverageBits(16);
    }

    public double getDistance() {
        //return lookupTable(sensor.getAverageVoltage(), number);
        double x = sensor.getAverageVoltage();

        if (x < 0.42) {
            return -2;
        }

        if (number == 1) {
            return ((a1 * Math.pow(x, b1)) + c1) / ((d1 * Math.pow(x, e1)) + f1);
        } else if (number == 2) {
            return ((a2 * Math.pow(x, b2)) + c2) / ((d2 * Math.pow(x, e2)) + f2);
        } else {
            return -1;
        }
    }

    public double getLookup() {
        return lookupTable(sensor.getAverageVoltage(), number);
    }

    private double[][] getTable(int sensorNumber) {
        if (sensorNumber == 1) {
            return table1;
        } if (sensorNumber == 2) {
            return table2;
        } else {
            return null;
        }
    }

    private double lookupTable(double input, int sensorNumber) {
        double[] pdata = new double[2];
        double m = -1; //Slope
        double distance = -1;

        if (input < 0.42) {
            return -2;
        }

        for (double[] data : table2) {
            if (input < data[0]) {
                m = (data[1] - pdata[1]) / (data[0] - pdata[0]);
                System.out.println("Slope: " + m);
                distance = (m * (input - data[0])) + data[1];
                //System.out.println("Distance: " + distance);
                return distance;
            }
            pdata[0] = data[0];
            pdata[1]= data[1];
        }
        return -1;
    }


}

class SensorException extends Exception {
    public SensorException(String message) {
        super(message);
    }
}