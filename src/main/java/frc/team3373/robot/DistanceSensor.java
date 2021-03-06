package frc.team3373.robot;

import org.json.JSONArray;
import org.json.JSONException;

import edu.wpi.first.wpilibj.AnalogInput;

public class DistanceSensor {

    double[][] table;

    private AnalogInput sensor; // Analog sensor

    private boolean useLookup = false;

    private double a;
    private double b;
    private double c;
    /* private double d;
    private double e;
    private double f; */

    private double pDist;
    private long lastUpdateMicros;

    Constants con = new Constants();

    /**
     * Class for instantiating a distance sensor using a lookup table (Unused)
     * @param port The analog port that the distance sensor is plugged into
     * @param lookupTable A 2D double array, first value is the distance, second value is the analog value 
     */
    public DistanceSensor(int port, double[][] lookupTable) {
        sensor = new AnalogInput(port);

        table = lookupTable;

        useLookup = true;

        sensor.setAccumulatorCenter(1401);
        sensor.setAccumulatorDeadband(1056);
        sensor.setAverageBits(8); // Sets how many readings should be averaged, 2^bits
    }

    /**
     * Class for instantiating a distance sensor using an exponential fit equation
     * @param port The analog port that the distance sensor is plugged into
     * @param serial The serial number of the sensor, used to find the index of the array with the coefficients
     */
    public DistanceSensor(int port, int serial) {
        sensor = new AnalogInput(port);
        pDist = 0;
        JSONArray array;
        try{
            array = Constants.getArray("distanceSensorValues").getJSONArray(serial);
        }catch(JSONException e){
            array = Constants.getArray("distanceSensorValues").getJSONArray(0);
        }
        a = array.getDouble(0);
        b = array.getDouble(1);
        c = array.getDouble(2);
        /* d = array.getDouble(3);
        e = array.getDouble(4);
        f = array.getDouble(5); */
        System.out.println(a);
        
        useLookup = false;
        
        sensor.setAverageBits(10); // Sets how many readings should be averaged
    }
    
    /**
     * @return The raw value for the voltage from the sensor
     */
    public double getVoltage() {
        return sensor.getVoltage();
    }

    /**
     * @return 
     */
    public double getAverage() {
        return sensor.getAverageVoltage();
    }

    public double getSmartAverage() {
        long micros = System.nanoTime() / 1000;
        int sample_wait_micros = 38000;
        int burst_delay_micros = 1500;
        int numBurstSamples = 8;

        if (micros - lastUpdateMicros < sample_wait_micros)
            return pDist;

        lastUpdateMicros = micros;

        double currReading;
        double lowestReading = 5;
        for (int i = 0; i < numBurstSamples; i++) {
            currReading = sensor.getVoltage();

            if (currReading < lowestReading)
                lowestReading = currReading;

            try {
                Thread.sleep(burst_delay_micros / 1000, (int)(burst_delay_micros % 1000));
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        pDist = lowestReading;
        return lowestReading;
    }

    public double getDistance() {
        double x = sensor.getAverageVoltage();

        if (x < 0.42) { // Voltage is out of range
            return -2;
        }

        if (!useLookup) {
            //return ((a * Math.pow(x, b)) + c) / ((d * Math.pow(x, e)) + f); // Curve fit equation:
                                                                                  // y\sim\frac{ax^b+c}{dx^f+g}
                                                                                  // (Desmos)
            return ((a * Math.pow(x, b)) + c); // Curve fit equation: y\sim ax^b+c
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