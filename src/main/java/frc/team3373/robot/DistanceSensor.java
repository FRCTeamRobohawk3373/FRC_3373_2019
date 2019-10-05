package frc.team3373.robot;

import org.json.JSONArray;
import org.json.JSONException;

import edu.wpi.first.wpilibj.AnalogInput;

public class DistanceSensor {
    private AnalogInput sensor; // Analog distance sensor

    // For power fit
    private double a; // Coefficient
    private double b; // Power
    private double c; // Offset

    private double pDist; // Previous distance for smart voltage (if not enough time has passed)
    private long lastUpdateMicros; // The time (in microseconds) when the smart voltage (pDist) has been updated

    private double[] rolling; // Array for storing values for the moving average
    private int index; // The index in rolling that has been updated the least recently

    private String dist = "distanceSensorValues";

    /**
     * Class for instantiating a distance sensor using an power fit equation
     * @param port The analog port that the distance sensor is plugged into
     * @param serial The serial number of the sensor, used to find the index of the array with the coefficients
     */
    public DistanceSensor(int port, int serial) {
        sensor = new AnalogInput(port);
        pDist = 0;
        JSONArray array;
        try {
            array = Constants.getArray("distanceSensorValues").getJSONArray(serial);
        } catch (JSONException e) {
            array = Constants.getArray("distanceSensorValues").getJSONArray(0);
        }
        a = array.getDouble(0);
        b = array.getDouble(1);
        c = array.getDouble(2);
        System.out.println(a);
        
        sensor.setAverageBits(8); // Sets how many readings should be averaged
        rolling = new double[4];
    }

    /**
     * Class for instantiating a distance sensor using an power fit equation
     * @param port The analog port that the distance sensor is plugged into
     * @param serial The serial number of the sensor, used to find the index of the array with the coefficients
     * @param smart Whether to use smart calibration values for finding distance
     */
    public DistanceSensor(int port, int serial, boolean smart) {
        sensor = new AnalogInput(port);
        pDist = 0;
        JSONArray array;
        if (smart)
            dist += "-smart";
        try{
            array = Constants.getArray(dist).getJSONArray(serial);
        }catch(JSONException e){
            array = Constants.getArray(dist).getJSONArray(0);
        }
        a = array.getDouble(0);
        b = array.getDouble(1);
        c = array.getDouble(2);
        System.out.println(a);
        
        sensor.setAverageBits(8); // Sets how many readings should be averaged
    }
    
    /**
     * Returns the raw value for the voltage from the sensor
     */
    public double getVoltage() {
        return sensor.getVoltage();
    }

    /**
     * Returns the RoboRio FPGA average
     */
    public double getAverage() {
        return sensor.getAverageVoltage();
    }

    /**
     * Finds voltage by using the smallest value from a burst of samples
     * @param samples The number of samples to take
     */
    public double getSmartVoltage(int samples) {
        long micros = System.nanoTime() / 1000;
        int sample_wait_micros = 38000;
        int burst_delay_micros = 1500;

        if (micros - lastUpdateMicros < sample_wait_micros)
            return pDist;

        lastUpdateMicros = micros;

        double currReading;
        double lowestReading = 5;
        for (int i = 0; i < samples; i++) {
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

    public double getSmartAverage(int samp) {
        rolling[index] = getSmartVoltage(samp);
        double sum = 0;
        for (int i = 0; i < rolling.length; i++) {
            sum += rolling[i];
        }
        index++;
        if (index + 1 > rolling.length)
            index = 0;
        return sum / rolling.length;
    }

    public double getDistance() {
        double x = sensor.getAverageVoltage();

        if (x < 0.42) { // Voltage is out of range
            return -2;
        }

        return ((a * Math.pow(x, b)) + c); // Curve fit equation: y\sim ax^b+c
    }

    public double getSmartDistance(int samp) {
        double x = getSmartVoltage(samp);

        if (x < 0.42) { // Voltage is out of range
            return -2;
        }

        return ((a * Math.pow(x, b)) + c); // Curve fit equation: y\sim ax^b+c
    }

}