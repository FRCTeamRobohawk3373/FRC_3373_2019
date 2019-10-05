package frc.team3373.robot;

import org.json.JSONArray;
import org.json.JSONException;

import edu.wpi.first.wpilibj.AnalogInput;

public class DistanceSensor {
    private AnalogInput sensor; // Analog sensor

    private double a;
    private double b;
    private double c;

    private double pDist;
    private long lastUpdateMicros;

    Constants con = new Constants();

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
        System.out.println(a);
        
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

        return ((a * Math.pow(x, b)) + c); // Curve fit equation: y\sim ax^b+c
    }

}