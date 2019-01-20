package frc.team3373.robot;

//import edu.wpi.first.wpilibj.AnalogInput;;

public class DistanceSensor {
    private double[][] table = {
        {0, 2},
        {1, 4},
        {2, 6},
        {3, 8},
        {4, 10},
        {5, 12}
    };

    //private AnalogInput sensor;

    public DistanceSensor(int port) {
        //sensor = new AnalogInput(port);
    }

    public double lookupTable(double input) {
        double[] pdata = new double[2];
        double m = -1; //Slope
        double distance = -1;

        for (double[] data : table) {
            if (input < data[0]) {
                m = (data[1] - pdata[1]) / (data[0] - pdata[0]);
                System.out.println("Slope: " + m);
                distance = (m * (input - data[0])) + data[1];
                System.out.println("Distance: " + distance);
                return distance;
            }
            pdata[0] = data[0];
            pdata[1]= data[1];
        }
        return -1;
    }


}