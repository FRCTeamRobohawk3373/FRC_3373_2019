package frc.team3373.robot;

import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.DigitalOutput;

public class Ultrasonic {
	AnalogInput sensor;

	double previousDistance;
	double previousAccurateDistance;
	int stabilityCounter;
	boolean inaccurate;

	public Ultrasonic(int analogport) {
		sensor = new AnalogInput(analogport);
		previousAccurateDistance = this.getRawDistance();
		previousDistance = previousAccurateDistance;
		stabilityCounter = 0;
		inaccurate = false;
	}

	public double getDistance() {
		double currentDistance = ((sensor.getAverageVoltage() / .00097656258) / 25.4) - 2;// Vcc/5120 Vcc =.00097656258
																							// supplied voltage (5v)
																							// v/vcc/5120 = mm /25.4 =
																							// in. - 2 inches because
																							// range 0 is the inside of
																							// the cone
		if (Math.abs(currentDistance - previousDistance) > 10 && !inaccurate) {// If there is an inaccurate data point
																				// e.g. spike larger than 10 inches in
																				// 1/100 seconds
			previousAccurateDistance = previousDistance;// record the last accurate distance
			inaccurate = true; // it is inaccurate

		}

		if (inaccurate) {// if the data is inaccurate
			if (Math.abs(currentDistance - previousDistance) < 10) {// if there is a non inaccurate data point
				stabilityCounter++;// increase counter for reseting inaccuracy if the signal is stable
			} else {
				stabilityCounter = 0;// if another inaccurate measurement comes in, reduce stability counter back to
										// 0
			}
		}
		if (stabilityCounter > 20 && inaccurate) {// if the signal has been stable for
			inaccurate = false;
			stabilityCounter = 0;
		} else if (inaccurate) {
			previousDistance = currentDistance;
			return previousAccurateDistance;
		}

		if (currentDistance < 180) {
			previousDistance = currentDistance;
		} else {
			// System.out.println("Signal Lost");
			return previousDistance;
		}
		return currentDistance;
	}

	public double getVoltage() {
		return sensor.getAverageVoltage();
	}

	public double getRawDistance() {
		return ((sensor.getAverageVoltage() / .00097656258) / 25.4) - 2;
	}
}