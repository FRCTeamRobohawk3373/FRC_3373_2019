package frc.team3373.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class SuperAHRS extends AHRS {
	// Reset function, get 0-360 rotation, bump detection (get bump), get altitude (starts at zero, positive numbers up)
	
	private double previousAccelerationZ;
	private boolean hasBumped;
	
	SuperAHRS (SPI.Port port) {
		super(port);
		previousAccelerationZ = super.getWorldLinearAccelZ();
		hasBumped = false;
	}
	public float getRotation() {
		float rotation;
		rotation=(360-super.getYaw())%360;
		
		/*if (super.getYaw() >= 0) {
			rotation = super.getYaw();
		} else {
			rotation = (180 - Math.abs(super.getYaw())) + 180;
		}*/
		return rotation;
	}
	public float getRawRotation() {
		return super.getYaw();
	}
	
	private double getZJerk() {
		double currentAccel = super.getWorldLinearAccelZ();
		double deltaAccel = currentAccel - previousAccelerationZ;
		previousAccelerationZ = currentAccel;
		return deltaAccel/.01;
	}
	
	public boolean hasHitBump (double amount) {
		if(Math.abs(this.getZJerk())>amount)
			hasBumped = true;
		return hasBumped;
	}
	
	public void resetBump() {
		hasBumped = false;
	}
}