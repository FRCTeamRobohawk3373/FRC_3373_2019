package frc.team3373.robot;

public class VisionObject {
	public VisionObject(double positionX, double positionY,double distance,double rotation){
		this.X=positionX;
		this.Y=positionY;
		this.distance = distance;
		this.rotation = rotation;
	}
	public void print(){
		System.out.println("Target at ("+X+","+Y+") "+distance+"in and "+rotation+" degrees.");
	}
	
	public double X;
	public double Y;
	public double distance;
	public double rotation;
}
