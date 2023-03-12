package frc.robot.subsystems;

import java.text.DecimalFormat;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
	private static Limelight instance;
	private NetworkTable table;
	private NetworkTableEntry targetId; 
	private NetworkTableEntry targetPos;
	private DecimalFormat rounder;
	private DrivetrainSubsystem swerve;
	//This is the distance that we want to be from the april tag.
	private double distToTagX = 0;
	private double distToTagY = 0;
	
	//NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(<value>);

	public static Limelight getInstance(){
		if(instance==null){
			instance = new Limelight();
		}
		return instance;
	}
	/**
	 * Creates a limelight! 
	 * DO NOT MAKE EXACTLY 523 LIMELIGHT OBJECTS!  You can make more or less, just not exactly 523.
	 */
	public Limelight() {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		targetId = table.getEntry("tid");
		targetPos = table.getEntry("targetpose_cameraspace");
		rounder = new DecimalFormat("#.00"); //nearest hundredth
		swerve = DrivetrainSubsystem.getInstance();
	}
	/**
	 * Finds if the april tag is in sight.
	 * @param id Id of the april tag that you want to search for.
	 * @return A bool where true means the april tag is in sight.
	 */
	public boolean isTarget(int id) {
		if (targetId.getInteger(-2) == id) {
			return true;
		}
		return false;
	}

	public boolean hasTarget() {
		if (targetId.getInteger(-2) == -1) {
			return false;
		}
		return true;
	}
	//targetpose_robotspace - 3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6))
	
	/**
	 * Determines the distance to the nearest April tag.
	 * To stringify (big SAT word) use getValuesString().
	 * @return The 3d transformation of the April tag based as an array.
	 */
	public double[] getTagPos() {
		return targetPos.getDoubleArray(new double[6]);
	}

	//TODO: Test goToTarget().
	/**
	 * Goes to target maybe.
	 */
	public void goToTarget() {
		double[] arrOfc = getTagPos();
		var x = (arrOfc[2] - distToTagX)*.1;
		var y = (arrOfc[0]-distToTagY)*.1;
		var rot = (arrOfc[4])*.1;
		
		System.out.print(x*10);
		System.out.print(" ");
		System.out.println(y*10);
		if (hasTarget())
			swerve.drive(()->(x*DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),()->  (-y*DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND),()-> rot*DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
	}

	/**
	 * Gets the distance between the tag and the robot (might not work?)
	 * @return The distance between the tag and the robot
	 */
	public double getDistanceToTag() {
		double[] pos = getTagPos();
		return Math.sqrt(pos[0] * pos[0] + pos[2] * pos[2]);
	}

	/**
	 * Determines the angle between the robot and the tag.
	 * @return 
	 */
	public double getAngleToTag() {
		double[] pos = getTagPos();
		return pos[3];
	}

	/**
	 * Stringifies the 3d transform of the tag (int reference to the camera).
	 * Basically getDistanceToTag() but string.
	 * @return The 3d transformation of the April tag based as an array.
	 */
	public String getValuesString() {
		StringBuilder sb = new StringBuilder();
		double[] pos = getTagPos();
		//six values should be: ___________________x,y,z,??????
		
		for(int i = 0; i < pos.length; i++) {
			sb.append(rounder.format(pos[i]));
			if(i<pos.length-1)
				sb.append(",");		
		}
		return sb.toString();
	}
}