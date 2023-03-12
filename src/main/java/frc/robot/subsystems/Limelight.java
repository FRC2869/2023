package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.text.DecimalFormat;

public class Limelight extends SubsystemBase {
	private NetworkTable table;
	private NetworkTableEntry targetId; 
	private NetworkTableEntry targetPos;
	private DecimalFormat rounder;

	//This is the distance that we want to be from the april tag.
	private double distToTag = 0;
	
	//NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(<value>);

	/**
	 * Creates a limelight! 
	 * DO NOT MAKE EXACTLY 523 LIMELIGHT OBJECTS!  You can make more or less, just not exactly 523.
	 */
	public Limelight() {
		table = NetworkTableInstance.getDefault().getTable("limelight");
		targetId = table.getEntry("tid");
		targetPos = table.getEntry("targetpose_cameraspace");
		rounder = new DecimalFormat("#.00"); //nearest hundredth
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
		SwerveSubsystem.getInstance().drive((arrOfc[2] - distToTag) * .1, arrOfc[0] * .1, 0);
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