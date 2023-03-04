package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JacksonInject;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Limelight extends SubsystemBase {
		NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
		NetworkTableEntry targetId = table.getEntry("tid");
		NetworkTableEntry targetPos = table.getEntry("targetpose_robotspace");
		NetworkTableEntry ta = table.getEntry("ta");
		//NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(<value>);
	
		/**
		 * Finds if the april tag is in sight.
		 * @param id 
		 * @return whether the apriltag is in sight
		 */
		public boolean isTarget(int id) {
			if (targetId.getInteger(-1) == id) {
				return true;
			}
			return false;
		}

		//targetpose_robotspace - 3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6))
		/*public float[] getTagPos(int id) {
			if (isTarget(id)) {
				return targetPos.;
			}
		}*/
}
