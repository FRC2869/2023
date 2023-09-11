package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;

public class ArmFloorPickupCone extends SequentialCommandGroup {
	public ArmFloorPickupCone(){
		addCommands(new ArmMove(PositionsPivot.FLOOR_CONE, PositionsWrist.FLOOR_CONE, 0,1));
	}
}