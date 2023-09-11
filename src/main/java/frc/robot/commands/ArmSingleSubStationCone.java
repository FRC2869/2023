package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;

public class ArmSingleSubStationCone extends SequentialCommandGroup {
	public ArmSingleSubStationCone(){
		addCommands(new ArmMove(PositionsPivot.SINGLE_CONE, PositionsWrist.SINGLE_CONE, 0,1));
	}
}