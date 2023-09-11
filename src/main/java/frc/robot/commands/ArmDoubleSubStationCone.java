package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;

public class ArmDoubleSubStationCone extends SequentialCommandGroup {
	public ArmDoubleSubStationCone(){
		addCommands(new ArmMove(PositionsPivot.DOUBLE_CONE, PositionsWrist.DOUBLE_CONE, 0,1));
	}
}