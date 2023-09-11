package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;

public class ArmDoubleSubStationCube extends SequentialCommandGroup {
	public ArmDoubleSubStationCube(){
		addCommands(new ArmMove(PositionsPivot.DOUBLE_CUBE, PositionsWrist.DOUBLE_CUBE, 0,1));
	}
}