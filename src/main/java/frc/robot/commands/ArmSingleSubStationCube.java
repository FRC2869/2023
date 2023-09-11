package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;

public class ArmSingleSubStationCube extends SequentialCommandGroup {
	public ArmSingleSubStationCube(){
		addCommands(new ArmMove(PositionsPivot.SINGLE_CUBE, PositionsWrist.SINGLE_CUBE, 0,1));
	}
}