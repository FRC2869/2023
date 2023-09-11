package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;

public class ArmLowFront extends SequentialCommandGroup {
	public ArmLowFront(){
		addCommands(new ArmMove(PositionsPivot.LOW_FRONT, PositionsWrist.LOW_FRONT, 0,1));
	}
}