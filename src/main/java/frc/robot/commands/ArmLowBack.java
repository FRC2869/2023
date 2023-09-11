package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;

public class ArmLowBack extends SequentialCommandGroup {
	public ArmLowBack(){
		addCommands(new ArmMove(PositionsPivot.LOW_BACK, PositionsWrist.LOW_BACK, 0,1));
	}
}