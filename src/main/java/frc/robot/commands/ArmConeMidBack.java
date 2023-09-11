package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;

public class ArmConeMidBack extends SequentialCommandGroup {
	public ArmConeMidBack(){
		addCommands(new ArmMove(PositionsPivot.MID_CONE_BACK, PositionsWrist.MID_CONE_BACK, 0,1));
	}
}