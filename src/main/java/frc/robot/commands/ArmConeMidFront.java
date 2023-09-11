package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;

public class ArmConeMidFront extends SequentialCommandGroup {
	public ArmConeMidFront(){
		addCommands(new ArmMove(PositionsPivot.MID_CONE_FRONT, PositionsWrist.MID_CONE_FRONT, 0,1));
	}
}