package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;

public class ArmCubeMidFront extends SequentialCommandGroup {
	public ArmCubeMidFront(){
		addCommands(new ArmMove(PositionsPivot.MID_CUBE_FRONT, PositionsWrist.MID_CUBE_FRONT, 0,1));
	}
}