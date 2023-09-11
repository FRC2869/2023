package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;

public class ArmCubeMidBack extends SequentialCommandGroup {
	public ArmCubeMidBack(){
		addCommands(new ArmMove(PositionsPivot.MID_CUBE_BACK, PositionsWrist.MID_CUBE_BACK, 0,1));
	}
}