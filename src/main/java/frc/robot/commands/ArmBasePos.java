package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;

public class ArmBasePos extends SequentialCommandGroup {
	public ArmBasePos(){
		addCommands(new ArmMove(PositionsPivot.BASE, PositionsWrist.BASE, 1,0));
	}
}
