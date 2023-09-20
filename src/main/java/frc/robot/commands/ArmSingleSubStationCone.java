package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PivotConstants.PositionsPivot;
import frc.robot.Constants.WristConstants.PositionsWrist;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmSingleSubStationCone extends SequentialCommandGroup {
	public ArmSingleSubStationCone(){
		var cmd = WristSubsystem.getInstance().getCurrentCommand();
		if(cmd != null){
			cmd.cancel();
		}
		var cmd2 = PivotSubsystem.getInstance().getCurrentCommand();
		if(cmd2 != null){
			cmd2.cancel();
		}
		addCommands(new ArmMove(PositionsPivot.SINGLE_CONE, PositionsWrist.SINGLE_CONE, 0,1));
	}
}