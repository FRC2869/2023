package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

public class ArmBasePos extends CommandBase{
	// private ArmSubsystem arm;
	private PivotSubsystem pivot;
	// private int armCounter;
	private int pivotCounter;
	private boolean hasRun;

	public ArmBasePos(){
		// arm = ArmSubsystem.getInstance();
		pivot = PivotSubsystem.getInstance();

		// addRequirements(arm);
		addRequirements(pivot);
	}

	@Override
	public void execute(){
		if(!hasRun){
			hasRun = true;
			System.out.println(Constants.autoTimer.get()+": Arm Base Pos Start");
		}
		// arm.setPositionControl(true);
		// arm.position(ArmConstants.Extension.highConeDistance);
		pivot.setPositionControl(true);
		pivot.position(PivotConstants.basePosition);
	}

	@Override
	public boolean isFinished(){
		
		// boolean armDone = Math.abs(arm.getPosition()-ArmConstants.Extension.highConeDistance) < ArmConstants.Extension.tolerance;
		boolean pivotDone = Math.abs(pivot.getAngle()-PivotConstants.basePosition) < PivotConstants.tolerance;

		// if(armDone){
		// 	armCounter++;
		// }else{
		// 	armCounter=0;
		// }
		if(pivotDone){
			pivotCounter++;
		}else{
			pivotCounter=0;
		}

		if(pivotCounter>Constants.pidTimer){
			System.out.println(Constants.autoTimer.get()+": Arm Base Pos End");
			pivot.setPositionControl(false);
			return true;
		}else{
			return false;
		}
		// return (arm.getPosition()==ArmConstants.Extension.lowConeDistance) && (pivot.getAngle() == PivotConstants.lowConeAngle);
	}
}
