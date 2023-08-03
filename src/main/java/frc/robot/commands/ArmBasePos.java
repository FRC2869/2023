package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmBasePos extends CommandBase{
	// private ArmSubsystem arm;
	private PivotSubsystem pivot;
	private boolean hasRun;
	private WristSubsystem wrist;
	private double startTime;

	public ArmBasePos(){
		// arm = ArmSubsystem.getInstance();
		pivot = PivotSubsystem.getInstance();
		wrist = WristSubsystem.getInstance();

		// addRequirements(arm);
		
		addRequirements(pivot);
		addRequirements(wrist);
	}

	@Override
	public void execute(){
		if(!hasRun){
			hasRun = true;
			startTime = Constants.autoTimer.get();
			System.out.println(startTime+": Arm Base Pos Start");
		}
		// arm.setPositionControl(true);
		// arm.position(ArmConstants.Extension.highConeDistance);
		if((Constants.autoTimer.get()-startTime)>1.5){
		pivot.setPositionControl(false);
		pivot.position(PivotConstants.basePosition);
		}
		wrist.setPositionControl(true);
		wrist.position(WristConstants.basePosition);
		

	}

	@Override
	public boolean isFinished(){
		
		// boolean armDone = Math.abs(arm.getPosition()-ArmConstants.Extension.highConeDistance) < ArmConstants.Extension.tolerance;
		boolean pivotDone = Math.abs(pivot.getAngle()-PivotConstants.basePosition)<PivotConstants.tolerance;
		boolean wristDone = Math.abs(wrist.getAngle()-WristConstants.basePosition) < WristConstants.tolerance;

		// if(armDone){
		// 	armCounter++;
		// }else{
		// 	armCounter=0;
		// }

		if(pivotDone&&wristDone){
			System.out.println(Constants.autoTimer.get()+": Arm Base Pos End");
			pivot.setPositionControl(false);
			wrist.setPositionControl(false);
			return true;
		}else{
			return false;
		}
		// return (arm.getPosition()==ArmConstants.Extension.lowConeDistance) && (pivot.getAngle() == PivotConstants.lowConeAngle);
	}
}
