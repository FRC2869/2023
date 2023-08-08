package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmLowFront extends CommandBase{
	// private ArmSubsystem arm;
	private PivotSubsystem pivot;
	// private int armCounter;
	private int pivotCounter;
	private WristSubsystem wrist;
	private int wristCounter;

	public ArmLowFront(){
		// arm = ArmSubsystem.getInstance();
		pivot = PivotSubsystem.getInstance();
		wrist = WristSubsystem.getInstance();
		System.out.println("Low");
		// addRequirements(arm);
		addRequirements(pivot);
		addRequirements(wrist);
	}

	@Override
	public void execute(){
		// arm.setPositionControl(true);
		// arm.position(ArmConstants.Extension.lowConeDistance);
		pivot.position(PivotConstants.lowFrontAngle);
		wrist.position(WristConstants.lowFrontAngle);
	}

	@Override
	public boolean isFinished(){
		
		// boolean armDone = Math.abs(arm.getPosition()-ArmConstants.Extension.lowConeDistance) < ArmConstants.Extension.tolerance;
		boolean pivotDone = Math.abs(pivot.getAngle()-PivotConstants.lowFrontAngle) < PivotConstants.tolerance;
		boolean wristDone = Math.abs(wrist.getAngle()-WristConstants.lowFrontAngle) < WristConstants.tolerance;

		if(wristDone){
			wristCounter++;
		}else{
			wristCounter=0;
		}
		if(pivotDone){
			pivotCounter++;
		}else{
			pivotCounter=0;
		}

		if(pivotCounter>Constants.pidTimer&&wristCounter>Constants.pidTimer){
			return true;
		}else{
			return false;
		}
		// return (arm.getPosition()==ArmConstants.Extension.lowConeDistance) && (pivot.getAngle() == PivotConstants.lowConeAngle);
	}
}
