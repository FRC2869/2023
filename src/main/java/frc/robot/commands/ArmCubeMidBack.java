package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmCubeMidBack extends CommandBase{
	// private ArmSubsystem arm;
	private PivotSubsystem pivot;
	// private int armCounter;
	private int pivotCounter;
	private WristSubsystem wrist;
	private int wristCounter;

	public ArmCubeMidBack(){
		// arm = ArmSubsystem.getInstance();
		pivot = PivotSubsystem.getInstance();
		wrist = WristSubsystem.getInstance();
		System.out.println("Cube Mid");
		// addRequirements(arm);
		addRequirements(pivot);
		addRequirements(wrist);
	}

	@Override
	public void execute(){
		// arm.setPositionControl(true);
		// arm.position(ArmConstants.Extension.midCubeDistance);
		pivot.setPositionControl(true);
		pivot.position(PivotConstants.midCubeBackAngle);
		wrist.setPositionControl(true);
		wrist.position(WristConstants.midCubeBackAngle);
	}

	@Override
	public boolean isFinished(){
		
		// boolean armDone = Math.abs(arm.getPosition()-ArmConstants.Extension.midCubeDistance) < ArmConstants.Extension.tolerance;
		boolean pivotDone = Math.abs(pivot.getAngle()-PivotConstants.midCubeBackAngle) < PivotConstants.tolerance;
		boolean wristDone = Math.abs(wrist.getAngle()-WristConstants.midCubeBackAngle) < WristConstants.tolerance;

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
			pivot.setPositionControl(false);
			return true;
		}else{
			return false;
		}
		// return (arm.getPosition()==ArmConstants.Extension.lowConeDistance) && (pivot.getAngle() == PivotConstants.lowConeAngle);
	}
}