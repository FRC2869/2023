package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ArmCubeMid extends CommandBase{
	private ArmSubsystem arm;
	private PivotSubsystem pivot;
	private int armCounter;
	private int pivotCounter;

	public ArmCubeMid(){
		arm = ArmSubsystem.getInstance();
		pivot = PivotSubsystem.getInstance();
		
		addRequirements(arm);
		addRequirements(pivot);
	}

	@Override
	public void execute(){
		arm.setPositionControl(true);
		arm.position(ArmConstants.Extension.midCubeDistance);
		pivot.setPositionControl(true);
		pivot.position(PivotConstants.midCubeAngle);
	}

	@Override
	public boolean isFinished(){
		
		boolean armDone = Math.abs(arm.getPosition()-ArmConstants.Extension.midCubeDistance) < ArmConstants.Extension.tolerance;
		boolean pivotDone = Math.abs(pivot.getAngle()-PivotConstants.midCubeAngle) < PivotConstants.tolerance;

		if(armDone){
			armCounter++;
		}else{
			armCounter=0;
		}
		if(pivotDone){
			pivotCounter++;
		}else{
			pivotCounter=0;
		}

		if(armCounter>Constants.pidTimer&&pivotCounter>Constants.pidTimer){
			return true;
		}else{
			return false;
		}
		// return (arm.getPosition()==ArmConstants.Extension.lowConeDistance) && (pivot.getAngle() == PivotConstants.lowConeAngle);
	}
}
