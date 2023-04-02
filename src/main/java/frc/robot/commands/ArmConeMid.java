package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

public class ArmConeMid extends CommandBase{
	// private ArmSubsystem arm;
	private PivotSubsystem pivot;
	// private int armCounter;
	private int pivotCounter;
	private boolean hasRun = false;

	public ArmConeMid(){
		// arm = ArmSubsystem.getInstance();
		pivot = PivotSubsystem.getInstance();
		// System.out.println("Cone Mid");
		// addRequirements(arm);
		addRequirements(pivot);
	}

	@Override
	public void execute(){
		if(!hasRun){
			System.out.println(Constants.autoTimer.get()+": Arm Cone Mid Start");
			hasRun = true;
		}
		// arm.setPositionControl(true);
		// arm.position(ArmConstants.Extension.midConeDistance);
		pivot.setPositionControl(true);
		pivot.position(PivotConstants.midConeAngle);
	}

	@Override
	public boolean isFinished(){
		// boolean armDone = Math.abs(arm.getPosition()-ArmConstants.Extension.midConeDistance) < ArmConstants.Extension.tolerance;
		boolean pivotDone = Math.abs(pivot.getAngle()-PivotConstants.midConeAngle) < PivotConstants.tolerance;
		
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
				// System.out.println(pivotCounter);
		if(pivotCounter>Constants.pidTimer){
			// System.out.println("DONE");
			System.out.println(Constants.autoTimer.get()+": Arm Cone Mid Done");
			pivot.setPositionControl(false);
			return true;
		}else{
			return false;
		}
		// return (arm.getPosition()==ArmConstants.Extension.lowConeDistance) && (pivot.getAngle() == PivotConstants.lowConeAngle);
	}

	@Override
	public void end(boolean isInterrupted){
		pivot.setPositionControl(false);
	}
}
