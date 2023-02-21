package org.team2869.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
	private static PivotSubsystem instance;
	public static PivotSubsystem getInstance(){
		if(instance==null){
			instance = new PivotSubsystem();
		}
		return instance;
	}

	public PivotSubsystem(){
		
	}
}
