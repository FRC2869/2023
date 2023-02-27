package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class GrabberSubsystem extends SubsystemBase{
	private static GrabberSubsystem instance;
	private Compressor compressor; 
	private DoubleSolenoid solenoid1;

	public static GrabberSubsystem getInstance(){
		if(instance == null){
			instance = new GrabberSubsystem();
		}
		return instance;
	}

	public GrabberSubsystem(){
		compressor = new Compressor(16, PneumaticsModuleType.CTREPCM);
		solenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.solenoidPortForwards, PneumaticsConstants.solenoidPortBackwards);
	}

	public void compressorOn(){
		compressor.enableDigital();
	}

	public void compressorOff(){
		compressor.disable();
	}

	public void closeGrabber(){
		solenoid1.set(Value.kForward);
	}

	public void openGrabber(){
		solenoid1.set(Value.kReverse);
	}

	public void offGrabber(){
		solenoid1.set(Value.kOff);
	}

	/**
	 * 
	 * @return if the compressor is on
	 */
	public boolean getCompressorOn(){
		return compressor.isEnabled();
	}


}
