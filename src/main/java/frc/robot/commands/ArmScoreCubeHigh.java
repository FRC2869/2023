package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomous.AutoForwardsDist;
import frc.robot.commands.grabber.OffGrabber;
import frc.robot.commands.grabber.OpenGrabber;

public class ArmScoreCubeHigh extends SequentialCommandGroup {
    public ArmScoreCubeHigh(){
        super(new ArmCubeHighBack(), new ParallelRaceGroup(new OpenGrabber(), new WaitCommand(1)), new ParallelRaceGroup(new OffGrabber(), new WaitCommand(.25)), new ArmBasePos(), new AutoForwardsDist(1));
    }
}
