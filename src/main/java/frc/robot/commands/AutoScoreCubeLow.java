package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.grabber.OffGrabber;
import frc.robot.commands.grabber.OpenGrabber;

public class AutoScoreCubeLow extends SequentialCommandGroup {
    public AutoScoreCubeLow(){
        super(new ArmLowFront(), new ParallelRaceGroup(new OpenGrabber(), new WaitCommand(1)), new ParallelRaceGroup(new OffGrabber(), new WaitCommand(.25)), new ArmBasePos());
    }
}
