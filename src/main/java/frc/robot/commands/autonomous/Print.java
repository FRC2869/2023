package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Print extends CommandBase {
    public Print(){

    }

    @Override
    public void execute(){
        System.out.println("PRINTING");
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
