package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class movePath extends CommandBase {
	PathPlannerTrajectory examplePath;
	PathPlannerState exampleState;

	public movePath () {
		// This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
		examplePath = PathPlanner.loadPath("forward1m", new PathConstraints(1, 3));

		// This trajectory can then be passed to a path follower such as a PPSwerveControllerCommand
		// Or the path can be sampled at a given point in time for custom path following

		// Sample the state of the path at 1.2 seconds
	}

	@Override
	public void initialize() {
		exampleState = (PathPlannerState) examplePath.sample(1.2);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
