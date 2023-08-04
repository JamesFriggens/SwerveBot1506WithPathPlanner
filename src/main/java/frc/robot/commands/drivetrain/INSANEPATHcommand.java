// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.SwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class INSANEPATHcommand extends SequentialCommandGroup {
  
  /** Creates a new INSANEPATHcommand. */
  
  public INSANEPATHcommand(SwerveDrivetrain m_drive, String pathName) {

    PathPlannerTrajectory traj = PathPlanner.loadPath(pathName, new PathConstraints(Auton.MAX_SPEED_MPS, Auton.MAX_ACCELERATION_MPSS));

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      m_drive.getAutoBuilder(new HashMap<>()).fullAuto(traj)
    );
  }
}
