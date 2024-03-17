// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoShoot_B extends Command {
  /** Creates a new Preload2. */
  public AutoShoot_B() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.feedsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.feedsubsystem.Feed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.feedsubsystem.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!Constants.k_NoteisReady) { //no note in shooter start to home wrist and shoulder
      return true;
    } else {
      return false;
    }
  }
}
