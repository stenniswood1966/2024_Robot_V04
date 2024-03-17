// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakeLoadCommand extends Command {
  /** Creates a new A_IntakeCommand. */
  public IntakeLoadCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intakesubsystem, RobotContainer.feedsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Commands.race(new IntakeCommand(), new LoadCommand()).withTimeout(3);
    RobotContainer.intakesubsystem.Intake();
    RobotContainer.feedsubsystem.Intake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intakesubsystem.Stop();
    RobotContainer.feedsubsystem.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    if (Constants.k_NoteisReady) {
      return true;
    } else {
      return false;
    }
  }
}
