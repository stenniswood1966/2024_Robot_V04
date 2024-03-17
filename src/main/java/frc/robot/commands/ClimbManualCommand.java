// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ClimbManualCommand extends Command {
  /** Creates a new ClimbCommand. */
  public ClimbManualCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climbsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double speed = RobotContainer.joystick2.getRawAxis(4) * 0.3;
    //RobotContainer.climbsubsystem.set(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climbsubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
