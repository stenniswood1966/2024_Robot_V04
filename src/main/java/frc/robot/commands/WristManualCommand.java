// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class WristManualCommand extends Command {
  /** Creates a new WristManualCommand. */
  public WristManualCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.wristsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = RobotContainer.joystick2.getRawAxis(5) * 0.3; //slow speed fown by 70%
    RobotContainer.wristsubsystem.set(speed);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.wristsubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
}
