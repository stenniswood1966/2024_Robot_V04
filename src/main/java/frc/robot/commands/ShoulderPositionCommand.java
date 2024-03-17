// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShoulderPositionCommand extends Command {
  private double targetpos;

  /** Creates a new ShoulderStowCommand. */
  public ShoulderPositionCommand(double targetpos) {
    this.targetpos = targetpos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shouldersubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shouldersubsystem.enablemotionmagic(targetpos);
    //System.out.println(targetpos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    if (Constants.k_ShoulderMMisMoving) {
      return false;
    } else {
      return true;
    }
  }
}
