// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoShoot_A extends Command {
  /** Creates a new A1. */
  public AutoShoot_A() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shootsubsystem, RobotContainer.shouldersubsystem, RobotContainer.wristsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.shootsubsystem.Shoot(Constants.k_FiringSolutionSpeed);
    RobotContainer.shouldersubsystem.enablemotionmagic(Constants.k_ShoulderShootPosition);
    RobotContainer.wristsubsystem.enablemotionmagic(Constants.k_FiringSolutionAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!Constants.k_ShoulderMMisMoving && !Constants.k_WristMMisMoving && Constants.k_shootmotor1speed >= Constants.k_FiringSolutionSpeed) {
      return true;
    } else {
      return false;
    }
  }
}
