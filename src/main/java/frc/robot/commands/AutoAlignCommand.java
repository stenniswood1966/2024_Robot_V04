// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class AutoAlignCommand extends Command {
  double tx = 0;
  CommandSwerveDrivetrain _drivetrain;
  /** Creates a new AutoAlignCommand. */
  public AutoAlignCommand(CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    _drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
      tx = LimelightHelpers.getTX("limelight");
      SwerveDriveState pose = _drivetrain.getState();
      var alliance = DriverStation.getAlliance();

    if (alliance.get() == DriverStation.Alliance.Blue) {

      if (tx <= 25) { //blue alliance
        Constants.k_steering_target = new Rotation2d(Math.toRadians(pose.Pose.getRotation().getDegrees() - tx));
        //SmartDashboard.putNumber("Steering target angle: ", pose.Pose.getRotation().getDegrees() - tx);
      }
      else {
        Constants.k_steering_target = new Rotation2d(Math.toRadians(pose.Pose.getRotation().getDegrees()));
        //SmartDashboard.putNumber("Steering target angle: ", pose.Pose.getRotation().getDegrees());
      }
    } else { //Red alliance
      if (tx <= 25) {
        Constants.k_steering_target = new Rotation2d(Math.toRadians(pose.Pose.getRotation().getDegrees() - tx + 180));
        //SmartDashboard.putNumber("Steering target angle: ", pose.Pose.getRotation().getDegrees() - tx + 180);
      }
      else {
        Constants.k_steering_target = new Rotation2d(Math.toRadians(pose.Pose.getRotation().getDegrees()));
        //SmartDashboard.putNumber("Steering target angle: ", pose.Pose.getRotation().getDegrees());
      }
    }

    //SmartDashboard.putNumber("Steering target angle: ", pose.Pose.getRotation().getDegrees());  
    System.out.println("limelight: " + tx);
    System.out.println("Rotation2d: " +_drivetrain.getState().Pose.getRotation().getDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    if (tx < 1) {
      return true;
    } else {
      return false;
    }
  }
}