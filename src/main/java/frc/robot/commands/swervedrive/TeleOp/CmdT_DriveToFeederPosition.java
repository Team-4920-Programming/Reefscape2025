// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.TeleOp;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_DriveToFeederPosition extends Command {
  /** Creates a new CmdT_DriveToFeederPosition. */
  Integer Feeder_Side;
  Integer Feeder_Slot;
  SwerveSubsystem DriveSS;
  Pose2d FeederPose;
  public CmdT_DriveToFeederPosition(SwerveSubsystem DriveSubsystem, Integer FeederSide, Integer FeederSlot) {
    // Use addRequirements() here to declare subsystem dependencies.
    Feeder_Side = FeederSide;
    Feeder_Slot = FeederSlot;
    DriveSS = DriveSubsystem;
    addRequirements(DriveSS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double Feeder_X =0.85;
    double Feeder_Y = 0.65;
    double Feeder_Rot = 54;
    FeederPose = new Pose2d(Feeder_X, Feeder_Y,Rotation2d.fromDegrees(Feeder_Rot));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveSS.driveToPose(FeederPose);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSS.drive(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
