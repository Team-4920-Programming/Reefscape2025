// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.TeleOp;


import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_EnableAutoAim extends Command {
  /** Creates a new CmdT_EnableAutoAim. */
  SwerveSubsystem Drive_SS;
  public CmdT_EnableAutoAim(SwerveSubsystem DriveSS) {
    // Use addRequirements() here to declare subsystem dependencies.
    Drive_SS = DriveSS;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putNumber("ElevatorTestingHeight", Level4.height);
    // SmartDashboard.putNumber("ElbowTestingAngle", Level4.elbow);
    // SmartDashboard.putNumber("WristTestingAngle", Level4.wrist);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Drive_SS.EnableAutoAim();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Drive_SS.isAutoAim();
  }
}
