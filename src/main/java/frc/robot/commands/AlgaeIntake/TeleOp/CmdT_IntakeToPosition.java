// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeIntake.TeleOp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BoltLog;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_IntakeToPosition extends Command {
  /** Creates a new Cmd_IntakeOn. */
  private AlgaeIntakeSubsystem IntakeSS;
  private double rAngle;
    private final BoltLog BoltLogger = new BoltLog();
  public CmdT_IntakeToPosition(AlgaeIntakeSubsystem Intake_Subsystem, double reqAngle) {
    IntakeSS = Intake_Subsystem;
    rAngle = reqAngle;
    addRequirements(IntakeSS);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    BoltLogger.Log(BoltLogger.HighLog, getSubsystem(), getName(), "execute", "Executing", true);
    if (!IntakeSS.HasAlgae() )
      IntakeSS.SetIntakeAngle(rAngle);;
    if (IntakeSS.HasAlgae() && rAngle >24)
      IntakeSS.SetIntakeAngle(rAngle);;
      if (IntakeSS.HasAlgae() && rAngle <24)
      IntakeSS.SetIntakeAngle(25);;
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    BoltLogger.Log(BoltLogger.HighLog, getSubsystem(), getName(), "Execute", "Executing", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // boolean atAngle = false;
    // if (Math.abs(IntakeSS.GetIntakeAngle() - rAngle) < 2)
    // {
    //   atAngle = true;
    // }
    return true;// atAngle;
  }
}
