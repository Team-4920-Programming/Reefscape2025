// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DataHighway;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.BoltLog;
import frc.robot.Robot;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;
import frc.robot.subsystems.DataHighway.DataHighwaySubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DataHighway_Def_Cmd extends Command {
  /** Creates a new DataHighway_Def_Cmd. */
  private final BoltLog BoltLogger = new BoltLog();
  private AlgaeIntakeSubsystem IntakeSS;
  private SwerveSubsystem SwerveSS;
  private CoralElevatorSubsystem ShooterSS;

  private boolean IntakeHasGP = false;

  private boolean ShooterComplete = false;

  private Pose2d robotSimulationWorldPose;
  private ChassisSpeeds fieldChassisSpeeds;

  public DataHighway_Def_Cmd(DataHighwaySubsystem data_Subsystem,AlgaeIntakeSubsystem Intake_Subsystem, SwerveSubsystem Swerve_SubSystem, CoralElevatorSubsystem Shooter_Subsystem) {
    IntakeSS = Intake_Subsystem;
     SwerveSS = Swerve_SubSystem;
    ShooterSS = Shooter_Subsystem;
    addRequirements(data_Subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    BoltLogger.Log(BoltLogger.HighLog, getSubsystem(), getName(), "Execute", "Executing", true);

    getIntakeSS();

    getSwerveSS();
    getShooterSS();

    setIntakeSS();

    setSwerveSS();
    setShooterSS();
    //log status at end of execute

   
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    BoltLogger.Log(BoltLogger.HighLog, getSubsystem(), getName(), "Execute", "Executing", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  private void getIntakeSS(){
    //get all intake data from intake subsystem
    
   
  }
  private void setIntakeSS(){
    //set all intake sub system data
    
    
   
  }
  private void getSwerveSS(){
    fieldChassisSpeeds = SwerveSS.getFieldVelocity();
    robotSimulationWorldPose = SwerveSS.getPose();
    
    if (Robot.isSimulation())
      if (SwerveSS.getAIntakeGamePiecesAmount() > 0)
      {
        IntakeHasGP = true;
        
      }
      else
      {
        IntakeHasGP = false;
        
      }
  }

  private void setSwerveSS(){

 
  }
  private void getShooterSS(){
      //ShooterComplete = ShooterSS.ShotComplete;
  }
  private void setShooterSS(){
    //ShooterSS.chassisSpeedsFieldRelative = fieldChassisSpeeds;
    //ShooterSS.robotSimulationWorldPose = robotSimulationWorldPose;
    //ShooterSS.RobotHasGP = IntakeHasGP;
  }

}
