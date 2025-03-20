// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.TeleOp;

import dev.doglog.DogLog;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DataHighway.DataHighwaySubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_DriveToPose extends Command {

  Integer branch;
 
  SwerveSubsystem DriveSS;
  DataHighwaySubsystem DataSS;
  Pose2d target;
  double maxZoom;
  PIDController XPID = new PIDController(2, 0, 0.01);
  PIDController YPID = new PIDController(2, 0, 0.01);
  PIDController RotPID = new PIDController(0.125,0,0);
  boolean inter;
  public CmdT_DriveToPose(SwerveSubsystem DriveSubsystem, Pose2d targetPose, double maxSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    target = targetPose;
    DriveSS = DriveSubsystem;
    addRequirements(DriveSS);
    maxZoom = maxSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriveSS.DisableAutoAim();

 
   
    DogLog.log ("ReefScore/ReefPose", target);


    
    
    XPID.setTolerance(0.05);
    YPID.setTolerance(0.05);
    RotPID.setTolerance(2);
    RotPID.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double CurrentX = DriveSS.getPose().getX();
    double CurrentY = DriveSS.getPose().getY();
    double CurrentRot = DriveSS.getPose().getRotation().getDegrees();
  //   double Reef_X = ReefPose.getX();
  //   double Reef_Y = ReefPose.getY();
  //   double Reef_Rot = ReefPose.getRotation().getDegrees()+180;
    double XVel = 0;  
    double YVel =0;
    // if (RotPID.atSetpoint()){
      
      XVel = XPID.calculate(CurrentX, target.getX());
      YVel = YPID.calculate(CurrentY, target.getY());
    // }
    double RotVel = RotPID.calculate(CurrentRot,target.getRotation().getDegrees());
    XVel = MathUtil.clamp(XVel, -maxZoom, maxZoom);
    YVel = MathUtil.clamp(YVel, -maxZoom,maxZoom);
    RotVel = MathUtil.clamp(RotVel, -3, 3);

    DriveSS.drive(new Translation2d(XVel,YVel),RotVel,true);

    DogLog.log("ReefScore/Xvel",XVel);
    DogLog.log("ReefScore/Yvel",YVel);
    DogLog.log("ReefScore/Rotvel",RotVel);
    DogLog.log("Reef XPID current", CurrentX);
    DogLog.log("Reef YPID current", CurrentY);
    
   
    //System.out.println("We be climbin'");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSS.drive(new ChassisSpeeds(0,0,0));
    //System.out.println("DrivetoReef Finished");
    //System.out.println("interrupted"+interrupted);
    inter = interrupted;
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return XPID.atSetpoint() & YPID.atSetpoint() & RotPID.atSetpoint();
  }

  public boolean wasInterrupted(){
    return inter;
  }
}
