// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.TeleOp;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DataHighway.DataHighwaySubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_DriveToReefPositionV2 extends Command {

  Integer branch;
 
  SwerveSubsystem DriveSS;
  Pose2d ReefPose;
  Pose2d targetPose;
  PIDController XPID = new PIDController(2.0, 0, 0.02);
  PIDController YPID = new PIDController(2.0, 0, 0.02);
  PIDController RotPID = new PIDController(0.15,0,0);
  double minSpeed = 0.3;
  boolean inter;
  public CmdT_DriveToReefPositionV2(SwerveSubsystem DriveSubsystem, int Position) {
    // Use addRequirements() here to declare subsystem dependencies.
    branch = Position;
    DriveSS = DriveSubsystem;
    addRequirements(DriveSS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    double branchoffset = Units.inchesToMeters(6.5);//7.825
    double distanceFromFace = Units.inchesToMeters(18.375);
    inter = false;
    Pose2d targetAprilTagPose = DriveSS.GetClosestReefSegment();

    targetPose = new Pose2d();
    targetPose = targetAprilTagPose;
    if (branch == 1){

      Transform2d test = new Transform2d(distanceFromFace, -branchoffset
      ,new Rotation2d(0));

      targetPose = targetAprilTagPose.plus(test);

      //System.out.println("target Pose = " + targetPose.toString());
      //System.out.println("target April Tag Pose = " + targetAprilTagPose.toString());
      //System.out.println("transform = " + test.toString());

    }
    if (branch == 2){

      Transform2d test = new Transform2d(distanceFromFace, branchoffset + Units.inchesToMeters(1)
      ,new Rotation2d(0));

      targetPose = targetAprilTagPose.plus(test);

      //System.out.println("target Pose = " + targetPose.toString());
      //System.out.println("target April Tag Pose = " + targetAprilTagPose.toString());
      //System.out.println("transform = " + test.toString());
    }

    DogLog.log("ReefScore/ReefPose", targetPose);


    XPID.setTolerance(0.02);
    YPID.setTolerance(0.02);
    RotPID.setTolerance(0.75);
    RotPID.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double CurrentX = DriveSS.getPose().getX();
    double CurrentY = DriveSS.getPose().getY();
    double CurrentRot = DriveSS.getPose().getRotation().getDegrees();

    double XVel = 0;  
    double YVel =0;
    double rawXVel = 0;
    double rawYVel = 0;
    double rawrotVel = 0;
      
    XVel = XPID.calculate(CurrentX, targetPose.getX());
    rawXVel = XVel;
    YVel = YPID.calculate(CurrentY, targetPose.getY());
    rawYVel = YVel;

    double RotVel = RotPID.calculate(CurrentRot,targetPose.getRotation().getDegrees() + 180);
    rawrotVel = RotVel;
    XVel = MathUtil.clamp(XVel, -3, 3);
    YVel = MathUtil.clamp(YVel, -3,3);
    RotVel = MathUtil.clamp(RotVel, -3, 3);

  if (!XPID.atSetpoint()){
    if (XVel > 0 ){
      XVel = Math.max(XVel, minSpeed);
    }
    else if (XVel < 0 ){
      XVel = Math.min(XVel, -minSpeed);
    }
  }
  if (!YPID.atSetpoint()){
    if (  YVel > 0 ){
      YVel = Math.max(YVel, minSpeed);
    }
    else if (YVel < 0 ){
      YVel = Math.min(YVel, -minSpeed);
    }
  }

  if (!RotPID.atSetpoint()){
    if (RotVel > 0 ){
      RotVel = Math.max(RotVel, 0.15);
    }
    else if (RotVel < 0 ){
      RotVel = Math.min(RotVel, -0.15);
    }
  }

  if (XPID.atSetpoint()){
    XVel = rawXVel;
  }
  if (YPID.atSetpoint()){
    YVel = rawYVel;
  }
  if (RotPID.atSetpoint()){
    RotVel = rawrotVel;
  }

  DriveSS.drive(new Translation2d(XVel,YVel),RotVel,true);

  DogLog.log("ReefScore/XPIDval",XVel);
  DogLog.log("ReefScore/YPIDval",YVel);
  DogLog.log("ReefScore/RotPIDval",RotVel);
  DogLog.log("Reef XPID current", CurrentX);
  DogLog.log("Reef YPID current", CurrentY);
  DogLog.log("ReefScore/CurrentRobotPose",DriveSS.getPose());

   
    //System.out.println("Driving toReef");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSS.drive(new ChassisSpeeds(0,0,0));
    //System.out.println("DrivetoReef Finished");
    DogLog.log("ReefScore/FinalPose",DriveSS.getPose());
    DogLog.log("ReefScore/FinalPoseRotation",DriveSS.getPose().getRotation().getDegrees());
    //System.out.println("interrupted"+interrupted);
    inter = interrupted;
    DogLog.log("ReefScore/IsStopped", DriveSS.isRobotStopped());
    DogLog.log("ReefScore/Interrupted", inter);
    DogLog.log("ReefScore/RobotXvel",DriveSS.getFieldVelocity().vxMetersPerSecond);
    DogLog.log("ReefScore/RobotYvel",DriveSS.getFieldVelocity().vyMetersPerSecond);
    DogLog.log("ReefScore/RobotRotvel",DriveSS.getFieldVelocity().omegaRadiansPerSecond);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return XPID.atSetpoint() && YPID.atSetpoint() && RotPID.atSetpoint() && DriveSS.isRobotStopped();
  }

  public boolean wasInterrupted(){
    return inter;
  }
}
