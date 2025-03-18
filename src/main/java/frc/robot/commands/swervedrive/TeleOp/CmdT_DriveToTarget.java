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
import frc.robot.Constants.RobotAutomationInformation.AutoAlignCoralFeederStation;
import frc.robot.Constants.RobotAutomationInformation.AutoAlignReef;
import frc.robot.subsystems.DataHighway.DataHighwaySubsystem;
import frc.robot.subsystems.DataHighway.DataHighwaySubsystem.Target;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_DriveToTarget extends Command {

  Integer branch;
 
  SwerveSubsystem DriveSS;
  DataHighwaySubsystem DataSS;
  Pose2d targetPose;
  double maxZoom;
  PIDController XPID = new PIDController(2.0, 0, 0.02);
  PIDController YPID = new PIDController(2.0, 0, 0.02);
  PIDController RotPID = new PIDController(0.15,0,0);
  double minSpeed = 0.3;
  boolean inter;
  Target whereAmIGoing;
  int option;
  public CmdT_DriveToTarget(SwerveSubsystem DriveSubsystem, Target t, double maxSpeed, int o) {
    // Use addRequirements() here to declare subsystem dependencies.
    whereAmIGoing = t;
    DriveSS = DriveSubsystem;
    addRequirements(DriveSS);
    maxZoom = maxSpeed;
    option = o;
  }

  // Called when the command is initially scheduled.

  //  public enum Target{
  //   ClosestPickupSlot,
  //   ClosestReefSegment,
  //   CLIMBSTART,
  // } 

  @Override
  public void initialize() {

    DriveSS.DisableAutoAim();

    if (whereAmIGoing == Target.ClosestPickupSlot){
      
      Pose2d tmp = DriveSS.GetClosestPickupSlot();
      Transform2d translation = new Transform2d(AutoAlignCoralFeederStation.distanceFromFace, 0, new Rotation2d(180));
      targetPose = tmp.plus(translation);

    }
    else if (whereAmIGoing == Target.ClosestReefSegment){

      Pose2d tmp = DriveSS.GetClosestReefSegment();

      if (option == 1){

        Transform2d test = new Transform2d(AutoAlignReef.distanceFromFace, -AutoAlignReef.branchOffset
        ,new Rotation2d(0));
  
        targetPose = tmp.plus(test);
  
      }
      if (option == 2){
  
        Transform2d test = new Transform2d(AutoAlignReef.distanceFromFace, AutoAlignReef.branchOffset
        ,new Rotation2d(0));
  
        targetPose = tmp.plus(test);
      }
      

    }
    else if (whereAmIGoing == Target.CLIMBSTART){

      if (DriveSS.DH_OUT_isRedAlliance){
        targetPose = new Pose2d(10.438,2.952, new Rotation2d((Units.degreesToRadians(180))));
      }
      else{
        targetPose = new Pose2d(7.11,5.1, new Rotation2d((Units.degreesToRadians(0))));
      }

    }
    
    DogLog.log ("DriveToTarget/Target", whereAmIGoing);
    DogLog.log ("DriveToTarget/TargetPose", targetPose);
    DogLog.log("DriveToTarget/CurrentPose", DriveSS.getPose());

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

    DogLog.log("DriveToTarget/XPIDval",XVel);
    DogLog.log("DriveToTarget/YPIDval",YVel);
    DogLog.log("DriveToTarget/RotPIDval",RotVel);
    DogLog.log("DriveToTarget/CurrentRobotPose",DriveSS.getPose());
    DogLog.log("DriveToTarget/CurrentRobotPoseRotation",DriveSS.getPose().getRotation().getDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSS.drive(new ChassisSpeeds(0,0,0));

    DogLog.log("DriveToTarget/FinalPose",DriveSS.getPose());
    DogLog.log("DriveToTarget/FinalPoseRotation",DriveSS.getPose().getRotation().getDegrees());
    inter = interrupted;
    DogLog.log("DriveToTarget/IsStopped", DriveSS.isRobotStopped());
    DogLog.log("DriveToTarget/Interrupted", inter);
    DogLog.log("DriveToTarget/RobotXvel",DriveSS.getFieldVelocity().vxMetersPerSecond);
    DogLog.log("DriveToTarget/RobotYvel",DriveSS.getFieldVelocity().vyMetersPerSecond);
    DogLog.log("DriveToTarget/RobotRotvel",DriveSS.getFieldVelocity().omegaRadiansPerSecond);
    
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
