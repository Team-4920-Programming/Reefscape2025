// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.dyn4j.geometry.Rotation;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.PIDs;
import frc.robot.Constants.PIDs.CoralElevator.DriveToPoseAuto;
import frc.robot.Constants.RobotAutomationInformation.AutoAlignReef;
import frc.robot.subsystems.DataHighway.DataHighwaySubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdA_DriveToReefPositionV9_PreProvincials extends Command {

  private final SwerveSubsystem DriveSS;
  private Pose2d target;

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
        DriveToPoseAuto.thetakP, 0.0, DriveToPoseAuto.thetakD, new TrapezoidProfile.Constraints(DriveToPoseAuto.thetaMaxVelocity, DriveToPoseAuto.thetaMaxAcceleration), Constants.LOOP_TIME);
  private final ProfiledPIDController xdriveController =
      new ProfiledPIDController(
        6.0, 0.0, 0.0, new TrapezoidProfile.Constraints(DriveToPoseAuto.driveMaxVelocity, DriveToPoseAuto.driveMaxAcceleration), Constants.LOOP_TIME);

  private final ProfiledPIDController ydriveController =
      new ProfiledPIDController(
        5.0, 0.0, 0.0, new TrapezoidProfile.Constraints(DriveToPoseAuto.driveMaxVelocity, DriveToPoseAuto.driveMaxAcceleration), Constants.LOOP_TIME);

  
  private Translation2d lastSetpointTranslation = Translation2d.kZero;
  private Rotation2d lastSetpointRotation = Rotation2d.kZero;
  private double lastTime = 0.0;
  private double thetaFromTarget = 0.0;
  private int pos;
  
  public CmdA_DriveToReefPositionV9_PreProvincials(SwerveSubsystem DriveSubsystem, int position) {
    DriveSS = DriveSubsystem;
    pos = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    

    target = GetTargetPose(DriveSS.GetClosestReefSegment());
    Pose2d currentPose = DriveSS.getPose();

    ChassisSpeeds robotVelocity = DriveSS.getRobotVelocity();

    Pose2d robotRelativeError = currentPose.relativeTo(target);
        xdriveController.reset(robotRelativeError.getX(), -robotVelocity.vxMetersPerSecond);
        ydriveController.reset(robotRelativeError.getY(), -robotVelocity.vyMetersPerSecond);
        thetaController.reset(currentPose.getRotation().getRadians(), robotVelocity.omegaRadiansPerSecond);


    lastSetpointTranslation = robotRelativeError.getTranslation();
    lastSetpointRotation = target.getRotation();
    lastTime = Timer.getTimestamp();
    thetaController.setTolerance(DriveToPoseAuto.thetaTolerance);
    xdriveController.setTolerance(DriveToPoseAuto.driveTolerance);
    ydriveController.setTolerance(DriveToPoseAuto.driveTolerance);

    DogLog.log("Auto/DriveToReefV9/Init/currentPose", currentPose);
    DogLog.log("Auto/DriveToReefV9/Init/targetPose", target);
    DogLog.log("Auto/DriveToReefV9/Init/drivePIDTolerance", xdriveController.getPositionTolerance());
    DogLog.log("Auto/DriveToReefV9/Init/thetaPIDTolerance", thetaController.getPositionTolerance());
    DogLog.log("Auto/DriveToReefV9/Init/xPIDError", xdriveController.getPositionError());
    DogLog.log("Auto/DriveToReefV9/Init/yPIDError", ydriveController.getPositionError());
    DogLog.log("Auto/DriveToReefV9/Init/thetaPIDError", thetaController.getPositionError());
    DogLog.log("Auto/DriveToReefV9/Status", "Initialized");

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    Pose2d currentPose = DriveSS.getPose();
    
    
    target = GetTargetPose(DriveSS.GetClosestReefSegment());
    Pose2d robotRelativeError = currentPose.relativeTo(target);

    
    double ffScalerX = MathUtil.clamp(
        (robotRelativeError.getX() - DriveToPoseAuto.ffMinRadius) / (DriveToPoseAuto.ffMaxRadius - DriveToPoseAuto.ffMinRadius),
        0.0,
        1.0);

    double ffScalerY = MathUtil.clamp(
      (robotRelativeError.getY() - DriveToPoseAuto.ffMinRadius) / (DriveToPoseAuto.ffMaxRadius - DriveToPoseAuto.ffMinRadius),
      0.0,
      1.0);

    DogLog.log("Auto/DriveToReefV9/Exec/currentPose", currentPose);
    DogLog.log("Auto/DriveToReefV9/Exec/targetPose", target);
    DogLog.log("Auto/DriveToReefV9/Exec/ffScalerX", ffScalerX);
    DogLog.log("Auto/DriveToReefV9/Exec/ffScalerY", ffScalerY);
        

    xdriveController.reset(lastSetpointTranslation.getX(), xdriveController.getSetpoint().velocity);

    ydriveController.reset(lastSetpointTranslation.getY(), ydriveController.getSetpoint().velocity);
    
    
    double driveVelocityScalarX = xdriveController.calculate(robotRelativeError.getX(), 0.0) + xdriveController.getSetpoint().velocity * ffScalerX;
    double driveVelocityScalarY = ydriveController.calculate(robotRelativeError.getY(), 0.0) + ydriveController.getSetpoint().velocity * ffScalerY;

    if (Math.abs(robotRelativeError.getX()) <= xdriveController.getPositionTolerance()) driveVelocityScalarX = 0.0;
    if (Math.abs(robotRelativeError.getY()) <= ydriveController.getPositionTolerance()) driveVelocityScalarY = 0.0;

    lastSetpointTranslation = robotRelativeError.getTranslation();

    DogLog.log("Auto/DriveToReefV9/Exec/driveVelocityScalarX", driveVelocityScalarX);
    DogLog.log("Auto/DriveToReefV9/Exec/driveVelocityScalarY", driveVelocityScalarY);
    DogLog.log("Auto/DriveToReefV9/Exec/lastSetpointTranslation", lastSetpointTranslation);

    // Calculate theta speed
    double thetaVelocity = thetaController.calculate( currentPose.getRotation().getRadians(),
                    new TrapezoidProfile.State( target.getRotation().getRadians(),
                    (target.getRotation().minus(lastSetpointRotation)).getRadians() / (Timer.getTimestamp() - lastTime)))
            + thetaController.getSetpoint().velocity * ffScalerX;
            thetaFromTarget =
        Math.abs(currentPose.getRotation().minus(target.getRotation()).getRadians());
    if (thetaFromTarget <= thetaController.getPositionTolerance()) thetaVelocity = 0.0;
    lastSetpointRotation = target.getRotation();



  DogLog.log("Auto/DriveToReefV9/Check/X", robotRelativeError.getX());
  DogLog.log("Auto/DriveToReefV9/Check/Y", robotRelativeError.getY());
  DogLog.log("Auto/DriveToReefV9/Check/R", robotRelativeError.getRotation().getDegrees());
  DogLog.log("Auto/DriveToReefV9/Exec/xPIDError", xdriveController.getPositionError());
  DogLog.log("Auto/DriveToReefV9/Exec/yPIDError", ydriveController.getPositionError());
  double driveXVel = driveVelocityScalarX;
  double driveYVel = driveVelocityScalarY;

  if ((Math.abs(ydriveController.getPositionError()) >=   DriveToPoseAuto.driveTolerance * 2 || Math.abs(thetaController.getPositionError()) >= thetaController.getPositionTolerance()*3) && Math.abs(xdriveController.getPositionError()) <= 0.9 ){
    DriveSS.DH_Out_DriveToPose = true;
    // driveXVel = 0.0;
    }


  
  // if (!DriveSS.DH_In_MechAtGoal && Math.abs(xdriveController.getPositionError()) <= 1.5  ){
  //   driveXVel *= 0.5;
  // }
  // else if (!DriveSS.DH_In_MechAtGoal){
  //   driveXVel  *=0.75;
  // }
  
    if (thetaVelocity < 0.0){
      DogLog.log("Auto/DriveToReefV9/Check/A", -1);
      thetaVelocity = Math.min(thetaVelocity, -0.25);
    }
    if (thetaVelocity > 0.0){
      DogLog.log("Auto/DriveToReefV9/Check/A", 1);
      thetaVelocity = Math.max(thetaVelocity, 0.25);
    }
    if (driveXVel < 0.0){
      DogLog.log("Auto/DriveToReefV9/Check/B", -1);
      driveXVel = Math.min(driveXVel, -0.25);
    }
    if (driveXVel > 0.0){
      DogLog.log("Auto/DriveToReefV9/Check/B", 1);
      driveXVel = Math.max(driveXVel, 0.25);
    }

    if (driveYVel < 0.0){
      DogLog.log("Auto/DriveToReefV9/Check/C", -1);
      driveYVel = Math.min(driveYVel, -0.25);
    }
    if (driveYVel > 0.0){
      DogLog.log("Auto/DriveToReefV9/Check/C", 1);
      driveYVel = Math.max(driveYVel, 0.25);  
    }

    DogLog.log("Auto/DriveToReefV9/Exec/FieldRelDistanceFromTargetX", Math.abs(currentPose.getX() - target.getX()));
    DogLog.log("Auto/DriveToReefV9/Exec/FieldRelDistanceFromTargetY", Math.abs(currentPose.getY() - target.getY()));
    DogLog.log("Auto/DriveToReefV9/Exec/ThetaFromTarget", Units.radiansToDegrees(thetaFromTarget));


    DogLog.log("Auto/DriveToReefV9/Exec/RobotRelSuppliedDriveVelocityX", driveXVel);
    DogLog.log("Auto/DriveToReefV9/Exec/RobotRelSuppliedDriveVelocityY", driveYVel);
    DogLog.log("Auto/DriveToReefV9/Exec/RobotRelSuppliedThetaVelocity", thetaVelocity);
    DogLog.log("Auto/DriveToReefV9/Status", "Executing");
    DogLog.log("Auto/DriveToReefV9/Check/XDrivePIDAtGoal", xdriveController.atGoal());
    DogLog.log("Auto/DriveToReefV9/Check/YDrivePIDAtGoal", ydriveController.atGoal());
    DogLog.log("Auto/DriveToReefV9/Check/ThetaPIDAtGoal", thetaController.atGoal());

    DriveSS.drive(new Translation2d(driveXVel, driveYVel),thetaVelocity, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSS.DH_Out_DriveToPose = false;
    DogLog.log("Auto/DriveToReefV9/Status", "Finished");
    DogLog.log("Auto/DriveToReefV9/Interrupted", interrupted);
    DriveSS.drive(new ChassisSpeeds(0,0,0));
  
    
  }

  public boolean atGoal() {
    return  xdriveController.atSetpoint() && ydriveController.atSetpoint() && thetaController.atGoal() && DriveSS.isRobotStopped();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atGoal();
    
  }

  public Pose2d GetTargetPose(Pose2d targetPose){
    double branchoffset = AutoAlignReef.branchOffset;
    double distanceFromFace = AutoAlignReef.distanceFromFace;
    Transform2d offset;
    if (pos == 2){
      offset = new Transform2d(distanceFromFace, branchoffset, new Rotation2d(Units.degreesToRadians(180)));
    }
      else{
      offset = new Transform2d(distanceFromFace, -branchoffset, new Rotation2d(Units.degreesToRadians(180)));

    }
    Pose2d targetAprilTagPose = DriveSS.GetClosestReefSegment();
    target = targetAprilTagPose.plus(offset);
    return target;
  }
   
}
