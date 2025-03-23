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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.Constants.PIDs.CoralElevator.DriveToPose;
import frc.robot.Constants.RobotAutomationInformation.AutoAlignReef;
import frc.robot.subsystems.DataHighway.DataHighwaySubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdA_DriveToReefPositionV6_ActuallyWinningWindsor extends Command {

  private final SwerveSubsystem DriveSS;
  private Pose2d target;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
        DriveToPose.drivekP, 0.0, DriveToPose.drivekD, new TrapezoidProfile.Constraints(DriveToPose.driveMaxVelocity, DriveToPose.driveMaxAcceleration), Constants.LOOP_TIME);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
        DriveToPose.thetakP, 0.0, DriveToPose.thetakD, new TrapezoidProfile.Constraints(DriveToPose.thetaMaxVelocity, DriveToPose.thetaMaxAcceleration), Constants.LOOP_TIME);

  private Translation2d lastSetpointTranslation = Translation2d.kZero;
  private Rotation2d lastSetpointRotation = Rotation2d.kZero;
  private double lastTime = 0.0;
  private double distanceFromTarget = 0.0;
  private double thetaFromTarget = 0.0;
  private int pos;
  
  public CmdA_DriveToReefPositionV6_ActuallyWinningWindsor(SwerveSubsystem DriveSubsystem, int position) {
    DriveSS = DriveSubsystem;
    pos = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    

    target = GetTargetPose(DriveSS.GetClosestReefSegment());
    Pose2d currentPose = DriveSS.getPose();
    ChassisSpeeds fieldVelocity = DriveSS.getFieldVelocity();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    driveController.reset(
        currentPose.getTranslation().getDistance(target.getTranslation()),
        Math.min(
          0.0,
          -linearFieldVelocity.rotateBy(
                        target.getTranslation()
                          .minus(currentPose.getTranslation())
                          .getAngle()
                          .unaryMinus())
                          .getX()));
    
    thetaController.reset(
        currentPose.getRotation().getRadians(), 
        fieldVelocity.omegaRadiansPerSecond);

    lastSetpointTranslation = currentPose.getTranslation();
    lastSetpointRotation = target.getRotation();
    lastTime = Timer.getTimestamp();
    thetaController.setTolerance(DriveToPose.thetaTolerance);
    driveController.setTolerance(DriveToPose.driveTolerance);
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    Pose2d currentPose = DriveSS.getPose();
    Pose2d targetPose = target;

    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    
    double ffScaler = MathUtil.clamp(
        (currentDistance - DriveToPose.ffMinRadius) / (DriveToPose.ffMaxRadius - DriveToPose.ffMinRadius),
        0.0,
        2.0);

    DogLog.log("Auto/Test/Exec/currentPose", currentPose);
    DogLog.log("Auto/Test/Exec/targetPose", targetPose);
    DogLog.log("Auto/Test/Exec/currentDistance", currentDistance);
    
    DogLog.log("Auto/Test/Check/ffScaler", ffScaler);
        
    distanceFromTarget = currentDistance;

    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.calculate(distanceFromTarget, 0.0)
            + driveController.getSetpoint().velocity * ffScaler;

    if (currentDistance <= driveController.getPositionTolerance()) driveVelocityScalar = 0.0;

    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                new Rotation2d(
                    Math.atan2(
                        currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
                        currentPose.getTranslation().getX() - targetPose.getTranslation().getX())))
            .transformBy(new Transform2d(driveController.getSetpoint().position, 0.0, Rotation2d.kZero))
            .getTranslation();

    DogLog.log("Auto/Test/Check/driveVelocityScalar", driveVelocityScalar);
    DogLog.log("Auto/Test/Check/lastSetpointTranslation", lastSetpointTranslation);

    // Calculate theta speed
    double thetaVelocity = thetaController.calculate( currentPose.getRotation().getRadians(),
                    new TrapezoidProfile.State( targetPose.getRotation().getRadians(),
                    (targetPose.getRotation().minus(lastSetpointRotation)).getRadians() / (Timer.getTimestamp() - lastTime)))
            + thetaController.getSetpoint().velocity * ffScaler;
            thetaFromTarget =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaFromTarget <= thetaController.getPositionTolerance()) thetaVelocity = 0.0;
    lastSetpointRotation = targetPose.getRotation();

    
    Translation2d driveVelocity =
        new Pose2d(
                Translation2d.kZero,
                new Rotation2d(
                    Math.atan2(
                        currentPose.getTranslation().getY() - targetPose.getTranslation().getY(),
                        currentPose.getTranslation().getX() - targetPose.getTranslation().getX())))
            .transformBy(new Transform2d(driveVelocityScalar, 0.0, Rotation2d.kZero))
            .getTranslation();
    lastTime = Timer.getTimestamp();

    // // Scale feedback velocities by input ff
    // final double linearS = linearFF.get().getNorm() * 3.0;
    // final double thetaS = Math.abs(omegaFF.getAsDouble()) * 3.0;
    // driveVelocity =
    //     driveVelocity.interpolate(linearFF.get().times(4.0), linearS);
    // // thetaVelocity =
    // //     MathUtil.interpolate(
    // //         thetaVelocity, omegaFF.getAsDouble() * 4.0 / Units.inchesToMeters(Math.hypot(13.25, 13.25)), thetaS);

    // Command speeds

    double driveXVel = driveVelocity.getX();
    double driveYVel = driveVelocity.getY();
    if (Math.abs(currentPose.getX() - target.getX()) <=  DriveToPose.driveTolerance) 
       driveXVel = 0.0;
    if (Math.abs(currentPose.getY() - target.getY()) <=  DriveToPose.driveTolerance)
       driveYVel = 0.0;

    if ((Math.abs(currentPose.getY() - target.getY()) >=   DriveToPose.driveTolerance || !thetaController.atSetpoint()) && currentDistance <= 0.2){
      
      DriveSS.DH_Out_DriveToPose = true;
      driveXVel = 0.0;
    }
    if (thetaVelocity < 0){
      thetaVelocity = Math.min(thetaVelocity, -0.12);
    }
    if (thetaVelocity > 0){
      thetaVelocity = Math.max(thetaVelocity, 0.12);
    }
    if (driveXVel < 0){
      driveXVel = Math.min(driveXVel, -0.1);
    }
    if (driveXVel > 0){
      driveXVel = Math.max(driveXVel, 0.1);
    }

    if (driveYVel < 0){
      driveYVel = Math.min(driveYVel, -0.1);
    }
    if (driveYVel > 0){
      driveYVel = Math.max(driveYVel, 0.1);
    }
    DogLog.log("Auto/Test/Check/distanceFromTarget", distanceFromTarget);
    DogLog.log("Auto/Test/Check/distanceFromTargetX", Math.abs(currentPose.getX() - target.getX()));
    DogLog.log("Auto/Test/Check/distanceFromTargetY", Math.abs(currentPose.getY() - target.getY()));
    DogLog.log("Auto/Test/Check/thetaFromTarget", Units.radiansToDegrees(thetaFromTarget));
    DogLog.log("Auto/Test/Check/RawdriveVelocityX", driveVelocity.getX());
    DogLog.log("Auto/Test/Check/RealdriveVelocityX", driveXVel);

    DogLog.log("Auto/Test/Check/RealdriveVelocityY", driveYVel);
    DogLog.log("Auto/Test/Check/RawdriveVelocityY", driveVelocity.getY());
    DogLog.log("Auto/Test/Check/thetaVelocity", thetaVelocity);
    DogLog.log("Auto/Test/Check/currentPose.getRotation()",currentPose.getRotation());
    DogLog.log("Auto/Test/Check/FINISHED", false);



    DriveSS.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
          driveXVel, driveYVel, thetaVelocity, currentPose.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSS.DH_Out_DriveToPose = false;
    DogLog.log("Auto/Test/Check/FINISHED", true);
    DriveSS.drive(new ChassisSpeeds(0,0,0));
  
    
  }

  public boolean atGoal() {
    return  Math.abs(DriveSS.getPose().getX() - target.getX()) <=  DriveToPose.driveTolerance && Math.abs(DriveSS.getPose().getY() - target.getY()) <=  DriveToPose.driveTolerance && thetaController.atGoal();
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
    // target = target.rotateAround(target.getTranslation(), new Rotation2d(Units.degreesToRadians(180)));
    return target;
  }
   
}
