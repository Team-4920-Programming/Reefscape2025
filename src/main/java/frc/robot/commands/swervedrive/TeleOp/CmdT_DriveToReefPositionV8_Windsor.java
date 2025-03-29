// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.TeleOp;

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
import frc.robot.Constants.PIDs.CoralElevator.DriveToPoseTele;
import frc.robot.Constants.RobotAutomationInformation.AutoAlignReef;
import frc.robot.subsystems.DataHighway.DataHighwaySubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_DriveToReefPositionV8_Windsor extends Command {

  private final SwerveSubsystem DriveSS;
  private Pose2d target;

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
        DriveToPoseTele.drivekP, 0.0, DriveToPoseTele.drivekD, new TrapezoidProfile.Constraints(DriveToPoseTele.driveMaxVelocity, DriveToPoseTele.driveMaxAcceleration), Constants.LOOP_TIME);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
        DriveToPoseTele.thetakP, 0.0, DriveToPoseTele.thetakD, new TrapezoidProfile.Constraints(DriveToPoseTele.thetaMaxVelocity, DriveToPoseTele.thetaMaxAcceleration), Constants.LOOP_TIME);

  private Translation2d lastSetpointTranslation = Translation2d.kZero;
  private Rotation2d lastSetpointRotation = Rotation2d.kZero;
  private double lastTime = 0.0;
  private double distanceFromTarget = 0.0;
  private double thetaFromTarget = 0.0;
  private int pos;
  
  public CmdT_DriveToReefPositionV8_Windsor(SwerveSubsystem DriveSubsystem, int position) {
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
    thetaController.setTolerance(DriveToPoseTele.thetaTolerance);
    driveController.setTolerance(DriveToPoseTele.driveTolerance);

    DogLog.log("Tele/DriveToReefV8/Init/currentPose", currentPose);
    DogLog.log("Tele/DriveToReefV8/Init/targetPose", target);
    DogLog.log("Tele/DriveToReefV8/Init/drivePIDTolerance", driveController.getPositionTolerance());
    DogLog.log("Tele/DriveToReefV8/Init/thetaPIDTolerance", thetaController.getPositionTolerance());
    DogLog.log("Tele/DriveToReefV8/Init/drivePIDError", driveController.getPositionError());
    DogLog.log("Tele/DriveToReefV8/Init/thetaPIDError", thetaController.getPositionError());
    DogLog.log("Tele/DriveToReefV8/Status", "Initialized");
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    Pose2d currentPose = DriveSS.getPose();
    Pose2d targetPose = target;

    double currentDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    
    double ffScaler = MathUtil.clamp(
        (currentDistance - DriveToPoseTele.ffMinRadius) / (DriveToPoseTele.ffMaxRadius - DriveToPoseTele.ffMinRadius),
        0.0,
        1.0);

        DogLog.log("Tele/DriveToReefV8/Exec/currentPose", currentPose);
        DogLog.log("Tele/DriveToReefV8/Exec/targetPose", targetPose);
        DogLog.log("Tele/DriveToReefV8/Exec/ffScaler", ffScaler);
        
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

            DogLog.log("Tele/DriveToReefV8/Exec/driveVelocityScalar", driveVelocityScalar);
            DogLog.log("Tele/DriveToReefV8/Exec/lastSetpointTranslation", lastSetpointTranslation);

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

    // double driveXVel = driveVelocity.getX();
    // double driveYVel = driveVelocity.getY();
    double driveXVel = 999;
    double driveYVel = 999;
    // if (Math.abs(currentPose.getX() - target.getX()) <=  DriveToPoseTele.driveTolerance) 
    //    driveXVel = 0.0;
    // if (Math.abs(currentPose.getY() - target.getY()) <=  DriveToPoseTele.driveTolerance)
    //    driveYVel = 0.0;
    Translation2d PositionErrorRobotRel = new Translation2d(driveController.getPositionError()*Math.cos(thetaController.getPositionError()),driveController.getPositionError()*Math.sin(thetaController.getPositionError()));
    Translation2d test23  = new Translation2d(Math.abs(currentPose.getX() - target.getX()), Math.abs(currentPose.getY() - target.getY())).rotateBy(new Rotation2d(-1*targetPose.getRotation().getRadians()));
    Translation2d test32  = new Translation2d(Math.abs(currentPose.getX() - target.getX()), Math.abs(currentPose.getY() - target.getY())).rotateBy(new Rotation2d(targetPose.getRotation().getRadians()));

//Math.abs(currentPose.getY() - target.getY()) >=   DriveToPoseTele.driveTolerance ||
if (( Math.abs(PositionErrorRobotRel.getY()) >=   DriveToPoseTele.driveTolerance * 3 || Math.abs(thetaController.getPositionError()) >= thetaController.getPositionTolerance()*3) && currentDistance <= 0.9 ){
  DriveSS.DH_Out_DriveToPose = true;
  // driveYVel = 0.0;
  driveXVel = 0.0;
  }
  Translation2d FieldToRobotVel = driveVelocity.rotateBy(new Rotation2d(-1*DriveSS.getPose().getRotation().getRadians()));
  DogLog.log("Tele/DriveToReefV8/Exec/RawFieldDriveVelX", driveVelocity.getX());
  DogLog.log("Tele/DriveToReefV8/Exec/RawFieldDriveVelY", driveVelocity.getY());
  DogLog.log("Tele/DriveToReefV8/Exec/RawRobotVelY", FieldToRobotVel.getY());
  DogLog.log("Tele/DriveToReefV8/Exec/RawRobotVelX", FieldToRobotVel.getX());
  DogLog.log("Tele/DriveToReefV8/Exec/RotatedErrorX", PositionErrorRobotRel.getX());
  DogLog.log("Tele/DriveToReefV8/Exec/RotatedErrorY", PositionErrorRobotRel.getY());


  
  if (driveXVel != 0){
      driveXVel = FieldToRobotVel.getX();
  }
  if (driveYVel != 0){
      driveYVel = FieldToRobotVel.getY();
  }

  if (!DriveSS.DH_In_MechAtGoal){
      
    // driveYVel *=0.25;
    driveXVel  *=0.75;
  
  }
  
  if (!DriveSS.DH_In_MechAtGoal && currentDistance <= 1.5  ){
      
    // driveYVel *=0.25;
    driveXVel *= 0.5;
  }
    if (thetaVelocity < 0){
      thetaVelocity = Math.min(thetaVelocity, -0.15);
    }
    if (thetaVelocity > 0){
      thetaVelocity = Math.max(thetaVelocity, 0.15);
    }
    if (driveXVel < 0){
      driveXVel = Math.min(driveXVel, -0.2);
    }
    if (driveXVel > 0){
      driveXVel = Math.max(driveXVel, 0.8);
    }

    if (driveYVel < 0){
      driveYVel = Math.min(driveYVel, -0.15);
    }
    if (driveYVel > 0){
      driveYVel = Math.max(driveYVel, 0.15);
    }

    DogLog.log("Tele/DriveToReefV8/Exec/FieldRelDistanceFromTarget", distanceFromTarget);
    DogLog.log("Tele/DriveToReefV8/Exec/FieldRelDistanceFromTargetX", Math.abs(currentPose.getX() - target.getX()));
    DogLog.log("Tele/DriveToReefV8/Exec/FieldRelDistanceFromTargetY", Math.abs(currentPose.getY() - target.getY()));
    DogLog.log("Tele/DriveToReefV8/Exec/ThetaFromTarget", Units.radiansToDegrees(thetaFromTarget));


    DogLog.log("Tele/DriveToReefV8/Exec/RobotRelSuppliedDriveVelocityX", driveXVel);
    DogLog.log("Tele/DriveToReefV8/Exec/RobotRelSuppliedDriveVelocityY", driveYVel);
    DogLog.log("Tele/DriveToReefV8/Exec/RobotRelSuppliedThetaVelocity", thetaVelocity);
    DogLog.log("Tele/DriveToReefV8/Status", "Executing");
    DogLog.log("Tele/DriveToReefV8/Check/DrivePIDAtGoal", driveController.atGoal());
    DogLog.log("Tele/DriveToReefV8/Check/ThetaPIDAtGoal", thetaController.atGoal());


    DogLog.log("Tele/DriveToReefV8/Check/test23x", test23.getX());
    DogLog.log("Tele/DriveToReefV8/Check/test23y", test23.getY());
    DogLog.log("Tele/DriveToReefV8/Check/test32x", test32.getX());
    DogLog.log("Tele/DriveToReefV8/Check/test32y", test32.getY());
    


    // DriveSS.drive(
    //     ChassisSpeeds.fromRobotRelativeSpeeds(
    //       -driveXVel, -driveYVel, thetaVelocity, currentPose.getRotation()));
    DriveSS.drive(new Translation2d(driveXVel, driveYVel),thetaVelocity, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSS.DH_Out_DriveToPose = false;
    DogLog.log("Tele/DriveToReefV8/Status", "Finished");
    DogLog.log("Tele/DriveToReefV8/Interrupted", interrupted);
    DriveSS.drive(new ChassisSpeeds(0,0,0));
  
    
  }

  public boolean atGoal() {
    return  driveController.atGoal() && thetaController.atGoal();
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
