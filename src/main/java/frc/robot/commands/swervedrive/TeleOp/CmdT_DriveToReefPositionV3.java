// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.TeleOp;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DataHighway.DataHighwaySubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_DriveToReefPositionV3 extends Command {

  Integer branch;
 
  SwerveSubsystem DriveSS;
  Pose2d ReefPose;
  Pose2d targetPose;
  boolean inter;
  Trajectory trajectory;
  HolonomicDriveController controller;
  private Timer DriveTimer = new Timer();
  public CmdT_DriveToReefPositionV3(SwerveSubsystem DriveSubsystem, int Position) {
    // Use addRequirements() here to declare subsystem dependencies.
    branch = Position;
    DriveSS = DriveSubsystem;
    addRequirements(DriveSS);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    double branchoffset = Units.inchesToMeters(7.825);
    double distanceFromFace = Units.inchesToMeters(19);
    inter = false;
    Pose2d targetAprilTagPose = DriveSS.GetClosestReefSegment();

    targetPose = new Pose2d();
    targetPose = targetAprilTagPose;
    if (branch == 1){

      Transform2d test = new Transform2d(distanceFromFace, -branchoffset
      ,new Rotation2d(0));

      targetPose = targetAprilTagPose.plus(test);

      System.out.println("target Pose = " + targetPose.toString());
      System.out.println("target April Tag Pose = " + targetAprilTagPose.toString());
      System.out.println("transform = " + test.toString());

    }
    if (branch == 2){

      Transform2d test = new Transform2d(distanceFromFace, branchoffset
      ,new Rotation2d(0));

      targetPose = targetAprilTagPose.plus(test);

      System.out.println("target Pose = " + targetPose.toString());
      System.out.println("target April Tag Pose = " + targetAprilTagPose.toString());
      System.out.println("transform = " + test.toString());
    }

    controller = new HolonomicDriveController(
    new PIDController(2, 0, 0.02), new PIDController(2, 0, 0.02),
    new ProfiledPIDController(0.15, 0, 0,
    new TrapezoidProfile.Constraints(
      RadiansPerSecond.convertFrom(360, DegreesPerSecond), 
      RadiansPerSecondPerSecond.convertFrom(180,RadiansPerSecondPerSecond))));

      controller.setTolerance(new Pose2d(0.01,0.01,new Rotation2d(Units.degreesToRadians(1))));

        TrajectoryConfig config = new TrajectoryConfig(3.5, 1);
        config.setReversed(false);

    var trajectory = TrajectoryGenerator.generateTrajectory(
        DriveSS.getPose(), null,targetPose,
        config);

      DriveTimer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Trajectory.State goal = trajectory.sample(DriveTimer.get());
    ChassisSpeeds adjustedSpeeds = controller.calculate(
      DriveSS.getPose(), goal, goal.poseMeters.getRotation());

    DriveSS.drive(adjustedSpeeds);

    DogLog.log("DriveV3/Xvel",adjustedSpeeds.vxMetersPerSecond);
    DogLog.log("DriveV3/Yvel",adjustedSpeeds.vyMetersPerSecond);
    DogLog.log("DriveV3/Rotvel",adjustedSpeeds.omegaRadiansPerSecond);
    DogLog.log("DriveV3/XPIDError",controller.getXController().getError());
    DogLog.log("DriveV3/YPIDError",controller.getYController().getError());
    DogLog.log("DriveV3/RotPIDError",Units.radiansToDegrees(controller.getThetaController().getPositionError()));
    DogLog.log ("DriveV3/ScoreTargetPose", targetPose);
    
   
    System.out.println("Driving V3 toReef");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DriveSS.drive(new ChassisSpeeds(0,0,0));
    DriveTimer.stop();
    DriveTimer.reset();
    
    System.out.println("DrivetoReef Finished");
    System.out.println("interrupted"+interrupted);
    inter = interrupted;
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atReference();
  }

  public boolean wasInterrupted(){
    return inter;
  }
}
