// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.TeleOp;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.util.ArrayList;
import java.util.List;


import dev.doglog.DogLog;

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

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_DriveToReefPositionTesting extends Command {

  Integer branch;
 
  SwerveSubsystem DriveSS;
  Pose2d ReefPose;
  Pose2d targetPose;
  boolean inter;
  Trajectory trajectory;
  HolonomicDriveController controller;
  List<Translation2d> waypoints;
  private Timer DriveTimer = new Timer();
  public CmdT_DriveToReefPositionTesting(SwerveSubsystem DriveSubsystem, int Position) {
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
    waypoints = new ArrayList<Translation2d>();
    // targetPose = new Pose2d();
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

      Transform2d test = new Transform2d(distanceFromFace, branchoffset
      ,new Rotation2d(0));

      targetPose = targetAprilTagPose.plus(test);

      //System.out.println("target Pose = " + targetPose.toString());
      //System.out.println("target April Tag Pose = " + targetAprilTagPose.toString());
      //System.out.println("transform = " + test.toString());
    }

    controller = new HolonomicDriveController(
    new PIDController(0.1, 0, 0.0), new PIDController(0.1, 0, 0.0),
    new ProfiledPIDController(1, 0, 0,
    new TrapezoidProfile.Constraints(
      RadiansPerSecond.convertFrom(360, DegreesPerSecond), 
      RadiansPerSecondPerSecond.convertFrom(180,RadiansPerSecondPerSecond))));

      controller.setTolerance(new Pose2d(0.01,0.01,new Rotation2d(Units.degreesToRadians(1))));

        TrajectoryConfig config = new TrajectoryConfig(3.5, 1);
        config.setReversed(false);
        config.setKinematics(DriveSS.getKinematics());  
        
        for (int i = 1; i <= 20; i ++){
          waypoints.add(DriveSS.getPose().interpolate(targetPose, 0.05*i).getTranslation());
        }
        
        

        trajectory = TrajectoryGenerator.generateTrajectory(
        DriveSS.getPose(), waypoints,targetPose,
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
    
   
    //System.out.println("Driving V3 toReef");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    DriveSS.drive(new ChassisSpeeds(0,0,0));
    DriveTimer.stop();
    DriveTimer.reset();
    
    //System.out.println("DrivetoReef Finished");
    //System.out.println("interrupted"+interrupted);
    inter = interrupted;
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(targetPose.getX()- DriveSS.getPose().getX()) <= 0.01 && Math.abs(targetPose.getY() - DriveSS.getPose().getY()) <= 0.01 && Math.abs(targetPose.getRotation().getDegrees()- DriveSS.getPose().getRotation().getDegrees()) <= 1.0);
  }

  public boolean wasInterrupted(){
    return inter;
  }
}
