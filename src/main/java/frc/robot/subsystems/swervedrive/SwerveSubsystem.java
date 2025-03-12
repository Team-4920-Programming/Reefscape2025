// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.ISerializable;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.Vision.Cameras;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralAlgaeStack;
import org.json.simple.parser.ParseException;
import org.opencv.features2d.FlannBasedMatcher;
import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.math.Matrix;
import frc.robot.Vision4920;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public class SwerveSubsystem extends SubsystemBase
{

  /**
   * Swerve drive object.
   */
  private final SwerveDrive         swerveDrive;
  /**
   * AprilTag field layout.
   */
  private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  /**
   * Enable vision odometry updates while driving.
   */
  private final boolean             visionDriveTest     =false; //YAGSL Vision
  /**
   * PhotonVision class to keep an accurate odometry.
   */
  private       Vision              vision;

  private     IntakeSimulation AintakeSimulation;
  private     IntakeSimulation CintakeSimulation;
  private boolean AutoAimEnabled = true;
  // Datahighway information
  public boolean DH_In_HasCoral = false;
  public boolean DH_In_InRedZone = false;
  public boolean DH_In_CoralYellow =false;
  public boolean DH_InStationZone = false;
  public boolean DH_In_InLeftCoralZone = false;
  public boolean DH_In_InRightCoralZone = false;
  public Pose2d DH_In_ClosestReefSegment;
  public int DH_Out_ReefSegment = 0;
  public double DH_Out_ReefDistance = 0;
  public boolean DH_OUT_isBlueAlliance = false;
  public boolean DH_OUT_isRedAlliance = false;
  public boolean DH_Out_AtCoralStation = false;

  public Pose2d DH_In_LeftCoralPose;
  public Pose2d DH_In_RightCoralPose;

  //Vision
  private Vision4920 GreyFeederCamera;
  private Vision4920 GreyReefCamera;
  private Vision4920 RedReefCamera;
  private Vision4920 RedGeneralCamera;
  private Vision4920 BlueGeneralCamera;
  private Vision4920 BlueFrontCamera;
  public Pose3d GreyFeederCameraPose3d = new Pose3d();
  public Pose3d GreyReefCameraPose3d = new Pose3d();
  public Pose3d RedReefCameraPose3d = new Pose3d();
  public Pose3d RedGeneralCameraPose3d = new Pose3d();
  public Pose3d BlueGeneralCameraPose3d = new Pose3d();
  public Pose3d BlueFrontCameraPose3d = new Pose3d();
  private PIDController RotPID = new PIDController(0.1,0,0);

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory)
  {
    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    //  In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
    //  In this case the wheel diameter is 4 inches, which must be converted to meters to get meters/second.
    //  The gear ratio is 6.75 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(3), 3.95);
    System.out.println("\"conversionFactors\": {");
    System.out.println("\t\"angle\": {\"factor\": " + angleConversionFactor + " },");
    System.out.println("\t\"drive\": {\"factor\": " + driveConversionFactor + " }");
    System.out.println("}");

    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
                                                                  new Pose2d(new Translation2d(Meter.of(1),
                                                                                               Meter.of(4)),
                                                                             Rotation2d.fromDegrees(0)));
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    if (Robot.isReal())
    {
      swerveDrive.setHeadingCorrection(true); // Heading correction should only be used while controlling the robot via angle.
      swerveDrive.setCosineCompensator(true);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
      
    }
    else
    {
      swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
      swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
         
    }
   swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
    //swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
    if (visionDriveTest)
    {
      setupPhotonVision();
      // Stop the odometry thread if we are using vision that way we can synchronize updates better.
      swerveDrive.stopOdometryThread();
    }

    //4920 Vision
    GreyFeederCamera = new Vision4920(Constants.Vision4920.kGreyFeederCam, Constants.Vision4920.kRobotToGreyFeederCam);
    GreyReefCamera = new Vision4920(Constants.Vision4920.kGreyReefCam, Constants.Vision4920.kRobotToGreyReefCam);
    RedReefCamera = new Vision4920(Constants.Vision4920.kRedReefCam, Constants.Vision4920.kRobotToRedReefCam);
    RedGeneralCamera = new Vision4920(Constants.Vision4920.kRedGeneralCam, Constants.Vision4920.kRobotToRedGeneralCam);
    BlueGeneralCamera = new Vision4920(Constants.Vision4920.kBlueGeneralCam, Constants.Vision4920.kRobotToBlueGeneralCam);
    BlueFrontCamera = new Vision4920(Constants.Vision4920.kBlueFrontCam, Constants.Vision4920.kRobotToBlueFrontCam);



    setupPathPlanner();
    if (Robot.isSimulation())
    {
      SetupIntake();
      setupSimulatedField();
    }
    
  }
  public void EnableAutoAim()
  {
    AutoAimEnabled = true;
  }
  public void DisableAutoAim()
  {
    AutoAimEnabled = false;
  }
  public boolean isAutoAim()
  {
    return AutoAimEnabled;
  }
  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg,
                                  controllerCfg,
                                  Constants.MAX_SPEED,
                                  new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                                             Rotation2d.fromDegrees(0)));
  }

  /**
   * Setup the photon vision class.
   */
  public void setupPhotonVision()
  {
    vision = new Vision(swerveDrive::getPose, swerveDrive.field);
  }
//*************************************************Start of 4920 Modificatons************************************ */
  @Override
  public void periodic()
  {
    // When vision is enabled we must manually update odometry in SwerveDrive
    if (visionDriveTest)
    {
      swerveDrive.updateOdometry();
      vision.updatePoseEstimation(swerveDrive);
      //vision.getEstimatedGlobalPose(vision.)
    }
    //4920 Vision
    swerveDrive.updateOdometry();
    ProcessVision4920();


    DogLog.log("Data/RobotOdo",swerveDrive.getPose());
    DogLog.log("Data/RoboSpeed", swerveDrive.getRobotVelocity());
    DogLog.log("Data/Reefposition",getReefSegment());
    DogLog.log("Data/ReefDistance",getReefDistance());

    DH_Out_ReefSegment = getReefSegment();
    DH_Out_ReefDistance = getReefDistance();

    DH_OUT_isRedAlliance = isRedAlliance();
    DH_OUT_isBlueAlliance = !isRedAlliance();
  }
  public int getReefSegment()
{
  Pose2d CurrentPose = getPose();
  double ReefX = 4.5;
  double ReefY = 4;
  if (isRedAlliance())
    ReefX = 17.5-4;

  double RobottoReefX = CurrentPose.getX() - ReefX;
  double RobottoReefY = CurrentPose.getY() - ReefY;
  double RobotAngletoReef = Math.atan2(RobottoReefX, RobottoReefY);
  double Segment = Units.radiansToDegrees(RobotAngletoReef);
  DogLog.log("Data/Reefsegmentangle", Segment);
//-180 to 180  
  //Segment = Segment/ 60;
  //- 3 to 3
  int seg  =0;
  if (Segment > 60 && Segment < 120 )
    seg = 0; 
  if (Segment >= 120 && Segment < 180)
    seg = 5;
  if (Segment <=0 && Segment > -60)
    seg =2;
  if (Segment <= -60 && Segment > -120)
    seg =3;
  if (Segment <= -120 && Segment > -180)
    seg =4;
  if (Segment >= 0 && Segment <= 60)
    seg =1;
    
  

  // 0 in front of blue driver station
  // 1 bottem corner near blue
  // 2 near processor 
  // 3 
  // 4 - top corner near blue
  //5
  //5

  return(seg);
}
public double getReefDistance(){
  Pose2d CurrentPose = getPose();
  double ReefX = 4.5;
  double ReefY = 4.0;
  if (isRedAlliance())
    ReefY = 17.5-4.5;

  double RobottoReefX = CurrentPose.getX();
  double RobottoReefY = CurrentPose.getY();

  double distance = Math.sqrt(Math.pow(RobottoReefX - ReefX,2)+Math.pow(RobottoReefY-ReefY,2));
  
  return distance;

}
private void ProcessVision4920()
{
  //Process Vision
  Pose2d GreyFeederPose = new Pose2d(0.0 ,0.0, Rotation2d.fromDegrees(0.0));
  double GreyFeederVisionTimestamp;

  
  if ( DriverStation.isDSAttached() && GreyFeederCamera != null)
  {
     var visionEst = GreyFeederCamera.getEstimatedGlobalPose();
     SmartDashboard.putBoolean("GreyFeederCamera Present", GreyFeederCamera.isConnected());
     
      if (visionEst.isPresent()){
          GreyFeederPose = visionEst.get().estimatedPose.toPose2d();
          GreyFeederCameraPose3d = visionEst.get().estimatedPose;

          
          GreyFeederVisionTimestamp = visionEst.get().timestampSeconds;
          DogLog.log("Grey Feeder Camera Pose", GreyFeederCameraPose3d);
          DogLog.log("Grey Feeder TimeStamp",GreyFeederVisionTimestamp);
            VisionReading(GreyFeederPose, GreyFeederVisionTimestamp, GreyFeederCamera.confidenceCalculator(visionEst.get()));
      }
  
  }
 
  Pose2d GreyReefPose= new Pose2d(0.0 ,0.0, Rotation2d.fromDegrees(0.0));;
  double GreyReefVisionTimestamp;
  
  
  if ( DriverStation.isDSAttached() && GreyReefCamera != null)
  {
     var visionEst = GreyReefCamera.getEstimatedGlobalPose();
     SmartDashboard.putBoolean("GreyReefCamera Present", GreyReefCamera.isConnected());
     
      if (visionEst.isPresent()){
          GreyReefPose = visionEst.get().estimatedPose.toPose2d();
          GreyReefCameraPose3d = visionEst.get().estimatedPose;

          
          GreyReefVisionTimestamp = visionEst.get().timestampSeconds;
          DogLog.log("Grey Reef Camera Pose", GreyReefCameraPose3d);
          DogLog.log("Grey Reef TimeStamp",GreyReefVisionTimestamp);
            VisionReading(GreyReefPose, GreyReefVisionTimestamp, GreyReefCamera.confidenceCalculator(visionEst.get()));
      }
  
  }

  Pose2d RedReefPose= new Pose2d(0.0 ,0.0, Rotation2d.fromDegrees(0.0));;
  double RedReefVisionTimestamp;

  if ( DriverStation.isDSAttached() && RedReefCamera != null)
  {
     var visionEst = RedReefCamera.getEstimatedGlobalPose();
     SmartDashboard.putBoolean("RedReefCamera Present", RedReefCamera.isConnected());
     
      if (visionEst.isPresent()){
          RedReefPose = visionEst.get().estimatedPose.toPose2d();
          RedReefCameraPose3d = visionEst.get().estimatedPose;

          
          RedReefVisionTimestamp = visionEst.get().timestampSeconds;
          DogLog.log("Red Reef Camera Pose", RedReefPose);
          DogLog.log("Red Reef TimeStamp",RedReefVisionTimestamp);
            VisionReading(RedReefPose, RedReefVisionTimestamp, RedReefCamera.confidenceCalculator(visionEst.get()));
      }
  
  }

  Pose2d RedGeneralPose= new Pose2d(0.0 ,0.0, Rotation2d.fromDegrees(0.0));;
  double RedGeneralVisionTimestamp;

  if ( DriverStation.isDSAttached() && RedGeneralCamera != null)
  {
     var visionEst = RedGeneralCamera.getEstimatedGlobalPose();
     SmartDashboard.putBoolean("RedGeneralCamera Present", RedGeneralCamera.isConnected());
     
      if (visionEst.isPresent()){
          RedGeneralPose = visionEst.get().estimatedPose.toPose2d();
          RedGeneralCameraPose3d = visionEst.get().estimatedPose;

          
          RedGeneralVisionTimestamp = visionEst.get().timestampSeconds;
          DogLog.log("Red General Camera Pose", RedGeneralCameraPose3d);
          DogLog.log("Red General TimeStamp",RedGeneralVisionTimestamp);
          VisionReading(RedGeneralPose, RedGeneralVisionTimestamp, RedGeneralCamera.confidenceCalculator(visionEst.get()));
      }
  
  }

  Pose2d BlueFrontPose= new Pose2d(0.0 ,0.0, Rotation2d.fromDegrees(0.0));;
  double BlueFrontVisionTimestamp;

  if ( DriverStation.isDSAttached() && BlueFrontCamera != null)
  {
     var visionEst = BlueFrontCamera.getEstimatedGlobalPose();
     SmartDashboard.putBoolean("BlueFrontCamera Present", BlueFrontCamera.isConnected());
     
      if (visionEst.isPresent()){
          BlueFrontPose = visionEst.get().estimatedPose.toPose2d();
          BlueFrontCameraPose3d = visionEst.get().estimatedPose;

          
          BlueFrontVisionTimestamp = visionEst.get().timestampSeconds;
          DogLog.log("Blue Front Camera Pose", BlueFrontCameraPose3d);
          DogLog.log("Blue Front TimeStamp",BlueFrontVisionTimestamp);
            VisionReading(BlueFrontPose, BlueFrontVisionTimestamp, BlueFrontCamera.confidenceCalculator(visionEst.get()));
      }
  
  }
  
  Pose2d BlueGeneralPose= new Pose2d(0.0 ,0.0, Rotation2d.fromDegrees(0.0));;
  double BlueGeneralVisionTimestamp;

  if ( DriverStation.isDSAttached() && BlueGeneralCamera != null)
  {
     var visionEst = BlueGeneralCamera.getEstimatedGlobalPose();
     SmartDashboard.putBoolean("BlueGeneralCamera Present", BlueGeneralCamera.isConnected());
     
      if (visionEst.isPresent()){
          BlueGeneralPose = visionEst.get().estimatedPose.toPose2d();
          BlueGeneralCameraPose3d = visionEst.get().estimatedPose;

          
          BlueGeneralVisionTimestamp = visionEst.get().timestampSeconds;
          DogLog.log("Blue General Camera Pose", BlueGeneralCameraPose3d);
          DogLog.log("Blue General TimeStamp",BlueGeneralVisionTimestamp);
          VisionReading(BlueGeneralPose, BlueGeneralVisionTimestamp, BlueGeneralCamera.confidenceCalculator(visionEst.get()));
      }
  
  }


}
/*4920 modificaitons for simulation */

  @Override
  public void simulationPeriodic()
  {
    DogLog.log("FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    DogLog.log("FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    //DogLog.log("Simulated Position",Simulate)
    SmartDashboard.putNumber("y",100);
    DogLog.log("FieldSimulation/Robot",swerveDrive.getMapleSimDrive().get().getSimulatedDriveTrainPose());
    //swerveDrive.addVisionMeasurement(getPose(), getAIntakeGamePiecesAmount());

    //swerveDrive.addVisionMeasurement(swerveDrive.getMapleSimDrive().get().getSimulatedDriveTrainPose(),Timer.getFPGATimestamp());

      //swerveDrive.getMapleSimDrive().get().getSimulatedDriveTrainPose())

  }

  public void SetupIntake(){
        Distance width = Inches.of(20);
        Distance Extension = Inches.of(13);
        AintakeSimulation = AintakeSimulation.OverTheBumperIntake(
              "Algae", 
              swerveDrive.getMapleSimDrive().get(), 
              width, Extension, IntakeSimulation.IntakeSide.LEFT, 1);
        AintakeSimulation.register(SimulatedArena.getInstance());

        Distance CWidth = Inches.of(5);
        Distance cExtension = Inches.of(12);
        CintakeSimulation = CintakeSimulation.OverTheBumperIntake(
              "Coral", 
              swerveDrive.getMapleSimDrive().get(), 
              width, Extension, IntakeSimulation.IntakeSide.BACK, 1);
        CintakeSimulation.register(SimulatedArena.getInstance());
  }
  public void setupSimulatedField()
  { 
      SimulatedArena.getInstance().resetFieldForAuto();
    
  }
  public void startCIntake()
  {
    CintakeSimulation.startIntake();
  }
  public void stopCIntake(){
        CintakeSimulation.stopIntake();
  }
  public void getGamePieceFromCIntake(){
    CintakeSimulation.obtainGamePieceFromIntake();
  }
  public int getCIntakeGamePiecesAmount(){
    return CintakeSimulation.getGamePiecesAmount();
  }
  public void startAIntake()
  {
    AintakeSimulation.startIntake();
  }
  public void stopAIntake(){
        AintakeSimulation.stopIntake();
  }
  public void getGamePieceFromAIntake(){
    AintakeSimulation.obtainGamePieceFromIntake();
  }
  public int getAIntakeGamePiecesAmount(){
    return AintakeSimulation.getGamePiecesAmount();
  }




  //**************************************************************End of 4920 Modificatons*************************** */
  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try
    {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces()
                               );
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
              // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent())
            {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
          // Reference to this subsystem to set requirements
                           );

    } catch (Exception e)
    {
      // Handle exception as needed
      e.printStackTrace();
    }

    //Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * Aim the robot at the target returned by PhotonVision.
   *
   * @return A {@link Command} which will run the alignment.
   */
  public Command aimAtTarget(Cameras camera)
  {

    return run(() -> {
      Optional<PhotonPipelineResult> resultO = camera.getBestResult();
      if (resultO.isPresent())
      {
        var result = resultO.get();
        if (result.hasTargets())
        {
          drive(getTargetSpeeds(0,
                                0,
                                Rotation2d.fromDegrees(result.getBestTarget()
                                                             .getYaw()))); // Not sure if this will work, more math may be required.
        }
      }
    });
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose)
  {
// Create the constraints to use while pathfinding
    System.out.println("PathPlanner");
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity()*.05, 4.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

// Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
                                     );
  }

  /**
   * Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.
   *
   * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to achieve.
   * @return {@link Command} to run.
   * @throws IOException    If the PathPlanner GUI settings is invalid
   * @throws ParseException If PathPlanner GUI settings is nonexistent.
   */
  private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
  throws IOException, ParseException
  {
    SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
                                                                            swerveDrive.getMaximumChassisAngularVelocity());
    AtomicReference<SwerveSetpoint> prevSetpoint
        = new AtomicReference<>(new SwerveSetpoint(swerveDrive.getRobotVelocity(),
                                                   swerveDrive.getStates(),
                                                   DriveFeedforwards.zeros(swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
                    () -> {
                      double newTime = Timer.getFPGATimestamp();
                      SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
                                                                                      robotRelativeChassisSpeed.get(),
                                                                                      newTime - previousTime.get());
                      swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
                                        newSetpoint.moduleStates(),
                                        newSetpoint.feedforwards().linearForces());
                      prevSetpoint.set(newSetpoint);
                      previousTime.set(newTime);

                    });
  }

  /**
   * Drive with 254's Setpoint generator; port written by PathPlanner.
   *
   * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
   * @return Command to drive the robot using the setpoint generator.
   */
  public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds)
  {
    try
    {
      return driveWithSetpointGenerator(() -> {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());

      });
    } catch (Exception e)
    {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();

  }


  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand()
  {
    SwerveDriveTest.setModulesToRotaryPosition(swerveDrive);
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12, true),
        3.0, 5.0, 3.0);
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand()
  {
    SwerveDriveTest.setModulesToRotaryPosition(swerveDrive);
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(swerveDrive.getModules())
                           .forEach(it -> it.setAngle(0.0)));
  }

  /**
   * Returns a Command that drives the swerve drive to a specific distance at a given speed.
   *
   * @param distanceInMeters       the distance to drive in meters
   * @param speedInMetersPerSecond the speed at which to drive in meters per second
   * @return a Command that drives the swerve drive to a specific distance at a given speed
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond)
  {
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
        .until(() -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) >
                     distanceInMeters);
  }

  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA)
  {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother controls.
   * @param translationY Translation in the Y direction. Cubed for smoother controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
   * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
   *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      //4920 modifications limit Driver (Jackson) speed in Red Zones

      ChassisSpeeds speed = velocity.get();
        
      if (DH_In_InRedZone)
      {
        if (speed.vxMetersPerSecond> 1)  speed.vxMetersPerSecond =1;
        if (speed.vyMetersPerSecond >1)  speed.vyMetersPerSecond =1;
         

      }
      if (AutoAimEnabled){

        

        if (DH_In_HasCoral && !DH_In_InLeftCoralZone && !DH_In_InRightCoralZone)
        {
        // double RotVel = getRotationVelocity();
        double RotVel = getRotationVelocityToTarget(DH_In_ClosestReefSegment,180);
        speed.omegaRadiansPerSecond = RotVel;
        //swerveDrive.driveFieldOriented(speed);
      }
        if(!DH_In_HasCoral && DH_In_InLeftCoralZone){
          double RotVel = getRotationVelocityToTarget(DH_In_LeftCoralPose,0);
        speed.omegaRadiansPerSecond = RotVel;
        }
        if(!DH_In_HasCoral && DH_In_InRightCoralZone){
          double RotVel = getRotationVelocityToTarget(DH_In_RightCoralPose, 0);
        speed.omegaRadiansPerSecond = RotVel;
        }
    }

    swerveDrive.driveFieldOriented(speed);
    });
  }

  private double getRotationVelocity()
  {
    Pose2d ReefPose;
    double CenterofReefX  = 4.481; //wall to wall
    double CenterofReefY = 4.0; //aliance Wall to Alliance Wall
    double ReefRadius = Units.inchesToMeters(65.5)/2;
    //onshape cordinates of blue 
    //y along alliance wall 
    //x from blue to red
    double LeftOffsetAng=-8;
    double RightOFfsetAng=8;
    double RobotOffset = Units.inchesToMeters(15); 
    double CurrentRot = getPose().getRotation().getDegrees();
    double offsetAng = 0;
    //  System.out.println("Initializing Drive to Reef *****************");
    int ReefSegment = getReefSegment();
  
     double Reefrot = ReefSegment * 60;
  
    RotPID.setTolerance(5);
    RotPID.enableContinuousInput(-180, 180);
    double Reef_Rot = Reefrot +180;

    double RotVel = RotPID.calculate(CurrentRot,Reef_Rot);
    RotVel = MathUtil.clamp(RotVel, -3, 3);

    return RotVel;
  }

  private double getRotationVelocityToTarget(Pose2d target, double offset)
  {

    // Rotation2d targetPose = target.getRotation().minus(getPose().getRotation());

    // double targetAngle = targetPose.getDegrees();


  
    RotPID.setTolerance(5);
    RotPID.enableContinuousInput(-180, 180);

    double RotVel = RotPID.calculate(getPose().getRotation().getDegrees(),target.getRotation().getDegrees()-offset);
    RotVel = MathUtil.clamp(RotVel, -3, 3);

    return RotVel;
  }


  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }


  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
   * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance())
    {
      zeroGyro();
      //Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        Constants.MAX_SPEED);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock()
  {
    swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }
  public void VisionReading(Pose2d visionPose,double Timestamp, Matrix<N3, N1> visionMeasurementStdDevs)
  {
   
    swerveDrive.addVisionMeasurement(visionPose, Timestamp, visionMeasurementStdDevs);
  }
  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }
}