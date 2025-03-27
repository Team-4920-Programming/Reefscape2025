// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.AbsoluteEncoderID.Climber;
import frc.robot.commands.Cmd_SeqRemoveLowAlgea;
import frc.robot.commands.Cmd_SeqClimb;
import frc.robot.commands.Cmd_SeqClimbV2;
import frc.robot.commands.Cmd_SeqRemoveHighAlgea;
import frc.robot.commands.Cmd_SeqScoreLeft;
import frc.robot.commands.Cmd_SeqScoreRight;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;
import frc.robot.subsystems.DataHighway.DataHighwaySubsystem;
import frc.robot.subsystems.ReefSurvey.ReefSurveySubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import frc.robot.commands.Elevator.Auto.CmdA_CoralIntake;
import frc.robot.commands.Elevator.Auto.CmdA_CoralOutTake;
import frc.robot.commands.Elevator.Auto.CmdA_Level4;
import frc.robot.commands.Elevator.Auto.CmdA_SafePosition;
import frc.robot.commands.Elevator.Auto.CmdA_Station;
import frc.robot.commands.Elevator.TeleOp.*;
import frc.robot.commands.ReefSurvey.AddToReefTest;
import frc.robot.commands.swervedrive.auto.CmdA_DriveToFeederPosition_Relative;
import frc.robot.commands.swervedrive.auto.CmdA_DriveToReefPosition;
import frc.robot.commands.swervedrive.auto.CmdA_DriveToReefPositionV2;
import frc.robot.commands.swervedrive.auto.CmdA_DriveToReefPositionV3_Relative;
import frc.robot.commands.swervedrive.auto.CmdA_DriveToReefPositionV3_RelativeTest;
import frc.robot.commands.swervedrive.auto.CmdA_DriveToReefPositionV4_Test;
import frc.robot.commands.swervedrive.auto.CmdA_DriveToReefPositionV5_WinningWindsor;
import frc.robot.commands.swervedrive.auto.CmdA_DriveToReefPositionV6_ActuallyWinningWindsor;
import frc.robot.commands.swervedrive.auto.CmdA_DriveToReefPositionV7_Test;
import frc.robot.commands.swervedrive.TeleOp.*;
import swervelib.SwerveDriveTest;
import frc.robot.commands.Climber.Auto.CmdA_BridgeOut;
import frc.robot.commands.Climber.TeleOp.*;
import swervelib.SwerveInputStream;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  private final SendableChooser<Command> autoChooser;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandJoystick OperatorJoystick = new CommandJoystick(1);
  final CommandJoystick ReefJoystick = new CommandJoystick(2);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/maxSwerve"));
  private final CoralElevatorSubsystem CoralElevatorSS = new CoralElevatorSubsystem();
  private final ClimberSubsystem ClimberSS = new ClimberSubsystem();
  //private final AlgaeIntakeSubsystem AlgaeIntakeSS = new AlgaeIntakeSubsystem();
  private final ReefSurveySubsystem ReefSurveySS = new ReefSurveySubsystem();
  private final DataHighwaySubsystem DataHighwaySS = new DataHighwaySubsystem(drivebase,CoralElevatorSS,ReefSurveySS);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                               OperatorConstants.DEADBAND),
                                                                 () -> MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driverXbox.getHID()::getYButtonPressed,
                                                                 driverXbox.getHID()::getAButtonPressed,
                                                                 driverXbox.getHID()::getXButtonPressed,
                                                                 driverXbox.getHID()::getBButtonPressed);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -driverXbox.getLeftY(),
                                                                   () -> -driverXbox.getLeftX())
                                                               .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("CmdA_RobotStartConfig", new CmdA_BridgeOut (ClimberSS));
    NamedCommands.registerCommand("CmdA_Level4", new CmdA_Level4 (CoralElevatorSS));
    NamedCommands.registerCommand("CmdA_SafePosition", new CmdA_SafePosition (CoralElevatorSS));
    NamedCommands.registerCommand("CmdA_Station", new CmdA_Station (CoralElevatorSS));
    NamedCommands.registerCommand("CmdA_CoralOutTake", new CmdA_CoralOutTake (CoralElevatorSS));
    NamedCommands.registerCommand("CmdA_CoralIntake", new CmdA_CoralIntake (CoralElevatorSS));
    NamedCommands.registerCommand("CmdA_DriveToReefPositionR", new CmdA_DriveToReefPositionV3_Relative(drivebase, 2));
    NamedCommands.registerCommand("CmdA_DriveToReefPositionL", new CmdA_DriveToReefPositionV3_Relative(drivebase, 1));
    NamedCommands.registerCommand("CmdA_DriveToFeederRelative", new CmdA_DriveToFeederPosition_Relative(drivebase));
    NamedCommands.registerCommand("CmdA_DriveToReefPositionRTest", new CmdA_DriveToReefPositionV6_ActuallyWinningWindsor(drivebase, 2));
    NamedCommands.registerCommand("CmdA_DriveToReefPositionLTest", new CmdA_DriveToReefPositionV6_ActuallyWinningWindsor(drivebase, 1));
    NamedCommands.registerCommand("CmdA_BackupFromReef", new CmdT_DriveToPoseRelativeBackAwayFromReef(drivebase, 3.0, -12.0, 0, 0));
    NamedCommands.registerCommand("CmdA_DRTest2", new CmdT_DriveToReefPositionV8_Windsor(drivebase, 2));
    NamedCommands.registerCommand("CmdA_DLTest2", new CmdT_DriveToReefPositionV8_Windsor(drivebase, 1));


    DogLog.setOptions(new DogLogOptions().withNtPublish(true));
    DogLog.setOptions(new DogLogOptions().withCaptureDs(true));

    DogLog.setPdh(new PowerDistribution());
    DogLog.setEnabled(true);

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("AutoChooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // (Condition) ? Return-On-True : Return-on-False
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
                                driveFieldOrientedAnglularVelocity :
                                driveFieldOrientedDirectAngleSim);

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
  //   if (DriverStation.isTest())
  //   {
  //     drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
  //     CoralElevatorSS.setDefaultCommand(new CmdT_Def_Elevator(CoralElevatorSS));
  //     // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  //     // driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(3.0, 0.2));
  //     //driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
  //     // driverXbox.back().whileTrue(drivebase.centerModulesCommand());
  //     // driverXbox.leftBumper().onTrue(Commands.none());
  //     // driverXbox.rightBumper().onTrue(Commands.none());
  //     driverXbox.leftTrigger().whileTrue(new Cmd_SeqScoreLeft(CoralElevatorSS, drivebase));
  //     driverXbox.rightTrigger().whileTrue(new Cmd_SeqScoreRight(CoralElevatorSS, drivebase));
  //     driverXbox.a().whileTrue(new CmdT_DriveToFeederPosition(drivebase));
  //     driverXbox.axisGreaterThan(4,0.1).onTrue(new CmdT_DisableAutoAim(drivebase));
  //     driverXbox.axisLessThan(4,-0.1).onTrue(new CmdT_DisableAutoAim(drivebase));
  //     driverXbox.rightBumper().onTrue(new CmdT_EnableAutoAim(drivebase));

        
  //     //********************* */
  //     //SysID Nonsense (Comment out when done)  

  //     //DriveBase
  //     // driverXbox.a().whileTrue(drivebase.sysIdDriveMotorCommand());
  //     // driverXbox.b().whileTrue(drivebase.sysIdAngleMotorCommand());
 
      
  //     //Algae

  //     //driverXbox.a().whileTrue(AlgaeIntakeSS.sysIDPivotAll());
  //    // OperatorJoystick.button(1).whileTrue(new CmdT_IntakeToPosition(AlgaeIntakeSS, 0));
  //  //Algae Intake
  //     //OperatorJoystick.button(2).whileTrue(new CmdT_OuttakeAlgae(AlgaeIntakeSS));
  //     //OperatorJoystick.button(3).whileTrue(new CmdT_IntakeToPosition(AlgaeIntakeSS, 60).
  //     //  andThen(new CmdT_IntakeAlgae(AlgaeIntakeSS)).
  //     //  andThen(new CmdT_IntakeToPosition(AlgaeIntakeSS, 25)));
  //     OperatorJoystick.button(1 ).whileTrue(new Cmd_SeqClimb(drivebase,ClimberSS));
  //     // OperatorJoystick.button(1 ).whileTrue(new CmdT_ClimberIn(ClimberSS));
  //     OperatorJoystick.button(2 ).whileTrue(new CmdT_ClimberIn(ClimberSS));
  //     OperatorJoystick.button(3 ).whileTrue(new CmdT_RunClimberIn(ClimberSS));
  //     OperatorJoystick.button(4).whileTrue(new CmdT_Station(CoralElevatorSS));
  //     OperatorJoystick.button(5).whileTrue(new CmdT_Level4(CoralElevatorSS));
  //     OperatorJoystick.button(6).whileTrue(new CmdT_Level3(CoralElevatorSS));
  //     OperatorJoystick.button(8).whileTrue(new CmdT_Level2(CoralElevatorSS));
  //     OperatorJoystick.button(9).whileTrue(new CmdT_Level1(CoralElevatorSS));
      
  //     // OperatorJoystick.button(11).whileTrue(new Cmd_SeqRemoveHighAlgea(CoralElevatorSS, drivebase));
  //     OperatorJoystick.button(11).whileTrue(new Cmd_SeqRemoveLowAlgea(CoralElevatorSS, drivebase));
  //     OperatorJoystick.button(7).whileTrue(new CmdT_AlgaeL3(CoralElevatorSS));
  //     // OperatorJoystick.button(11).whileTrue(new CmdT_AlgaeTestLow(CoralElevatorSS));
  //     // OperatorJoystick.button(11).whileTrue(new Cmd_SeqRemoveHighAlgea(CoralElevatorSS, drivebase));

      
      
  //     ReefJoystick.button(1).whileTrue(new CmdT_LevelSelect(CoralElevatorSS, 1));
  //     ReefJoystick.button(2).whileTrue(new CmdT_LevelSelect(CoralElevatorSS, 2));
  //     ReefJoystick.button(3).whileTrue(new CmdT_LevelSelect(CoralElevatorSS, 3));
  //     ReefJoystick.button(4).whileTrue(new CmdT_LevelSelect(CoralElevatorSS, 4));
  //     //ReefJoystick.button(5).onTrue(new CmdT_MoveToLevel(CoralElevatorSS));
  //    // ReefJoystick.button(5).whileTrue(new Cmd_SeqScoreLeft(CoralElevatorSS, drivebase));
  //  //  ReefJoystick.button(5).whileTrue(new Cmd_SeqRemoveAlgea(CoralElevatorSS, drivebase));
  //  //ReefJoystick.button(5).whileTrue(new Cmd_SeqScoreLeft(CoralElevatorSS, drivebase));
       
  //    //ReefJoystick.button(6).whileTrue(new Cmd_SeqScoreRight(CoralElevatorSS, drivebase));
  //     //ReefJoystick.button(6).whileTrue(new CmdT_DriveToReefPosition(drivebase,2));
  //     // driverXbox.leftBumper().whileTrue(new CmdT_IntakeToPosition(AlgaeIntakeSS, 30));
  //     // driverXbox.rightBumper().whileTrue(new CmdT_IntakeToPosition(AlgaeIntakeSS, 80));
      
  //     //Elevator

  //     // driverXbox.a().whileTrue(CoralElevatorSS.sysIDElevatorAll());

  //     //driverXbox.leftBumper().whileTrue(new MoveElevatorToPosition(CoralElevatorSS, 0.03));
  //     //driverXbox.rightBumper().whileTrue(new MoveElevatorToPosition(CoralElevatorSS, 0.725));
     
  //     //Elbow

  //     // driverXbox.a().whileTrue(CoralElevatorSS.sysIDElbowAll());
  //     //driverXbox.leftTrigger().whileTrue(new MoveElbowToAngle(CoralElevatorSS, 185));
  //     //driverXbox.rightTrigger().whileTrue(new MoveElbowToAngle(CoralElevatorSS, 5));
 

      
  //     driverXbox.x().whileTrue(new CmdT_CoralIntake(CoralElevatorSS ));
  //     driverXbox.y().whileTrue(new CmdT_CoralOutTake(CoralElevatorSS ));

  //     //OperatorJoystick.button(10).whileTrue(new CmdT_DriveToFeederPosition(drivebase));
      

  //     //Scoring Buttons
  //     //OperatorJoystick.button(14).onTrue(CoralElevatorSubsystem().SetScoreSelection(2));
  //     //OperatorJoystick.button(15).onTrue(new CoralElevatorSubsystem().SetScoreSelection(3));
  //     //OperatorJoystick.button(16).onTrue(new CoralElevatorSubsystem().SetScoreSelection(4));
      

  //   } else
  //   {
  

  /****************Start of Teleop Controls*****************************/
  
  //Default Commands
    

  drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
  CoralElevatorSS.setDefaultCommand(new CmdT_Def_Elevator(CoralElevatorSS));
  // ClimberSS.setDefaultCommand(new CmdT_ClimberIn(ClimberSS));

  //Driver Controller
  driverXbox.leftTrigger().whileTrue(new Cmd_SeqScoreLeft(CoralElevatorSS, drivebase, ReefSurveySS)).onFalse(new CmdT_IsScoring(CoralElevatorSS, false));
  driverXbox.rightTrigger().whileTrue(new Cmd_SeqScoreRight(CoralElevatorSS, drivebase,ReefSurveySS)).onFalse(new CmdT_IsScoring(CoralElevatorSS, false));

  // driverXbox.leftBumper().whileTrue(new Cmd_SeqScoreLeftBak(CoralElevatorSS, drivebase, ReefSurveySS));
  // driverXbox.rightBumper().whileTrue(new Cmd_SeqScoreRightBak(CoralElevatorSS, drivebase,ReefSurveySS));

  //Autoaim
  driverXbox.axisGreaterThan(4,0.1).onTrue(new CmdT_DisableAutoAim(drivebase));
  driverXbox.axisLessThan(4,-0.1).onTrue(new CmdT_DisableAutoAim(drivebase));
  driverXbox.rightBumper().onTrue(new CmdT_EnableAutoAim(drivebase));

  driverXbox.x().whileTrue(new CmdT_CoralIntake(CoralElevatorSS ));
  driverXbox.y().whileTrue(new CmdT_CoralOutTake(CoralElevatorSS ));

  driverXbox.a().whileTrue(new CmdA_DriveToFeederPosition_Relative(drivebase));
  // Button 1: Abort climb
  OperatorJoystick.button(1 ).whileTrue(new CmdT_ClimberIn(ClimberSS));
  // Button 2: Setup Climb
  OperatorJoystick.button(2 ).whileTrue(new Cmd_SeqClimb(drivebase,ClimberSS,CoralElevatorSS));
  // Button 3: Climb
  OperatorJoystick.button(3 ).whileTrue(new CmdT_RunClimberIn(ClimberSS));
  
  // Button 4: Station
  OperatorJoystick.button(4).onTrue(new CmdT_Station(CoralElevatorSS));
  // Button 5: L4
  OperatorJoystick.button(5).onTrue(new CmdT_Level4(CoralElevatorSS));
  // Button 6: L3
  OperatorJoystick.button(6).onTrue(new CmdT_Level3(CoralElevatorSS));

  OperatorJoystick.button(7).whileTrue(new Cmd_SeqClimbV2(drivebase, ClimberSS, CoralElevatorSS));
  // Button 8: L2
  OperatorJoystick.button(8).onTrue(new CmdT_Level2(CoralElevatorSS));
  // Button 9: L1
  OperatorJoystick.button(9).onTrue(new CmdT_Level1(CoralElevatorSS));

  // Button 11: High Algae
  OperatorJoystick.button(12).whileTrue(new Cmd_SeqRemoveLowAlgea(CoralElevatorSS, drivebase));
  // Button 12: High Algae
  OperatorJoystick.button(11).whileTrue(new Cmd_SeqRemoveHighAlgea(CoralElevatorSS, drivebase));


  ReefJoystick.button(1).whileTrue(new CmdT_LevelSelect(CoralElevatorSS, 1));
  ReefJoystick.button(2).whileTrue(new CmdT_LevelSelect(CoralElevatorSS, 2));
  ReefJoystick.button(3).whileTrue(new CmdT_LevelSelect(CoralElevatorSS, 3));
  ReefJoystick.button(4).whileTrue(new CmdT_LevelSelect(CoralElevatorSS, 4));

  


  /****************End of Teleop Controls*****************************/

  // }
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setDriveMode(){
  
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
  
  public void StartUpConfig(){
    DataHighwaySS.StartUp();
    CoralElevatorSS.MatchSetup();
    DogLog.log("RobotContainer/IsSetupComplete", true);
  }

  public boolean IsSetupCompleted(){
    return DataHighwaySS.IsSetupCompleted();
  }
  public void ResetSetup(){
    DataHighwaySS.ResetSetup();
  }

  public void DisableLimelight(){
    LimelightHelpers.setLEDMode_ForceOff("limelight");

  }

  public void EnableLimelight(){
    LimelightHelpers.setLEDMode_ForceOn("limelight");
  }
  public void SetElevatorCoast(){
    CoralElevatorSS.setCoast();
  }
  public void SetElevatorBrake(){
    CoralElevatorSS.setBrake();
  }
}