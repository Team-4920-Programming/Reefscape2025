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
import frc.robot.commands.Cmd_SeqRemoveAlgea;
import frc.robot.commands.Cmd_SeqScoreLeft;
import frc.robot.commands.Cmd_SeqScoreRight;
import frc.robot.commands.AlgaeIntake.TeleOp.CmdT_IntakeAlgae;
import frc.robot.commands.AlgaeIntake.TeleOp.CmdT_OuttakeAlgae;
import frc.robot.commands.AlgaeIntake.TeleOp.CmdT_IntakeToPosition;
import frc.robot.commands.swervedrive.TeleOp.CmdT_DriveToFeederPosition;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.AlgaeIntake.AlgaeIntakeSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.CoralElevator.CoralElevatorSubsystem;
import frc.robot.subsystems.DataHighway.DataHighwaySubsystem;
import frc.robot.subsystems.ReefSurvey.ReefSurveySubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import frc.robot.commands.Elevator.TeleOp.*;
import frc.robot.commands.swervedrive.TeleOp.CmdT_DriveToReefPosition;
import swervelib.SwerveDriveTest;
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
  private final AlgaeIntakeSubsystem AlgaeIntakeSS = new AlgaeIntakeSubsystem();
  private final DataHighwaySubsystem DataHighwaySS = new DataHighwaySubsystem(drivebase,CoralElevatorSS);
  private final ReefSurveySubsystem ReefSurveySS = new ReefSurveySubsystem();
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

    if (Robot.isSimulation()) 
      DogLog.setOptions(new DogLogOptions().withNtPublish(true));
    else
      DogLog.setOptions(new DogLogOptions().withNtPublish(true));

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
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
      CoralElevatorSS.setDefaultCommand(new CmdT_Def_Elevator(CoralElevatorSS));
      // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(3.0, 0.2));
      //driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      // driverXbox.leftBumper().onTrue(Commands.none());
      // driverXbox.rightBumper().onTrue(Commands.none());
        driverXbox.leftTrigger().whileTrue(new Cmd_SeqScoreLeft(CoralElevatorSS, drivebase));
        driverXbox.rightTrigger().whileTrue(new Cmd_SeqScoreRight(CoralElevatorSS, drivebase));
        driverXbox.a().whileTrue(new CmdT_DriveToFeederPosition(drivebase));
        
      //********************* */
      //SysID Nonsense (Comment out when done)

      //DriveBase
      // driverXbox.a().whileTrue(drivebase.sysIdDriveMotorCommand());
      // driverXbox.b().whileTrue(drivebase.sysIdAngleMotorCommand());
 
      
      //Algae

      //driverXbox.a().whileTrue(AlgaeIntakeSS.sysIDPivotAll());
      OperatorJoystick.button(1).whileTrue(new CmdT_IntakeToPosition(AlgaeIntakeSS, 0));
   //Algae Intake
      OperatorJoystick.button(2).whileTrue(new CmdT_OuttakeAlgae(AlgaeIntakeSS));
      OperatorJoystick.button(3).whileTrue(new CmdT_IntakeToPosition(AlgaeIntakeSS, 60).
        andThen(new CmdT_IntakeAlgae(AlgaeIntakeSS)).
        andThen(new CmdT_IntakeToPosition(AlgaeIntakeSS, 25)));
      OperatorJoystick.button(4).whileTrue(new CmdT_Level2(CoralElevatorSS));
      OperatorJoystick.button(5).whileTrue(new CmdT_Level3(CoralElevatorSS));
      OperatorJoystick.button(6).whileTrue(new CmdT_Level4(CoralElevatorSS));
      OperatorJoystick.button(7).whileTrue(new CmdT_Station(CoralElevatorSS));
      OperatorJoystick.button(8).whileTrue(new CmdT_Level1(CoralElevatorSS));
      OperatorJoystick.button(9).whileTrue(new CmdT_ArmNeutral(CoralElevatorSS));
      OperatorJoystick.button(13).whileTrue(new CmdT_AlgaeLowApproach(CoralElevatorSS));
      OperatorJoystick.button(14).whileTrue(new CmdT_AlgaeLowRetract(CoralElevatorSS));
      OperatorJoystick.button(15).whileTrue(new Cmd_SeqRemoveAlgea(CoralElevatorSS, drivebase));
      
      ReefJoystick.button(1).whileTrue(new CmdT_LevelSelect(CoralElevatorSS, 1));
      ReefJoystick.button(2).whileTrue(new CmdT_LevelSelect(CoralElevatorSS, 2));
      ReefJoystick.button(3).whileTrue(new CmdT_LevelSelect(CoralElevatorSS, 3));
      ReefJoystick.button(4).whileTrue(new CmdT_LevelSelect(CoralElevatorSS, 4));
      //ReefJoystick.button(5).onTrue(new CmdT_MoveToLevel(CoralElevatorSS));
     // ReefJoystick.button(5).whileTrue(new Cmd_SeqScoreLeft(CoralElevatorSS, drivebase));
   //  ReefJoystick.button(5).whileTrue(new Cmd_SeqRemoveAlgea(CoralElevatorSS, drivebase));
   ReefJoystick.button(5).whileTrue(new Cmd_SeqScoreLeft(CoralElevatorSS, drivebase));
       
     ReefJoystick.button(6).whileTrue(new Cmd_SeqScoreRight(CoralElevatorSS, drivebase));
      //ReefJoystick.button(6).whileTrue(new CmdT_DriveToReefPosition(drivebase,2));
      // driverXbox.leftBumper().whileTrue(new CmdT_IntakeToPosition(AlgaeIntakeSS, 30));
      // driverXbox.rightBumper().whileTrue(new CmdT_IntakeToPosition(AlgaeIntakeSS, 80));
      
      //Elevator

      // driverXbox.a().whileTrue(CoralElevatorSS.sysIDElevatorAll());

      //driverXbox.leftBumper().whileTrue(new MoveElevatorToPosition(CoralElevatorSS, 0.03));
      //driverXbox.rightBumper().whileTrue(new MoveElevatorToPosition(CoralElevatorSS, 0.725));
     
      //Elbow

      // driverXbox.a().whileTrue(CoralElevatorSS.sysIDElbowAll());
      //driverXbox.leftTrigger().whileTrue(new MoveElbowToAngle(CoralElevatorSS, 185));
      //driverXbox.rightTrigger().whileTrue(new MoveElbowToAngle(CoralElevatorSS, 5));
 

      //Wrist

      // driverXbox.a().whileTrue(CoralElevatorSS.sysIDWristAll());
      //driverXbox.a().whileTrue(new MoveWristToAngle(CoralElevatorSS, 90));
     // driverXbox.b().whileTrue(new MoveWristToAngle(CoralElevatorSS, 5));
      driverXbox.x().whileTrue(new CmdT_CoralIntake(CoralElevatorSS ));
      driverXbox.y().whileTrue(new CmdT_CoralOutTake(CoralElevatorSS ));
      //Climber

      // driverXbox.a().whileTrue(ClimberSS.sysIDClimberAll());

      //Drive Subssystem
      OperatorJoystick.button(10).whileTrue(new CmdT_DriveToFeederPosition(drivebase));
      

      //Scoring Buttons
      //OperatorJoystick.button(14).onTrue(CoralElevatorSubsystem().SetScoreSelection(2));
      //OperatorJoystick.button(15).onTrue(new CoralElevatorSubsystem().SetScoreSelection(3));
      //OperatorJoystick.button(16).onTrue(new CoralElevatorSubsystem().SetScoreSelection(4));
      

    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      //driverXbox.y().whileTrue(drivebase.aimAtSpeaker(2));
      
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.rightBumper().onTrue(Commands.none());
    }

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
}