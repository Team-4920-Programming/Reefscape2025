// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.io.ObjectInputFilter.Status;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (115) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.63);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = -0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class PIDs {
    public static class Climber {
      public static final double kp = 0.1;
      public static final double ki = 0.1;
      public static final double kd = 0.1;
    
      public static final double ks = 0.1;
      public static final double kg = 0.1;
      public static final double kv = 0.1;

      public static final double maxVelocity = 0.01;
    }
    public static class CoralElevator {
      public static class Elevator{
        public static final double kp = 3.0;//3.5
        public static final double ki = 0.0;
        public static final double kd = 0.01;

        public static final double ks = 0.6016;
        public static final double kv = 11.991;
        public static final double ka = 1.946;
        public static final double kg = 0.57052;

        public static double maxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
        public static double maxAcceleration = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);
        public static final double pulleyRadius = Units.inchesToMeters(1.0);
        public static final double elevatorReduction = 16;
      }
      public static class Elbow{
        public static final double kp = 0.03;
        public static final double ki = 0.0;
        public static final double kd = 0.0;

        public static final double ks = 0.1;
        public static final double kg = 0.1;
        public static final double kv = 0.1;
        
        public static final double maxVelocity = 0.01;
      }
      public static class Wrist{
        public static final double kp = 0.01;
        public static final double ki = 0;
        public static final double kd = 0;
        
        public static final double ks = 0.1;
        public static final double kg = 0.1;
        public static final double kv = 0.1;
        
        public static final double maxVelocity = 0.01;
      }
      public static class RightFlap{
        public static final double kp = 0.01;
        public static final double ki = 0;
        public static final double kd = 0;
      }
      public static class LeftFlap{
        public static final double kp = 0.01;
        public static final double ki = 0;
        public static final double kd = 0;
      }
      
    }
    public static class AlgaeIntake {
      public static final double kp = 0.01;
      public static final double ki = 0;
      public static final double kd = 0;
      
      public static final double ks = 0.1;
      public static final double kg = 0.1;
      public static final double kv = 0.1;
      
      public static final double maxVelocity = 0.01;
    }
  }

  public static class CanIDs {
    public static class DriveMotors {
      public static final int FL = 3;
      public static final int FR = 5;
      public static final int BR = 7;
      public static final int BL = 9;
    }

    public static class  RotationMotors {
      public static final int FL = 2;
      public static final int FR = 4;
      public static final int BR = 6;
      public static final int BL = 8;
    }

    public static class CoralElevator {
      public static final int ElevatorStage = 10;
      public static final int Elbow = 11;
      public static final int Wrist = 12;
      public static final int CoralIntake = 13;
      public static final int CoralFlapRight = 21;
      public static final int CoralFlapLeft = 20;
    }

    public static class MitoCANdria {
      public static final int Red = 14;
      public static final int BlueGray = 15;
    }

    public static class Climber {
      public static final int Winch = 17;
      public static final int Pivot = 19;
    }

    public static class Sensor {
    public static final int LaserCAN = 16;
    public static final int Pigeon = 18;
    }
  }

  public static class DIO {
    public static class CoralElevator {
      public static final int DownStop = 1;
      public static final int UpStop = 2;
      public static final int CoralPresence = 3;
      public static final int UltrasonicTrigger = 5;
      public static final int UltrasonicEcho = 6;

    }

    public static class AlgaeIntake {
      public static final int AlgaePresence = 0;
    }

    public static class Climber {
      public static final int CagePresence = 4;
    }
  }

  public static class AbsoluteEncoderID {
    public static class Swerve {
      public static final int FLRot = CanIDs.RotationMotors.FL;
      public static final int FRRot = CanIDs.RotationMotors.FR;
      public static final int BRRot = CanIDs.RotationMotors.BR;
      public static final int BLRot = CanIDs.RotationMotors.BL;
    }

    public static class CoralElevator {
      public static final int ElbowAngle = CanIDs.CoralElevator.Elbow;
      public static final int WristAngle = CanIDs.CoralElevator.Wrist;

    }

    //public static class AlgaeIntake {
    //  public static final int AlgaeAngle  = CanIDs.AlgaeIntake.Pivot;
    //}//

    public static class Climber {
      public static final int ClimberAngle  = CanIDs.Climber.Winch;
    }
  }

  public static class Vision {
    public static class AprilTag {
      public static class Gray {
        public static final int Cam1 = 1;
        public static final int Cam2 = 2;
      }

      public static class Red {
        public static final int Cam3 = 3;
        public static final int Cam4 = 4;
      }

      public static class Blue {
        public static final int Cam5 = 5;
        public static final int Cam6 = 6;
      }
    }

    public static class CoralElevator {
      public static  final int CoralDetector = 7;
    }
  }
  public static class RobotPositions{
    public static class CoralStation {
      public static final double wrist = 151; //degrees //-202
      public static final double elbow =2; //degrees
      public static final double height = 0.15; //meters
  }
  public static class Level1 {
    public static final double wrist = 30; //degrees
    public static final double elbow =1; //degrees
    public static final double height = 0.05; //meters
}
    public static class Level2 {
        public static final double wrist = -35; //degrees
        public static final double elbow =10; //degrees
        public static final double height = 0.5; //meters
    }
    public static class Level3 {
      public static final double wrist = -35; //degrees
      public static final double elbow =170; //degrees
      public static final double height = 0.12; //meters
  }
  public static class Level4 {
    public static final double wrist = -38; //degrees
    public static final double elbow =185; //degrees
    public static final double height = 0.725; //meters
}
  public static class Level4_Far {
    public static final double wrist = -25; //degrees
    public static final double elbow =160; //degrees
    public static final double height = 0.725; //meters
  }
public static class AlgaeLowApproach {
  public static final double wrist = 0; //degrees
  public static final double elbow =90; //degrees
  public static final double height = 0.35; //meters
}
public static class AlgaeMidApproach {
  public static final double wrist = -35; //degrees
  public static final double elbow =10; //degrees
  public static final double height = 0.55; //meters
}
public static class AlgaeLowRetract {
  public static final double wrist = -50; //degrees
  public static final double elbow =90; //degrees
  public static final double height = 0.35; //meters
}
public static class AlgaeMidRetract {
  public static final double wrist = -35; //degrees
  public static final double elbow =10; //degrees
  public static final double height = 0.5; //meters
}
public static class ArmNeutral {
  public static final double wrist = 67; //degrees
  public static final double elbow = 0.5; //degrees
  public static final double height = 0.05; //meters
}
  }
  public static class RobotLimits {
    public static class Elbow {
      public static final double minAngle = -10;
      public static final double maxAngle = 190;
    }

    public static class Wrist {
      public static final double minAngle = -95;
      public static final double maxAngle = 100;
    }

    public static class Elevator {
      public static final double minHeight = 0;
      public static final double maxHeight = 0.75;
      public static final double offset = 0.04;
      public static final double elevatorRedZoneUpperHeight = 0.55;
      public static final double elevatorRedZoneLowerHeight = 0.36;
      
    }

    public static class Climber {
      public static final double minAngle = 20; // change this to correct limits
      public static final double maxAngle = 60; // change this to correct limits

    }
    public static class AlgaeIntake {
      public static final double minAngle = 0;
      public static final double maxAngle = 45;
    }
  }

  public static class RobotMotionLimits {
    public static class Elbow {
      public static final double minAngle = -5;
      public static final double maxAngle = 190;
    }

    public static class Wrist {
      public static final double minAngle = -90;
      public static final double maxAngle = 180;
    }

    public static class Elevator {
      public static final double minHeight = 0;
      public static final double maxHeight = 0.75;
      
    }
    
  }
  public static class Vision4920 {
    public static final String kGreyFeederCam = "GreyFeederCam";
    public static final String kGreyReefCam = "GreyReefCam";
    public static final String kRedReefCam = "RedReefCam";
    public static final String kRedGeneralCam = "RedGeneralCam";
    public static final String kBlueGeneralCam = "BlueGeneralCam";
    public static final String kBlueFrontCam = "BlueFrontCam";
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.

    // positive x to the left, positive y up
    public static final Transform3d kRobotToGreyFeederCam =
            new Transform3d(new Translation3d(Units.inchesToMeters(-3.25), Units.inchesToMeters(-13), Units.inchesToMeters(38.25)), 
            new Rotation3d(0, Units.degreesToRadians(314), Units.degreesToRadians(180))); //
  public static final Transform3d kRobotToGreyReefCam =
            new Transform3d(new Translation3d(Units.inchesToMeters(11.25), Units.inchesToMeters(-14), Units.inchesToMeters(38.75)), 
            new Rotation3d(0, Units.degreesToRadians(37), 0)); // 0.48
  public static final Transform3d kRobotToRedReefCam =
            new Transform3d(new Translation3d(Units.inchesToMeters(11.25), Units.inchesToMeters(14), Units.inchesToMeters(38.75)), 
            new Rotation3d(0, Units.degreesToRadians(37), 0)); // 0.48
  public 
  static final Transform3d kRobotToRedGeneralCam =
            new Transform3d(new Translation3d(Units.inchesToMeters(3.25), Units.inchesToMeters(14.25), Units.inchesToMeters(36.5)), 
            new Rotation3d(0, Units.degreesToRadians(350), Units.degreesToRadians(180))); // 0.48
  
  public static final Transform3d kRobotToBlueGeneralCam =
            new Transform3d(new Translation3d(Units.inchesToMeters(0.75), Units.inchesToMeters(-14.25), Units.inchesToMeters(40.25)), 
            new Rotation3d(Units.degreesToRadians(13), Units.degreesToRadians(0), Units.degreesToRadians(270))); // 0.48
   
            public static final Transform3d kRobotToBlueFrontCam =
            new Transform3d(new Translation3d(Units.inchesToMeters(12.5), Units.inchesToMeters(-13), Units.inchesToMeters(34.5)), 
            new Rotation3d(0, Units.degreesToRadians(350), 0)); // 0.48
  
  public static final Transform3d ROBOT_TO_CAMERA_Front = kRobotToGreyFeederCam.inverse();
  public static final Transform3d ROBOT_TO_CAMERA_Rear = kRobotToGreyReefCam.inverse();
  public static final Transform3d ROBOT_TO_CAMERA_Right = kRobotToRedReefCam .inverse();
  public static final Transform3d ROBOT_TO_CAMERA_Left = kRobotToRedGeneralCam.inverse();
  
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
   

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(.5, .5, 1);

// from Hemlock5712
    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
    public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
    public static final double NOISY_DISTANCE_METERS = 2.5;
    public static final double DISTANCE_WEIGHT = 7;
    public static final int TAG_PRESENCE_WEIGHT = 10;

    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
     * meters.
     */
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = VecBuilder
        .fill(
            // if these numbers are less than one, multiplying will do bad things
            1, // x
            1, // y
            1 * Math.PI // theta
        );

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians.
     */
   



    public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = VecBuilder
        .fill(
            // if these numbers are less than one, multiplying will do bad things
            .1, // x
            .1, // y
            .1);

}
}



     
 
