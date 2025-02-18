// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.io.ObjectInputFilter.Status;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

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

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
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
        public static final double kp = 2.5;
        public static final double ki = 0.0;
        public static final double kd = 0.0;

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
        public static final double kp = 0.1;
        public static final double ki = 0.1;
        public static final double kd = 0.1;

        public static final double ks = 0.1;
        public static final double kg = 0.1;
        public static final double kv = 0.1;
        
        public static final double maxVelocity = 0.01;
      }
      public static class Wrist{
        public static final double kp = 0.1;
        public static final double ki = 0.1;
        public static final double kd = 0.1;
        
        public static final double ks = 0.1;
        public static final double kg = 0.1;
        public static final double kv = 0.1;
        
        public static final double maxVelocity = 0.01;
      }
    }
    public static class AlgaeIntake {
      public static final double kp = 0.1;
      public static final double ki = 0.1;
      public static final double kd = 0.1;
      
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
    }

    public static class AlgaeIntake {
      public static final int Pivot = 14;
      public static final int BallIntake = 15;
    }

    public static class Climber {
      public static final int Winch = 17;
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

    public static class AlgaeIntake {
      public static final int AlgaeAngle  = CanIDs.AlgaeIntake.Pivot;
    }

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

  public static class RobotLimits {
    public static class Elbow {
      public static final double minAngle = 0;
      public static final double maxAngle = 360;
      public static final double offset = 0;
    }

    public static class Wrist {
      public static final double minAngle = 0;
      public static final double maxAngle = 360;
      public static final double offset = 0;
    }

    public static class Elevator {
      public static final double offset = 0.03;
      public static final double minHeight = 0+offset;
      public static final double maxHeight = 360+offset;
      
    }

    public static class Climber {
      public static final double minAngle = 0;
      public static final double maxAngle = 360;
      public static final double offset = 0;
    }
    public static class AlgaeIntake {
      public static final double minAngle = 0;
      public static final double maxAngle = 60;
      public static final double offset = 0;
    }
  }
}



     
 
