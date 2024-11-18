// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.Pheonix6Drivetrain;

import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double loopTime = 0.02;

    public static final Mode currentMode = Robot.isReal() ? Mode.REAL : Mode.SIM;

    // public static final Mode currentMode = Mode.REPLAY;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }
  public static class OperatorConstants {
    // Port numbers for driver and operator gamepads. These correspond with the numbers on the USB
    // tab of the DriverStation
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivetrainConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kLeftRearID = 1;
    public static final int kLeftFrontID = 2;
    public static final int kRightRearID = 3;
    public static final int kRightFrontID = 4;

    // Current limit for drivetrain motors
    public static final int kCurrentLimit = 40;

    // Invert motors
    public static final boolean kLeftInverted = true;
    public static final boolean kRightInverted = false;

    // PID Constants
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

  }

  public static final class DriveConstants {
        public static final double MaxSpeedMetPerSec = 6;
        public static final double MaxAngularRateRadiansPerSec = Math.PI * 2; // 2 PI is one full
        // rotation per second
        public static final double deadbandPercent = 0.16;
        public static final double maxAccelerationMetersPerSecSquared = 7.0;

        public static final Pose2d initialPose =
                new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90));

        public static final double anticipationTime = 0.01;
        public static final double minimumAnticipationVelocity = 0.0;

        public static final double pathfindTransformToleranceMeters = 0.1;
        public static final double pathfindRotationToleranceRadians = 0.1;

        public static final double alignToleranceRadians = 0.1;

        public static final double alignmentkPMax = 7.0; // 15
        public static final double alignmentkPMin = 5.0; // 8
        public static final double alignmentkI = 5.5; // 25
        public static final double alignmentkD = 0.0; // 0.4

        public static final double autoAlignmentkP = 5.0;
        public static final double autoAlignmentkI = 5.5;
        public static final double autoAlignmentkD = 0.0;

        public static final double vXkP = 5.0;
        public static final double vXkI = 2.5;
        public static final double vXkD = 0.0;

        public static final double vYkP = 5.0;
        public static final double vYkI = 2.5;
        public static final double vYkD = 0.0;
    }

    public static final class FieldConstants {
      public static final double lengthM = 16.451;
      public static final double widthM = 8.211;

      public static final double midfieldLowThresholdM = 5.87;
      public static final double midfieldHighThresholdM = 10.72;

      public static final Rotation2d ampHeading = new Rotation2d(-Math.PI / 2);

      public static final Rotation2d blueUpHeading = Rotation2d.fromRadians(0.0);
      public static final Rotation2d blueDownHeading = Rotation2d.fromRadians(Math.PI);
      public static final Rotation2d blueLeftHeading = Rotation2d.fromRadians(Math.PI / 2.0);
      public static final Rotation2d blueRightHeading = Rotation2d.fromRadians(-Math.PI / 2.0);

      public static final Rotation2d redUpHeading = Rotation2d.fromRadians(Math.PI);
      public static final Rotation2d redDownHeading = Rotation2d.fromRadians(0.0);
      public static final Rotation2d redLeftHeading = Rotation2d.fromRadians(-Math.PI / 2.0);
      public static final Rotation2d redRightHeading = Rotation2d.fromRadians(Math.PI / 2.0);

      public static final Rotation2d redSourceHeading =
              new Rotation2d(Math.PI * 4 / 3); // 60 degrees
      public static final Rotation2d blueSourceHeading =
              new Rotation2d(Math.PI * 5 / 3); // 120 degrees

      public static final Translation2d fieldToRedSpeaker =
              new Translation2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42));

      public static final Translation2d fieldToBlueSpeaker =
              new Translation2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42));

      public static final Pose2d robotAgainstBlueSpeaker =
              new Pose2d(1.39, 5.56, Rotation2d.fromDegrees(180));

      public static final Pose2d robotAgainstRedSpeaker =
              new Pose2d(15.19, 5.56, Rotation2d.fromDegrees(0));

      public static final Pose2d robotAgainstBlueSpeakerRight =
              new Pose2d(0.7, 4.38, Rotation2d.fromDegrees(120));

      public static final Pose2d robotAgainstRedSpeakerRight =
              new Pose2d(15.83, 6.73, Rotation2d.fromDegrees(-60));

      public static final Pose2d robotAgainstBlueSpeakerLeft =
              new Pose2d(0.7, 6.73, Rotation2d.fromDegrees(-120));

      public static final Pose2d robotAgainstRedSpeakerLeft =
              new Pose2d(15.83, 4.38, Rotation2d.fromDegrees(60));

      public static final Pose2d robotAgainstBluePodium =
              new Pose2d(2.57, 4.09, Rotation2d.fromDegrees(180));

      public static final Pose2d robotAgainstRedPodium =
              new Pose2d(13.93, 4.09, Rotation2d.fromDegrees(0));

      public static final Pose2d robotAgainstBlueAmpZone =
              new Pose2d(2.85, 7.68, Rotation2d.fromDegrees(-90));

      public static final Pose2d robotAgainstRedAmpZone =
              new Pose2d(13.74, 7.68, Rotation2d.fromDegrees(-90));

      public static final Pose2d robotAgainstBlueSource =
              new Pose2d(14.82, 0.69, Rotation2d.fromDegrees(-60));

      public static final Pose2d robotAgainstRedSource =
              new Pose2d(1.63, 0.69, Rotation2d.fromDegrees(60));

      // TODO: Find actual coordinates of shop source
      public static final Pose2d robotAgainstShopSource =
              new Pose2d(8.30, 6.80, Rotation2d.fromDegrees(60));
  }

  public static final class TunerConstants {
        // Both sets of gains need to be tuned to your individual robot.

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        public static final Slot0Configs steerGains =
                new Slot0Configs()
                        .withKP(150)
                        .withKI(50)
                        .withKD(0.2)
                        .withKS(0.25)
                        .withKV(1.5)
                        .withKA(0);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        public static final Slot0Configs driveGains =
                new Slot0Configs()
                        .withKP(0)
                        .withKI(0.02)
                        .withKD(0)
                        .withKS(0.26)
                        .withKV(0.12)
                        .withKA(0.01);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType steerClosedLoopOutput =
                ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType driveClosedLoopOutput =
                ClosedLoopOutputType.Voltage;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double kSlipCurrentA = 80;

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        public static final double kSpeedAt12VoltsMps = 5.02; // 5.21 OR 5.02

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285716;

        private static final double kDriveGearRatio = 6.122448979591837;
        private static final double kSteerGearRatio = 21.428571428571427;
        private static final double kWheelRadiusInches = 1.965;

        private static final boolean kSteerMotorReversed = true;
        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        private static final String kCANbusName = "Canivore";
        private static final int kPigeonId = 1;

        // These are only used for simulation
        private static final double kSteerInertia = 0.00001;
        private static final double kDriveInertia = 0.001;
        // Simulated voltage necessary to overcome friction
        private static final double kSteerFrictionVoltage = 0.25;
        private static final double kDriveFrictionVoltage = 0.25;

        private static final SwerveDrivetrainConstants DrivetrainConstants =
                new SwerveDrivetrainConstants()
                        .withPigeon2Id(kPigeonId)
                        .withCANbusName(kCANbusName);

        private static final SwerveModuleConstantsFactory ConstantCreator =
                new SwerveModuleConstantsFactory()
                        .withDriveMotorGearRatio(kDriveGearRatio)
                        .withSteerMotorGearRatio(kSteerGearRatio)
                        .withWheelRadius(kWheelRadiusInches)
                        .withSlipCurrent(kSlipCurrentA)
                        .withSteerMotorGains(steerGains)
                        .withDriveMotorGains(driveGains)
                        .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                        .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                        .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                        .withSteerInertia(kSteerInertia)
                        .withDriveInertia(kDriveInertia)
                        .withSteerFrictionVoltage(kSteerFrictionVoltage)
                        .withDriveFrictionVoltage(kDriveFrictionVoltage)
                        .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                        .withCouplingGearRatio(kCoupleRatio)
                        .withSteerMotorInverted(kSteerMotorReversed);

        // Front Left
        private static final int kBackRightDriveMotorId = 2;
        private static final int kBackRightSteerMotorId = 1;
        private static final int kBackRightEncoderId = 1;
        private static final double kBackRightEncoderOffset = 0.3486328125;

        private static final double kBackRightXPosInches = 10.375;
        private static final double kBackRightYPosInches = 10.375;

        // Front Right
        private static final int kBackLeftDriveMotorId = 4;
        private static final int kBackLeftSteerMotorId = 3;
        private static final int kBackLeftEncoderId = 2;
        private static final double kBackLeftEncoderOffset = 0.096435546875;

        private static final double kBackLeftXPosInches = 10.375;
        private static final double kBackLeftYPosInches = -10.375;

        // Back Left
        private static final int kFrontRightDriveMotorId = 8;
        private static final int kFrontRightSteerMotorId = 7;
        private static final int kFrontRightEncoderId = 4;
        private static final double kFrontRightEncoderOffset = 0.130859375;

        private static final double kFrontRightXPosInches = -10.375;
        private static final double kFrontRightYPosInches = 10.375;

        // Back Right
        private static final int kFrontLeftDriveMotorId = 6;
        private static final int kFrontLeftSteerMotorId = 5;
        private static final int kFrontLeftEncoderId = 3;
        private static final double kFrontLeftEncoderOffset = -0.372802734375;

        private static final double kFrontLeftXPosInches = -10.375;
        private static final double kFrontLeftYPosInches = -10.375;

        public static final double kModuleRadiusMeters =
                Units.inchesToMeters(Math.hypot(kFrontLeftXPosInches, kFrontLeftYPosInches));

        private static final SwerveModuleConstants FrontLeft =
                ConstantCreator.createModuleConstants(
                        kFrontLeftSteerMotorId,
                        kFrontLeftDriveMotorId,
                        kFrontLeftEncoderId,
                        kFrontLeftEncoderOffset,
                        Units.inchesToMeters(kFrontLeftXPosInches),
                        Units.inchesToMeters(kFrontLeftYPosInches),
                        kInvertLeftSide);
        private static final SwerveModuleConstants FrontRight =
                ConstantCreator.createModuleConstants(
                        kFrontRightSteerMotorId,
                        kFrontRightDriveMotorId,
                        kFrontRightEncoderId,
                        kFrontRightEncoderOffset,
                        Units.inchesToMeters(kFrontRightXPosInches),
                        Units.inchesToMeters(kFrontRightYPosInches),
                        kInvertRightSide);
        private static final SwerveModuleConstants BackLeft =
                ConstantCreator.createModuleConstants(
                        kBackLeftSteerMotorId,
                        kBackLeftDriveMotorId,
                        kBackLeftEncoderId,
                        kBackLeftEncoderOffset,
                        Units.inchesToMeters(kBackLeftXPosInches),
                        Units.inchesToMeters(kBackLeftYPosInches),
                        kInvertLeftSide);
        private static final SwerveModuleConstants BackRight =
                ConstantCreator.createModuleConstants(
                        kBackRightSteerMotorId,
                        kBackRightDriveMotorId,
                        kBackRightEncoderId,
                        kBackRightEncoderOffset,
                        Units.inchesToMeters(kBackRightXPosInches),
                        Units.inchesToMeters(kBackRightYPosInches),
                        kInvertRightSide);

        public static final SwerveDriveKinematics kinematics =
                new SwerveDriveKinematics(
                        new Translation2d(FrontLeft.LocationX, FrontLeft.LocationY),
                        new Translation2d(FrontLeft.LocationX, FrontRight.LocationY),
                        new Translation2d(BackLeft.LocationX, BackLeft.LocationY),
                        new Translation2d(BackRight.LocationX, BackRight.LocationY));

        public static final Pheonix6Drivetrain DriveTrain =
                new Pheonix6Drivetrain(
                        DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
    }
}
