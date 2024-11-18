package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;


import java.util.Optional;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class Pheonix6Drivetrain extends SwerveDrivetrain implements Subsystem {
    //
    public static Pose2d transformToPose(Transform2d transform) {
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }

    private double vx, vy, omega = 0.0;
    private boolean fieldCentric = true;

    private double alignError = 0.0;

    private double tuningVolts = 0.0;

    public enum AlignTarget {
        NONE,
        AMP,
        UP,
        DOWN,
        LEFT,
        RIGHT
    }

    public enum AlignState {
        MANUAL,
        ALIGNING,
        POSE_TARGET,
        SYS_ID_DRIVE,
        SYS_ID_ROTATION
    }

    private AlignTarget alignTarget = AlignTarget.NONE;
    private AlignState alignState = AlignState.MANUAL;

    private double alignDirection = 0.0;

    private Pose2d targetTightPose;



    private Supplier<Pose2d> getFieldToRobot = () -> new Pose2d();

    private Supplier<Translation2d> getRobotVelocity = () -> new Translation2d();

    

    private PIDController vXController =
            new PIDController(DriveConstants.vXkP, DriveConstants.vXkI, DriveConstants.vXkD);

    private PIDController vYController =
            new PIDController(DriveConstants.vYkP, DriveConstants.vYkI, DriveConstants.vYkD);

    private PIDController thetaController =
            new PIDController(
                    DriveConstants.alignmentkPMax,
                    DriveConstants.alignmentkI,
                    DriveConstants.alignmentkD);

    private double alignkPMax = DriveConstants.alignmentkPMax;
    private double alignkPMin = DriveConstants.alignmentkPMin;

    private SwerveRequest.FieldCentric driveFieldCentric = new SwerveRequest.FieldCentric();
    private SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric();
    private SwerveRequest.SysIdSwerveTranslation driveSysId =
            new SwerveRequest.SysIdSwerveTranslation();
    private SwerveRequest.SysIdSwerveSteerGains rotationSysId =
            new SwerveRequest.SysIdSwerveSteerGains();
    // private SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private static final double kSimLoopPeriod = 0.02; // Original: 5 ms
    private Notifier simNotifier = null;
    private double lastSimTime;

    private String lastCommandedPath = "";
    private Command pathfindCommand = null;

    private Pose2d pathfindPose = new Pose2d();

    private ChassisSpeeds stopSpeeds = new ChassisSpeeds(0, 0, 0);

    private Rotation2d desiredHeading = new Rotation2d();

    private boolean demo = false;

    public Pheonix6Drivetrain(
            SwerveDrivetrainConstants driveTrainConstants,
            double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Constants.currentMode == Constants.Mode.SIM) {
            startSimThread();
        }

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        /*
         * Since we're extending `SwerveDrivetrain`, we can't extend `SubsystemBase`, we can only
         * implement `Subsystem`. Because of this, we have to register ourself manaully.
         *
         * In short, never trust the Command Scheduler.
         */
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public Pheonix6Drivetrain(
            SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Constants.currentMode == Constants.Mode.SIM) {
            startSimThread();
        }

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        /*
         * Since we're extending `SwerveDrivetrain`, we can't extend `SubsystemBase`, we can only
         * implement `Subsystem`. Because of this, we have to register ourself manaully.
         *
         * In short, never trust the Command Scheduler.
         */
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void setAlignGains(double kPMax, double kPMin, double kI, double kD) {
        alignkPMax = kPMax;
        alignkPMin = kPMin;
        thetaController.setI(kI);
        thetaController.setD(kD);
    }

    public void setPoseSupplier(Supplier<Pose2d> getFieldToRobot) {
        this.getFieldToRobot = getFieldToRobot;
    }

    public void setAlignTarget(AlignTarget alignTarget) {
        this.alignTarget = alignTarget;
    }

    public void setAlignState(AlignState state) {
        this.alignState = state;
    }

    public AlignState getAlignState() {
        return alignState;
    }

    public AlignTarget getAlignTarget() {
        return alignTarget;
    }

    public void setVelocitySupplier(Supplier<Translation2d> getRobotVelocity) {
        this.getRobotVelocity = getRobotVelocity;
    }

    public void setDemo(boolean demo) {
        this.demo = demo;
    }

    


    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier =
                new Notifier(
                        () -> {
                            final double currentTime = Utils.getCurrentTimeSeconds();
                            double deltaTime = currentTime - lastSimTime;
                            lastSimTime = currentTime;

                            // System.out.println(System.currentTimeMillis());

                            /* use the measured time delta, get battery voltage from WPILib */
                            updateSimState(deltaTime, RobotController.getBatteryVoltage());
                        });
        simNotifier.startPeriodic(kSimLoopPeriod);
    }

 
    public void setAlignDirection(double direction, boolean allianceSpecific) {
        if (allianceSpecific) {
            if (!DriverStation.getAlliance().isPresent()) {
                alignDirection = direction;
            } else {
                switch (DriverStation.getAlliance().get()) {
                    case Blue:
                        alignDirection = direction;
                        break;
                    case Red:
                        alignDirection = direction + Math.PI;
                        break;
                }
            }
        } else {
            alignDirection = direction;
        }
    }

    public void setTuningVolts(double tuningVolts) {
        this.tuningVolts = tuningVolts;
    }

    private void controlDrivetrain() {
        Pose2d pose = getFieldToRobot.get();
        desiredHeading = pose.getRotation();
        if (alignState == AlignState.ALIGNING && !demo) {
            switch (alignTarget) {
                
                case UP:
                    if (!DriverStation.getAlliance().isPresent()) {
                        desiredHeading = FieldConstants.blueUpHeading;
                    } else {
                        switch (DriverStation.getAlliance().get()) {
                            case Blue:
                                desiredHeading = FieldConstants.blueUpHeading;
                                break;
                            case Red:
                                desiredHeading = FieldConstants.redUpHeading;
                                break;
                        }
                    }
                    break;
                case DOWN:
                    if (!DriverStation.getAlliance().isPresent()) {
                        desiredHeading = FieldConstants.blueDownHeading;
                    } else {
                        switch (DriverStation.getAlliance().get()) {
                            case Blue:
                                desiredHeading = FieldConstants.blueDownHeading;
                                break;
                            case Red:
                                desiredHeading = FieldConstants.redDownHeading;
                                break;
                        }
                    }
                    break;
                case LEFT:
                    if (!DriverStation.getAlliance().isPresent()) {
                        desiredHeading = FieldConstants.blueLeftHeading;
                    } else {
                        switch (DriverStation.getAlliance().get()) {
                            case Blue:
                                desiredHeading = FieldConstants.blueLeftHeading;
                                break;
                            case Red:
                                desiredHeading = FieldConstants.redLeftHeading;
                                break;
                        }
                    }
                    break;
                case RIGHT:
                    if (!DriverStation.getAlliance().isPresent()) {
                        desiredHeading = FieldConstants.blueRightHeading;
                    } else {
                        switch (DriverStation.getAlliance().get()) {
                            case Blue:
                                desiredHeading = FieldConstants.blueRightHeading;
                                break;
                            case Red:
                                desiredHeading = FieldConstants.redRightHeading;
                                break;
                        }
                    }
                    break;
                case NONE:
                    break;
        
            }

            alignError = thetaController.getPositionError();

            double t = Math.abs(alignError) / (Math.PI / 4);
            double minkP = alignkPMin;
            double maxkP = alignkPMax;
            double rotationkP = minkP * (1.0 - t) + t * maxkP;

            // double rotationkP1 = (maxkP-minkP)/(Math.PI/4) * (alignError) + 0.1;

            if (t > 1) { //
                rotationkP = maxkP;
            }

          
            if (!DriverStation.isAutonomous()) {
                thetaController.setP(rotationkP);
            }

            omega =
                    -thetaController.calculate(
                            pose.getRotation().getRadians(), desiredHeading.getRadians());

         
        } else if (alignState == AlignState.POSE_TARGET) {
            vx = vXController.calculate(pose.getX(), targetTightPose.getX());
            vy = vYController.calculate(pose.getY(), targetTightPose.getY());
            omega =
                    -thetaController.calculate(
                            pose.getRotation().getRadians(),
                            targetTightPose.getRotation().getRadians());
        }



    

        // if (vx == 0 && vy == 0 && omega == 0) {
        //     setControl(brake);
        if (alignState == AlignState.SYS_ID_DRIVE) {
            setControl(driveSysId.withVolts(Units.Volts.of(tuningVolts)));
        } else if (alignState == AlignState.SYS_ID_ROTATION) {
            setControl(rotationSysId.withVolts(Units.Volts.of(tuningVolts)));
        } else if (!fieldCentric) {
            setControl(
                    driveRobotCentric
                            .withVelocityX(demo ? MathUtil.clamp(vx, 0, 1) : vx)
                            .withVelocityY(demo ? MathUtil.clamp(vy, 0, 1) : vy)
                            .withRotationalRate(demo ? omega * 0.50 : omega)
                            .withDeadband(0.0)
                            .withRotationalDeadband(0.0)
                            .withDriveRequestType(DriveRequestType.Velocity));
        } else {
            setControl(
                    driveFieldCentric
                            .withVelocityX(demo ? MathUtil.clamp(vx, 0, 1) : vx)
                            .withVelocityY(demo ? MathUtil.clamp(vy, 0, 1) : vy)
                            .withRotationalRate(demo ? omega * 0.5 : omega)
                            .withDeadband(0.0)
                            .withRotationalDeadband(0.0)
                            .withDriveRequestType(DriveRequestType.Velocity));
        }
    }

    public Optional<Rotation2d> getOverrideRotation() {
        if (alignState == AlignState.ALIGNING) {
            return Optional.of(desiredHeading);
        } else {
            return Optional.empty();
        }
    }

    

  

    public double[] getWheelRadiusCharacterizationPosition() {
        return new double[] {
            this.getModule(0).getDriveMotor().getPosition().getValueAsDouble(),
            this.getModule(1).getDriveMotor().getPosition().getValueAsDouble(),
            this.getModule(2).getDriveMotor().getPosition().getValueAsDouble(),
            this.getModule(3).getDriveMotor().getPosition().getValueAsDouble()
        };
    }

    public void setDrivePID(double kP, double kI, double kD) {
        for (int i = 0; i < 4; i++) {
            getModule(i)
                    .getDriveMotor()
                    .getConfigurator()
                    .refresh(
                            new Slot0Configs()
                                    .withKP(kP)
                                    .withKI(kI)
                                    .withKD(kD)
                                    .withKS(TunerConstants.driveGains.kS)
                                    .withKV(TunerConstants.driveGains.kV)
                                    .withKA(TunerConstants.driveGains.kA));
        }
    }

    public void setDriveP(double kP) {
        for (int i = 0; i < 4; i++) {
            getModule(i)
                    .getDriveMotor()
                    .getConfigurator()
                    .apply(
                            new Slot0Configs()
                                    .withKP(kP)
                                    .withKI(TunerConstants.driveGains.kI)
                                    .withKD(TunerConstants.driveGains.kD)
                                    .withKS(TunerConstants.driveGains.kS)
                                    .withKV(TunerConstants.driveGains.kV)
                                    .withKA(TunerConstants.driveGains.kA));
        }
    }

    public void setDriveI(double kI) {
        for (int i = 0; i < 4; i++) {
            getModule(i)
                    .getDriveMotor()
                    .getConfigurator()
                    .apply(
                            new Slot0Configs()
                                    .withKP(TunerConstants.driveGains.kP)
                                    .withKI(kI)
                                    .withKD(TunerConstants.driveGains.kD)
                                    .withKS(TunerConstants.driveGains.kS)
                                    .withKV(TunerConstants.driveGains.kV)
                                    .withKA(TunerConstants.driveGains.kA));
        }
    }

    public void setDriveD(double kD) {
        for (int i = 0; i < 4; i++) {
            getModule(i)
                    .getDriveMotor()
                    .getConfigurator()
                    .apply(
                            new Slot0Configs()
                                    .withKP(TunerConstants.driveGains.kP)
                                    .withKI(TunerConstants.driveGains.kI)
                                    .withKD(kD)
                                    .withKS(TunerConstants.driveGains.kS)
                                    .withKV(TunerConstants.driveGains.kV)
                                    .withKA(TunerConstants.driveGains.kA));
        }
    }

    public void setBrakeMode(boolean brake) {
        this.getModule(0)
                .getDriveMotor()
                .setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        this.getModule(1)
                .getDriveMotor()
                .setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        this.getModule(2)
                .getDriveMotor()
                .setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        this.getModule(3)
                .getDriveMotor()
                .setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);

        this.getModule(0)
                .getSteerMotor()
                .setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        this.getModule(1)
                .getSteerMotor()
                .setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        this.getModule(2)
                .getSteerMotor()
                .setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        this.getModule(3)
                .getSteerMotor()
                .setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public void autoInit() {
        thetaController.setP(DriveConstants.autoAlignmentkP);
        thetaController.setI(DriveConstants.autoAlignmentkI);
        thetaController.setD(DriveConstants.autoAlignmentkD);

        setAlignState(AlignState.ALIGNING);
        
    }

    public void teleopInit() {
        thetaController.setP(DriveConstants.alignmentkPMax);
        thetaController.setI(DriveConstants.alignmentkI);
        thetaController.setD(DriveConstants.alignmentkD);

        setAlignState(AlignState.MANUAL);

        vx = 0.0;
        vy = 0.0;
        omega = 0.0;

        setControl(
                driveRobotCentric
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withRotationalRate(omega)
                        .withDeadband(0.0)
                        .withRotationalDeadband(0.0)
                        .withDriveRequestType(DriveRequestType.Velocity));
    }

    @Override
    public void periodic() {
        controlDrivetrain();
    }
}