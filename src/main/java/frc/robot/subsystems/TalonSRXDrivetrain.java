// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

/* This class declares the subsystem for the robot drivetrain if controllers are connected via CAN. Make sure to go to
 * RobotContainer and uncomment the line declaring this subsystem and comment the line for PWMDrivetrain.
 *
 * The subsystem contains the objects for the hardware contained in the mechanism and handles low level logic
 * for control. Subsystems are a mechanism that, when used in conjuction with command "Requirements", ensure
 * that hardware is only being used by 1 command at a time.
 */
public class TalonSRXDrivetrain extends SubsystemBase {
  /*Class member variables. These variables represent things the class needs to keep track of and use between
  different method calls. */
  DifferentialDrive m_drivetrain;

  /*Constructor. This method is called when an instance of the class is created. This should generally be used to set up
   * member variables and perform any configuration or set up necessary on hardware.
   */
  public TalonSRXDrivetrain() {
    
    WPI_TalonSRX leftFront = new WPI_TalonSRX(DrivetrainConstants.kLeftFrontID);
    WPI_TalonSRX leftRear = new WPI_TalonSRX(DrivetrainConstants.kLeftRearID);
    WPI_TalonSRX rightFront = new WPI_TalonSRX(DrivetrainConstants.kRightFrontID);
    WPI_TalonSRX rightRear = new WPI_TalonSRX(DrivetrainConstants.kRightRearID);

    leftFront.configPeakCurrentLimit(DrivetrainConstants.kCurrentLimit);
    leftFront.configPeakCurrentDuration(100);
    leftFront.configContinuousCurrentLimit(20);
    leftFront.enableCurrentLimit(true);

    rightFront.configPeakCurrentLimit(DrivetrainConstants.kCurrentLimit);
    rightFront.configPeakCurrentDuration(100);
    rightFront.configContinuousCurrentLimit(20);
    rightFront.enableCurrentLimit(true);

    leftRear.configPeakCurrentLimit(DrivetrainConstants.kCurrentLimit);
    leftRear.configPeakCurrentDuration(100);
    leftRear.configContinuousCurrentLimit(20);
    leftRear.enableCurrentLimit(true);

    rightRear.configPeakCurrentLimit(DrivetrainConstants.kCurrentLimit);
    rightRear.configPeakCurrentDuration(100);
    rightRear.configContinuousCurrentLimit(20);
    rightRear.enableCurrentLimit(true);

    leftRear.follow(leftFront);
    rightRear.follow(rightFront);
    
    leftFront.setInverted(DrivetrainConstants.kLeftInverted);
    leftRear.setInverted(InvertType.FollowMaster);
    rightFront.setInverted(DrivetrainConstants.kRightInverted);
    rightRear.setInverted(InvertType.FollowMaster);


    m_drivetrain = new DifferentialDrive(leftFront, rightFront);
  }

  /*Method to control the drivetrain using arcade drive. Arcade drive takes a speed in the X (forward/back) direction
   * and a rotation about the Z (turning the robot about it's center) and uses these to control the drivetrain motors */
  public void arcadeDrive(double speed, double rotation) {
    m_drivetrain.arcadeDrive(speed, rotation);
  }

  @Override
  public void periodic() {
    /*This method will be called once per scheduler run. It can be used for running tasks we know we want to update each
     * loop such as processing sensor data. Our drivetrain is simple so we don't have anything to put here */
  }
}
