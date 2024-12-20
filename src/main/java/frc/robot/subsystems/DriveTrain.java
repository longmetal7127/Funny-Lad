// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a differential drive style drivetrain. */
public class DriveTrain extends SubsystemBase {
  public static final double kMaxSpeed = 35.0; // meters per second
  public static final double kMaxAngularSpeed = 7 * Math.PI; // one rotation per second

  private static final double kTrackWidth = 0.381 * 2; // meters
  private static final double kWheelRadius = 0.0762; // meters
  private static final int kEncoderResolution = 4096;
  private SparkMax m_leftLeader = new SparkMax(1, MotorType.kBrushless);
  private SparkMax m_leftFollower = new SparkMax(2, MotorType.kBrushless);
  private SparkMax m_rightLeader = new SparkMax(3, MotorType.kBrushless);
  private SparkMax m_rightFollower = new SparkMax(4, MotorType.kBrushless);

  DifferentialDrive m_robotDrive;

  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse
   * and resets the
   * gyro.
   */
  public DriveTrain() {
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.inverted(true);
    rightFollowerConfig.follow(m_rightLeader);

    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    rightFollowerConfig.inverted(true);

    m_rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    rightFollowerConfig.inverted(false);
    rightFollowerConfig.follow(m_rightLeader);

    SparkMaxConfig leftLeaderConfig = new SparkMaxConfig();
    rightFollowerConfig.inverted(false);

    m_leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftLeader.configure(leftLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_robotDrive = new DifferentialDrive(m_leftLeader::set, m_rightLeader::set);

  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot    Angular velocity in rad/s.
   */
  public void drive(double x, double y) {
    m_robotDrive.arcadeDrive(x* -1, y);
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
  }
}
