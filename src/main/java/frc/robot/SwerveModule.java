// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508; // meters // TODO verify wheel radius
  private static final double kDriveGearRatio = 7.0 / 57.0;
  private static final int kDriveEncoderResolution = 2048;
  private static final int kTurningEncoderResolution = 4096;
  private static final double gearRatio = 12.5;
  
  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
  2 * Math.PI; // radians per second squared
  
  @SuppressWarnings({"removal"})
  private final TalonFX m_driveMotor;
  private final CANSparkMax m_turningMotor;
  @SuppressWarnings({"removal"})
  private final WPI_CANCoder m_turningEncoder;

  private final String name;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorID PWM output for the drive motor.
   * @param turningMotorID PWM output for the turning motor.
   * @param driveEncoderID DIO input for the drive encoder channel A
   * @param turningEncoderID DIO input for the turning encoder channel A
   */
  public SwerveModule(
      int driveMotorID,
      int turningMotorID,
      int turningEncoderID,
      String name) {
    this.name = name;
    m_driveMotor = new TalonFX(driveMotorID);
    m_turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

    m_turningEncoder = new WPI_CANCoder(turningEncoderID);

    //m_driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); // TODO need to figure out which feedback device

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveMotor.getSelectedSensorVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotor.getSelectedSensorPosition(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(2 * Math.PI * m_turningEncoder.getPosition() / 2048. / gearRatio);

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveMotor.getSelectedSensorVelocity() * 10.0 / kDriveEncoderResolution * kDriveGearRatio * 2.0 * Math.PI * kWheelRadius, state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(2.0 * Math.PI * m_turningEncoder.getPosition() / kTurningEncoderResolution, state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    // TODO undo 
    if(name.equals("fl")) {
      //System.out.println(new Rotation2dm_turningEncoder.getPosition());
      // System.out.println(name + " - rotation " + encoderRotation.getDegrees() + "   - turn output: " + turnOutput);
      System.out.println(name + " - rotation " + m_turningEncoder.getPosition() + "   - turn output: " + turnOutput);
    }
    
    //m_driveMotor.set(TalonFXControlMode.PercentOutput, driveOutput + driveFeedforward);
    //m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }
}
