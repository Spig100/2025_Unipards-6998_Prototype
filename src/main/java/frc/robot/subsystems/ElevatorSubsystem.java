// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.*;

public class ElevatorSubsystem extends SubsystemBase {

  private final SparkMax motor_right_leader = new SparkMax(ELEVATOR_MOTOR_1, MotorType.kBrushless);
  private final SparkMax motor_left_follower = new SparkMax(ELEVATOR_MOTOR_2, MotorType.kBrushless);

  private final SparkMaxConfig motor_conf_glob = new SparkMaxConfig();
  private final SparkMaxConfig motor_conf_right_leader = new SparkMaxConfig();
  private final SparkMaxConfig motor_conf_left_follower = new SparkMaxConfig();

  private SparkClosedLoopController motor1_ctrl;
  private SparkClosedLoopController motor2_ctrl;

  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {
    configureMotors();

  }

  public void zeroEncoders() {
    // Setting the zero point when booting the robot.
    motor_right_leader.getEncoder().setPosition(0);
    motor_left_follower.getEncoder().setPosition(0);
  }

  public void setElevatorSpeed(double speed) {
    motor_right_leader.set(speed);
  }

  public void setElevatorVoltage(Voltage voltage) {
    motor_left_follower.setVoltage(voltage);
  }

  public void configureMotors() {
    /* Motor configuration modifying. */
    motor_conf_glob
            .smartCurrentLimit(ELEVATOR_SMART_CURRENT_LIMIT)
            .idleMode(SparkBaseConfig.IdleMode.kBrake);
    motor_conf_right_leader
            .apply(motor_conf_glob);
    motor_conf_left_follower
            .apply(motor_conf_glob)
            .follow(motor_right_leader, true);

    /* Motor configuration applying */
    motor_right_leader.configure(
            motor_conf_right_leader,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
    );
    motor_left_follower.configure(
            motor_conf_left_follower,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
    );
  }

  // System Identification Tool
  // Initialize System Identification Routine
  SysIdRoutine sysIdRoutine = new SysIdRoutine(
          new SysIdRoutine.Config(
                  null,
                  Volts.of(5),
                  null
          ),
          new SysIdRoutine.Mechanism(
                  this::setElevatorVoltage,
                  null,
                  this
          )
  );

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }


    /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
