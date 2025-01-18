// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//TODO: After the mechanism dept. finish the structure, write this subsystem.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static frc.robot.Constants.GamePieceConstants.*;

public class GamePieceSubsystem extends SubsystemBase {
  private final SparkMax motor = new SparkMax(GAME_PIECE_MOTOR, MotorType.kBrushless);
  private final SparkMaxConfig motor_config = new SparkMaxConfig();

  /** Creates a new GamePieceSubsystem. */
  public GamePieceSubsystem() {}

  private void configureMotors() {
    motor_config
            .smartCurrentLimit(GAME_PIECE_SMART_CURRENT_LIMIT)
            .idleMode(SparkBaseConfig.IdleMode.kCoast);
    motor.configure(motor_config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  // Basic methods.

  public void setMotorPercentage(double output) {
    motor.set(output);
  }

  public void setMotorVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  // System Identification

  public SysIdRoutine sysIdRoutine() = new SysIdRoutine(

  );
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
