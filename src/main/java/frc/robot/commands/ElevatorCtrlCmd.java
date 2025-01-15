// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;

/** An example command that uses an example subsystem. */
public class ElevatorCtrlCmd extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem elevator;
  private final int status;

  /**
   * Creates a new ExampleCommand.
   *
   * @param elevator The subsystem used by this command.
   */
  public ElevatorCtrlCmd(ElevatorSubsystem elevator, int status) {
    this.elevator = elevator;
    this.status = status;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (status) {
      case 1:
        elevator.setElevatorSpeed(-Constants.ElevatorConstants.ELEVATOR_SPEED);
        break;
      case -1:
        elevator.setElevatorSpeed(Constants.ElevatorConstants.ELEVATOR_SPEED);
        break;
      default:
        elevator.setElevatorSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
