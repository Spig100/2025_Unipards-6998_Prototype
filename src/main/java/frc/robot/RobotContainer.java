/*
 Copyright (c) FIRST and other WPILib contributors.
 Open Source Software; you can modify and/or share it under the terms of
 the WPILib BSD license file in the root directory of this project.
*/

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.ElevatorCtrlCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverXboxCtrl = new CommandXboxController(0);
    private final CommandXboxController operatorXboxCtrl = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ElevatorSubsystem elevator = new ElevatorSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // DRIVETRAIN BINDING

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverXboxCtrl.getLeftY() * MaxSpeed)
                        // Drive forward with negative Y (forward)
                    .withVelocityY(-driverXboxCtrl.getLeftX() * MaxSpeed)
                        // Drive left with negative X (left)
                    .withRotationalRate(-driverXboxCtrl.getRightX() * MaxAngularRate)
                    // Drive counterclockwise with negative X (left)
            )
        );

        driverXboxCtrl.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverXboxCtrl.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverXboxCtrl.getLeftY(), -driverXboxCtrl.getLeftX()))
        ));

        // Run SysId routines on the drivetrain when holding back/start and X/Y of the driver controller.
        // Note that each routine should be run exactly once in a single log.
        driverXboxCtrl.back().and(driverXboxCtrl.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverXboxCtrl.back().and(driverXboxCtrl.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverXboxCtrl.start().and(driverXboxCtrl.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverXboxCtrl.start().and(driverXboxCtrl.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverXboxCtrl.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // ELEVATOR BINDING
        operatorXboxCtrl.povUp().onTrue(new ElevatorCtrlCmd(elevator,1));
        operatorXboxCtrl.povDown().onTrue(new ElevatorCtrlCmd(elevator,-1));
        operatorXboxCtrl.povCenter().onTrue(new ElevatorCtrlCmd(elevator,0));

        // Run SysId routines on the elevator when holding back/start and X/Y of the operator controller.
        operatorXboxCtrl.back().and(operatorXboxCtrl.y().whileTrue(elevator.sysIdDynamic(Direction.kForward)));
        operatorXboxCtrl.back().and(operatorXboxCtrl.x().whileTrue(elevator.sysIdDynamic(Direction.kReverse)));
        operatorXboxCtrl.start().and(operatorXboxCtrl.y().whileTrue(elevator.sysIdQuasistatic(Direction.kForward)));
        operatorXboxCtrl.start().and(operatorXboxCtrl.x().whileTrue(elevator.sysIdQuasistatic(Direction.kReverse)));

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
