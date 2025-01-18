// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class ElevatorConstants {
    // Motors' CAN Bus ID
    public static final int ELEVATOR_MOTOR_1 = 1;
    public static final int ELEVATOR_MOTOR_2 = 2;

    public static final boolean ELEVATOR_MOTOR_1_INVERTED = true;
    public static final boolean ELEVATOR_MOTOR_2_INVERTED = false;

    // Elevator's physical factors
    // TODO: Check Elevator's specification and fill in.
    public static final int ELEVATOR_TRIP_DISTANCE = 0;
    public static final double ELEVATOR_GEAR_RATIO = 9; // X:1
    public static final double ELEVATOR_SPEED = 0.25; // percentage, only use in non-PID mode
    // public static final double ELEVATOR_ROTATION_PER_METER = 0.12 * ELEVATOR_GEAR_RATIO;
    public static final int ELEVATOR_SMART_CURRENT_LIMIT = 25; // Amp
  }

  public static final class GamePieceConstants {
    public static final int GAME_PIECE_MOTOR = 0;
    public static final int GAME_PIECE_SMART_CURRENT_LIMIT = 25;
  }
}
