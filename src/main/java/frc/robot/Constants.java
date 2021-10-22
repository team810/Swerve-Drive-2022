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

    //Robot stats
    public static final double LENGTH_TO_WHEELS = 0.381;
    public static final double WIDTH_TO_WHEELS = 0.381;

    public static final double kEncoderResolution = 7/4;

    public static final double GEAR_RATIO = 6.67;
    public static final double CIRCUMFERENCE = 4 * Math.PI;
    public static final double MAX_VELOCITY = 3; //units: m/s
 
    //Port Numbers
    // CAN ID
    public static final int FRONT_RIGHT_CAN = 1;
    public static final int FRONT_LEFT_CAN = 2;
    public static final int BACK_RIGHT_CAN = 4;
    public static final int BACK_LEFT_CAN = 3;

    // PWM
    public static final int FRONT_RIGHT = 0;
    public static final int FRONT_LEFT = 1;
    public static final int BACK_RIGHT = 2;
    public static final int BACK_LEFT = 3;

    // ENCODERS
    public static final int FRONT_RIGHT_PORT_1 = 4;
    public static final int FRONT_RIGHT_PORT_2 = 5;

    public static final int FRONT_LEFT_PORT_1 = 2;
    public static final int FRONT_LEFT_PORT_2 = 3;

    public static final int BACK_RIGHT_PORT_1 = 6;
    public static final int BACK_RIGHT_PORT_2 = 7;

    public static final int BACK_LEFT_PORT_1 = 0;
    public static final int BACK_LEFT_PORT_2 = 1;
}
