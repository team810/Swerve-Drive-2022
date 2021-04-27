// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Robot stats
    public static final double LENGTH_TO_WHEELS = 0.381;
    public static final double WIDTH_TO_WHEELS = 0.381;
    public static final double CIRCUMFERENCE = 1;
    public static final double GEAR_RATIO = 6.67;
    public static final double kEncoderResolution = 7/4;

    //DISTANCE TO MODULES - IMPORTANT
    /*  This is the distance from the CENTER of the drivetrain to the center of the wheel of the swerve modules
        Replace with values in METERS
        FRONT AND LEFT ARE POSITIVE, BACK AND RIGHT ARE NEGATIVE
    */
    public static final Translation2d m_frontLeftLocation = new Translation2d(Constants.LENGTH_TO_WHEELS, Constants.WIDTH_TO_WHEELS); 
    public static final Translation2d m_frontRightLocation = new Translation2d(Constants.LENGTH_TO_WHEELS, -Constants.WIDTH_TO_WHEELS);
    public static final Translation2d m_backLeftLocation = new Translation2d(-Constants.LENGTH_TO_WHEELS, Constants.WIDTH_TO_WHEELS);
    public static final Translation2d m_backRightLocation = new Translation2d(-Constants.LENGTH_TO_WHEELS, -Constants.WIDTH_TO_WHEELS);

    //Encoder stuff
    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
    public static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared


    //shuffleboard stuff
    public static final double kP = 5e-5; 
    public static final double kI = 1e-6;
    public static final double kD = 0; 
    public static final double kIz = 0; 
    public static final double kFF = 0.000156; 
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1;
    public static final double maxRPM = 5700;
    public static final double ERROR = 3;

    public static final double maxVel = 2000; // rpm
    public static final double maxAcc = 1500;
}
