// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class SwerveModule {

    public Spark angleMotor;
    public CANSparkMax speedMotor;

    private CANEncoder m_driveEncoder;
    private Encoder m_turningEncoder;

    private CANPIDController m_drivePIDController;
    private PIDController m_turningPIDController;

    private final double MAX_VOLTS = 4.95;
    private final int driveChannel;

    public SwerveModule(Drivetrain d, int driveChannel, int turnChannel, int encoder1, int encoder2){
        speedMotor = new CANSparkMax(driveChannel, MotorType.kBrushless);
        speedMotor.restoreFactoryDefaults();
        m_driveEncoder = speedMotor.getEncoder();
        //m_driveEncoder.setPositionConversionFactor(Units.inchesToMeters(Constants.CIRCUMFERENCE) / Constants.GEAR_RATIO);
        //Change conversion factor to meters/second
        m_driveEncoder.setVelocityConversionFactor(Units.inchesToMeters(Constants.CIRCUMFERENCE) / (60.0 * Constants.GEAR_RATIO));

        angleMotor = new Spark(turnChannel);
        m_turningEncoder = new Encoder(encoder2, encoder1);
        m_turningEncoder.reset();
        
        m_turningEncoder.setDistancePerPulse(2 * Math.PI / Constants.kEncoderResolution);

        m_drivePIDController = speedMotor.getPIDController();
        m_drivePIDController.setP(.5);
        m_drivePIDController.setI(0);
        m_drivePIDController.setD(0);

        m_turningPIDController = new PIDController(1.0/90.0,0,0);
        //m_turningPIDController.enableContinuousInput(-180, 180);

        this.driveChannel = driveChannel;
    }

    public void drive(double speed, double angle){
        //Angles to degrees to make code more readable
        angle = Math.toDegrees(angle);
        double output = m_turningPIDController.calculate(m_turningEncoder.get(), angle);

        //Smartdashboard debug info
        SmartDashboard.putNumber(getSide(driveChannel) + " Speed", speed);
        SmartDashboard.putNumber(getSide(driveChannel) + " Angle (Deg)", angle);
        SmartDashboard.putNumber(getSide(driveChannel) + " Angle (Reported)", m_turningEncoder.get());
        SmartDashboard.putNumber(getSide(driveChannel) + " Out", -output);
        
        //Set Motor powers
        m_drivePIDController.setReference(speed, ControlType.kVelocity);
        angleMotor.set(-output);
    }

    public void setIdleMode(IdleMode mode){
        speedMotor.setIdleMode(mode);
    }

    private String getSide(int driveChannel){
        if(driveChannel == 2){
            return "Front_Left";
        }else if(driveChannel == 1){
            return "Front_Right";
        }else if(driveChannel == 3){
            return "Back_Left";
        }else if(driveChannel == 4){
            return "Back_Right";
        }else{
            return "ERROR";
        }
    }
}
