// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class SwerveModule {

    private CANSparkMax m_driveMotor;
    private Spark m_turningMotor;
    private CANEncoder m_driveEncoder;
    private Encoder m_turningEncoder;
    private CANPIDController m_drivePIDController;
    private ProfiledPIDController m_turningPIDController;
    //private CANPIDController[] pidHolder = new CANPIDController[2];
    
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    private Drivetrain d;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr, kP2, kI2, kD2;

    public SwerveModule(Drivetrain d, int driveChannel, int turnChannel, int encoder1, int encoder2){
        m_driveMotor = new CANSparkMax(driveChannel, MotorType.kBrushless);
        m_driveMotor.restoreFactoryDefaults();
        m_turningMotor = new Spark(turnChannel);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turningEncoder = new Encoder(encoder1, encoder2);

        m_driveEncoder.setPositionConversionFactor(Units.inchesToMeters(Constants.CIRCUMFERENCE) / Constants.GEAR_RATIO);
        m_turningEncoder.setDistancePerPulse(2 * Math.PI / Constants.kEncoderResolution);

        m_driveEncoder.setVelocityConversionFactor(Units.inchesToMeters(Constants.CIRCUMFERENCE) / (60.0 * Constants.GEAR_RATIO));
        //m_turningEncoder.setVelocityConversionFactor(Units.inchesToMeters(Constants.CIRCUMFERENCE) / (60.0 * Constants.GEAR_RATIO));
        
        m_drivePIDController = m_driveMotor.getPIDController();
        m_turningPIDController = new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              Constants.kMaxSpeed, Constants.kModuleMaxAngularAcceleration));

        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        this.d = d;
        updatePID();
    }

    public void updatePID(){
        //DRIVE PID
        kP = d.P.getDouble(0); 
        kI = d.I.getDouble(0);
        kD = d.D.getDouble(0); 
        kIz = d.Iz.getDouble(0); 
        kFF = d.FF.getDouble(0); 
        kMaxOutput = d.MaxOutput.getDouble(1); 
        kMinOutput = -d.MaxOutput.getDouble(1);
        maxRPM = d.mxRPM.getDouble(0);
        maxVel = d.mxVel.getDouble(0); // rpm
        maxAcc = d.mxAcc.getDouble(0);
        allowedErr = d.Err.getDouble(0);

        //TURNING PID
        kP2 = d.P2.getDouble(0); 
        kI2 = d.I2.getDouble(0);
        kD2 = d.D2.getDouble(0); 

        //setup controllers
        m_drivePIDController.setP(kP);
        m_drivePIDController.setI(kI);
        m_drivePIDController.setD(kD);
        m_drivePIDController.setIZone(kIz);
        m_drivePIDController.setFF(kFF);
        m_drivePIDController.setOutputRange(kMinOutput, kMaxOutput);
        int smartMotionSlot = 0;
        m_drivePIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        m_drivePIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        m_drivePIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        m_drivePIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        m_turningPIDController = new ProfiledPIDController(
            kP2,
            kI2,
            kD2,
          new TrapezoidProfile.Constraints(
              Constants.kMaxSpeed, Constants.kModuleMaxAngularAcceleration));

        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.get()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        //Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
            SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.get()));
        //Drive
        m_drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kSmartVelocity);

        //Turn
        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.get(), state.angle.getRadians());

        final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
        m_turningMotor.setVoltage(turnOutput + turnFeedforward);
    }
}
