// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class SwerveModule {

    private CANSparkMax m_driveMotor, m_turningMotor;
    private CANEncoder m_driveEncoder, m_turningEncoder;
    private CANPIDController m_drivePIDController, m_turningPIDController;
    private CANPIDController[] pidHolder = new CANPIDController[2];

    private Drivetrain d;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr, kP2, kI2, kD2, kIz2, kFF2, kMaxOutput2, kMinOutput2, maxRPM2, maxVel2, minVel2, maxAcc2, allowedErr2;

    public SwerveModule(Drivetrain d, int driveChannel, int turnChannel){
        m_driveMotor = new CANSparkMax(driveChannel, MotorType.kBrushless);
        m_driveMotor.restoreFactoryDefaults();
        m_turningMotor = new CANSparkMax(turnChannel, MotorType.kBrushless);
        m_turningMotor.restoreFactoryDefaults();

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turningEncoder = m_turningMotor.getEncoder();

        m_driveEncoder.setPositionConversionFactor(Units.inchesToMeters(Constants.CIRCUMFERENCE) / Constants.GEAR_RATIO);
        m_turningEncoder.setPositionConversionFactor(2 * Math.PI / Constants.CAN_ENCODER_RES);
        m_driveEncoder.setVelocityConversionFactor(Units.inchesToMeters(Constants.CIRCUMFERENCE) / (60.0 * Constants.GEAR_RATIO));
        //m_turningEncoder.setVelocityConversionFactor(Units.inchesToMeters(Constants.CIRCUMFERENCE) / (60.0 * Constants.GEAR_RATIO));
        
        m_drivePIDController = m_driveMotor.getPIDController();
        pidHolder[0] = m_drivePIDController;
        m_turningPIDController = m_turningMotor.getPIDController();
        pidHolder[1] = m_turningPIDController;
        this.d = d;

        updatePID();
    }

    public void updatePID(){
        kP = d.P.getDouble(0); 
        kI = d.I.getDouble(0);
        kD = d.D.getDouble(0); 
        kIz = d.Iz.getDouble(0); 
        kFF = d.FF.getDouble(0); 
        kMaxOutput = d.MaxOutput.getDouble(1); 
        kMinOutput = -d.MaxOutput.getDouble(1);
        maxRPM = d.mxRPM.getDouble(0);

        // Smart Motion Coefficients
        maxVel = d.mxVel.getDouble(0); // rpm
        maxAcc = d.mxAcc.getDouble(0);
        allowedErr = d.Err.getDouble(0);

        kP2 = d.P2.getDouble(0); 
        kI2 = d.I2.getDouble(0);
        kD2 = d.D2.getDouble(0); 
        kIz2 = d.Iz2.getDouble(0); 
        kFF2 = d.FF2.getDouble(0); 
        kMaxOutput2 = d.MaxOutput2.getDouble(1); 
        kMinOutput2 = -d.MaxOutput2.getDouble(1);
        maxRPM2 = d.mxRPM2.getDouble(0);

        // Smart Motion Coefficients
        maxVel2 = d.mxVel2.getDouble(0); // rpm
        maxAcc2 = d.mxAcc2.getDouble(0);
        allowedErr2 = d.Err2.getDouble(0);

        //setup controllers
        pidHolder[0].setP(kP);
        pidHolder[0].setI(kI);
        pidHolder[0].setD(kD);
        pidHolder[0].setIZone(kIz);
        pidHolder[0].setFF(kFF);
        pidHolder[0].setOutputRange(kMinOutput, kMaxOutput);

        pidHolder[1].setP(kP2);
        pidHolder[1].setI(kI2);
        pidHolder[1].setD(kD2);
        pidHolder[1].setIZone(kIz2);
        pidHolder[1].setFF(kFF2);
        pidHolder[1].setOutputRange(kMinOutput2, kMaxOutput2);

        int smartMotionSlot = 0;
        pidHolder[0].setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        pidHolder[0].setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        pidHolder[0].setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        pidHolder[0].setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        pidHolder[1].setSmartMotionMaxVelocity(maxVel2, smartMotionSlot);
        pidHolder[1].setSmartMotionMinOutputVelocity(minVel2, smartMotionSlot);
        pidHolder[1].setSmartMotionMaxAccel(maxAcc2, smartMotionSlot);
        pidHolder[1].setSmartMotionAllowedClosedLoopError(allowedErr2, smartMotionSlot);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        //Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
            SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getPosition()));

        pidHolder[0].setReference(state.speedMetersPerSecond, ControlType.kSmartVelocity);
        pidHolder[1].setReference(state.angle.getRadians(), ControlType.kSmartMotion);
    }
}
