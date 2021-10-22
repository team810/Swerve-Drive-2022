// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.SwerveModule;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;


  //NavX
  public final AHRS navx = new AHRS(SPI.Port.kMXP); // change to I2C if not working
  
  public Drivetrain() {
    //Drivetrain, drive motor CAN ID, Turn motor PWM port, Encoder port 1, Encoder port 2
    m_frontRight = new SwerveModule(this, 
      Constants.FRONT_RIGHT_CAN, 
      Constants.FRONT_RIGHT, 
      Constants.FRONT_RIGHT_PORT_1, 
      Constants.FRONT_RIGHT_PORT_2);
    
    m_frontLeft = new SwerveModule(this,
      Constants.FRONT_LEFT_CAN, 
      Constants.FRONT_LEFT, 
      Constants.FRONT_LEFT_PORT_1, 
      Constants.FRONT_LEFT_PORT_2);

    m_backRight = new SwerveModule(this,
      Constants.BACK_RIGHT_CAN,
      Constants.BACK_RIGHT,
      Constants.BACK_RIGHT_PORT_1,
      Constants.BACK_RIGHT_PORT_2);

    m_backLeft = new SwerveModule(this,
      Constants.BACK_LEFT_CAN,
      Constants.BACK_LEFT, 
      Constants.BACK_LEFT_PORT_1, 
      Constants.BACK_LEFT_PORT_2);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double xSpeed, double ySpeed, double rot){ //All values have to be between -1 and 1
    double r = Math.sqrt ((Constants.LENGTH_TO_WHEELS * Constants.LENGTH_TO_WHEELS) + (Constants.WIDTH_TO_WHEELS * Constants.WIDTH_TO_WHEELS));
    ySpeed *= -1;

    double a = xSpeed - rot * (Constants.LENGTH_TO_WHEELS / r);
    double b = xSpeed + rot * (Constants.LENGTH_TO_WHEELS / r);
    double c = ySpeed - rot * (Constants.WIDTH_TO_WHEELS / r);
    double d = ySpeed + rot * (Constants.WIDTH_TO_WHEELS / r);

    double backRightSpeed = Math.sqrt ((a * a) + (d * d));
    double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
    double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
    double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

    double maxSpeed = Math.max(backRightSpeed, Math.max(backLeftSpeed, Math.max(frontRightSpeed, frontLeftSpeed)));
    //Normalize wheel speeds to between 0 and 1
    if(maxSpeed > 1){
      backRightSpeed /= maxSpeed;
      backLeftSpeed /= maxSpeed;
      frontRightSpeed /= maxSpeed;
      frontLeftSpeed /= maxSpeed;
    }

    //Angles are between -pi and pi
    double backRightAngle = Math.atan2 (a, d);
    double backLeftAngle = Math.atan2 (a, c);
    double frontRightAngle = Math.atan2 (b, d);
    double frontLeftAngle = Math.atan2 (b, c);

    m_backRight.drive(backRightSpeed, backRightAngle);
    m_backLeft.drive(backLeftSpeed, backLeftAngle);
    m_frontRight.drive(frontRightSpeed, frontRightAngle);
    m_frontLeft.drive(frontLeftSpeed, frontLeftAngle);
  }
}
