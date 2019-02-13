/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.commands.Drive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  CANSparkMax leftMotor = null;
  CANSparkMax leftMotorFollower = null;
  CANSparkMax rightMotor = null;
  CANSparkMax rightMotorFollower = null;

  DifferentialDrive differentialDrive = null;

  public Drivetrain() {
    leftMotor = new CANSparkMax(RobotMap.leftMotor, MotorType.kBrushless);
    leftMotorFollower = new CANSparkMax(RobotMap.leftMotorFollower, MotorType.kBrushless);
    rightMotor = new CANSparkMax(RobotMap.rightMotor, MotorType.kBrushless);
    rightMotorFollower = new CANSparkMax(RobotMap.rightMotorFollower, MotorType.kBrushless);

    leftMotorFollower.follow(leftMotor);
    rightMotorFollower.follow(rightMotor);

    differentialDrive = new DifferentialDrive(leftMotor, rightMotor);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new Drive());
  }
}
