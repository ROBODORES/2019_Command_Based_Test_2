/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.RobotMap;
import frc.robot.commands.SetArm;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Add your docs here.
 */
public class Arm extends PIDSubsystem {
  DigitalInput source_A = null;
  DigitalInput source_B = null;
  DigitalInput limitSwitch = null;
  Encoder encoder = null;
  VictorSPX armMotor = null;

  public Arm() {
    // Intert a subsystem name and PID values here
    super("Elevator", 1.0, 0.0, 0.0);
    setAbsoluteTolerance(0.05);
    getPIDController().setContinuous(false);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
    source_A = new DigitalInput(RobotMap.armSourceA);
    source_B = new DigitalInput(RobotMap.armSourceB);
    limitSwitch = new DigitalInput(RobotMap.armlimit);
    encoder = new Encoder(source_A, source_B);
    armMotor = new VictorSPX(RobotMap.armMotor);
  }

  public boolean isSwitchSet() {
    return limitSwitch.get();
  }

  public void setEncoder() {
    encoder.reset();
  }

  public void moveUp() {
    armMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void moveDown() {
    armMotor.set(ControlMode.PercentOutput, -0.5);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new SetArm(0));
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return encoder.getRaw();
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    armMotor.set(ControlMode.PercentOutput, output);
  }
}
