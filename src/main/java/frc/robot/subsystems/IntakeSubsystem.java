// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SystemConstants;

import static frc.robot.Constants.IntakeConstants.*;

// Simple class that has (1) double solenoid to simulate extending and retracting an intake

public class IntakeSubsystem extends SubsystemBase {

  private final DoubleSolenoid m_intakeSolenoid = 
    new DoubleSolenoid(
      SystemConstants.kPcmCanID,
      PneumaticsModuleType.CTREPCM,
      kIntakeSolenoidID[0],
      kIntakeSolenoidID[1]
    );

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeRetract();
  }

  /** method to extend the intake */
  public void intakeExtend() {
    m_intakeSolenoid.set(Value.kForward);
  }

  /** method to retract the intake */
  public void intakeRetract() {
    m_intakeSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
