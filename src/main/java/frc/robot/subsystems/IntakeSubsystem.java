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

public class IntakeSubsystem extends SubsystemBase {

  private final DoubleSolenoid m_intakeSolenoid = 
    new DoubleSolenoid(
      SystemConstants.kPcmCanID,
      PneumaticsModuleType.CTREPCM,
      kIntakeSolenoidID[0],
      kIntakeSolenoidID[1]
    );

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  public void intakeExtend() {
    m_intakeSolenoid.set(Value.kForward);
  }

  public void intakeRetract() {
    m_intakeSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
