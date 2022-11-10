// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Auto1;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SystemConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();

  private final Command m_auto1 = new Auto1(m_robotDrive, m_robotIntake);

  XboxController m_driveController = new XboxController(OIConstants.kDriveControllerInput);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerInput);

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // //PathPlanner Event Map
  // public static HashMap<String, Command> eventMap = new HashMap<String, Command>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
      new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    -m_driveController.getLeftY(), m_driveController.getRightX()),
            m_robotDrive)
    );

    m_chooser.setDefaultOption("Auto 1", m_auto1);

    setEventMap();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
      .whenHeld(new InstantCommand(m_robotIntake::intakeExtend, m_robotIntake))
      .whenReleased(new InstantCommand(m_robotIntake::intakeRetract, m_robotIntake));

    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
      .whenPressed(m_auto1);
  }

  public void setEventMap() {
    SystemConstants.eventMap.put("intakeExtend", new InstantCommand(m_robotIntake::intakeExtend, m_robotIntake));
    SystemConstants.eventMap.put("intakeRetract", new InstantCommand(m_robotIntake::intakeRetract, m_robotIntake));
  }


  
    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
      return m_chooser.getSelected();
      }
}