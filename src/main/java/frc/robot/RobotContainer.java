// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SystemConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();

  // Controller IO Objects
  XboxController m_driveController = new XboxController(OIConstants.kDriveControllerInput);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerInput);

  /* Auto Builder */
  // Using the PathPlanner RamseteAutoBuilder constructor to create an AutoBuilder
  RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(
    m_robotDrive::getPose,
    m_robotDrive::resetOdometry,
    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    m_robotDrive.kinematics,
    new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
    m_robotDrive::getWheelSpeeds,
    new PIDConstants(DriveConstants.kPDriveVel, 0.0, 0.0),
    m_robotDrive::tankDriveVolts,
    SystemConstants.eventMap,
    m_robotDrive
    );
  
  /* Auto Commands
      Using the autoBuilder object constructed above, we can create an entire auto routine with the following:
        - Name of the command
        - Path group for that specific auto
          - This is defined in the AutoPaths.java file
      
  */
  private final Command m_auto1 = autoBuilder.fullAuto(AutoPaths.pathGroupAuto1); 

  // A chooser for autonomous commands
  SendableChooser<Command> m_autonomouschooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    // Set the default command for the drive system
    m_robotDrive.setDefaultCommand(
      new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    -m_driveController.getLeftY(), m_driveController.getRightX()),
            m_robotDrive)
    );

    // Add a sendable chooser to shuffleboard on the "Auto Chooser" tab
    Shuffleboard.getTab("Auto Chooser").add(m_autonomouschooser);
    m_autonomouschooser.setDefaultOption("Auto 1", m_auto1);

    // Populate the autonomous event map
    setEventMap();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driveController, XboxController.Button.kA.value)
      .whenHeld(new InstantCommand(m_robotIntake::intakeExtend, m_robotIntake))
      .whenReleased(new InstantCommand(m_robotIntake::intakeRetract, m_robotIntake));

    new JoystickButton(m_driveController, XboxController.Button.kY.value)
      .whenPressed(m_auto1);
  }

  /**
   * Use this method to set any and all commands that are defined in the PathPlanner autonomous paths.
   * These events are case sensitive and should exactly match those in the PathPlanner app.
   */
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
      return m_autonomouschooser.getSelected();
      }
}