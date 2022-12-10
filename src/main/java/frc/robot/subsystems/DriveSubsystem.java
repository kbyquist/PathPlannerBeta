// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import static java.lang.Math.*;

// DriveSubsystem class that contains the hardware and methods needed for both autonomous and teleop control
public class DriveSubsystem extends SubsystemBase {

  // Kinematics object used in autonomous control
  public DifferentialDriveKinematics kinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

  // The gyro sensor (navX or navX2)
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // Define Motors in Drive Subsystem
  private final CANSparkMax m_leftLead = new CANSparkMax(kLeftDriveMotors[0], MotorType.kBrushless);
  private final CANSparkMax m_leftFollow1 = new CANSparkMax(kLeftDriveMotors[1], MotorType.kBrushless);
  private final CANSparkMax m_leftFollow2 = new CANSparkMax(kLeftDriveMotors[2], MotorType.kBrushless);
  private final CANSparkMax m_rightLead = new CANSparkMax(kRightDriveMotors[0], MotorType.kBrushless);
  private final CANSparkMax m_rightFollow1 = new CANSparkMax(kRightDriveMotors[1], MotorType.kBrushless);
  private final CANSparkMax m_rightFollow2 = new CANSparkMax(kRightDriveMotors[2], MotorType.kBrushless);

  // Separate wheel encoders used for path following
  private final Encoder m_leftWheelEncoder = 
      new Encoder(
        kLeftWheelEncoderPorts[0],
        kLeftWheelEncoderPorts[1],
        kLeftWheelEncoderReversed,
        EncodingType.k1X);

  private final Encoder m_rightWheelEncoder = 
      new Encoder(
        kRightWheelEncoderPorts[0],
        kRightWheelEncoderPorts[1],
        kRightWheelEncoderReversed,
        EncodingType.k1X);

  // Robot differential drive used in teleop control
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLead, m_rightLead);

  // Creates a SlewRateLimiter object that limits the rate of change of the signal to defined constant units per second
  public double kSlewRateDrive = 3.5;
  SlewRateLimiter driveFilter;

  // Shuffleboard setup to display debugging information
  private ShuffleboardTab tabDrive = Shuffleboard.getTab(kDriveTabName);

  NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
  NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
  NetworkTableEntry netGyroAngle = tabDrive.add("Gyro Angle", 0).getEntry();



  /** Creates a new Drivetrain. Initialize hardware here. This method is called on robot code start */
  public DriveSubsystem() {

    // Adds gyro object to the shuffleboard drive tab
    Shuffleboard.getTab(kDriveTabName).add(m_gyro);
    
    // Gyro reset and calibration
    zeroHeading();
    m_gyro.calibrate();

    // Wheel encoder zeroing
    resetWheelEncoders();

    // Creates new odometry object to store the pose of the robot
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-m_gyro.getAngle())); //Turns wrong way

    // Resets the odometry object to a (0,0) position with a rotation of the current gyro reading (which should be 0 as it was recently reset)
    m_odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(-m_gyro.getAngle()));

    /* Restore Defaults of Motors. 
    Doing this confirms the settings will be the same no matter what physical controller is used  */
    m_leftLead.restoreFactoryDefaults();
    m_leftFollow1.restoreFactoryDefaults();
    m_leftFollow2.restoreFactoryDefaults();
    m_rightLead.restoreFactoryDefaults();
    m_rightFollow1.restoreFactoryDefaults();
    m_rightFollow2.restoreFactoryDefaults();

    /* Create Follow Groups */
    //Left
    m_leftFollow1.follow(m_leftLead);
    m_leftFollow2.follow(m_leftLead);
    //Right
    m_rightFollow1.follow(m_rightLead);
    m_rightFollow2.follow(m_rightLead);

    // Set Motor and Encoder Inversions
    m_rightLead.setInverted(kRightMotorInverted);
    m_leftLead.setInverted(kLeftMotorInverted);

    // Set Encoder Conversion
    m_leftWheelEncoder.setDistancePerPulse(kEncoderDistancePerPulse);
    m_rightWheelEncoder.setDistancePerPulse(kEncoderDistancePerPulse);

    // Sets the slew rate of the previously constructed object
    driveFilter = new SlewRateLimiter(kSlewRateDrive);

    // Starting PathPlanner server to view the autonomous movements and send new paths quickly
    PathPlannerServer.startServer(5811);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(driveFilter.calculate(fwd), rot);
  }

  /**
   * reports the average distance travelled by the wheel encoders
   * @return meters travelled by the robot
   */
  public double getAverageWheelEncoderDistance() {
    return (m_leftWheelEncoder.getDistance() + m_rightWheelEncoder.getDistance()) / 2.0;
  }

  /**
   * reports the average speed of the wheel encoders
   * @return absolute value of the speed of the robot
   */
  public double getAverageWheelEncoderSpeed() {
    return (abs(m_leftWheelEncoder.getRate()) + abs(m_rightWheelEncoder.getRate())) / 2.0;
  }

  /**
   * gets the left wheel encoder object
   * @return left wheel encoder object
   */
  public Encoder getLeftWheelEncoder() { return m_leftWheelEncoder; }
  
  /**
   * gets the right wheel encoder object
   * @return right wheel encoder object
   */
  public Encoder getRightWheelEncoder() { return m_rightWheelEncoder; }

  /**
   * resets each wheel encoder using the {@link edu.wpi.first.wpilibj.Encoder} reset method
   */
  public void resetWheelEncoders() {
    m_leftWheelEncoder.reset();
    m_rightWheelEncoder.reset();
  }

  /**
   * gets the current position of the robot
   * @return x and y coordinates of the robot on the field
   */
  public Pose2d getPose() { return m_odometry.getPoseMeters(); }

  /**
   * gets the wheel speeds of a differential drive
   * @return left and right wheel speeds in m/s
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftWheelEncoder.getRate(), m_rightWheelEncoder.getRate());
  }

  /**
   * resets the gyro heading to 0 degrees
   */
  public void zeroHeading() { m_gyro.reset(); }

  /**
   * gets the heading of the gyro
   * @return current angle of the gyro
   */
  public double getHeading() { return m_gyro.getAngle(); }

  /**
   * gets the current turning rate of the gyro
   * @return the turn rate in degrees per second
   */
  public double getTurnRate() { return m_gyro.getRate(); }

  /**
   * gets the gyro object
   * @return navX gyro object
   */
  public Gyro getGyro() {
    return m_gyro;
  }

  /**
   * Method to drive the robot via applied voltage. Also will feed the motor safety object in the differential drive to remove timeout errors
   *
   * @param leftVolts Voltage to be applied to the left motors
   * @param rightVolts Voltage to be applied to the right motors
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftLead.setVoltage(leftVolts);
    m_rightLead.setVoltage(rightVolts);
    m_drive.feed();
  }

  /**
   * Resets the odometry for the robot. Uses the current rotation of the gyro and the supplied pose.
   * This will also reset the wheel encoders to 0.
   * @param pose current location of the robot
   */
  public void resetOdometry(Pose2d pose) {
    resetWheelEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(-m_gyro.getAngle()));
  }
  

  @Override
  public void periodic() {
  // Update the odometry in the periodic block as the robot is moving
  m_odometry.update(Rotation2d.fromDegrees(-m_gyro.getAngle()), m_leftWheelEncoder.getDistance(), m_rightWheelEncoder.getDistance());

  netGyroAngle.setValue(-m_gyro.getAngle());

  var translation = m_odometry.getPoseMeters().getTranslation();
  m_xEntry.setNumber(translation.getX());
  m_yEntry.setNumber(translation.getY());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // Old version of PathPlanner trajectory following NOT IN USE
  // // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
  // public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {

  //   return new SequentialCommandGroup(
  //       new InstantCommand(() -> {
  //         // Reset odometry for the first path you run during auto
  //         if(isFirstPath){
  //             this.resetOdometry(traj.getInitialPose());
  //         }
  //       }),
  //       new PPRamseteCommand(
  //           traj, 
  //           this::getPose, // Pose supplier
  //           new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
  //           new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter),
  //           this.kinematics, // DifferentialDriveKinematics
  //           this::getWheelSpeeds,
  //           new PIDController(kPDriveVel, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
  //           new PIDController(kPDriveVel, 0, 0), // Right controller (usually the same values as left controller)
  //           this::tankDriveVolts, // Voltage biconsumer
  //           // SystemConstants.eventMap, // This argument is optional if you don't use event markers
  //           this // Requires this drive subsystem
  //       )
  //   );
  // }

}


