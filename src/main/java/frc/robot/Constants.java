// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;

// All constants should have names that are easily understood to their use

public final class Constants {

    // Class containing any constants that need to be accesible in multiple subsystems or are relevant system wide
    public static final class SystemConstants {
        public static final int kPcmCanID = 33;
        public static HashMap<String, Command> eventMap = new HashMap<String, Command>();
    }

    // Class containing any constant that directly relates to any functions of the drive subsystem
    public static final class DriveConstants {
        public static final int[] kLeftDriveMotors = new int[] {4,5,6};
        public static final int[] kRightDriveMotors = new int[] {3,7,8};

        public static final boolean kLeftMotorInverted = true;
        public static final boolean kRightMotorInverted = false;

        public static final int[] kLeftWheelEncoderPorts = new int[] {0, 1};
        public static final int[] kRightWheelEncoderPorts = new int[] {2, 3};
        public static final boolean kLeftWheelEncoderReversed = true;
        public static final boolean kRightWheelEncoderReversed = false;
        public static final int kWheelEncoderCountsPerRevolution = 2048;

        public static final double kWheelDiameterIN = 6.25;

        public static final String kDriveTabName = "Drive Subsystem";
        public static final double kTrackwidthMeters = 1.0895;

        public static final double kWheelDiameterMeters = kWheelDiameterIN/39.37;
        public static final double kEncoderDistancePerPulse =
            // Encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kWheelEncoderCountsPerRevolution;

        public static final double ksVolts = 0.0091269;
        public static final double kvVoltSecondsPerMeter = 1.8872;
        public static final double kaVoltSecondsSquaredPerMeter = 1.0531;

        // Tuned for current code test base
        public static final double kPDriveVel = 2.776;
    }

    // Class containing constants for the controller input USB ID
    public static final class OIConstants {
        public static final int kDriveControllerInput = 0;
        public static final int kOperatorControllerInput = 1;
    }

    // Class containing constants that are relevant to autonomous construction
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = .25;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    // Class containing any constant that directly relates to the intake subsystem
    public static final class IntakeConstants {
        public static final int[] kIntakeSolenoidID = new int[] {0,1};
    }
}