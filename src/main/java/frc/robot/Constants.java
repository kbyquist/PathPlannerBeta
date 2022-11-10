// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;

public final class Constants {
    public static final class SystemConstants {
        public static final int kPcmCanID = 33;
        public static HashMap<String, Command> eventMap = new HashMap<String, Command>();
    }

    public static final class DriveConstants {
        public static final int[] kLeftDriveMotors = new int[] {0,1,2};
        public static final int[] kRightDriveMotors = new int[] {3,4,5};

        public static final boolean kLeftMotorInverted = true;
        public static final boolean kRightMotorInverted = false;

        public static final int[] kLeftWheelEncoderPorts = new int[] {0, 1};
        public static final int[] kRightWheelEncoderPorts = new int[] {2, 3};
        public static final boolean kLeftWheelEncoderReversed = true;
        public static final boolean kRightWheelEncoderReversed = false;
        public static final int kWheelEncoderCountsPerRevolution = 2048;

        public static final double kWheelDiameterIN = 6.;

        public static final String kDriveTabName = "Drive Subsystem";
        public static final double kTrackwidthMeters = 0.69;

        public static final double kWheelDiameterMeters = kWheelDiameterIN/39.37;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterMeters * Math.PI) / (double) kWheelEncoderCountsPerRevolution;

        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = .0002936;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0010696;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;
    }

    public static final class OIConstants {
        public static final int kDriveControllerInput = 0;
        public static final int kOperatorControllerInput = 1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class IntakeConstants {
        public static final int[] kIntakeSolenoidID = new int[] {0,1};
    }
}