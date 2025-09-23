// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * 
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    private Constants() {
        throw new UnsupportedOperationException("This is a constants class!");
    }

    public static class OperatorConstants {
        private OperatorConstants() {
            throw new UnsupportedOperationException("This is a constants class!");
        }

        // ports for the controllers
        public static final int kDriverControllerPort = 0;
        public static final int kCoDriverControllerPort = 1;
    }

    public static class DriveConstants {
        private DriveConstants() {
            throw new UnsupportedOperationException("This is a constants class!");
        }

        // ids for the motors
        public static final int kFrontLeftMotorId = 3;
        public static final int kFrontRightMotorId = 5;
        public static final int kBackLeftMotorId = 2;
        public static final int kBackRightMotorId = 4;

        // sets if an motor is inverted
        public static final boolean kLeftMotorsInverted = false;
        public static final boolean kRightMotorsInverted = true;

        // idle mode
        public static final IdleMode kMotorIdleMode = IdleMode.kBrake;

        // smartLimit
        public static final int kSmartCurrentLimit = 30;

        public static final double kMaxSpeedMetersPerSecond = 5.6;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5.6;

        // PID constants for controlling wheel velocity
        public static final double kVelocityP = 0.1;
        public static final double kVelocityI = 0.0;
        public static final double kVelocityD = 0.0;
        public static final double kVelocityFF = 1 / 473;

        // physical constants
        public static final double kWheelRadius = Units.inchesToMeters(3);
        public static final double kWheelDiameter = 2 * kWheelRadius;
        public static final double kWheelCircumference = kWheelDiameter * Math.PI;
        public static final double kGearReduction = 8.45865;

        public static final double kTrackWidth = 0.5;

        // encoder conversion factors
        public static final double kRotationsToMeters = kWheelCircumference / kGearReduction;
        public static final double kRotationsPerMinuteToMetersPerSecond = kRotationsToMeters / 60;

        // kinematics
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);

        // controller axis deadbands, sensitivities, and multipliers
        public static final double kDeadBand = 0.01;

        public static final double kForwardAxisSensitvity = 0.8;
        public static final double kRotatonAxisSenitvity = 0.8;

        public static final double kForwardAxisMultiplier = 1;
        public static final double kTurningAxisMultiplier = 1;
    }

    public static class AutoConstants {
        private AutoConstants() {
            throw new UnsupportedOperationException("This is a constants class!");
        }

        public static final double kMassKG = 50;
        public static final double kRobotMOI = 50;

        public static final double kWheelCOF = 1;

        public static final DCMotor kDriveMotorGearbox = DCMotor.getNEO(2)
                .withReduction(DriveConstants.kGearReduction);

        public static final double kTrueMaxSpeedMetersPerSecond = 0.85 * NEOMotorConstants.kFreeSpeedRPM
                * DriveConstants.kRotationsPerMinuteToMetersPerSecond;

        public static final RobotConfig kRobotConfig = new RobotConfig(kMassKG, kRobotMOI,
                new ModuleConfig(DriveConstants.kWheelRadius, kTrueMaxSpeedMetersPerSecond,
                        kWheelCOF, kDriveMotorGearbox, DriveConstants.kSmartCurrentLimit, 2),
                DriveConstants.kTrackWidth);

        public static final PathFollowingController kAutoController = new PPLTVController(TimedRobot.kDefaultPeriod,
                DriveConstants.kMaxSpeedMetersPerSecond);
    }

    public static class NEOMotorConstants {
        private NEOMotorConstants() {
            throw new UnsupportedOperationException("This is a constants class!");
        }

        public static final double kFreeSpeedRPM = 5676;
        public static final double kStallTorqueNm = 2.6;
        public static final double kStallCurrentAmps = 105;
        public static final double kFreeCurrentAmps = 1.5;
    }
}