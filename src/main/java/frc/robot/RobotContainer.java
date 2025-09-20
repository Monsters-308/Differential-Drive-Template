// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.CommandFlightHotasX;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystem defined here
    private final DriveTrain m_driveTrain = new DriveTrain();

    // controllers
    private final CommandFlightHotasX m_driverController = new CommandFlightHotasX(
            OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_coDriverController = new CommandXboxController(
            OperatorConstants.kCoDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();

        // prevent robot from moving until throttle is back to zero
        Command safetyCommand = new WaitUntilCommand(() -> m_driverController.getThrottle() == 0)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        safetyCommand.addRequirements(m_driveTrain);

        safetyCommand.schedule();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        m_driveTrain.setDefaultCommand(
                m_driveTrain.driveJoysticks(m_driverController::getThrottle,
                        m_driverController::getStickX,
                        m_driverController.getHID()::getR1Button));

        CommandScheduler.getInstance().schedule();
    }

    /**
     * Sets the idle mode of the drive motors.
     * 
     * @param mode The idle mode to set.
     */
    public void setDriveIdleMode(IdleMode mode) {
        m_driveTrain.setIdleMode(mode);
    }
}
