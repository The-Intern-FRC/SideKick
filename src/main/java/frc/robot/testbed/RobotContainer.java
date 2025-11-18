// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.testbed;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generic.subsystems.drive.Drive;
import frc.robot.generic.util.AbstractRobotContainer;
import frc.robot.generic.util.RobotConfig;
import frc.robot.generic.util.SwerveBuilder;
import frc.robot.testbed.subsystems.TestMotor;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Testbed robot container for testing motors and prototypes without programming. Uses generic
 * swerve drive as base with additional test motor capabilities.
 */
public class RobotContainer implements AbstractRobotContainer {
  public static RobotConfig config = RobotConfig.defaultCommandBased(RobotContainer::new);

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController codriverController = new CommandXboxController(1);

  // Subsystems - using generic swerve drive
  private final Drive drive = SwerveBuilder.buildDefaultDrive(driverController);

  // Test motors for prototype testing
  // 4 motors for joystick control (CAN IDs 10-13)
  private final TestMotor joystickMotor1 = new TestMotor(10, "JoystickMotor1");
  private final TestMotor joystickMotor2 = new TestMotor(11, "JoystickMotor2");
  private final TestMotor joystickMotor3 = new TestMotor(12, "JoystickMotor3");
  private final TestMotor joystickMotor4 = new TestMotor(13, "JoystickMotor4");

  // Paddle motors (CAN IDs 14-15)
  private final TestMotor paddleMotor1 = new TestMotor(14, "PaddleMotor1");
  private final TestMotor paddleMotor2 = new TestMotor(15, "PaddleMotor2");

  // Button-controlled motors with dashboard settings (CAN IDs 20-23)
  private final TestMotor buttonMotor1 = new TestMotor(20, "ButtonMotor1");
  private final TestMotor buttonMotor2 = new TestMotor(21, "ButtonMotor2");
  private final TestMotor buttonMotor3 = new TestMotor(22, "ButtonMotor3");
  private final TestMotor buttonMotor4 = new TestMotor(23, "ButtonMotor4");

  // Dashboard
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the testbed robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set up auto routines (using generic implementation)
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure button bindings
    configureButtonBindings();
  }

  /** Configures button bindings for testbed motor control. */
  private void configureButtonBindings() {
    // Codriver controller bindings:

    // Left joystick Y-axis -> JoystickMotor1 (vertical movement)
    codriverController
        .axisGreaterThan(1, 0.1)
        .whileTrue(
            Commands.run(() -> joystickMotor1.setVoltage(-codriverController.getLeftY() * 12.0)));
    codriverController
        .axisLessThan(1, -0.1)
        .whileTrue(
            Commands.run(() -> joystickMotor1.setVoltage(-codriverController.getLeftY() * 12.0)));
    codriverController.axisLessThan(1, 0.1).and(codriverController.axisGreaterThan(1, -0.1))
        .whileTrue(Commands.run(joystickMotor1::stop));

    // Left joystick X-axis -> JoystickMotor2 (horizontal movement)
    codriverController
        .axisGreaterThan(0, 0.1)
        .whileTrue(
            Commands.run(() -> joystickMotor2.setVoltage(-codriverController.getLeftX() * 12.0)));
    codriverController
        .axisLessThan(0, -0.1)
        .whileTrue(
            Commands.run(() -> joystickMotor2.setVoltage(-codriverController.getLeftX() * 12.0)));
    codriverController.axisLessThan(0, 0.1).and(codriverController.axisGreaterThan(0, -0.1))
        .whileTrue(Commands.run(joystickMotor2::stop));

    // Right joystick Y-axis -> JoystickMotor3
    codriverController
        .axisGreaterThan(5, 0.1)
        .whileTrue(
            Commands.run(() -> joystickMotor3.setVoltage(-codriverController.getRightY() * 12.0)));
    codriverController
        .axisLessThan(5, -0.1)
        .whileTrue(
            Commands.run(() -> joystickMotor3.setVoltage(-codriverController.getRightY() * 12.0)));
    codriverController.axisLessThan(5, 0.1).and(codriverController.axisGreaterThan(5, -0.1))
        .whileTrue(Commands.run(joystickMotor3::stop));

    // Right joystick X-axis -> JoystickMotor4
    codriverController
        .axisGreaterThan(4, 0.1)
        .whileTrue(
            Commands.run(() -> joystickMotor4.setVoltage(-codriverController.getRightX() * 12.0)));
    codriverController
        .axisLessThan(4, -0.1)
        .whileTrue(
            Commands.run(() -> joystickMotor4.setVoltage(-codriverController.getRightX() * 12.0)));
    codriverController.axisLessThan(4, 0.1).and(codriverController.axisGreaterThan(4, -0.1))
        .whileTrue(Commands.run(joystickMotor4::stop));

    // Left trigger (paddle) -> PaddleMotor1 and PaddleMotor2 forward
    codriverController
        .leftTrigger(0.1)
        .whileTrue(
            Commands.parallel(
                Commands.run(
                    () -> {
                      double value = codriverController.getLeftTriggerAxis();
                      paddleMotor1.setVoltage(value * 12.0);
                      paddleMotor2.setVoltage(value * 12.0);
                    }),
                Commands.run(paddleMotor1::stop).finallyDo(paddleMotor1::stop),
                Commands.run(paddleMotor2::stop).finallyDo(paddleMotor2::stop)));

    // Right trigger (paddle) -> PaddleMotor1 and PaddleMotor2 backward
    codriverController
        .rightTrigger(0.1)
        .whileTrue(
            Commands.parallel(
                Commands.run(
                    () -> {
                      double value = codriverController.getRightTriggerAxis();
                      paddleMotor1.setVoltage(-value * 12.0);
                      paddleMotor2.setVoltage(-value * 12.0);
                    }),
                Commands.run(paddleMotor1::stop).finallyDo(paddleMotor1::stop),
                Commands.run(paddleMotor2::stop).finallyDo(paddleMotor2::stop)));

    // Button bindings for motors with dashboard control:
    // A button -> Enable velocity control for ButtonMotor1
    codriverController.a().onTrue(buttonMotor1.enableVelocityControlCommand());
    codriverController.a().onFalse(buttonMotor1.stopCommand());

    // B button -> Enable velocity control for ButtonMotor2
    codriverController.b().onTrue(buttonMotor2.enableVelocityControlCommand());
    codriverController.b().onFalse(buttonMotor2.stopCommand());

    // X button -> Enable position control for ButtonMotor3
    codriverController.x().onTrue(buttonMotor3.enablePositionControlCommand());
    codriverController.x().onFalse(buttonMotor3.stopCommand());

    // Y button -> Enable position control for ButtonMotor4
    codriverController.y().onTrue(buttonMotor4.enablePositionControlCommand());
    codriverController.y().onFalse(buttonMotor4.stopCommand());

    // Left bumper -> Stop all paddle motors
    codriverController
        .leftBumper()
        .onTrue(
            Commands.parallel(paddleMotor1.stopCommand(), paddleMotor2.stopCommand()));

    // Right bumper -> Stop all joystick motors
    codriverController
        .rightBumper()
        .onTrue(
            Commands.parallel(
                joystickMotor1.stopCommand(),
                joystickMotor2.stopCommand(),
                joystickMotor3.stopCommand(),
                joystickMotor4.stopCommand()));

    // Back button -> Stop all test motors
    codriverController
        .back()
        .onTrue(
            Commands.parallel(
                joystickMotor1.stopCommand(),
                joystickMotor2.stopCommand(),
                joystickMotor3.stopCommand(),
                joystickMotor4.stopCommand(),
                paddleMotor1.stopCommand(),
                paddleMotor2.stopCommand(),
                buttonMotor1.stopCommand(),
                buttonMotor2.stopCommand(),
                buttonMotor3.stopCommand(),
                buttonMotor4.stopCommand()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link
   * frc.robot.generic.Robot} class.
   *
   * @return the command to run in autonomous
   */
  @Override
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
