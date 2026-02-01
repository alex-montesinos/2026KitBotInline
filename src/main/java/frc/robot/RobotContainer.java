// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Commands;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.FuelConstants.*;
import frc.robot.commands.Autos;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final CANDriveSubsystem driveSubsystem = new CANDriveSubsystem();
  private final CANFuelSubsystem ballSubsystem = new CANFuelSubsystem();

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Limit acceleration to 3 units per second (0 to 100% in ~0.33 seconds)
  private final SlewRateLimiter speedLimiter = new SlewRateLimiter(3.0);
  // You can often set rotation higher (sharper turns) or the same. 
  private final SlewRateLimiter turnLimiter = new SlewRateLimiter(3.0);

  //Tracks if robot is driving backwards.
  private boolean isDriveReversed = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureBindings();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    autoChooser.setDefaultOption("Autonomous", Autos.exampleAuto(driveSubsystem, ballSubsystem));
    autoChooser.addOption("Mobility Only (Just Drive)", Autos.mobilityAuto(driveSubsystem)); // Adds "Mobility Only" to the dropdown menu on the driver station
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // While the left bumper on operator controller is held, intake Fuel
    operatorController.leftBumper()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    operatorController.rightBumper()
        .whileTrue(ballSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
            .andThen(ballSubsystem.launchCommand())
            .finallyDo(() -> ballSubsystem.stop()));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    operatorController.a()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop()));
    // When Driver presses 'X', flip the direction
    driverController.x().onTrue(Commands.runOnce(() -> isDriveReversed = !isDriveReversed));

// Turbo and Precision Modes
driveSubsystem.setDefaultCommand(
    driveSubsystem.driveArcade(
        () -> {
            // 1. Get raw input
            double input = -driverController.getLeftY();

            // 2. REVERSE LOGIC: If the mode is active, flip the sign
            // This makes the Intake the "Front" or the Shooter the "Front"
            if (isDriveReversed) {
                input = -input;
            }

            // 3. TURBO LOGIC
            double scale = 0.7; // Default
            if (driverController.getRightTriggerAxis() > 0.5) scale = 1.0;
            else if (driverController.getLeftTriggerAxis() > 0.5) scale = 0.35;

            double targetSpeed = input * scale;

            // 4. SLEW RATE LIMITER (Smooths the movement)
            return speedLimiter.calculate(targetSpeed);
        },
        () -> {
            // Turning usually stays the same (Right Stick Right = Clockwise)
            // regardless of which side is "front"
            double turnInput = -driverController.getRightX();
            
            // Apply scale/slew logic to turning
            double turnScale = 0.7; 
            if (driverController.getRightTriggerAxis() > 0.5) turnScale = 1.0;
            
            return turnLimiter.calculate(turnInput * turnScale);
        }
    )
);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
