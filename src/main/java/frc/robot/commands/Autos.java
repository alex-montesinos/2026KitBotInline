// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CANDriveSubsystem;

public final class Autos {
  // Example autonomous command which drives forward for 1 second.
  public static final Command exampleAuto(CANDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
    return new SequentialCommandGroup(
        // Drive backwards for .25 seconds. The driveArcadeAuto command factory
        // creates a command which does not end which allows us to control
        // the timing using the withTimeout decorator
        driveSubsystem.driveArcade(() -> 0.5, () -> 0).withTimeout(.25),
        // Stop driving. This line uses the regular driveArcade command factory so it
        // ends immediately after commanding the motors to stop
        driveSubsystem.driveArcade(() -> 0, () -> 0),
        // Spin up the launcher for 1 second and then launch balls for 9 seconds, for a
        // total of 10 seconds
        ballSubsystem.spinUpCommand().withTimeout(1),
        ballSubsystem.launchCommand().withTimeout(9),
        // Stop running the launcher
        ballSubsystem.runOnce(() -> ballSubsystem.stop()));
  }
  // A simple auto that just drives out of the community zone for points.
  // Useful if the shooter is broken or you are starting far away.
  public static final Command mobilityAuto(CANDriveSubsystem driveSubsystem) {
    return new SequentialCommandGroup(
        // Drive at 50% speed for 2.0 seconds. 
        // TIP: Test this on the field! 2.0 seconds is usually safe to clear the line.
        driveSubsystem.driveArcade(() -> 0.5, () -> 0).withTimeout(2.0),
        
        // Stop the motors immediately after the time is up
        driveSubsystem.driveArcade(() -> 0, () -> 0)
    );
  }
}
