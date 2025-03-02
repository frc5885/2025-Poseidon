// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.WaitUntilFarFromCommand;
import frc.robot.subsystems.drive.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class JustDrive extends SequentialCommandGroup {
  /** Creates a new JustDrive. */
  public JustDrive(Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new WaitUntilFarFromCommand(drive::getPose, 1.5)
            .deadlineFor(
                new RunCommand(() -> drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0)), drive)));
  }
}
