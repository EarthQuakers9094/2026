// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopperservo.HopperServoSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualServoMoveCommand extends Command {
  public enum ServoMoveDirection {
    TOWARDS_180,
    TOWARDS_0
  }

  private final HopperServoSubsystem servo;
  private final ServoMoveDirection direction;

  public ManualServoMoveCommand(HopperServoSubsystem servo, ServoMoveDirection direction) {
    this.servo = servo;
    this.direction = direction;
    addRequirements(servo);
  }

  @Override
  public void initialize() {
    Commands.print("Servo manual controller enabled");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double position = servo.getPWMPos();

    if (direction == ServoMoveDirection.TOWARDS_180) {
      servo.setSetpointPWM(position + 0.05);
    } else if (direction == ServoMoveDirection.TOWARDS_0) {
      servo.setSetpointPWM(position - 0.05);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
