// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopperservo;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class HopperServoSubsystem extends SubsystemBase {
  private final HopperServoIO io;
  private final HopperServoIOInputsAutoLogged inputs = new HopperServoIOInputsAutoLogged();

  public HopperServoSubsystem(HopperServoIO io) {
    this.io = io;
  }

  public void setSetpoint(Angle setpoint) {
    io.setSetpoint(setpoint);
  }

  public void setSetpointPWM(double PWM) {
    if (PWM > 1.0) {
      io.setSetpointPWM(1.0);
      return;
    }
    if (PWM < 0.0) {
      io.setSetpointPWM(0.0);
      return;
    }
    io.setSetpointPWM(PWM);
  }

  public double getPWMPos() {
    return io.getPWMPos();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }
}
