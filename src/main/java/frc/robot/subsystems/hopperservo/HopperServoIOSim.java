package frc.robot.subsystems.hopperservo;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class HopperServoIOSim implements HopperServoIO {
  public void setSetpoint(Angle setpoint) {
    System.out.println(setpoint.in(Degrees));
  }
}
