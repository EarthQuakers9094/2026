package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MovingAverage;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final MovingAverage speedAverage = new MovingAverage(40);

  @AutoLogOutput private boolean isShooting = false;
  @AutoLogOutput private boolean isIndexing = false;

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  public void beginShooting() {
    isShooting = true;
    io.setVelocitySetpoint(Constants.ShooterConstants.launchSpeed);
    Logger.recordOutput(
        "Shooter/SpeedSetpointRadPerSec",
        Constants.ShooterConstants.launchSpeed.in(RadiansPerSecond));
  }

  public void endShooting() {
    isShooting = false;
    io.setVelocitySetpoint(RPM.of(0.0));
    Logger.recordOutput("Shooter/SpeedSetpointRadPerSec", 0.0);
  }

  public boolean isActivelyShooting() {
    return this.isIndexing;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    double averageSpeed = speedAverage.addValue(inputs.shooterSpeed.baseUnitMagnitude());

    Logger.recordOutput("Shooter/AverageSpeed", averageSpeed);
    if (isShooting
        && averageSpeed >= Constants.ShooterConstants.minLaunchSpeed.baseUnitMagnitude()
        && speedAverage.getStandardDeviation(averageSpeed) <= 12.) {
      io.startIndexing();
      isIndexing = true;

    } else {
      io.stopIndexing();
      isIndexing = false;
    }
    Logger.processInputs("Shooter", inputs);
  }

  public AngularVelocity getShooterSpeed() {
    return inputs.shooterSpeed;
  }

  public void setPitch(Rotation2d pitch) {
    io.setPitch(pitch);
  }

  public void setYaw(Rotation2d yaw) {
    io.setYaw(yaw);
  }
}
