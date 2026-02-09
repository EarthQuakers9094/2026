package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.Vision.PoseObservation;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public PoseObservation poseObservation;
    public boolean isCameraConnected = false;
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
