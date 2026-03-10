package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.Vision.PoseObservation;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public PoseObservation poseObservation;
    public boolean isCameraConnected = false;
    public Pose3d[] targetPoses = new Pose3d[] {};
  }

  public default String getName() {
    return "Camera";
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
