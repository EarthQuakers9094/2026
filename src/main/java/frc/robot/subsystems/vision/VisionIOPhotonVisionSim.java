package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {

  private final PhotonCameraSim cameraSim;
  private static VisionSystemSim visionSim;
  private final Supplier<Pose2d> poseSupplier;
  private final String name;

  public VisionIOPhotonVisionSim(
      String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
    super(name, robotToCamera);

    this.name = name;

    this.poseSupplier = poseSupplier;

    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(Constants.Field.aprilTagLayout);
    }

    SimCameraProperties cameraProperties = new SimCameraProperties();
    cameraProperties.setFPS(30);
    cameraProperties.setCalibration(320, 240, Rotation2d.fromDegrees(70));
    cameraProperties.setCalibError(0.05, 0.01);
    cameraProperties.setAvgLatencyMs(30);

    cameraSim = new PhotonCameraSim(camera, cameraProperties, Constants.Field.aprilTagLayout);
    cameraSim.enableDrawWireframe(true);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    Logger.recordOutput(
        "Vision/" + name + "Location", new Pose3d(poseSupplier.get()).plus(robotToCamera));

    visionSim.update(poseSupplier.get());

    super.updateInputs(inputs);
  }
}
