// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Comparator;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

public class CameraSubsystem extends SubsystemBase {
  public final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("Camera");
  PhotonCamera m_camera = new PhotonCamera("FuelCam");

  /** Creates a new RubikPiSubsystem. */
  public CameraSubsystem() {
  }

  public List<TargetCorner> getTargetCorners(PhotonTrackedTarget target) {
    List<TargetCorner> output = target.getMinAreaRectCorners();
    output.sort(Comparator.comparingDouble((TargetCorner corner) -> {
      return corner.x;
    }).thenComparingDouble((TargetCorner corner) -> {
      return corner.y;
    })); // Top left is 0, bottom left is 1, top right is 2, bottom right is 3 (probably)

    return output;
  }

  public double getDistanceToTarget(double targetY) {
    double pitchToTarget = degreesToRadians((targetY / CameraConstants.cameraResY) * CameraConstants.cameraFovY
        - (CameraConstants.cameraFovY / 2));

    double distance = (CameraConstants.cameraHeight - CameraConstants.fuelRadius) / Math.tan(pitchToTarget);

    return distance;
  }

  public double degreesToRadians(double degrees) {
    return degrees * Math.PI / 180;
  }

  @Override
  public void periodic() {
    List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();

    for (int i = 0; i < results.size(); i++) {
      if (results.get(i).hasTargets()) {
        PhotonTrackedTarget target = results.get(i).getBestTarget();
        List<TargetCorner> targetCorners = getTargetCorners(target);

        m_table.putValue("Top Left X", NetworkTableValue.makeDouble(targetCorners.get(0).x));
        m_table.putValue("Top Left Y", NetworkTableValue.makeDouble(targetCorners.get(0).y));
        m_table.putValue("Bottom Left X", NetworkTableValue.makeDouble(targetCorners.get(1).x));
        m_table.putValue("Bottom Left Y", NetworkTableValue.makeDouble(targetCorners.get(1).y));
        m_table.putValue("Top Right X", NetworkTableValue.makeDouble(targetCorners.get(2).x));
        m_table.putValue("Top Right Y", NetworkTableValue.makeDouble(targetCorners.get(2).y));
        m_table.putValue("Bottom Right X", NetworkTableValue.makeDouble(targetCorners.get(3).x));
        m_table.putValue("Bottom Right Y", NetworkTableValue.makeDouble(targetCorners.get(3).y));

        // double targetX = (targetCorners.get(1).x + targetCorners.get(3).x) / 2;
        double targetBottomY = targetCorners.get(1).y;
        double targetTopY = targetCorners.get(0).y;

        double distance = getDistanceToTarget((targetBottomY + targetTopY) / 2);

        m_table.putValue("Distance", NetworkTableValue.makeDouble(distance));
      }
    }
  }
}
