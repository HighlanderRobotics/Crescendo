// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.util.protobuf.ProtobufSerializable;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.proto.PhotonTrackedTargetProto;

/** Add your docs here. */
public class PhotonTrackedTargetProtobufWrapper extends PhotonTrackedTarget
    implements ProtobufSerializable {
  public PhotonTrackedTargetProtobufWrapper(PhotonTrackedTarget photonTrackedTarget) {
    super(
        photonTrackedTarget.getYaw(),
        photonTrackedTarget.getPitch(),
        photonTrackedTarget.getArea(),
        photonTrackedTarget.getSkew(),
        photonTrackedTarget.getFiducialId(),
        photonTrackedTarget.getBestCameraToTarget(),
        photonTrackedTarget.getAlternateCameraToTarget(),
        photonTrackedTarget.getPoseAmbiguity(),
        photonTrackedTarget.getMinAreaRectCorners(),
        photonTrackedTarget.getDetectedCorners());
  }

  public static final PhotonTrackedTargetProto proto = PhotonTrackedTarget.proto;
}
