/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  PhotonCamera visionCamera = new PhotonCamera("Microsoft LifeCam HD-3000");
  PhotonPipelineResult powercellPipeline = visionCamera.getLatestResult();
  PhotonTrackedTarget powercellTarget = powercellPipeline.getBestTarget();
  
  /** 
   * Creates a new VisionSubsystem.
   */
  public VisionSubsystem() {

  }

  /**
   * A way to get the distance between the robot and the powercell
   * @return the distance between the robot and the powercell
   */
  public double getTargetDistance() {
    return powercellTarget.getCameraToTarget().getY();
  }

  /**
   * Gets the angle of the powercell relative to the robot
   * @return the angle of the powercell to the robot in degrees
   */
  public double getAngle(){
    return powercellTarget.getCameraToTarget().getRotation().getDegrees();
  }

  /**
   * Gets the yaw displacement of the powercell relative to the robot
   * @return the yaw of the powercell relative to the robot
   */
   public double getTargetYaw() {
    return powercellTarget.getYaw();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
