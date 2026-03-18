// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.*;
import static frc.robot.Constants.DriveConstants.*;

public class CANDriveSubsystem extends SubsystemBase {
  private final SparkMax leftLeader;
  private final SparkMax leftFollower;
  private final SparkMax rightLeader;
  private final SparkMax rightFollower;

  private final DifferentialDrive drive;

  //Camera Setup
  PhotonCamera camera = new PhotonCamera("photonvision"); //create camera
  PIDController drivePid = new PIDController(0.4, 0, 0); //PID loop for range
  PIDController turnPid = new PIDController(0.1, 0, 0);  //PID loop for rotation
  Translation3d robotToCameraTrl = new Translation3d(-0.2, 0, 0.2); //Measure on robot
  Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-45), Math.toRadians(180)); //Measure on Robot
  Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot); //Set Robot to camera transform
  Pose2d BlueHubPose = new Pose2d(4.62,4.03,new Rotation2d().fromDegrees(0)); //Position of Blue Hub
  Pose2d RedHubPose = new Pose2d(11.91,4.03,new Rotation2d().fromDegrees(180)); //Posirion of Red Hub
  AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  //Publish 3d robot position to Network Table
  StructPublisher<Pose3d> pubPose = NetworkTableInstance.getDefault().getStructTopic("robotPose3D",Pose3d.struct).publish();

  public CANDriveSubsystem() {
    // create brushed motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushed);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushed);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushed);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushed);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
  }

  // Command factory to create command to drive the robot with joystick inputs.
  public Command driveArcade(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
    return this.run(
        () -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
  }

    public void autoAlign(){
     // Read in relevant data from the Camera
        boolean PoseEstEnable = true; //0 = align to apriltag, 1=Pose estimation
        double targetYaw = 0.0;
        double targetRange = 0.0;
        Rotation2d PoseTargetYaw =  new Rotation2d(0); 
        double PoseTargetRange = 0.0;
        double PoseYaw;
        double forward = 0.0;
        double turn = 0.0;
        boolean ReadyToAlign=false;
        var results = camera.getAllUnreadResults();

        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (PoseEstEnable == true){
                      Pose3d robotPose3d = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), tagLayout.getTagPose(target.getFiducialId()).get(), robotToCamera);
                      Pose2d robotPose2d = robotPose3d.toPose2d();
                      pubPose.set(robotPose3d);
                      if (target.getFiducialId()>23 && target.getFiducialId()<28) {
                        PoseTargetYaw = PhotonUtils.getYawToPose(robotPose2d, BlueHubPose);
                        PoseTargetRange = PhotonUtils.getDistanceToPose(robotPose2d, BlueHubPose);
                        ReadyToAlign = true;
                      }
                      if (target.getFiducialId()>7 && target.getFiducialId()<12) {
                        PoseTargetYaw  = PhotonUtils.getYawToPose(robotPose2d, RedHubPose);
                        PoseTargetRange = PhotonUtils.getDistanceToPose(robotPose2d, RedHubPose);
                        ReadyToAlign = true;
                      }

                      //Find Rotation from center of hub
                      PoseYaw = 180-PoseTargetYaw.getDegrees();
                      if (PoseTargetYaw.getDegrees()<0){
                        PoseYaw = (-PoseTargetYaw.getDegrees()-180);
                      }

                      if(ReadyToAlign == true){
                        forward = -1 * MathUtil.clamp(drivePid.calculate(1.1-PoseTargetRange,0),-0.6,0.6);
                        turn = MathUtil.clamp(turnPid.calculate(PoseYaw,0), -0.4, 0.4);
                    }
                    }
                    if (PoseEstEnable == false){

                    
                      if (target.getFiducialId()== 10 || target.getFiducialId()==26) {
                        //Found Hub center tag, record its information
                        targetYaw = target.getYaw();
                        targetRange =
                                PhotonUtils.calculateDistanceToTargetMeters(
                                        0.5, // Measured with a tape measure, or in CAD.
                                        1.123, // From 2024 game manual for ID 7
                                        Units.degreesToRadians(15.0), // Measured with a protractor, or in CAD.
                                        Units.degreesToRadians(target.getPitch()));

                        SmartDashboard.putNumber("Yaw to target", targetYaw);
                        SmartDashboard.putNumber("Dist to target", targetRange);
                        forward = -1* drivePid.calculate(0.7-targetRange,0);
                        turn = 0.4 * turnPid.calculate(targetYaw,0);
                      }
                        

                    }
                    drive.arcadeDrive(forward, turn); //drive based on results
                }
            }
        }
    }
  public Command autoAlignCommand() {
    return this.run(() -> autoAlign());
  }
}
