// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DriveConstants.DRIVE_GEAR_RATIO;
import static frc.robot.Constants.DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.DriveConstants.LEFT_FOLLOWER_ID;
import static frc.robot.Constants.DriveConstants.LEFT_LEADER_ID;
import static frc.robot.Constants.DriveConstants.PIGEON2_ID;
import static frc.robot.Constants.DriveConstants.RIGHT_FOLLOWER_ID;
import static frc.robot.Constants.DriveConstants.RIGHT_LEADER_ID;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightAlign;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class CANDriveSubsystem extends SubsystemBase {
  private final CANBus kCanBus = new CANBus("rio");

  private final double kGearRatio = DRIVE_GEAR_RATIO;
  private final Distance kWheelRadius = Inches.of(3);

  private final TalonFX rightLeader = new TalonFX(RIGHT_LEADER_ID, kCanBus);
  private final TalonFX leftLeader = new TalonFX(LEFT_LEADER_ID, kCanBus);
  private final TalonFX rightFollower = new TalonFX(RIGHT_FOLLOWER_ID, kCanBus);
  private final TalonFX leftFollower = new TalonFX(LEFT_FOLLOWER_ID, kCanBus);

  private final Pigeon2 pigeon2 = new Pigeon2(PIGEON2_ID, kCanBus);

  private final DutyCycleOut rightOut = new DutyCycleOut(0);
  private final DutyCycleOut leftOut = new DutyCycleOut(0);

  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(pigeon2.getRotation2d(), 0, 0);

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.550);

  private final Field2d field = new Field2d();

  public CANDriveSubsystem() {
    initializeRightDrive(rightLeader.getConfigurator());
    initializeRightDrive(rightFollower.getConfigurator());
    initializeLeftDrive(leftLeader.getConfigurator());
    initializeLeftDrive(leftFollower.getConfigurator());
    initializePigeon2(pigeon2.getConfigurator());

    leftFollower.setControl(new Follower(leftLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    rightFollower.setControl(new Follower(rightLeader.getDeviceID(), MotorAlignmentValue.Aligned));

    BaseStatusSignal.setUpdateFrequencyForAll(100,
        leftLeader.getPosition(),
        rightLeader.getPosition(),
        pigeon2.getYaw());

    rightOut.UpdateFreqHz = 0;
    leftOut.UpdateFreqHz = 0;

    // Autos from Pathplanner
    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that willdrive the robot given ROBOT RELATIVE
                                                              // ChassisSpeeds. Also optionally outputs individual
                                                              // module feed forwards
        new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential
                                   // drive trains
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the
          // red  alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

    SmartDashboard.putData("Field", field);
  }

  public void initializeLeftDrive(TalonFXConfigurator cfg) {
    var apply = new TalonFXConfiguration();

    apply.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    apply.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    cfg.apply(apply);

    cfg.setPosition(0);
  }

  public void initializeRightDrive(TalonFXConfigurator cfg) {
    var apply = new TalonFXConfiguration();

    apply.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    apply.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    cfg.apply(apply);

    cfg.setPosition(0);
  }

  public void initializePigeon2(Pigeon2Configurator cfg) {
    var toApply = new Pigeon2Configuration();

    cfg.apply(toApply);

    cfg.setYaw(0);
  }

  private Distance rotationsToMeters(Angle rotations) {

    var gearedRadians = rotations.in(Radians) / this.kGearRatio;

    return this.kWheelRadius.times(gearedRadians);
  }

  public DifferentialDriveWheelPositions getPositions() {
    return new DifferentialDriveWheelPositions(rotationsToMeters(leftLeader.getPosition().getValue()).in(Meters), rotationsToMeters(rightLeader.getPosition().getValue()).in(Meters));
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    double leftMotorRPS = leftLeader.getVelocity().getValueAsDouble();
    double rightMotorRPS = rightLeader.getVelocity().getValueAsDouble();
    double leftWheelRPS = leftMotorRPS / DRIVE_GEAR_RATIO;
    double rightWheelRPS = rightMotorRPS / DRIVE_GEAR_RATIO;
    return new DifferentialDriveWheelSpeeds(leftWheelRPS * Math.PI * 0.152, rightWheelRPS * Math.PI * 0.152);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getSpeeds());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(pigeon2.getRotation2d(), getPositions(), pose);;
  }
  
  public void driveRobotRelative(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    double leftRPS = wheelSpeeds.leftMetersPerSecond / (Math.PI * 0.152) * DRIVE_GEAR_RATIO;
    double rightRPS = wheelSpeeds.rightMetersPerSecond / (Math.PI * 0.152) * DRIVE_GEAR_RATIO;

    leftLeader.setControl(new VelocityDutyCycle(leftRPS));
    rightLeader.setControl(new VelocityDutyCycle(rightRPS));
  }

  @Override
  public void simulationPeriodic() {
  }

  @Override
  public void periodic() {
    odometry.update(pigeon2.getRotation2d(),
        rotationsToMeters(leftLeader.getPosition().getValue()).in(Meters),
        rotationsToMeters(rightLeader.getPosition().getValue()).in(Meters));
    field.setRobotPose(odometry.getPoseMeters());
  }

  // Direct control for use inside alignment
  public void arcadeDrive(double fwd, double rot) {
    if (fwd > 0.1 || fwd < -0.1 || rot > 0.1 || rot < -0.1) {
      rightOut.Output = fwd + rot;
      leftOut.Output = fwd - rot;

      leftLeader.setControl(leftOut);
      rightLeader.setControl(rightOut);
    }
  }
}
