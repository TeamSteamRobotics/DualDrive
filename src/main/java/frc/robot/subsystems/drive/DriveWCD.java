// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.Constants.CANID;
import frc.robot.Constants.DigitalIOID;
import frc.robot.Constants.OdometryConsts;

//import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel;

public class DriveWCD extends SubsystemBase {
  private SparkMax frontLeftMotor;
  private SparkMax frontRightMotor;
  private SparkMax backLeftMotor;
  private SparkMax backRightMotor;

  private RelativeEncoder frontLeftEncoder;
  private RelativeEncoder frontRightEncoder;
  private RelativeEncoder backLeftEncoder;
  private RelativeEncoder backRightEncoder;

  private Encoder leftThroughBoreEncoder;
  private Encoder rightThroughBoreEncoder;

  //private AHRS navX;

  private DifferentialDrive diffDrive;
  
  private DifferentialDriveOdometry odometry;

  private Pose2d currentRobotPose;

  public DriveWCD() {
    frontLeftMotor = new SparkMax(CANID.frontLeft, SparkLowLevel.MotorType.kBrushless);
    frontRightMotor = new SparkMax(CANID.frontRight, SparkLowLevel.MotorType.kBrushless);
    backLeftMotor = new SparkMax(CANID.backLeft, SparkLowLevel.MotorType.kBrushless);
    backRightMotor = new SparkMax(CANID.backRight, SparkLowLevel.MotorType.kBrushless);
    
    diffDrive = new DifferentialDrive(backLeftMotor, backRightMotor);

    frontLeftEncoder = frontLeftMotor.getEncoder();
    frontRightEncoder = frontRightMotor.getEncoder();
    backLeftEncoder = backLeftMotor.getEncoder();
    backRightEncoder = backRightMotor.getEncoder();

    leftThroughBoreEncoder = new Encoder(DigitalIOID.leftDriveEncoder1, DigitalIOID.leftDriveEncoder2);
    rightThroughBoreEncoder = new Encoder(DigitalIOID.rightDriveEncoder1, DigitalIOID.rightDriveEncoder2); 

    leftThroughBoreEncoder.setDistancePerPulse(OdometryConsts.wheelCircumfrenceMeters / 2048);
    rightThroughBoreEncoder.setDistancePerPulse(OdometryConsts.wheelCircumfrenceMeters / 2048);

    leftThroughBoreEncoder.setReverseDirection(true);
    rightThroughBoreEncoder.setReverseDirection(false); 

    //navX = new AHRS(SPI.Port.kMXP);

    //odometry = new DifferentialDriveOdometry(navX.getRotation2d(), getLeftSideMeters(), getRightSideMeters(), new Pose2d(1 ,1 , new Rotation2d(0)));

    SmartDashboard.putData("Reset Encoders", new InstantCommand(() -> resetEncoders(), this));
    //SmartDashboard.putData("Reset Gyro", new InstantCommand(() -> resetGyro(), this));
  }
  //trying to use a SlewRateLimiter so motor doesn't suddenly go full speed and make chain skip.
  SlewRateLimiter filter = new SlewRateLimiter(0.9);
  //Sejun notes, set up an timer variable and change for loops with while loops creating two seperate scenarios for auto (half speed) and teleop (full speed)
  long timeSinceStart = System.currentTimeMillis();

  //while(timeSinceStart<100000){
  
  public void drive(double speed, double rotation){
    for(int i = 0; i < 15; i++){
    speed = speed/2;
    
    diffDrive.arcadeDrive(filter.calculate(speed), rotation);

    }
  }

  public void curveDrive(double speed, double rotation) {
    for(int i = 0; i < 15; i++){
    speed = speed/2;
    
    diffDrive.curvatureDrive(filter.calculate(speed), rotation, true);

    }
  }
  //} COMMENTED OUT TO ALLOW FOR IT TO BUILD
  public void stop(){
    frontLeftMotor.set(0);
    frontRightMotor.set(0);
    backLeftMotor.set(0);
    backRightMotor.set(0);
  }

  //Average of both left side motor encoders
  public double getLeftSideBuiltInRotations() {
    return (frontLeftEncoder.getPosition() + backLeftEncoder.getPosition()) / 2;
  }

  //Average of both right side motor encoders
  public double getRightSideBuiltInRotations() {
    return (frontRightEncoder.getPosition() + backRightEncoder.getPosition()) / 2;
  }

  public double getLeftSideDistanceBuiltInMeters() {
    return (getLeftSideBuiltInRotations() * Constants.OdometryConsts.rotationsToMeters);
  }

  public double getRightSideDistanceBuiltInMeters() {
    return (getRightSideBuiltInRotations() * Constants.OdometryConsts.rotationsToMeters);
  }

  public double getBuiltInEncoderDistanceMeters() {
    return (getLeftSideDistanceBuiltInMeters() + getRightSideDistanceBuiltInMeters()) / 2;
  }

  public void resetEncoders() {
    rightThroughBoreEncoder.reset();
    leftThroughBoreEncoder.reset();
  }

  public double getRightSideMeters() {
    return rightThroughBoreEncoder.getDistance();
  }

  public double getLeftSideMeters() {
    return leftThroughBoreEncoder.getDistance();
  }

  public double getDistanceMeters() {
    return -(getLeftSideMeters()); //+ getRightSideMeters()); / 2;
  }

  public double getRateMetersPerSecond() {
    return (rightThroughBoreEncoder.getRate() + leftThroughBoreEncoder.getRate()) / 2;
  }

  //public void resetGyro() {
    //navX.reset();
  //}

  //public double getAngleDegrees() {
   // return navX.getAngle();
  //}

 // public Rotation2d getRotation2d() {
    //return navX.getRotation2d();
  //}
  
  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  public Pose2d getRobotPose() {
    return currentRobotPose;
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //currentRobotPose = odometry.update(getRotation2d(), getLeftSideMeters(), getRightSideMeters());

    SmartDashboard.putNumber("Through Bore Encoder Distance", getDistanceMeters());

    //SmartDashboard.putNumber("Gyro reading", getAngleDegrees());    
  }
}