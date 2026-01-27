// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.gearing.Sprocket;

public final class Constants {
  public static class OperatorConstants {
    public static final int DriverControllerPort = 0;
    public static final int OperatorControllerPort = 1;

    public static final int coralBeamSensorPort= 5;
    public static final int shooterMotorPort = 9; 
    public static final int armMotorPort = 10;
    public static final int intakeSubsystem = 11;
    public static final int indexerSubsystem = 11;

    public static final double DEADBAND = 0.05;
  }
  public static final double maxSpeed = 7;
  
  public static final int MotorPort = 0;
  public static final int EncoderAChannel = 0;
  public static final int EncoderBChannel = 1;
  public static final int JoystickPort = 0;

  public static class ArmConstants{

    public static final String ArmPositionKey = "ArmPosition";
    public static final String ArmPKey = "ArmP";

    // The P gain for the PID controller that drives this arm.
    public static final double Kp = 50.0;
    public static final double Ki = 0.0;
    public static final double Kd = 0.0;

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    //  = (2 * PI rads) / (4096 pulses)
    public static final double ArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;
    
    public static final MechanismGearing ArmGearBox = new MechanismGearing(GearBox.fromReductionStages(80), Sprocket.fromStages("16:48"));
    public static final Mass ArmMass = Pounds.of(8); // Kilograms
    public static final Distance ArmLength = Inches.of(30);
    public static final Angle softMaxAngle = Degrees.of(225);
    public static final Angle softMinAngle = Degrees.of(-45);
    public static final Angle hardMaxAngle = Degrees.of(225);
    public static final Angle hardMinAngle = Degrees.of(-45);
    public static final Angle startingAngle = Degrees.of(225);

    public static final Angle groundAngle = Degrees.of(0); //Rotations.of(-0.74);
    public static final Angle manualAngle = Degrees.of(15); //Rotations.of(-0.788);
    public static final Angle shooterAngle = Degrees.of(45); //Rotations.of(-0.69);
  }

  public static class shooterConstants
  {
    public static final double Kp = 50;
    public static final double Ki = 0;
    public static final double Kd = 0;

    public static final double kS = 0.0;
    public static final double kG = 0.762;
    public static final double kV = 0.762;
    public static final double kA = 0.0;

    public static final MechanismGearing shooterGearBox = new MechanismGearing(GearBox.fromReductionStages(25));
    public static final Distance Radius = Inches.of(4);
    public static final Distance Diameter = Radius.times(2);
    public static final Mass Mass = Kilograms.of(4.0); 

    public static final AngularVelocity MaxRPM = RPM.of(1000);
    public static final AngularAcceleration MaxAcceleration = DegreesPerSecondPerSecond.of(2.5);
    public static final Voltage MaxVolts = Volts.of(10);

  }
}
