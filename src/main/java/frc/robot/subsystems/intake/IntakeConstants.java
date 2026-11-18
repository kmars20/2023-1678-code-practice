package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {

  // PID

  public static final double kp = 0.5; // all guesses, can get from sysid
  public static final double kd = 0.2;
  public static final double ki = 0;
  public static final Rotation2d pidTolerance = Rotation2d.fromDegrees(5.0);
  public static final Rotation2d pidIzone = Rotation2d.fromDegrees(10.0);

  // CANs

  public static final int rollerID = 20;
  public static final int pivotID = 21;

  // Motor config

  public static final int pivotCurrentLim = 40;
  public static final int rollerCurrentLim = 40;
  public static final double pivotVoltageCompensation = 12.00;
  public static final double rollerVoltageCompensation = 12.00;

  // Gravity

  public static final double kG =
      0.252; // calculation on recalc (10:72 * 20:72) * 12:48 gear reduc 5lb 8.11 in, or sysid

  // Pivot constants

  public static final Rotation2d stowAngle = Rotation2d.fromDegrees(0); // stowed will be 0
  public static final Rotation2d activeAngle =
      Rotation2d.fromDegrees(170); // looks like it idk onshape wont let me measure it
  public static final double pivotRatio = 4; // encoder sprocket to final pivot sprocket (12:48)
  public static final int angleFromZero = 30;
  public static final int activeTolerance = 5; // max range to consider the pivot as active

  // Roller constants

  public static final double intakeSpeed = 0.6;
  public static final int holdingStatorSpike =
      40; // complete guess, can tune with real values when intaking objects

  // Limits

  public static final int minAngleDeg = 0;
  public static final int maxAngleDeg = 175;
}
