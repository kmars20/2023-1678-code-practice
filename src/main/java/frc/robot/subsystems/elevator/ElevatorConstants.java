package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;

public class ElevatorConstants {

  // PID

  public static final double kp = 0.5; // all guesses, can get from sysid
  public static final double kd = 0.2;
  public static final double ki = 0;
  public static final Rotation2d pidTolerance = Rotation2d.fromDegrees(5.0);
  public static final Rotation2d pidIzone = Rotation2d.fromDegrees(10.0);

  public static final double Lkp = 0.5; // all guesses, can get from sysid
  public static final double Lkd = 0.2;
  public static final double Lki = 0;
  public static final double LpidTolerance = 5;
  public static final double LpidIzone = 10;

  // CANs

  public static final int lengthID = 22;
  public static final int oppositeLengthID = 23;
  public static final int pivotID = 24;
  public static final int oppositePivotID = 25;

  // Motor config

  public static final int pivotCurrentLim = 40;
  public static final int lengthCurrentLim = 40;
  public static final double pivotVoltageCompensation = 12.00;
  public static final double lengthVoltageCompensation = 12.00;

  // Gravity

  public static final double PkG = 0.545; // pivot kg at base position
  public static final double PkGPerLength =
      0.2; // recalc for different levels (different lengths, this a guess)
  public static final double LkG = 0.375; //

  // Pivot constants

  public static final Rotation2d stowAngle = Rotation2d.fromDegrees(0); // stowed will be 0
  public static final double pivotRatio = 4; // encoder sprocket to final pivot sprocket (12:48)
  public static final Rotation2d angleFromZero = Rotation2d.fromDegrees(-20);
  public static final int activeTolerance = 5; // max range to consider the pivot as active

  public static final Rotation2d L1Angle = Rotation2d.fromDegrees(12);
  public static final Rotation2d L2Angle = Rotation2d.fromDegrees(45);
  public static final Rotation2d L3Angle = Rotation2d.fromDegrees(75);

  // Length constants

  public static final double spoolDiam = 1.5;
  public static final double spoolRatio = 3;
  public static final int L1Length = 37;
  public static final int L2Length = 45;
  public static final int L3Length = 63;

  // limits

  public static final double pivotMinDeg = 0.0;
  public static final double pivotMaxDeg = 110.0;
  public static final double lengthMin = 0.0;
  public static final double lengthMax = 0.9;
}
