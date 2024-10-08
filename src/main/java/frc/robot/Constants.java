package frc.robot;

//Measurement Units
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

//Pathplanner
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PIDConstants;

//AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

//Geometry
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;

//Kinematics
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

//Units and stuff
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

//Swerves and utils
import frc.lib.swerve.SwerveModuleConstants;
import frc.lib.swerve.COTSFalconSwerveConstants;
import frc.lib.utils.AllianceFlipUtil;

public final class Constants{
  public static final class SwerveConstants{
    public static final int pigeonID = 1;
    public static final boolean invertGyro = true; //Always make sure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2); //Must be tuned to specific robot

    /** Drivetrain Constants 
     * Must be tuned to specific robot
    */
    public static final double trackWidth = Units.inchesToMeters(24);
    public static final double wheelBase = Units.inchesToMeters(24);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /**
     * Swerve kinematics
     * Don't change this unless you're not doing a traditional 4 module rectangle/square swerve
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      Mod0.position,
      Mod1.position,
      Mod2.position,
      Mod3.position);
    
    /** Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /**Motor Inverts*/
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert - chosenModule.driveMotorInvert;


  }
}
