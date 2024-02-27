package frc.robot.ConstantsF;

import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Util.PIDFConstants;

public class RobotConstants {

    public static final double wheelBase = 0.679;
    public static final double trackWidth = 0.578;

    public static final double motorToSwerveDriveRatio = 1.0 / 8.14;

    //Swerve Module Locations as Translation 2d Objects (For easy integration into wpilib)
    //Coordinates input as (Y, X)
    //Positive Y is towards the front of the robot
    //Positive X is towards the left side of the robot (if looking down from above and the robot is facing north)
    public static final Translation2d frontLeftSwerveModuleLocation = new Translation2d(wheelBase/2, trackWidth/2) ;
    public static final Translation2d frontRightSwerveModuleLocation = new Translation2d(wheelBase/2, -trackWidth/2) ;

    public static final Translation2d backLeftSwerveModuleLocation = new Translation2d(-wheelBase/2, trackWidth/2) ;
    public static final Translation2d backRightSwerveModuleLocation = new Translation2d(-wheelBase/2, -trackWidth/2) ;


    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(frontLeftSwerveModuleLocation,
                                                                frontRightSwerveModuleLocation,
                                                                backLeftSwerveModuleLocation,
                                                                backRightSwerveModuleLocation) ;

    //used for module velocity desaturation in Velocity Control Loops
    public static final double maxAttainableSpeedMetersPerSecond = 4.55;
    
    //PID Constants Container for the Swerve Turning Motor PID Controller
    public static final PIDFConstants swerveAnglePID = new PIDFConstants(0.0035, 0, 0);

    //PID Constants for pathplanner
    public static final PIDConstants ppTranslate = new PIDConstants(5, 0, 0);
    public static final PIDConstants ppRotate = new PIDConstants(0, 0, 0);

    public static final double translationScale = 0.2;
    public static final double rotateScale = 0.5;

    //Angle Offsets for the Swerve CANcoder offsets
    public static final double swerveModuleAngleOffsetFL = (0.022) * 360;
    public static final double swerveModuleAngleOffsetFR = (-0.361) * 360;
    public static final double swerveModuleAngleOffsetHL = (0.244 - 0.5) * 360;
    public static final double swerveModuleAngleOffsetHR = (0.457 - 0.5) * 360;

    public static final double swerveWheelRadius_Meters = 0.1016;

    //Feedforward constants for the Swerve Module Velocity Control Loops
    public static final double swerveThrottlekV = 1.4375;
    public static final double swerveThrottlekS = 0.5631;
    public static final double swerveThrottlekA = 0.19433;
    // public static final double swerveThrottlekV = 0.00000001;
    // public static final double swerveThrottlekS = 0.0005631;
    // public static final double swerveThrottlekA = 0.00000001;

    public static final double moduleRPM_to_MetersPerSecond = 0.00531976356008;
    public static final double moduleRPM_to_FeetPerSecond = 0.209439510239;

    public static final String cameraInstanceName = "photonvision";
    
    public static final double shooterGearRatio = 1;

    public static final double shootTospeaker = 78.5;
}
