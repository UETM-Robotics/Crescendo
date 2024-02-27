package frc.robot.Subsystems;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConstantsF.PortConstants;
import frc.robot.ConstantsF.RobotConstants;
import frc.robot.Simulation.SimSwerveModule;
import frc.robot.Util.OdometryU;
import frc.robot.Util.SwerveModule;
import frc.robot.Util.SwerveModuleStateU;
import frc.robot.Util.Vec3d;

public class Drivetrain extends SubsystemBase{    
    private final SwerveModule[] swerveModules;

    private SwerveModuleStateU[] moduleStates = new SwerveModuleStateU[4];

    private final OperatingMode dMode;

    private final VSLAM vslam;
    private final SwerveDriveKinematics kinematics = RobotConstants.swerveKinematics;

    /**
     * Instantiates a DriveTrain Object. Can be in SIM mode or DRIVE mode.
     * @param DriveTrainMode Directs what mode the drivetrain is instantiated and operated in
     */

    private static Drivetrain instance;

    public static Drivetrain getInstance()
    {
        if(instance == null)
        {
            instance = new Drivetrain(OperatingMode.DRIVE);
        }

        return instance;
    }

    private Drivetrain(OperatingMode DriveTrainMode)
    {

        this.dMode = DriveTrainMode;

        switch( dMode ) {
            case DRIVE:
                swerveModules = new SwerveModule[]{ new SwerveModule(PortConstants.FrontLeftThrottlePort,
                                                                    PortConstants.FrontLeftAnglePort,
                                                                    PortConstants.FrontLeftAzimuthEncoder,
                                                                    RobotConstants.swerveModuleAngleOffsetFL),
                                                    new SwerveModule(PortConstants.FrontRightThrottlePort,
                                                                    PortConstants.FrontRightAnglePort,
                                                                    PortConstants.FrontRightAzimuthEncoder,
                                                                    RobotConstants.swerveModuleAngleOffsetFR, false),
                                                    new SwerveModule(PortConstants.HindLeftThrottlePort,
                                                                    PortConstants.HindLeftAnglePort,
                                                                    PortConstants.HindLeftAzimuthEncoder,
                                                                    RobotConstants.swerveModuleAngleOffsetHL),
                                                    new SwerveModule(PortConstants.HindRightThrottlePort,
                                                                    PortConstants.HindRightAnglePort,
                                                                    PortConstants.HindRightAzimuthEncoder,
                                                                    RobotConstants.swerveModuleAngleOffsetHR),
                                                };
                break;
            case SIM:
                swerveModules = new SwerveModule[]{ new SimSwerveModule(),
                                                    new SimSwerveModule(),
                                                    new SimSwerveModule(),
                                                    new SimSwerveModule()
                                                };
                break;
            default:
                swerveModules = new SwerveModule[]{ new SwerveModule(PortConstants.FrontLeftThrottlePort,
                                                                    PortConstants.FrontLeftAnglePort,
                                                                    PortConstants.FrontLeftAzimuthEncoder,
                                                                    RobotConstants.swerveModuleAngleOffsetFL),
                                                    new SwerveModule(PortConstants.FrontRightThrottlePort,
                                                                    PortConstants.FrontRightAnglePort,
                                                                    PortConstants.FrontRightAzimuthEncoder,
                                                                    RobotConstants.swerveModuleAngleOffsetHL),
                                                    new SwerveModule(PortConstants.HindLeftThrottlePort,
                                                                    PortConstants.HindLeftAnglePort,
                                                                    PortConstants.HindLeftAzimuthEncoder,
                                                                    RobotConstants.swerveModuleAngleOffsetHL),
                                                    new SwerveModule(PortConstants.HindRightThrottlePort,
                                                                    PortConstants.HindRightAnglePort,
                                                                    PortConstants.HindRightAzimuthEncoder,
                                                                    RobotConstants.swerveModuleAngleOffsetHR),
                                                };
                break;
        }

        this.vslam = new VSLAM( this.dMode, getSwerveModuleStates() );

        AutoBuilder.configureHolonomic(this::getRobotPose,
        this::resetPose,
        this::getChassisSpeeds,
        this::set,
        new HolonomicPathFollowerConfig(
            RobotConstants.ppTranslate,
            RobotConstants.ppRotate,
            RobotConstants.maxAttainableSpeedMetersPerSecond,
            0.892,
            new ReplanningConfig()),

        () -> {
            var alliance = DriverStation.getAlliance();

            if(alliance.isPresent())
            {
                return alliance.get() == Alliance.Red;
            }
            return false;
        },
        
        this
        );
    }

    /**
     * Sets the desired module states
     * @param chassisSpeeds
     * @param voltageControl
     */
    public void set(ChassisSpeeds chassisSpeeds, boolean velocityControl) {

        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond,
                                                            chassisSpeeds.vyMetersPerSecond,
                                                            -chassisSpeeds.omegaRadiansPerSecond, 
                                                            vslam.getHeading());

        moduleStates = SwerveModuleStateU.toModuleStatesU(kinematics.toSwerveModuleStates(chassisSpeeds));

        setSwerveModuleStates(moduleStates, velocityControl);
        
    }

    public void set(ChassisSpeeds chassisSpeeds)
    {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        targetSpeeds = new ChassisSpeeds(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond, targetSpeeds.omegaRadiansPerSecond);

        moduleStates = SwerveModuleStateU.toModuleStatesU(kinematics.toSwerveModuleStates(targetSpeeds));

        setSwerveModuleStates(moduleStates, false);
    }

    public void resetPose(Pose2d pose)
    {
        vslam.resetOdometry(pose, vslam.getHeading(), moduleStates);
    }

    /**
     * Call to update the VSLAM odometry reading
     */
    public void updateOdometry() {
        vslam.updateOdometry( getSwerveModuleStates() );
    }

    /**
     * Resets robot's position on the field to (0,0,0), (X,Y,Theta)
     */
    public void resetOdometry() {

        resetOdometry(new Vec3d());
    }

    /**
     * Resets robot's position on the field to some initial position vector
     * @param initalPositionVector Robot's starting position on the field
     */
    public void resetOdometry(Vec3d initalPositionVector) {
        vslam.resetOdometry(initalPositionVector, vslam.getHeading(), getSwerveModuleStates());
    }

    public Rotation2d getHeading()
    {
        return vslam.getHeading();
    }

    public void resetHeading()
    {
        vslam.resetHeading();
    }

    /**
     * 
     * @return The robot's position on the field in meters and degrees
     */
    public Vec3d getRobotPosition() {
        return vslam.getRobotPoseMeters();
    }

    public Pose2d getRobotPose()
    {
        return vslam.getRobotPoseMeters().asPose2d();
    }

    public VSLAM getVslam()
    {
        return vslam;
    }


    /**
     * 
     * @return Array of the Drivetrain's Swerve Module's states according to sensor readings
     */
    public SwerveModuleStateU[] getSwerveModuleStates() {
        return new SwerveModuleStateU[]{
            swerveModules[0].getState(),
            swerveModules[1].getState(),
            swerveModules[2].getState(),
            swerveModules[3].getState()
        };
    }

    public void setSwerveModuleStates(SwerveModuleStateU[] mSwerveModuleStates, boolean velocityControl) {

        if(velocityControl) { SwerveDriveKinematics.desaturateWheelSpeeds(mSwerveModuleStates,
                                                                        RobotConstants.maxAttainableSpeedMetersPerSecond); }
        else { SwerveDriveKinematics.desaturateWheelSpeeds(mSwerveModuleStates, 1);}

        this.moduleStates = mSwerveModuleStates;
        for(int i = 0; i < 4; i++)
        {
            swerveModules[i].setState(this.moduleStates[i], velocityControl);
        }
    }

    public SwerveModule[] getModules()
    {
        return swerveModules;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public void updateSimModules() {
        swerveModules[0].update();
        swerveModules[1].update();
        swerveModules[2].update();
        swerveModules[3].update();
    }


    public OperatingMode getDriveTrainOperatingMode() {
        return this.dMode;
    }


    public static enum OperatingMode {
        SIM, DRIVE;
    }
}
