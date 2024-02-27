// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class ConstantsU
{
  private static ConstantsU instance;

  private CommandPS4Controller driverController;
  private Pose3d[] aprilTags;

  private ConstantsU()
  {
    driverController = new CommandPS4Controller(0);

    Pose3d[] aprilTags = new Pose3d[16];

    aprilTags[0] = new Pose3d(new Translation3d(593.68 * 0.0254, 9.68 * 0.0254, 53.38 * 0.0254), new Rotation3d(0, 0, 120 * Math.PI / 180));
    aprilTags[1] = new Pose3d(new Translation3d(637.21 * 0.0254, 34.79 * 0.0254, 53.38 * 0.0254), new Rotation3d(0, 0, 120 * Math.PI / 180));
    aprilTags[2] = new Pose3d(new Translation3d(652.73 * 0.0254, 196.17 * 0.0254, 57.13 * 0.0254), new Rotation3d(0, 0, 180 * Math.PI / 180));
    aprilTags[3] = new Pose3d(new Translation3d(652.73 * 0.0254, 218.42 * 0.0254, 57.13 * 0.0254), new Rotation3d(0, 0, 180 * Math.PI / 180));
    aprilTags[4] = new Pose3d(new Translation3d(578.77 * 0.0254, 323 * 0.0254, 53.38 * 0.0254), new Rotation3d(0, 0, 270 * Math.PI / 180));
    aprilTags[5] = new Pose3d(new Translation3d(72.5 * 0.0254, 323 * 0.0254, 53.38 * 0.0254), new Rotation3d(0, 0, 270 * Math.PI / 180));
    aprilTags[6] = new Pose3d(new Translation3d(-1.5 * 0.0254, 218.42 * 0.0254, 57.13 * 0.0254), new Rotation3d(0, 0, 0 * Math.PI / 180));
    aprilTags[7] = new Pose3d(new Translation3d(-1.5 * 0.0254, 196.17 * 0.0254, 57.13 * 0.0254), new Rotation3d(0, 0, 0 * Math.PI / 180));
    aprilTags[8] = new Pose3d(new Translation3d(14.02 * 0.0254, 34.79 * 0.0254, 53.38 * 0.0254), new Rotation3d(0, 0, 60 * Math.PI / 180));
    aprilTags[9] = new Pose3d(new Translation3d(57.54 * 0.0254, 9.68 * 0.0254, 53.38 * 0.0254), new Rotation3d(0, 0, 60 * Math.PI / 180));
    aprilTags[10] = new Pose3d(new Translation3d(468.69 * 0.0254, 146.19 * 0.0254, 52 * 0.0254), new Rotation3d(0, 0, 300 * Math.PI / 180));
    aprilTags[11] = new Pose3d(new Translation3d(468.69 * 0.0254, 177.1 * 0.0254, 52 * 0.0254), new Rotation3d(0, 0, 60 * Math.PI / 180));
    aprilTags[12] = new Pose3d(new Translation3d(441.74 * 0.0254, 161.62 * 0.0254, 52 * 0.0254), new Rotation3d(0, 0, 180 * Math.PI / 180));
    aprilTags[13] = new Pose3d(new Translation3d(209.48 * 0.0254, 161.62 * 0.0254, 52 * 0.0254), new Rotation3d(0, 0, 0 * Math.PI / 180));
    aprilTags[14] = new Pose3d(new Translation3d(182.73 * 0.0254, 177.1 * 0.0254, 52 * 0.0254), new Rotation3d(0, 0, 120 * Math.PI / 180));
    aprilTags[15] = new Pose3d(new Translation3d(182.73 * 0.0254, 146.19 * 0.0254, 52 * 0.0254), new Rotation3d(0,0, 240 * Math.PI / 180));
  }

  public static ConstantsU getInstance()
  {
    if(instance == null)
    {
      instance = new ConstantsU();
    }

    return instance;
  }

  public CommandPS4Controller getDriverController()
  {
    return driverController;
  }

  public Pose3d[] getAprilTagPoses()
  {
    return aprilTags;
  }
}
