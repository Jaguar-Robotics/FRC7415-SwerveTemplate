// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.MechanicalConfiguration;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //Drive Constants
  public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.0;
  public static final double DRIVETRAIN_WHEELBASE_METERS = 0.0;

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2; 
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1; 
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9; 
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); 

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4; 
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3; 
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10; 
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); 

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 8;
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 7; 
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12; 
  public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0);

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6; 
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5; 
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11; 
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); 

  public static final MechanicalConfiguration MK4I_L2 = new MechanicalConfiguration(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            false,
            (14.0 / 50.0) * (10.0 / 60.0),
            true
    );

  //PathPlanner Config
  public static final HolonomicPathFollowerConfig pathPlannerConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(0.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.0, 0.0, 0.0), // Rotation PID constants
                    3.0, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            );
}
