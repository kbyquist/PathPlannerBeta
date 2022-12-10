// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.Constants.AutoConstants;

/** 
 * This class is where all autonomous paths should be constructed
 */

public class AutoPaths {
    /** 
     * Construct an ArrayList using {@link com.pathplanner.lib.PathPlanner.loadPathGroup}
     * where the ArrayList is named pathGroupAuto#
     * and takes the name of the path file created
     * in the PathPlanner app with the required path constraints
    */

    // Auto 1 Path
    public static ArrayList<PathPlannerTrajectory> pathGroupAuto1 = PathPlanner
    .loadPathGroup(
        "Auto1",
        new PathConstraints(
        AutoConstants.kMaxSpeedMetersPerSecond, //Velocity
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)); //Acceleration

}
