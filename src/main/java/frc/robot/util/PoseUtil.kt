package frc.robot.util

import edu.wpi.first.math.geometry.Pose2d
import frc.robot.util.FieldConstants.Side

object PoseUtil {
    // Cache the L4 poses to avoid recalculating them
    private val allL4Poses by lazy { FieldConstants.getAllL4Poses().map { it.toPose2d() } }
    
    // Cache side-specific poses
    private val rightSidePoses by lazy { allL4Poses.filterIndexed { index, _ -> index % 2 == 0 } }
    private val leftSidePoses by lazy { allL4Poses.filterIndexed { index, _ -> index % 2 == 1 } }

    fun getClosestDesiredBranchID(robotPose: Pose2d, side: Side): Int {
        val posesOnSide = if (side == Side.RIGHT) rightSidePoses else leftSidePoses
        val closestPose = robotPose.nearest(posesOnSide)
        return allL4Poses.indexOf(closestPose)
    }

    fun getBranchIDFromPose(branchPose: Pose2d): Int {
        val closestL4Pose = branchPose.nearest(allL4Poses)
        return allL4Poses.indexOf(closestL4Pose)
    }
}