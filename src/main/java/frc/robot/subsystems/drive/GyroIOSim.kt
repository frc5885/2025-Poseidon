package frc.robot.subsystems.drive

import edu.wpi.first.units.Units.RadiansPerSecond

import edu.wpi.first.math.util.Units
import frc.robot.subsystems.drive.GyroIO.GyroIOInputs
import frc.robot.util.SparkUtil
import org.ironmaple.simulation.drivesims.GyroSimulation

class GyroIOSim(private val m_gyroSimulation: GyroSimulation) : GyroIO {
    override fun updateInputs(inputs: GyroIOInputs) {
        inputs.connected = true
        inputs.yawPosition = m_gyroSimulation.gyroReading
        inputs.yawVelocityRadPerSec =
            Units.degreesToRadians(m_gyroSimulation.measuredAngularVelocity.`in`(RadiansPerSecond))

        inputs.odometryYawTimestamps = SparkUtil.getSimulationOdometryTimeStamps().toList()
        inputs.odometryYawPositions = m_gyroSimulation.cachedGyroReadings.toList()
    }
}
