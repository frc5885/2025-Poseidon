// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.io.operatorPanel;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.ReefLevel;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

/** Interface for physical override switches on operator console. */
public class OperatorPanel {
  private final GenericHID joystick;
  private final NetworkTable m_networkTable;
  private final String NETWORK_ENTRY = "ReefTargets";
  private final String NETWORK_ENTRY_level = "ReefTargetsLevel";

  public OperatorPanel(int port) {
    joystick = new GenericHID(port);
    m_networkTable = NetworkTableInstance.getDefault().getTable("ReefPanel");
    m_networkTable.getIntegerTopic(NETWORK_ENTRY).subscribe(0);
    m_networkTable.getIntegerTopic(NETWORK_ENTRY_level).subscribe(0);
  }

  /** Returns whether the controller is connected. */
  public boolean isConnected() {
    return joystick.isConnected() && !DriverStation.getJoystickIsXbox(joystick.getPort());
  }

  /** Gets the state of a switch (0-7 from left to right). */
  public BooleanSupplier getOverrideSwitch(int index) {
    if (index < 0 || index > 7) {
      throw new RuntimeException(
          "Invalid driver override index " + Integer.toString(index) + ". Must be 0-7.");
    }
    return () -> joystick.getRawButton(index + 1);
  }

  public BooleanSupplier getNegatedOverrideSwitch(int index) {
    return () -> !getOverrideSwitch(index).getAsBoolean();
  }

  /** Returns a trigger for an switch (0-7 from left to right). */
  public Trigger overrideSwitch(int index) {
    return new Trigger(getOverrideSwitch(index));
  }

  /** Returns the reef level from 1 to 4 */
  @AutoLogOutput(key = "OperatorPanel/ReefLevel")
  public int getReefLevel() {
    return m_networkTable.getEntry(NETWORK_ENTRY_level).getNumber(1).intValue();
  }

  /** Returns the target reef branch from 0 to 11 */
  @AutoLogOutput(key = "OperatorPanel/ReefTarget")
  public int getReefTarget() {
    return m_networkTable.getEntry(NETWORK_ENTRY).getNumber(0).intValue();
  }

  public Pose3d getTargetPose() {
    return FieldConstants.Reef.branchPositions
        .get(getReefTarget())
        .get(ReefLevel.values()[getReefLevel() - 1]);
  }
}
