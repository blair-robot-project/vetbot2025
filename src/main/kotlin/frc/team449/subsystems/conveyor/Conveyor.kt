package frc.team449.subsystems.conveyor

import com.revrobotics.spark.SparkMax
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team449.system.motor.createSparkMax

class Conveyor (
  val conveyor_motor: SparkMax
) : SubsystemBase() {
  fun intake(): Command {
    return runOnce {
      conveyor_motor.setVoltage(ConveyorConstants.CONVEYOR_INTAKE_VOLTAGE)
    }
  }
  fun outtake(): Command {
    return runOnce {
      conveyor_motor.setVoltage(ConveyorConstants.CONVEYOR_OUTTAKE_VOLTAGE)
    }
  }
  fun stop(): Command {
    return runOnce {
      conveyor_motor.stopMotor()
    }
  }

  companion object {
    fun createConveyor(): Conveyor {
      return Conveyor(
        createSparkMax(
          ConveyorConstants.CONVEYOR_ID,
          ConveyorConstants.CONVEYOR_INVERTED
        )
      )
    }
  }
}