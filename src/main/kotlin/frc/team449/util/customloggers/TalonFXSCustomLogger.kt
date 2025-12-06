package frc.team449.util.customloggers

import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.hardware.TalonFXS
import edu.wpi.first.epilogue.CustomLoggerFor
import edu.wpi.first.epilogue.logging.ClassSpecificLogger
import edu.wpi.first.epilogue.logging.EpilogueBackend
import edu.wpi.first.units.Units.Rotations

@CustomLoggerFor(TalonFXS::class)
class TalonFXSCustomLogger : ClassSpecificLogger<TalonFXS>(TalonFXS::class.java) {
  override fun update(backend: EpilogueBackend, motor: TalonFXS) {
    backend.log("Motor Position", motor.position.value.`in`(Rotations))
    backend.log("Motor Velocity", motor.velocity.value)
    backend.log("Motor Acceleration", motor.acceleration.value)
    backend.log("Closed Loop Reference", motor.closedLoopReference.valueAsDouble)
    backend.log("Closed Loop Error", motor.closedLoopError.value)
    backend.log("Closed Loop PID output", motor.closedLoopOutput.value)
    backend.log("Motor Voltage", motor.motorVoltage.value)
    backend.log("Supply Current", motor.supplyCurrent.value)
    backend.log("Stator Current", motor.statorCurrent.value)
    backend.log("Device Temperature", motor.deviceTemp.value)
  }
}
