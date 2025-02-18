package frc.robot.utils.pingu

import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj2.command.SubsystemBase

/**
 * A subsystem for aggregating Alerts
 */
object AlertPingu : SubsystemBase() {
    private val motors = ArrayList<TalonFX>()
    private val canCoders = ArrayList<CANcoder>()
    private val motorAlerts = ArrayList<Alert>()
    private val canCoderAlerts = ArrayList<Alert>()

    /**
     * Automatically periodically set the alerts to each of the motors
     */
    override fun periodic() {
        motors.indices.forEach { i ->
            motorAlerts[i].set(!motors[i].isConnected)
        }
        canCoders.indices.forEach { i ->
            canCoderAlerts[i].set(!canCoders[i].isConnected)
        }
    }

    /** Add a motor to the [AlertPingu] subsystem
     *
     * @param motor The motor to add
     * @param motorName The name of the motor being added
     */
    @JvmStatic
    fun add(
        motor: TalonFX,
        motorName: String,
    ) {
        motors.add(motor)
        motorAlerts.add(Alert("Disconnected " + motorName + " motor " + motor.deviceID, AlertType.kError))
    }

    /** Add a canCoder to the [AlertPingu] subsystem
     *
     * @param canCoder The canCoder to add
     */
    @JvmStatic
    fun add(canCoder: CANcoder) {
        canCoders.add(canCoder)
        canCoderAlerts.add(Alert("Disconnected canCoder " + canCoder.deviceID, AlertType.kError))
    }
}
