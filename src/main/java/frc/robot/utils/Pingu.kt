package frc.robot.utils

import edu.wpi.first.math.controller.PIDController
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

/**
 * A class that extends the PIDController to include additional parameters.
 *
 * @property p The proportional gain.
 * @property i The integral gain.
 * @property d The derivative gain.
 * @property v The velocity gain.
 * @property s The static gain.
 * @property g The gravity gain.
 * @throws NullPointerException if v, s, or g are not set and are then accessed.
 */
data class Pingu
    @JvmOverloads
    constructor(
        var p: Double,
        var i: Double,
        var d: Double,
        var v: Double? = null,
        var s: Double? = null,
        var g: Double? = null,
    ) {
        val pidController
            get() = PIDController(p, i, d)

        fun setPID(pidController: PIDController) {
            p = pidController.p
            i = pidController.i
            d = pidController.d
        }
    }

/**
 * A class that represents a PID controller with network logging capabilities.
 *
 * @property p The proportional gain as a LoggedNetworkNumber.
 * @property i The integral gain as a LoggedNetworkNumber.
 * @property d The derivative gain as a LoggedNetworkNumber.
 * @property v The velocity gain as a LoggedNetworkNumber.
 * @property s The static gain as a LoggedNetworkNumber.
 * @property g The gravity gain as a LoggedNetworkNumber.
 * @throws NullPointerException if v, s, or g are not set and are then accessed.
 */
class NetworkPingu
    @JvmOverloads
    constructor(
        var p: LoggedNetworkNumber,
        var i: LoggedNetworkNumber,
        var d: LoggedNetworkNumber,
        var v: LoggedNetworkNumber? = null,
        var s: LoggedNetworkNumber? = null,
        var g: LoggedNetworkNumber? = null,
    ) : PIDController(p.get(), i.get(), d.get())
