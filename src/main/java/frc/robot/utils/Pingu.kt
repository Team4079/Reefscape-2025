package frc.robot.utils

import edu.wpi.first.math.controller.PIDController

class Pingu
    @JvmOverloads
    constructor(
        var p: Double,
        var i: Double,
        var d: Double,
        v: Double? = null,
        s: Double? = null,
        g: Double? = null,
    ) {
        var v: Double = v ?: throw NullPointerException("V is not set but is trying to be called.")
        var s: Double = s ?: throw NullPointerException("S is not set but is trying to be called.")
        var g: Double = g ?: throw NullPointerException("G is not set but is trying to be called.")

        fun toPIDController(): PIDController = PIDController(p, i, d)
    }
