package com.eb.obd2.models

data class VehicleData(
    val rpm: Float = 0f,
    val speed: Float = 0f,
    val engineTemp: Float = 70f,
    val throttlePercentage: Float = 0f,
    val fuelLevel: Float = 0f,
    val fuelConsumptionRate: Float = 0f,
    val gear: Int = 1,
    val gearPosition: String = "N",
    val brakePercentage: Float = 0f
) 