package com.eb.obd2.models

enum class DrivingStyle(val level: Float) {
    Obscene(-1.0f),
    Normal(0.0f),
    Aggressive(1.0f),
}