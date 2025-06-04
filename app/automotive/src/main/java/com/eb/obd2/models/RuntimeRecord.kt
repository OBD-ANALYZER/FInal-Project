package com.eb.obd2.models

import java.time.LocalDateTime

interface RuntimeRecord {
    /** The value of the record (raw value) */
    val value: String

    /** The unit of the record */
    val unit: String

    /** The time of the record */
    val time: LocalDateTime
    
    /** The raw value of the record for data recording */
    val rawValue: String
        get() = value  // Default implementation returns value

    /**
     * Converts the record to a [Map] for Firestore.
     */
    fun toMap(): Map<String, Any>

    /**
     * @param command The command of the record. `OBD_Hex` reserved by obd service 1 data,
     * @param value The OBD standard value
     * @param unit The OBD standard unit of the value
     * @param time The record time
     */
    data class PlainRecord(
        val command: String,
        override val value: String,
        override val unit: String,
        override val time: LocalDateTime
    ) : RuntimeRecord {
        override val rawValue: String
            get() = value

        override fun toMap(): Map<String, Any> {
            return mapOf(
                "command" to command,
                "value" to value,
                "unit" to unit,
                "time" to time.toString()
            )
        }
    }

    /**
     * @param speed The speed of the record in SI unit
     * @param acceleration The acceleration of the record in SI unit between two last records
     * @param deltaTime The delta time of record since last record.
     * @param value The OBD standard value
     * @param unit The OBD standard unit of the value
     * @param time The record time
     */
    data class SpeedRecord(
        val speed: Float,
        val acceleration: Float,
        val deltaTime: Float,
        override val value: String,
        override val unit: String,
        override val time: LocalDateTime
    ) : RuntimeRecord {
        override val rawValue: String
            get() = value
            
        override fun toMap(): Map<String, Any> {
            return mapOf(
                "speed" to speed,
                "acceleration" to acceleration,
                "deltaTime" to deltaTime,
                "value" to value,
                "unit" to unit,
                "time" to time.toString()
            )
        }
    }
}