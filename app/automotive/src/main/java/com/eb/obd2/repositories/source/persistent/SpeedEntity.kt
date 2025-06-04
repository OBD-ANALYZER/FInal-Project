package com.eb.obd2.repositories.source.persistent

import androidx.room.Entity
import androidx.room.ForeignKey
import androidx.room.Index
import androidx.room.PrimaryKey

/**
 * Speed entity class for data transform object
 *
 * @param speedId The id of the speed record
 * @param recordId The record id of the speed record
 * @param speed The speed of the speed record (m/s)
 * @param acceleration The acceleration of the speed record (m/s^2)
 * @param deltaTime The delta time of the speed record (s)
 */
@Entity(
    tableName = "speed",
    foreignKeys = [ForeignKey(
        entity = RecordEntity::class,
        parentColumns = ["recordId"],
        childColumns = ["recordId"],
        onDelete = ForeignKey.CASCADE
    )],
    indices = [Index(value = ["recordId"])]
)
data class SpeedEntity(
    @PrimaryKey(autoGenerate = true) val speedId: Long = 0,
    val recordId: Long,
    val speed: Float = 0.0f,
    val acceleration: Float = 0.0f,
    val deltaTime: Float = 0.0f
)