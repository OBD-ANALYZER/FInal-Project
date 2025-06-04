package com.eb.obd2.repositories.source.persistent

import androidx.room.Entity
import androidx.room.PrimaryKey
import java.time.LocalDateTime

/**
 * Record entity class for data transform object
 *
 * @param recordId The id of the record
 * @param command The command of the record
 * @param value The value of the record
 * @param unit The unit of the record
 * @param time The record time of the record
 */
@Entity(tableName = "record")
data class RecordEntity(
    @PrimaryKey(autoGenerate = true) val recordId: Long = 0,
    val command: String,
    val value: String,
    val unit: String,
    val time: LocalDateTime,
)