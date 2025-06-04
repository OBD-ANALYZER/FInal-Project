package com.eb.obd2.services

import androidx.room.Database
import androidx.room.RoomDatabase
import androidx.room.TypeConverter
import androidx.room.TypeConverters
import com.eb.obd2.repositories.source.persistent.RecordDao
import com.eb.obd2.repositories.source.persistent.RecordEntity
import com.eb.obd2.repositories.source.persistent.SpeedDao
import com.eb.obd2.repositories.source.persistent.SpeedEntity
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter

@Database(entities = [RecordEntity::class, SpeedEntity::class], version = 1, exportSchema = false)
@TypeConverters(DatabaseService.Converters::class)
abstract class DatabaseService : RoomDatabase() {

    abstract fun RecordDao(): RecordDao

    abstract fun SpeedDao(): SpeedDao

    class Converters {
        @TypeConverter
        fun fromTimestamp(value: String): LocalDateTime =
            LocalDateTime.parse(value, DateTimeFormatter.ISO_DATE_TIME)

        @TypeConverter
        fun dateToTimestamp(date: LocalDateTime): String =
            date.format(DateTimeFormatter.ISO_DATE_TIME)
    }
}