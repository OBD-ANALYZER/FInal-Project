package com.eb.obd2.repositories.source.persistent

import androidx.room.Dao
import androidx.room.Insert
import androidx.room.Query
import androidx.room.RewriteQueriesToDropUnusedColumns
import androidx.room.Transaction
import androidx.room.RoomWarnings
import java.time.LocalDateTime

@Dao
interface RecordDao {
    @Insert
    suspend fun insert(record: RecordEntity) : Long

    @Transaction
    @RewriteQueriesToDropUnusedColumns
    @SuppressWarnings(RoomWarnings.CURSOR_MISMATCH)
    @Query("""
        SELECT * FROM record r
        LEFT JOIN speed s ON r.recordId = s.recordId
        WHERE r.time BETWEEN :start AND :end
    """)
    suspend fun getRecordsInRange(start: LocalDateTime, end: LocalDateTime): List<DynamicRecord>
}