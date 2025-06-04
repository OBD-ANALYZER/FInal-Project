package com.eb.obd2.repositories.source.persistent

import androidx.room.Embedded
import androidx.room.Relation

data class DynamicRecord(
    @Embedded val record: RecordEntity,
    @Relation(
        parentColumn = "recordId",
        entityColumn = "recordId"
    )
    val speed: SpeedEntity?,
)