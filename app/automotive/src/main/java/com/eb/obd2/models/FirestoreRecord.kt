package com.eb.obd2.models

data class FirestoreRecord(
    val recordId: Long,
    val time: String,  // Store the time as a string (ISO format)
    val speed: Float,
)
