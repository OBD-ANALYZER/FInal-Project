package com.eb.obd2.repositories

import com.eb.obd2.models.FirestoreRecord
import com.eb.obd2.models.RuntimeRecord
import com.eb.obd2.repositories.source.persistent.RecordDao
import com.eb.obd2.repositories.source.persistent.RecordEntity
import com.eb.obd2.repositories.source.persistent.SpeedDao
import com.eb.obd2.repositories.source.persistent.SpeedEntity
import com.google.firebase.firestore.FirebaseFirestore
import java.time.Duration
import java.time.LocalDateTime
import javax.inject.Inject

class RecordRepository @Inject constructor(
    private val recordDao: RecordDao,
    private val speedDao: SpeedDao,
) {
    // Firestore instance
    private val firestore = FirebaseFirestore.getInstance()

    suspend fun getRecords(
        start: LocalDateTime,
        end: LocalDateTime
    ): List<RuntimeRecord> {
        return recordDao.getRecordsInRange(start, end).mapNotNull { detail ->
            when {
                detail.speed != null -> {
                    RuntimeRecord.SpeedRecord(
                        speed = detail.speed.speed,
                        acceleration = detail.speed.acceleration,
                        deltaTime = detail.speed.deltaTime,
                        value = detail.record.value,
                        unit = detail.record.unit,
                        time = detail.record.time
                    )
                }
                else -> null
            }
        }
    }

    suspend fun saveRecord(record: RuntimeRecord.PlainRecord): RuntimeRecord {
        val entityId = recordDao.insert(
            RecordEntity(
                command = record.command,
                value = record.value,
                unit = record.unit,
                time = record.time
            )
        )
        // Insert into Firestore
        val firestoreRecord = FirestoreRecord(
            recordId = entityId,
            time = record.time.toString(), // Store time as String
            speed = record.value.toFloatOrNull() ?: 0f // Use value from record (convert if necessary)
        )

        firestore.collection("records")
            .document(entityId.toString())  // Use recordId as the document ID
            .set(firestoreRecord)
            .addOnSuccessListener {
                // Handle success, maybe log or notify user
            }
            .addOnFailureListener {
                // Handle failure, show error message
            }

        when(record.command) {
            "0D" -> {
                val lastRecord = speedDao.getLatestSpeedRecord()
                val speed = (record.value.toFloatOrNull() ?: Float.NaN) * 1000.0f / 3600.0f
                val dv = speed - (lastRecord?.speed?.speed ?: 0.0f)
                val dt = lastRecord?.record?.let {
                    Duration.between(it.time, record.time).toNanos() / 1e9f
                } ?: 0.0f

                speedDao.insertSpeed(
                    SpeedEntity(
                        recordId = entityId,
                        speed = speed,
                        acceleration = if (dt != 0f) dv / dt else 0f,
                        deltaTime = dt
                    )
                )

                return RuntimeRecord.SpeedRecord(
                    speed = speed,
                    acceleration = if (dt != 0f) dv / dt else 0f,
                    deltaTime = dt,
                    value = record.value,
                    unit = record.unit,
                    time = record.time
                )
            }
        }

        return RuntimeRecord.PlainRecord(
            command = record.command,
            value = record.value,
            unit = record.unit,
            time = record.time
        )
    }
    // Helper function to sync records from Firestore (optional)
    fun getRecordsFromFirestore(onResult: (List<FirestoreRecord>) -> Unit) {
        firestore.collection("records")
            .get()
            .addOnSuccessListener { documents ->
                val records = documents.map { document ->
                    document.toObject(FirestoreRecord::class.java)
                }
                onResult(records)  // Return the records to the caller
            }
            .addOnFailureListener {
                // Handle Firestore error
            }
    }
}