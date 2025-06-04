package com.eb.obd2.services

import android.util.Log
import com.google.firebase.firestore.FirebaseFirestore
import kotlinx.coroutines.tasks.await
import javax.inject.Inject
import javax.inject.Singleton

@Singleton
class FirebaseService @Inject constructor() {

    private val firestore = FirebaseFirestore.getInstance()

    suspend fun saveData(collection: String, documentId: String, data: Map<String, Any>) {
        try {
            firestore.collection(collection)
                .document(documentId)
                .set(data)
                .await() // Ensures it's run inside a coroutine
            Log.d("FirebaseService", "Data successfully written to Firestore")
        } catch (e: Exception) {
            Log.e("FirebaseService", "Firestore save error: ${e.message}")
        }
    }

    suspend fun getData(collection: String, documentId: String): Map<String, Any>? {
        return try {
            val snapshot = firestore.collection(collection).document(documentId).get().await()
            snapshot.data
        } catch (e: Exception) {
            Log.e("FirebaseService", "Firestore read error: ${e.message}")
            null
        }
    }
}
