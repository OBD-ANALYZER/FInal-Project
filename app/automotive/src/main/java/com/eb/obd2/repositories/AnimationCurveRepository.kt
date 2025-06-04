package com.eb.obd2.repositories

import android.content.Context
import com.eb.obd2.models.AnimationCurve
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.withContext
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import java.io.File
import javax.inject.Inject
import javax.inject.Singleton

@Singleton
class AnimationCurveRepository @Inject constructor(
    private val appContext: Context
) {
    private val _savedCurves = MutableStateFlow<List<AnimationCurve>>(emptyList())
    val savedCurves: Flow<List<AnimationCurve>> = _savedCurves.asStateFlow()
    
    private val json = Json { prettyPrint = true; ignoreUnknownKeys = true }
    private val curvesDirectory get() = File(appContext.filesDir, "curves")
    
    init {
        // Ensure directory exists
        curvesDirectory.mkdirs()
        
        // Load saved curves
        loadSavedCurves()
    }
    
    /**
     * Get a list of all saved curves
     */
    suspend fun getAllCurves(): List<AnimationCurve> = withContext(Dispatchers.IO) {
        return@withContext _savedCurves.value
    }
    
    /**
     * Save a new animation curve
     */
    suspend fun saveCurve(curve: AnimationCurve): Boolean = withContext(Dispatchers.IO) {
        try {
            val fileName = "${curve.name.replace(" ", "_")}_${curve.timestamp}.json"
            val file = File(curvesDirectory, fileName)
            
            val jsonString = json.encodeToString(curve)
            file.writeText(jsonString)
            
            // Update the saved curves list
            loadSavedCurves()
            
            return@withContext true
        } catch (e: Exception) {
            e.printStackTrace()
            return@withContext false
        }
    }
    
    /**
     * Delete an animation curve
     */
    suspend fun deleteCurve(curve: AnimationCurve): Boolean = withContext(Dispatchers.IO) {
        try {
            val fileName = "${curve.name.replace(" ", "_")}_${curve.timestamp}.json"
            val file = File(curvesDirectory, fileName)
            
            val result = file.delete()
            
            // Update the saved curves list
            if (result) {
                loadSavedCurves()
            }
            
            return@withContext result
        } catch (e: Exception) {
            e.printStackTrace()
            return@withContext false
        }
    }
    
    /**
     * Record a driving pattern as an animation curve
     */
    suspend fun recordDrivingPattern(
        speedSamples: List<Pair<Long, Float>>,
        name: String,
        description: String
    ): AnimationCurve = withContext(Dispatchers.Default) {
        // Sort samples by timestamp
        val sortedSamples = speedSamples.sortedBy { it.first }
        
        if (sortedSamples.isEmpty()) {
            return@withContext AnimationCurve(
                name = name,
                description = description
            )
        }
        
        // Normalize timestamps to start from 0
        val startTime = sortedSamples.first().first
        val normalizedSamples = sortedSamples.map { 
            (it.first - startTime).toFloat() to it.second 
        }
        
        // Create points from samples
        val points = normalizedSamples.map { (time, speed) ->
            AnimationCurve.Point(time, speed)
        }
        
        // Generate simple tangents (can be improved later)
        val tangents = generateTangents(points)
        
        return@withContext AnimationCurve(
            points = points,
            tangents = tangents,
            name = name,
            description = description
        )
    }
    
    /**
     * Import an animation curve from JSON
     */
    suspend fun importCurve(jsonString: String): AnimationCurve? = withContext(Dispatchers.Default) {
        try {
            return@withContext json.decodeFromString<AnimationCurve>(jsonString)
        } catch (e: Exception) {
            e.printStackTrace()
            return@withContext null
        }
    }
    
    /**
     * Export an animation curve to JSON
     */
    suspend fun exportCurve(curve: AnimationCurve): String = withContext(Dispatchers.Default) {
        return@withContext json.encodeToString(curve)
    }
    
    /**
     * Generate simple tangents for a list of points
     */
    private fun generateTangents(points: List<AnimationCurve.Point>): List<AnimationCurve.Tangent> {
        if (points.isEmpty()) return emptyList()
        
        return points.mapIndexed { index, point ->
            when (index) {
                0 -> {
                    // First point - outgoing tangent
                    if (points.size > 1) {
                        val nextPoint = points[1]
                        val dx = (nextPoint.x - point.x) * 0.3f
                        val dy = (nextPoint.y - point.y) * 0.3f
                        AnimationCurve.Tangent(dx, dy)
                    } else {
                        AnimationCurve.Tangent(100f, 0f)
                    }
                }
                points.size - 1 -> {
                    // Last point - incoming tangent
                    val prevPoint = points[index - 1]
                    val dx = (point.x - prevPoint.x) * 0.3f
                    val dy = (point.y - prevPoint.y) * 0.3f
                    AnimationCurve.Tangent(dx, dy)
                }
                else -> {
                    // Middle points - average of neighboring points
                    val prevPoint = points[index - 1]
                    val nextPoint = points[index + 1]
                    val dx = (nextPoint.x - prevPoint.x) * 0.15f
                    val dy = (nextPoint.y - prevPoint.y) * 0.15f
                    AnimationCurve.Tangent(dx, dy)
                }
            }
        }
    }
    
    /**
     * Load all saved curves from storage
     */
    private fun loadSavedCurves() {
        val curves = mutableListOf<AnimationCurve>()
        
        curvesDirectory.listFiles()?.filter { it.isFile && it.extension == "json" }?.forEach { file ->
            try {
                val jsonString = file.readText()
                val curve = json.decodeFromString<AnimationCurve>(jsonString)
                curves.add(curve)
            } catch (e: Exception) {
                e.printStackTrace()
            }
        }
        
        // Sort curves by timestamp (newest first)
        _savedCurves.value = curves.sortedByDescending { it.timestamp }
    }
} 