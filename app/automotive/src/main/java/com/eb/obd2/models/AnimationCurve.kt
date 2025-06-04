package com.eb.obd2.models

import kotlinx.serialization.Serializable

/**
 * Animation curve model that matches the format used by the OBD emulator.
 * Represents a Hermite curve used for speed profile animations.
 */
@Serializable
data class AnimationCurve(
    val points: List<Point> = emptyList(),
    val tangents: List<Tangent> = emptyList(),
    val name: String = "Unnamed Curve",
    val description: String = "",
    val timestamp: Long = System.currentTimeMillis()
) {
    @Serializable
    data class Point(
        val x: Float, // Time (ms)
        val y: Float  // Speed (km/h)
    )
    
    @Serializable
    data class Tangent(
        val x: Float, // Tangent X component
        val y: Float  // Tangent Y component
    )
    
    /**
     * Calculate the duration of the curve in milliseconds
     */
    fun duration(): Float {
        return if (points.isEmpty()) 0f else points.maxByOrNull { it.x }?.x ?: 0f
    }
    
    /**
     * Calculate the maximum speed in the curve
     */
    fun maxSpeed(): Float {
        return if (points.isEmpty()) 0f else points.maxByOrNull { it.y }?.y ?: 0f
    }
    
    /**
     * Convert to JSON format used by the emulator
     */
    fun toJson(): Map<String, Any> {
        return mapOf(
            "points" to points.map { mapOf("x" to it.x, "y" to it.y) },
            "tangents" to tangents.map { mapOf("x" to it.x, "y" to it.y) },
            "name" to name,
            "description" to description,
            "timestamp" to timestamp
        )
    }
    
    companion object {
        /**
         * Create an animation curve from JSON data
         */
        fun fromJson(jsonData: Map<String, Any>): AnimationCurve {
            try {
                @Suppress("UNCHECKED_CAST")
                val pointsData = jsonData["points"] as? List<Map<String, Any>> ?: emptyList()
                @Suppress("UNCHECKED_CAST")
                val tangentsData = jsonData["tangents"] as? List<Map<String, Any>> ?: emptyList()
                
                val points = pointsData.map {
                    Point(
                        (it["x"] as? Number)?.toFloat() ?: 0f,
                        (it["y"] as? Number)?.toFloat() ?: 0f
                    )
                }
                
                val tangents = tangentsData.map {
                    Tangent(
                        (it["x"] as? Number)?.toFloat() ?: 0f,
                        (it["y"] as? Number)?.toFloat() ?: 0f
                    )
                }
                
                return AnimationCurve(
                    points = points,
                    tangents = tangents,
                    name = jsonData["name"] as? String ?: "Unnamed Curve",
                    description = jsonData["description"] as? String ?: "",
                    timestamp = (jsonData["timestamp"] as? Number)?.toLong() ?: System.currentTimeMillis()
                )
            } catch (e: Exception) {
                // Return empty curve if parsing fails
                return AnimationCurve()
            }
        }
        
        /**
         * Create a sample animation curve for testing
         */
        fun createSample(): AnimationCurve {
            return AnimationCurve(
                points = listOf(
                    Point(0f, 0f),
                    Point(2000f, 30f),
                    Point(5000f, 60f),
                    Point(7000f, 45f),
                    Point(10000f, 0f)
                ),
                tangents = listOf(
                    Tangent(500f, 0f),
                    Tangent(500f, 0f),
                    Tangent(0f, 0f),
                    Tangent(-500f, 0f),
                    Tangent(-500f, 0f)
                ),
                name = "Sample Acceleration and Deceleration",
                description = "A sample curve showing acceleration and gentle deceleration"
            )
        }
    }
} 