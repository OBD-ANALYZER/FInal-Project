package com.eb.obd2.models

import kotlinx.serialization.Serializable

/**
 * Represents a driving profile that contains a collection of animation curves
 * and configuration settings
 */
@Serializable
data class DrivingProfile(
    val id: String = "",
    val name: String,
    val description: String = "",
    val category: ProfileCategory = ProfileCategory.GENERAL,
    val animationCurves: List<AnimationCurve> = emptyList(),
    val defaultSettings: Map<String, Float> = emptyMap(),
    val timestamp: Long = System.currentTimeMillis(),
    val isRemote: Boolean = false
) {
    /**
     * Get the total number of curves in this profile
     */
    fun curveCount(): Int = animationCurves.size
    
    /**
     * Get a specific curve by index
     */
    fun getCurve(index: Int): AnimationCurve? {
        return if (index in animationCurves.indices) animationCurves[index] else null
    }
    
    /**
     * Check if this profile contains any curves
     */
    fun isEmpty(): Boolean = animationCurves.isEmpty()
    
    companion object {
        /**
         * Create a sample profile for testing
         */
        fun createSample(): DrivingProfile {
            return DrivingProfile(
                id = "sample_profile_1",
                name = "City Driving Sample",
                description = "A sample profile with city driving patterns",
                category = ProfileCategory.CITY,
                animationCurves = listOf(
                    AnimationCurve.createSample()
                ),
                defaultSettings = mapOf(
                    "animation_speed" to 1.0f
                )
            )
        }
    }
}

/**
 * Categories for driving profiles
 */
@Serializable
enum class ProfileCategory {
    ALL,       // All categories
    GENERAL,   // General driving profiles
    CITY,      // City driving profiles
    HIGHWAY,   // Highway driving profiles
    MOUNTAIN,  // Mountain driving profiles
    RACE_TRACK,// Race track driving profiles 
    ECO,       // Economy-focused driving profiles
    CUSTOM;    // Custom user-defined profiles
    
    fun displayName(): String {
        return when (this) {
            ALL -> "All Categories"
            GENERAL -> "General"
            CITY -> "City Driving"
            HIGHWAY -> "Highway"
            MOUNTAIN -> "Mountain Roads"
            RACE_TRACK -> "Race Track"
            ECO -> "Eco-Friendly"
            CUSTOM -> "Custom"
        }
    }
    
    companion object {
        fun fromString(value: String): ProfileCategory {
            return values().find { it.name.equals(value, ignoreCase = true) } ?: GENERAL
        }
        
        fun getAllCategories(): List<ProfileCategory> = values().toList()
    }
} 