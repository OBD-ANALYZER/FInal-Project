package com.eb.obd2.repositories

import android.content.Context
import com.eb.obd2.models.AnimationCurve
import com.eb.obd2.models.DrivingProfile
import com.eb.obd2.models.ProfileCategory
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.withContext
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import java.io.File
import javax.inject.Inject
import javax.inject.Singleton
import kotlin.random.Random
import kotlinx.coroutines.launch

@Singleton
class ProfileRepository @Inject constructor(
    private val appContext: Context,
    private val animationCurveRepository: AnimationCurveRepository
) {
    private val _localProfiles = MutableStateFlow<List<DrivingProfile>>(emptyList())
    val localProfiles: Flow<List<DrivingProfile>> = _localProfiles.asStateFlow()
    
    private val _remoteProfiles = MutableStateFlow<List<DrivingProfile>>(emptyList())
    val remoteProfiles: Flow<List<DrivingProfile>> = _remoteProfiles.asStateFlow()
    
    private val _isSyncing = MutableStateFlow(false)
    val isSyncing: Flow<Boolean> = _isSyncing.asStateFlow()
    
    private val json = Json { prettyPrint = true; ignoreUnknownKeys = true }
    private val profilesDirectory get() = File(appContext.filesDir, "profiles")
    
    init {
        // Ensure directory exists
        profilesDirectory.mkdirs()
        
        // Load local profiles
        loadLocalProfiles()
        
        // Load sample remote profiles for testing
        loadSampleRemoteProfiles()
    }
    
    /**
     * Get all local profiles
     */
    suspend fun getAllLocalProfiles(): List<DrivingProfile> = withContext(Dispatchers.IO) {
        return@withContext _localProfiles.value
    }
    
    /**
     * Get all remote profiles
     */
    suspend fun getAllRemoteProfiles(): List<DrivingProfile> = withContext(Dispatchers.IO) {
        return@withContext _remoteProfiles.value
    }
    
    /**
     * Get all profiles (both local and remote)
     */
    suspend fun getAllProfiles(): List<DrivingProfile> = withContext(Dispatchers.IO) {
        return@withContext _localProfiles.value + _remoteProfiles.value
    }
    
    /**
     * Get profiles by category
     */
    suspend fun getProfilesByCategory(category: ProfileCategory): List<DrivingProfile> = withContext(Dispatchers.IO) {
        return@withContext getAllProfiles().filter { it.category == category }
    }
    
    /**
     * Save a profile locally
     */
    suspend fun saveProfile(profile: DrivingProfile): Boolean = withContext(Dispatchers.IO) {
        try {
            val profileWithId = if (profile.id.isEmpty()) {
                profile.copy(id = generateProfileId())
            } else {
                profile
            }
            
            val fileName = "${profileWithId.id}.json"
            val file = File(profilesDirectory, fileName)
            
            val jsonString = json.encodeToString(profileWithId)
            file.writeText(jsonString)
            
            // Update the saved profiles list
            loadLocalProfiles()
            
            return@withContext true
        } catch (e: Exception) {
            e.printStackTrace()
            return@withContext false
        }
    }
    
    /**
     * Delete a profile
     */
    suspend fun deleteProfile(profile: DrivingProfile): Boolean = withContext(Dispatchers.IO) {
        try {
            // For remote profiles, we'd make a server call here
            if (profile.isRemote) {
                // Simulate API call
                delay(500)
                
                // Update remote profiles list - remove the profile
                _remoteProfiles.value = _remoteProfiles.value.filter { it.id != profile.id }
                
                return@withContext true
            } else {
                // Local profile
                val fileName = "${profile.id}.json"
                val file = File(profilesDirectory, fileName)
                
                val result = file.delete()
                
                // Update the saved profiles list
                if (result) {
                    loadLocalProfiles()
                }
                
                return@withContext result
            }
        } catch (e: Exception) {
            e.printStackTrace()
            return@withContext false
        }
    }
    
    /**
     * Import a profile from the server to local storage
     */
    suspend fun importProfileFromServer(profile: DrivingProfile): Boolean = withContext(Dispatchers.IO) {
        try {
            if (!profile.isRemote) return@withContext false
            
            // Create a local copy of the remote profile
            val localProfile = profile.copy(
                id = generateProfileId(),
                isRemote = false,
                timestamp = System.currentTimeMillis()
            )
            
            return@withContext saveProfile(localProfile)
        } catch (e: Exception) {
            e.printStackTrace()
            return@withContext false
        }
    }
    
    /**
     * Upload a profile to the server
     */
    suspend fun uploadProfileToServer(profile: DrivingProfile): Boolean = withContext(Dispatchers.IO) {
        try {
            // Simulate network call
            _isSyncing.value = true
            delay(1000)
            
            // In a real app, we'd make an API call here
            // For now, just simulate success
            _isSyncing.value = false
            
            return@withContext true
        } catch (e: Exception) {
            e.printStackTrace()
            _isSyncing.value = false
            return@withContext false
        }
    }
    
    /**
     * Sync with server to refresh remote profiles
     */
    suspend fun syncWithServer(): Boolean = withContext(Dispatchers.IO) {
        try {
            _isSyncing.value = true
            
            // Simulate network delay
            delay(1500)
            
            // In a real app, we'd make an API call to get the latest profiles
            // For now, just refresh our sample data
            loadSampleRemoteProfiles()
            
            _isSyncing.value = false
            return@withContext true
        } catch (e: Exception) {
            e.printStackTrace()
            _isSyncing.value = false
            return@withContext false
        }
    }
    
    /**
     * Create a new profile from a collection of animation curves
     */
    suspend fun createProfileFromCurves(
        name: String, 
        description: String,
        category: ProfileCategory,
        curves: List<AnimationCurve>,
        settings: Map<String, Float> = emptyMap()
    ): DrivingProfile = withContext(Dispatchers.Default) {
        val profile = DrivingProfile(
            id = generateProfileId(),
            name = name,
            description = description,
            category = category,
            animationCurves = curves,
            defaultSettings = settings,
            timestamp = System.currentTimeMillis()
        )
        
        // Save the profile
        saveProfile(profile)
        
        return@withContext profile
    }
    
    /**
     * Generate a unique profile ID
     */
    private fun generateProfileId(): String {
        val timestamp = System.currentTimeMillis()
        val random = Random.nextInt(1000, 9999)
        return "profile_${timestamp}_$random"
    }
    
    /**
     * Load all local profiles from storage
     */
    private fun loadLocalProfiles() {
        val profiles = mutableListOf<DrivingProfile>()
        
        profilesDirectory.listFiles()?.filter { it.isFile && it.extension == "json" }?.forEach { file ->
            try {
                val jsonString = file.readText()
                val profile = json.decodeFromString<DrivingProfile>(jsonString)
                profiles.add(profile)
            } catch (e: Exception) {
                e.printStackTrace()
            }
        }
        
        // Sort profiles by timestamp (newest first)
        _localProfiles.value = profiles.sortedByDescending { it.timestamp }
        
        // If no profiles exist, create a sample
        if (_localProfiles.value.isEmpty()) {
            val sampleProfile = DrivingProfile.createSample()
            kotlinx.coroutines.runBlocking {
                saveProfile(sampleProfile)
            }
        }
    }
    
    /**
     * Load sample remote profiles for testing
     */
    private fun loadSampleRemoteProfiles() {
        val remoteProfiles = listOf(
            DrivingProfile(
                id = "remote_profile_1",
                name = "Urban Commute",
                description = "Perfect for daily city commuting with traffic stops",
                category = ProfileCategory.CITY,
                animationCurves = listOf(AnimationCurve.createSample()),
                timestamp = System.currentTimeMillis() - 86400000, // 1 day ago
                isRemote = true
            ),
            DrivingProfile(
                id = "remote_profile_2",
                name = "Highway Cruising",
                description = "Smooth acceleration patterns for highway driving",
                category = ProfileCategory.HIGHWAY,
                animationCurves = listOf(AnimationCurve.createSample()),
                timestamp = System.currentTimeMillis() - 172800000, // 2 days ago
                isRemote = true
            ),
            DrivingProfile(
                id = "remote_profile_3",
                name = "Mountain Pass",
                description = "Varied acceleration and deceleration for winding roads",
                category = ProfileCategory.MOUNTAIN,
                animationCurves = listOf(AnimationCurve.createSample()),
                timestamp = System.currentTimeMillis() - 259200000, // 3 days ago
                isRemote = true
            ),
            DrivingProfile(
                id = "remote_profile_4",
                name = "Eco Drive",
                description = "Gentle acceleration patterns for maximum fuel efficiency",
                category = ProfileCategory.ECO,
                animationCurves = listOf(AnimationCurve.createSample()),
                timestamp = System.currentTimeMillis() - 345600000, // 4 days ago
                isRemote = true
            ),
            DrivingProfile(
                id = "remote_profile_5",
                name = "Track Day",
                description = "High performance driving patterns for track testing",
                category = ProfileCategory.RACE_TRACK,
                animationCurves = listOf(AnimationCurve.createSample()),
                timestamp = System.currentTimeMillis() - 432000000, // 5 days ago
                isRemote = true
            )
        )
        
        _remoteProfiles.value = remoteProfiles
    }
} 