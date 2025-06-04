package com.eb.obd2.views.components

import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Check
import androidx.compose.material.icons.filled.PlayArrow
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material.icons.filled.Stop
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedCard
import androidx.compose.material3.Slider
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import com.eb.obd2.models.DrivingProfile
import com.eb.obd2.services.OBDService
import com.eb.obd2.viewmodels.OBDViewModel

/**
 * Emulator control panel allowing adjustment of settings and triggering of simulations
 */
@Composable
fun EmulatorControlPanel(
    viewModel: OBDViewModel,
    modifier: Modifier = Modifier
) {
    val commandStatusMessage = viewModel.commandStatusMessage.collectAsState(null).value
    val twoWayCommEnabled = viewModel.twoWayCommEnabled.collectAsState(true).value
    val emulatorSettings = viewModel.emulatorSettings.collectAsState(emptyMap()).value
    val profiles = viewModel.localProfiles
    
    var showSettingsPanel by remember { mutableStateOf(false) }
    var showProfilesPanel by remember { mutableStateOf(false) }
    var showScenarioPanel by remember { mutableStateOf(false) }
    
    // Show any command status messages
    LaunchedEffect(commandStatusMessage) {
        // In a real app, we might want to show a toast or snackbar
        // for status messages rather than just logging them
        commandStatusMessage?.let {
            println("Command status: $it")
        }
    }
    
    Card(
        modifier = modifier
            .fillMaxWidth()
            .padding(16.dp)
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp)
        ) {
            // Header with two-way communication toggle
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = "Emulator Controls",
                    style = MaterialTheme.typography.titleLarge,
                    fontWeight = FontWeight.Bold
                )
                
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Text(
                        text = if (twoWayCommEnabled) "Enabled" else "Disabled",
                        style = MaterialTheme.typography.labelMedium
                    )
                    
                    Spacer(modifier = Modifier.width(8.dp))
                    
                    Switch(
                        checked = twoWayCommEnabled,
                        onCheckedChange = { enabled ->
                            viewModel.setTwoWayCommunicationEnabled(enabled)
                        }
                    )
                }
            }
            
            Spacer(modifier = Modifier.height(16.dp))
            
            if (twoWayCommEnabled) {
                // Control buttons
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceEvenly
                ) {
                    Button(
                        onClick = { showSettingsPanel = true }
                    ) {
                        Icon(
                            imageVector = Icons.Default.Settings,
                            contentDescription = "Adjust Settings",
                            modifier = Modifier.padding(end = 8.dp)
                        )
                        Text("Adjust Settings")
                    }
                    
                    Button(
                        onClick = { showProfilesPanel = true }
                    ) {
                        Icon(
                            imageVector = Icons.Default.PlayArrow,
                            contentDescription = "Run Profile",
                            modifier = Modifier.padding(end = 8.dp)
                        )
                        Text("Run Profile")
                    }
                    
                    Button(
                        onClick = { showScenarioPanel = true }
                    ) {
                        Icon(
                            imageVector = Icons.Default.PlayArrow,
                            contentDescription = "Run Scenario",
                            modifier = Modifier.padding(end = 8.dp)
                        )
                        Text("Run Scenario")
                    }
                }
                
                Spacer(modifier = Modifier.height(16.dp))
                
                // Stop button
                Button(
                    onClick = { viewModel.stopSimulation() },
                    modifier = Modifier.fillMaxWidth()
                ) {
                    Icon(
                        imageVector = Icons.Default.Stop,
                        contentDescription = "Stop Simulation",
                        modifier = Modifier.padding(end = 8.dp)
                    )
                    Text("Stop All Simulations")
                }
                
                // Current status display (if command status message is not null)
                commandStatusMessage?.let { message ->
                    Spacer(modifier = Modifier.height(16.dp))
                    
                    Text(
                        text = message,
                        style = MaterialTheme.typography.bodyMedium,
                        modifier = Modifier
                            .fillMaxWidth()
                            .padding(8.dp)
                            .border(
                                width = 1.dp,
                                color = MaterialTheme.colorScheme.outline,
                                shape = RoundedCornerShape(4.dp)
                            )
                            .padding(8.dp)
                    )
                }
            } else {
                // Two-way communication disabled message
                Text(
                    text = "Two-way communication is disabled. Enable it to control the emulator.",
                    style = MaterialTheme.typography.bodyMedium,
                    modifier = Modifier.padding(vertical = 16.dp)
                )
            }
        }
    }
    
    // Settings adjustment dialog
    if (showSettingsPanel) {
        EmulatorSettingsDialog(
            currentSettings = emulatorSettings,
            onSettingChange = { setting, value ->
                viewModel.adjustEmulatorSetting(setting, value)
            },
            onDismiss = { showSettingsPanel = false }
        )
    }
    
    // Profile selection dialog
    if (showProfilesPanel) {
        ProfileSelectionDialog(
            profiles = profiles,
            onProfileSelected = { profile ->
                viewModel.startProfileSimulation(profile)
                showProfilesPanel = false
            },
            onDismiss = { showProfilesPanel = false }
        )
    }
    
    // Scenario selection dialog
    if (showScenarioPanel) {
        ScenarioSelectionDialog(
            onScenarioSelected = { scenarioId ->
                viewModel.startDrivingScenario(scenarioId)
                showScenarioPanel = false
            },
            onDismiss = { showScenarioPanel = false }
        )
    }
}

/**
 * Dialog for adjusting emulator settings
 */
@Composable
fun EmulatorSettingsDialog(
    currentSettings: Map<OBDService.EmulatorSetting, Float>,
    onSettingChange: (OBDService.EmulatorSetting, Float) -> Unit,
    onDismiss: () -> Unit
) {
    AlertDialog(
        onDismissRequest = onDismiss,
        title = {
            Text("Adjust Emulator Settings")
        },
        text = {
            Column {
                // Display all settings with sliders
                OBDService.EmulatorSetting.values().forEach { setting ->
                    val currentValue = currentSettings[setting] ?: 1.0f
                    var sliderPosition by remember { mutableStateOf(currentValue) }
                    
                    Text(
                        text = getSettingDisplayName(setting),
                        style = MaterialTheme.typography.bodyMedium,
                        fontWeight = FontWeight.Bold
                    )
                    
                    Row(
                        modifier = Modifier
                            .fillMaxWidth()
                            .padding(vertical = 8.dp),
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Slider(
                            value = sliderPosition,
                            onValueChange = {
                                sliderPosition = it
                            },
                            onValueChangeFinished = {
                                onSettingChange(setting, sliderPosition)
                            },
                            valueRange = getSettingRange(setting),
                            steps = getSettingSteps(setting),
                            modifier = Modifier.weight(1f)
                        )
                        
                        Text(
                            text = formatSettingValue(setting, sliderPosition),
                            style = MaterialTheme.typography.bodySmall,
                            modifier = Modifier.padding(start = 8.dp)
                        )
                    }
                    
                    Spacer(modifier = Modifier.height(8.dp))
                }
            }
        },
        confirmButton = {
            Button(
                onClick = onDismiss
            ) {
                Text("Done")
            }
        }
    )
}

/**
 * Dialog for selecting a profile to run
 */
@Composable
fun ProfileSelectionDialog(
    profiles: List<DrivingProfile>,
    onProfileSelected: (DrivingProfile) -> Unit,
    onDismiss: () -> Unit
) {
    AlertDialog(
        onDismissRequest = onDismiss,
        title = {
            Text("Select Profile to Run")
        },
        text = {
            if (profiles.isEmpty()) {
                Text("No profiles available. Create a profile first.")
            } else {
                LazyColumn(
                    modifier = Modifier.height(300.dp)
                ) {
                    items(profiles) { profile ->
                        ProfileItem(
                            profile = profile,
                            onClick = { onProfileSelected(profile) }
                        )
                    }
                }
            }
        },
        confirmButton = {
            TextButton(
                onClick = onDismiss
            ) {
                Text("Cancel")
            }
        }
    )
}

/**
 * Dialog for selecting a predefined scenario to run
 */
@Composable
fun ScenarioSelectionDialog(
    onScenarioSelected: (String) -> Unit,
    onDismiss: () -> Unit
) {
    // Predefined scenarios - these would normally come from your backend
    val scenarios = listOf(
        "urban_driving" to "Urban Driving",
        "highway_cruise" to "Highway Cruising",
        "mountain_drive" to "Mountain Drive",
        "traffic_jam" to "Traffic Jam",
        "aggressive_driving" to "Aggressive Driving"
    )
    
    AlertDialog(
        onDismissRequest = onDismiss,
        title = {
            Text("Select Scenario")
        },
        text = {
            LazyColumn(
                modifier = Modifier.height(300.dp)
            ) {
                items(scenarios) { (id, name) ->
                    ScenarioItem(
                        id = id,
                        name = name,
                        onClick = { onScenarioSelected(id) }
                    )
                }
            }
        },
        confirmButton = {
            TextButton(
                onClick = onDismiss
            ) {
                Text("Cancel")
            }
        }
    )
}

/**
 * Item displaying a profile for selection
 */
@Composable
fun ProfileItem(
    profile: DrivingProfile,
    onClick: () -> Unit,
    modifier: Modifier = Modifier
) {
    OutlinedCard(
        modifier = modifier
            .fillMaxWidth()
            .padding(vertical = 4.dp),
        onClick = onClick
    ) {
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Column(
                modifier = Modifier.weight(1f)
            ) {
                Text(
                    text = profile.name,
                    style = MaterialTheme.typography.titleMedium,
                    fontWeight = FontWeight.Bold
                )
                
                Text(
                    text = profile.description,
                    style = MaterialTheme.typography.bodySmall,
                    maxLines = 2
                )
                
                Text(
                    text = "Category: ${profile.category.name}",
                    style = MaterialTheme.typography.labelSmall
                )
            }
            
            Icon(
                imageVector = Icons.Default.PlayArrow,
                contentDescription = "Run Profile",
                tint = MaterialTheme.colorScheme.primary
            )
        }
    }
}

/**
 * Item displaying a scenario for selection
 */
@Composable
fun ScenarioItem(
    id: String,
    name: String,
    onClick: () -> Unit,
    modifier: Modifier = Modifier
) {
    OutlinedCard(
        modifier = modifier
            .fillMaxWidth()
            .padding(vertical = 4.dp),
        onClick = onClick
    ) {
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp),
            verticalAlignment = Alignment.CenterVertically
        ) {
            Text(
                text = name,
                style = MaterialTheme.typography.titleMedium,
                fontWeight = FontWeight.Bold,
                modifier = Modifier.weight(1f)
            )
            
            Icon(
                imageVector = Icons.Default.PlayArrow,
                contentDescription = "Run Scenario",
                tint = MaterialTheme.colorScheme.primary
            )
        }
    }
}

/**
 * Format setting name to be more user-friendly
 */
private fun getSettingDisplayName(setting: OBDService.EmulatorSetting): String {
    return when (setting) {
        OBDService.EmulatorSetting.ENGINE_TEMP_FACTOR -> "Engine Temperature Factor"
        OBDService.EmulatorSetting.FUEL_CONSUMPTION_FACTOR -> "Fuel Consumption Factor"
        OBDService.EmulatorSetting.RPM_RESPONSE_FACTOR -> "RPM Response Factor"
        OBDService.EmulatorSetting.SIMULATION_SPEED -> "Simulation Speed"
        OBDService.EmulatorSetting.FAULT_SIMULATION -> "Fault Simulation Rate"
        OBDService.EmulatorSetting.BATTERY_VOLTAGE_FACTOR -> "Battery Voltage Factor"
        // Add a catch-all else branch for future enum values
        else -> "Unknown Setting"
    }
}

/**
 * Format setting value for display
 */
private fun formatSettingValue(setting: OBDService.EmulatorSetting, value: Float): String {
    return when (setting) {
        OBDService.EmulatorSetting.ENGINE_TEMP_FACTOR,
        OBDService.EmulatorSetting.FUEL_CONSUMPTION_FACTOR,
        OBDService.EmulatorSetting.RPM_RESPONSE_FACTOR,
        OBDService.EmulatorSetting.SIMULATION_SPEED -> String.format("%.1fx", value)
        OBDService.EmulatorSetting.FAULT_SIMULATION -> String.format("%.0f%%", value * 100)
        OBDService.EmulatorSetting.BATTERY_VOLTAGE_FACTOR -> String.format("%.1fV", value)
        else -> value.toString()
    }
}

/**
 * Get value range for a setting
 */
private fun getSettingRange(setting: OBDService.EmulatorSetting): ClosedFloatingPointRange<Float> {
    return when (setting) {
        OBDService.EmulatorSetting.ENGINE_TEMP_FACTOR,
        OBDService.EmulatorSetting.FUEL_CONSUMPTION_FACTOR,
        OBDService.EmulatorSetting.RPM_RESPONSE_FACTOR -> 0.5f..2.0f
        OBDService.EmulatorSetting.SIMULATION_SPEED -> 0.1f..2.0f
        OBDService.EmulatorSetting.FAULT_SIMULATION -> 0.0f..1.0f
        OBDService.EmulatorSetting.BATTERY_VOLTAGE_FACTOR -> 0.8f..1.2f
        else -> 0.0f..1.0f
    }
}

/**
 * Get setting steps
 */
private fun getSettingSteps(setting: OBDService.EmulatorSetting): Int {
    return when (setting) {
        OBDService.EmulatorSetting.ENGINE_TEMP_FACTOR -> 15 // 0.1 increments
        OBDService.EmulatorSetting.FUEL_CONSUMPTION_FACTOR -> 15 // 0.1 increments
        OBDService.EmulatorSetting.RPM_RESPONSE_FACTOR -> 15 // 0.1 increments
        OBDService.EmulatorSetting.SIMULATION_SPEED -> 29 // 0.1 increments
        OBDService.EmulatorSetting.FAULT_SIMULATION -> 10 // 10% increments
        OBDService.EmulatorSetting.BATTERY_VOLTAGE_FACTOR -> 4 // 0.1 increments
        else -> 10 // default value
    }
}

/**
 * Get default value for setting
 */
private fun getDefaultValue(setting: OBDService.EmulatorSetting): Float {
    return when (setting) {
        OBDService.EmulatorSetting.ENGINE_TEMP_FACTOR -> 1.0f
        OBDService.EmulatorSetting.FUEL_CONSUMPTION_FACTOR -> 1.0f
        OBDService.EmulatorSetting.RPM_RESPONSE_FACTOR -> 1.0f
        OBDService.EmulatorSetting.SIMULATION_SPEED -> 1.0f
        OBDService.EmulatorSetting.FAULT_SIMULATION -> 0.0f
        OBDService.EmulatorSetting.BATTERY_VOLTAGE_FACTOR -> 1.0f
        else -> 1.0f
    }
}

/**
 * Reset all settings to defaults
 */
private fun resetSettings(viewModel: OBDViewModel) {
    viewModel.adjustEmulatorSetting(OBDService.EmulatorSetting.ENGINE_TEMP_FACTOR, 1.0f)
    viewModel.adjustEmulatorSetting(OBDService.EmulatorSetting.FUEL_CONSUMPTION_FACTOR, 1.0f)
    viewModel.adjustEmulatorSetting(OBDService.EmulatorSetting.RPM_RESPONSE_FACTOR, 1.0f)
    viewModel.adjustEmulatorSetting(OBDService.EmulatorSetting.SIMULATION_SPEED, 1.0f)
    viewModel.adjustEmulatorSetting(OBDService.EmulatorSetting.FAULT_SIMULATION, 0.0f)
    viewModel.adjustEmulatorSetting(OBDService.EmulatorSetting.BATTERY_VOLTAGE_FACTOR, 1.0f)
}

/**
 * Get setting description
 */
private fun getSettingDescription(setting: OBDService.EmulatorSetting): String {
    return when (setting) {
        OBDService.EmulatorSetting.ENGINE_TEMP_FACTOR -> "Controls how quickly engine temperature changes"
        OBDService.EmulatorSetting.FUEL_CONSUMPTION_FACTOR -> "Multiplier for fuel consumption rate"
        OBDService.EmulatorSetting.RPM_RESPONSE_FACTOR -> "Controls how responsive RPM is to throttle changes"
        OBDService.EmulatorSetting.SIMULATION_SPEED -> "Overall speed of the simulation"
        OBDService.EmulatorSetting.FAULT_SIMULATION -> "Probability of random fault generation"
        OBDService.EmulatorSetting.BATTERY_VOLTAGE_FACTOR -> "Controls battery voltage behavior"
        else -> "No description available"
    }
}

/**
 * Get setting minimum value
 */
private fun getSettingMinValue(setting: OBDService.EmulatorSetting): Float {
    return when (setting) {
        OBDService.EmulatorSetting.ENGINE_TEMP_FACTOR -> 0.5f
        OBDService.EmulatorSetting.FUEL_CONSUMPTION_FACTOR -> 0.5f
        OBDService.EmulatorSetting.RPM_RESPONSE_FACTOR -> 0.5f
        OBDService.EmulatorSetting.SIMULATION_SPEED -> 0.1f
        OBDService.EmulatorSetting.FAULT_SIMULATION -> 0.0f
        OBDService.EmulatorSetting.BATTERY_VOLTAGE_FACTOR -> 0.8f
        else -> 0.0f
    }
}

/**
 * Get setting maximum value
 */
private fun getSettingMaxValue(setting: OBDService.EmulatorSetting): Float {
    return when (setting) {
        OBDService.EmulatorSetting.ENGINE_TEMP_FACTOR -> 2.0f
        OBDService.EmulatorSetting.FUEL_CONSUMPTION_FACTOR -> 2.0f
        OBDService.EmulatorSetting.RPM_RESPONSE_FACTOR -> 2.0f
        OBDService.EmulatorSetting.SIMULATION_SPEED -> 3.0f
        OBDService.EmulatorSetting.FAULT_SIMULATION -> 1.0f
        OBDService.EmulatorSetting.BATTERY_VOLTAGE_FACTOR -> 1.2f
        else -> 1.0f
    }
} 