package com.eb.obd2.views.components

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Battery5Bar
import androidx.compose.material.icons.filled.BatteryAlert
import androidx.compose.material.icons.filled.DirectionsCar
import androidx.compose.material.icons.filled.EnergySavingsLeaf
import androidx.compose.material.icons.filled.LocalGasStation
import androidx.compose.material.icons.filled.PriorityHigh
import androidx.compose.material.icons.filled.Sensors
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material.icons.filled.Shield
import androidx.compose.material.icons.outlined.Shield
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.Divider
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp

enum class SystemStatus {
    NORMAL,
    WARNING,
    CRITICAL,
    UNKNOWN
}

data class SystemStatusItem(
    val name: String,
    val status: SystemStatus,
    val icon: ImageVector,
    val details: String = ""
)

@Composable
fun VehicleStatusMonitor(
    engineStatus: SystemStatus = SystemStatus.NORMAL,
    transmissionStatus: SystemStatus = SystemStatus.NORMAL,
    fuelSystemStatus: SystemStatus = SystemStatus.NORMAL,
    emissionSystemStatus: SystemStatus = SystemStatus.NORMAL,
    brakeSystemStatus: SystemStatus = SystemStatus.NORMAL,
    batteryStatus: SystemStatus = SystemStatus.NORMAL,
    engineTemperature: Float = 90f,
    batteryVoltage: Float = 12.6f,
    oilPressure: Float = 30f,
    fuelLevel: Float = 50f,
    engineDetails: String = "",
    transmissionDetails: String = "",
    fuelSystemDetails: String = "",
    emissionSystemDetails: String = "",
    brakeSystemDetails: String = "",
    batteryDetails: String = "",
    modifier: Modifier = Modifier
) {
    val systems = listOf(
        SystemStatusItem(
            name = "Engine",
            status = engineStatus,
            icon = Icons.Filled.Settings,
            details = engineDetails.ifEmpty { getDefaultStatusDetails(engineStatus, "engine") }
        ),
        SystemStatusItem(
            name = "Transmission",
            status = transmissionStatus,
            icon = Icons.Filled.Settings,
            details = transmissionDetails.ifEmpty { getDefaultStatusDetails(transmissionStatus, "transmission") }
        ),
        SystemStatusItem(
            name = "Fuel System",
            status = fuelSystemStatus,
            icon = Icons.Filled.LocalGasStation,
            details = fuelSystemDetails.ifEmpty { getDefaultStatusDetails(fuelSystemStatus, "fuel system") }
        ),
        SystemStatusItem(
            name = "Emission",
            status = emissionSystemStatus,
            icon = Icons.Filled.EnergySavingsLeaf,
            details = emissionSystemDetails.ifEmpty { getDefaultStatusDetails(emissionSystemStatus, "emission system") }
        ),
        SystemStatusItem(
            name = "Brakes",
            status = brakeSystemStatus,
            icon = Icons.Filled.Shield,
            details = brakeSystemDetails.ifEmpty { getDefaultStatusDetails(brakeSystemStatus, "brake system") }
        ),
        SystemStatusItem(
            name = "Battery",
            status = batteryStatus,
            icon = Icons.Filled.Battery5Bar,
            details = batteryDetails.ifEmpty { getDefaultStatusDetails(batteryStatus, "battery") }
        )
    )
    
    Card(
        modifier = modifier
            .fillMaxWidth()
            .padding(16.dp),
        elevation = CardDefaults.cardElevation(defaultElevation = 4.dp),
        shape = RoundedCornerShape(12.dp)
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp)
        ) {
            // Title with overall status indicator
            Row(
                modifier = Modifier.fillMaxWidth(),
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.SpaceBetween
            ) {
                Text(
                    text = "Vehicle Health Status",
                    style = MaterialTheme.typography.titleLarge,
                    fontWeight = FontWeight.Bold
                )
                
                val overallStatus = getOverallStatus(systems.map { it.status })
                
                StatusIndicator(
                    status = overallStatus,
                    size = 24.dp
                )
            }
            
            Spacer(modifier = Modifier.height(8.dp))
            
            Divider()
            
            // Main parameters
            Spacer(modifier = Modifier.height(16.dp))
            
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceEvenly
            ) {
                // Engine temp
                KeyValueMetric(
                    label = "Engine",
                    value = String.format("%.2f°C", engineTemperature),
                    icon = Icons.Filled.Settings,
                    critical = engineTemperature > 110f || engineTemperature < 50f,
                    warning = engineTemperature > 100f || engineTemperature < 60f
                )
                
                // Battery voltage
                KeyValueMetric(
                    label = "Battery",
                    value = String.format("%.2fV", batteryVoltage),
                    icon = Icons.Filled.Battery5Bar,
                    critical = batteryVoltage < 11.8f || batteryVoltage > 14.8f,
                    warning = batteryVoltage < 12.2f || batteryVoltage > 14.4f
                )
                
                // Oil pressure
                KeyValueMetric(
                    label = "Oil",
                    value = String.format("%.2fpsi", oilPressure),
                    icon = Icons.Filled.DirectionsCar,
                    critical = oilPressure < 10f,
                    warning = oilPressure < 20f
                )
                
                // Fuel level
                KeyValueMetric(
                    label = "Fuel",
                    value = String.format("%.2f%%", fuelLevel),
                    icon = Icons.Filled.LocalGasStation,
                    critical = fuelLevel < 10f,
                    warning = fuelLevel < 20f
                )
            }
            
            Spacer(modifier = Modifier.height(16.dp))
            
            Divider()
            
            // System status icons
            Spacer(modifier = Modifier.height(16.dp))
            
            // Display systems in a grid (2x3)
            systems.chunked(3).forEach { rowItems ->
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceEvenly
                ) {
                    rowItems.forEach { system ->
                        SystemStatusIndicator(system = system)
                    }
                    
                    // Add spacers for incomplete rows
                    repeat(3 - rowItems.size) {
                        Spacer(modifier = Modifier.width(80.dp))
                    }
                }
                
                Spacer(modifier = Modifier.height(16.dp))
            }
            
            // Display warning/critical messages if any
            val issueMessages = systems
                .filter { it.status != SystemStatus.NORMAL }
                .map { it.details }
            
            if (issueMessages.isNotEmpty()) {
                Divider()
                
                Spacer(modifier = Modifier.height(8.dp))
                
                Text(
                    text = "Issues Detected",
                    style = MaterialTheme.typography.titleMedium,
                    fontWeight = FontWeight.Bold,
                    color = MaterialTheme.colorScheme.error
                )
                
                Spacer(modifier = Modifier.height(8.dp))
                
                Column(
                    verticalArrangement = Arrangement.spacedBy(4.dp)
                ) {
                    issueMessages.forEach { message ->
                        Row(
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Icon(
                                imageVector = Icons.Filled.PriorityHigh,
                                contentDescription = "Warning",
                                tint = MaterialTheme.colorScheme.error,
                                modifier = Modifier.size(16.dp)
                            )
                            
                            Spacer(modifier = Modifier.width(8.dp))
                            
                            Text(
                                text = message,
                                style = MaterialTheme.typography.bodyMedium
                            )
                        }
                    }
                }
            }
        }
    }
}

@Composable
fun SystemStatusIndicator(system: SystemStatusItem) {
    Column(
        horizontalAlignment = Alignment.CenterHorizontally,
        verticalArrangement = Arrangement.Center,
        modifier = Modifier.width(80.dp)
    ) {
        Box(
            modifier = Modifier
                .size(48.dp)
                .border(
                    width = 2.dp,
                    color = getStatusColor(system.status).copy(alpha = 0.3f),
                    shape = CircleShape
                )
                .padding(4.dp),
            contentAlignment = Alignment.Center
        ) {
            Icon(
                imageVector = system.icon,
                contentDescription = system.name,
                tint = getStatusColor(system.status),
                modifier = Modifier.size(28.dp)
            )
        }
        
        Spacer(modifier = Modifier.height(4.dp))
        
        Text(
            text = system.name,
            style = MaterialTheme.typography.bodyMedium,
            color = MaterialTheme.colorScheme.onSurface
        )
        
        Text(
            text = getStatusText(system.status),
            style = MaterialTheme.typography.bodySmall,
            color = getStatusColor(system.status),
            fontWeight = FontWeight.Bold
        )
    }
}

@Composable
fun StatusIndicator(
    status: SystemStatus,
    size: androidx.compose.ui.unit.Dp = 16.dp,
    modifier: Modifier = Modifier
) {
    Box(
        modifier = modifier
            .size(size)
            .clip(CircleShape)
            .background(getStatusColor(status))
    )
}

@Composable
fun KeyValueMetric(
    label: String,
    value: String,
    icon: ImageVector,
    critical: Boolean = false,
    warning: Boolean = false
) {
    val status = when {
        critical -> SystemStatus.CRITICAL
        warning -> SystemStatus.WARNING
        else -> SystemStatus.NORMAL
    }
    
    Column(
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Icon(
            imageVector = icon,
            contentDescription = label,
            tint = getStatusColor(status),
            modifier = Modifier.size(24.dp)
        )
        
        Spacer(modifier = Modifier.height(4.dp))
        
        Text(
            text = value,
            style = MaterialTheme.typography.bodyLarge,
            fontWeight = FontWeight.Bold,
            color = if (critical || warning) getStatusColor(status) else MaterialTheme.colorScheme.onSurface
        )
        
        Text(
            text = label,
            style = MaterialTheme.typography.bodySmall,
            color = MaterialTheme.colorScheme.onSurface.copy(alpha = 0.7f)
        )
    }
}

/**
 * Helper functions
 */
private fun getOverallStatus(statusList: List<SystemStatus>): SystemStatus {
    return when {
        statusList.any { it == SystemStatus.CRITICAL } -> SystemStatus.CRITICAL
        statusList.any { it == SystemStatus.WARNING } -> SystemStatus.WARNING
        statusList.all { it == SystemStatus.NORMAL } -> SystemStatus.NORMAL
        else -> SystemStatus.UNKNOWN
    }
}

private fun getStatusColor(status: SystemStatus): Color {
    return when (status) {
        SystemStatus.NORMAL -> Color(0xFF4CAF50) // Green
        SystemStatus.WARNING -> Color(0xFFFFC107) // Amber
        SystemStatus.CRITICAL -> Color(0xFFF44336) // Red
        SystemStatus.UNKNOWN -> Color.Gray
    }
}

private fun getStatusText(status: SystemStatus): String {
    return when (status) {
        SystemStatus.NORMAL -> "OK"
        SystemStatus.WARNING -> "Warning"
        SystemStatus.CRITICAL -> "Critical"
        SystemStatus.UNKNOWN -> "Unknown"
    }
}

private fun getDefaultStatusDetails(status: SystemStatus, system: String): String {
    return when (status) {
        SystemStatus.WARNING -> "Potential issue detected with $system"
        SystemStatus.CRITICAL -> "$system requires immediate attention"
        else -> ""
    }
}

@Preview(showBackground = true)
@Composable
fun VehicleStatusMonitorPreview() {
    MaterialTheme {
        VehicleStatusMonitor(
            engineStatus = SystemStatus.NORMAL,
            transmissionStatus = SystemStatus.WARNING,
            fuelSystemStatus = SystemStatus.NORMAL,
            emissionSystemStatus = SystemStatus.CRITICAL,
            brakeSystemStatus = SystemStatus.NORMAL,
            batteryStatus = SystemStatus.NORMAL,
            engineTemperature = 95f,
            batteryVoltage = 12.2f,
            oilPressure = 28f,
            fuelLevel = 35f,
            transmissionDetails = "Transmission fluid temperature high",
            emissionSystemDetails = "Check engine light on: Oxygen sensor fault"
        )
    }
} 