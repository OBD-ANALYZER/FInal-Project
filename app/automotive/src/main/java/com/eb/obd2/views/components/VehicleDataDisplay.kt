package com.eb.obd2.views.components

import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.eb.obd2.models.VehicleData
import com.eb.obd2.services.OBDService

@Composable
fun VehicleDataDisplay(
    obdService: OBDService,
    modifier: Modifier = Modifier
) {
    val vehicleData by obdService.vehicleData.collectAsState()

    Column(
        modifier = modifier
            .fillMaxWidth()
            .padding(16.dp),
        verticalArrangement = Arrangement.spacedBy(16.dp)
    ) {
        // Primary Data Row (RPM, Speed, Gear)
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween
        ) {
            DataCard(
                label = "RPM",
                value = "${vehicleData.rpm.toInt()}",
                modifier = Modifier.weight(1f)
            )
            Spacer(modifier = Modifier.width(8.dp))
            DataCard(
                label = "Speed",
                value = "${vehicleData.speed.toInt()} km/h",
                modifier = Modifier.weight(1f)
            )
            Spacer(modifier = Modifier.width(8.dp))
            DataCard(
                label = "Gear",
                value = "${vehicleData.gear} (${vehicleData.gearPosition})",
                modifier = Modifier.weight(1f)
            )
        }

        // Secondary Data Row (Engine Temp, Throttle, Brake)
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween
        ) {
            DataCard(
                label = "Engine Temp",
                value = "${vehicleData.engineTemp.toInt()}°C",
                modifier = Modifier.weight(1f)
            )
            Spacer(modifier = Modifier.width(8.dp))
            DataCard(
                label = "Throttle",
                value = "${vehicleData.throttlePercentage.toInt()}%",
                modifier = Modifier.weight(1f)
            )
            Spacer(modifier = Modifier.width(8.dp))
            DataCard(
                label = "Brake",
                value = "${vehicleData.brakePercentage.toInt()}%",
                modifier = Modifier.weight(1f)
            )
        }

        // Fuel Data Row
        Row(
            modifier = Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween
        ) {
            DataCard(
                label = "Fuel Level",
                value = "${vehicleData.fuelLevel.toInt()}%",
                modifier = Modifier.weight(1f)
            )
            Spacer(modifier = Modifier.width(8.dp))
            DataCard(
                label = "Fuel Consumption",
                value = "${String.format("%.2f", vehicleData.fuelConsumptionRate)} L/100km",
                modifier = Modifier.weight(1f)
            )
        }
    }
}

@Composable
private fun DataCard(
    label: String,
    value: String,
    modifier: Modifier = Modifier
) {
    Card(
        modifier = modifier,
        elevation = CardDefaults.cardElevation(defaultElevation = 4.dp)
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .padding(12.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Text(
                text = label,
                style = MaterialTheme.typography.bodyMedium,
                color = MaterialTheme.colorScheme.onSurfaceVariant
            )
            Spacer(modifier = Modifier.height(4.dp))
            Text(
                text = value,
                style = MaterialTheme.typography.titleLarge.copy(
                    fontWeight = FontWeight.Bold,
                    fontSize = 24.sp
                ),
                color = MaterialTheme.colorScheme.onSurface
            )
        }
    }
} 