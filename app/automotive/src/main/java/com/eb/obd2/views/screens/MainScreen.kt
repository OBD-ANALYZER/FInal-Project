package com.eb.obd2.views.screens

import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import com.eb.obd2.services.OBDService
import com.eb.obd2.views.components.VehicleDataDisplay

@Composable
fun MainScreen(
    obdService: OBDService,
    modifier: Modifier = Modifier
) {
    Column(
        modifier = modifier
            .fillMaxSize()
            .padding(16.dp)
    ) {
        // Vehicle Data Display
        VehicleDataDisplay(
            obdService = obdService,
            modifier = Modifier.fillMaxWidth()
        )
    }
} 