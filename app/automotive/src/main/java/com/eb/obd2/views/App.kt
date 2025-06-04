package com.eb.obd2.views

import androidx.compose.animation.animateContentSize
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.AccountCircle
import androidx.compose.material.icons.filled.Assessment
import androidx.compose.material.icons.filled.BugReport
import androidx.compose.material.icons.filled.DirectionsCar
import androidx.compose.material.icons.filled.Extension
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.ElevatedCard
import androidx.compose.material3.FilledTonalButton
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.NavigationBar
import androidx.compose.material3.NavigationBarItem
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.window.Dialog
import androidx.hilt.navigation.compose.hiltViewModel
import androidx.navigation.NavDestination.Companion.hierarchy
import androidx.navigation.NavGraph.Companion.findStartDestination
import androidx.navigation.NavType
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import androidx.navigation.compose.currentBackStackEntryAsState
import androidx.navigation.compose.rememberNavController
import androidx.navigation.navArgument
import com.eb.obd2.models.RenderStatus
import com.eb.obd2.services.PluginService
import com.eb.obd2.services.DrivingScenarioSimulator
import com.eb.obd2.viewmodels.OBDViewModel
import com.eb.obd2.viewmodels.PluginViewModel
import com.eb.obd2.views.components.ComprehensiveDashboard
import com.eb.obd2.views.components.DataRecordingPanel
import com.eb.obd2.views.components.DrivingAnalysisPanel
import com.eb.obd2.views.components.EmulatorControlPanel
import com.eb.obd2.views.components.ErrorScreen
import com.eb.obd2.views.components.HistoricalDataDisplay
import com.eb.obd2.views.components.LoadingScreen
import com.eb.obd2.views.components.ScoreGraph
import com.eb.obd2.views.components.SimulationControlPanel
import com.eb.obd2.views.components.StyleIndicator
import com.eb.obd2.views.components.VehicleStatusMonitor
import com.eb.obd2.views.components.WarningNotificationsPanel
import com.eb.obd2.views.plugins.PluginDashboardScreen
import com.eb.obd2.views.plugins.PluginDetailScreen
import com.eb.obd2.views.plugins.PluginListScreen
import com.eb.obd2.views.screens.ProfilesScreen
import com.eb.obd2.views.components.ScenarioSelectionPanel
import com.eb.obd2.viewmodels.DrivingScenarioViewModel
import javax.inject.Inject

// Navigation route constants
sealed class Screen(val route: String, val title: String, val icon: @Composable () -> Unit) {
    object Dashboard : Screen(
        route = "dashboard",
        title = "Dashboard",
        icon = { Icon(Icons.Filled.DirectionsCar, contentDescription = null) }
    )
    
    object Analysis : Screen(
        route = "analysis",
        title = "Analysis",
        icon = { Icon(imageVector = Icons.Default.Assessment, contentDescription = null) }
    )
    
    object Plugins : Screen(
        route = "plugins",
        title = "Plugins",
        icon = { Icon(Icons.Filled.Extension, contentDescription = null) }
    )
    
    object Profiles : Screen(
        route = "profiles",
        title = "Profiles",
        icon = { Icon(Icons.Filled.AccountCircle, contentDescription = null) }
    )
    
    object Settings : Screen(
        route = "settings",
        title = "Settings",
        icon = { Icon(Icons.Filled.Settings, contentDescription = null) }
    )
    
    companion object {
        val items = listOf(Dashboard, Analysis, Plugins, Profiles, Settings)
    }
}

@Composable
fun App(
    viewModel: OBDViewModel = hiltViewModel()
) {
    val navController = rememberNavController()
    
    Scaffold(
        bottomBar = {
            NavigationBar {
                val navBackStackEntry by navController.currentBackStackEntryAsState()
                val currentDestination = navBackStackEntry?.destination
                
                Screen.items.forEach { screen ->
                    val isEnabled = when (viewModel.status) {
                        RenderStatus.SUCCESS -> true  // Allow navigation when connected
                        else -> screen.route == Screen.Dashboard.route  // Only Dashboard allowed when not connected
                    }
                    
                    NavigationBarItem(
                        icon = { screen.icon() },
                        label = { Text(screen.title) },
                        selected = currentDestination?.hierarchy?.any { it.route == screen.route } == true,
                        enabled = isEnabled,
                        onClick = {
                            if (isEnabled) {
                                navController.navigate(screen.route) {
                                    popUpTo(navController.graph.findStartDestination().id) {
                                        saveState = true
                                    }
                                    launchSingleTop = true
                                    restoreState = true
                                }
                            }
                        }
                    )
                }
            }
        }
    ) { innerPadding ->
        NavHost(
            navController = navController,
            startDestination = Screen.Dashboard.route,
            modifier = Modifier.padding(innerPadding)
        ) {
            composable(Screen.Dashboard.route) {
                when (viewModel.status) {
                    RenderStatus.LOADING -> LoadingScreen()
                    RenderStatus.ERROR -> ErrorScreen()
                    RenderStatus.SUCCESS -> DashboardScreen(viewModel)
                }
            }
            
            composable(Screen.Analysis.route) {
                if (viewModel.status == RenderStatus.SUCCESS) {
                    AnalysisScreen(viewModel)
                } else {
                    navController.navigate(Screen.Dashboard.route)
                }
            }
            
            composable(Screen.Plugins.route) {
                if (viewModel.status == RenderStatus.SUCCESS) {
                    val pluginViewModel: PluginViewModel = hiltViewModel()
                    PluginListScreen(
                        navController = navController,
                        viewModel = pluginViewModel
                    )
                } else {
                    navController.navigate(Screen.Dashboard.route)
                }
            }
            
            composable(
                route = "plugins/{pluginId}",
                arguments = listOf(navArgument("pluginId") { type = NavType.StringType })
            ) { backStackEntry ->
                val pluginId = backStackEntry.arguments?.getString("pluginId") ?: return@composable
                val pluginViewModel: PluginViewModel = hiltViewModel()
                PluginDetailScreen(
                    navController = navController,
                    pluginId = pluginId,
                    viewModel = pluginViewModel
                )
            }
            
            composable(
                route = "plugins/dashboard/{pluginId}",
                arguments = listOf(navArgument("pluginId") { type = NavType.StringType })
            ) { backStackEntry ->
                val pluginId = backStackEntry.arguments?.getString("pluginId") ?: return@composable
                val pluginViewModel: PluginViewModel = hiltViewModel()
                
                PluginDashboardScreen(
                    navController = navController,
                    viewModel = pluginViewModel,
                    pluginService = pluginViewModel.getPluginService()
                )
            }
            
            composable(Screen.Profiles.route) {
                if (viewModel.status == RenderStatus.SUCCESS) {
                    ProfilesScreen()
                } else {
                    navController.navigate(Screen.Dashboard.route)
                }
            }
            
            composable(Screen.Settings.route) {
                if (viewModel.status == RenderStatus.SUCCESS) {
                    val scenarioViewModel: DrivingScenarioViewModel = hiltViewModel()
                    SettingsScreen(
                        viewModel = viewModel,
                        scenarioSimulator = scenarioViewModel.getSimulator()
                    )
                } else {
                    navController.navigate(Screen.Dashboard.route)
                }
            }
        }
    }
}

@Composable
fun DashboardScreen(viewModel: OBDViewModel) {
    var showDebug by remember { mutableStateOf(false) }
    
    Box(
        modifier = Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background)
    ) {
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(16.dp)
                .verticalScroll(rememberScrollState()),
            horizontalAlignment = Alignment.CenterHorizontally,
            verticalArrangement = Arrangement.spacedBy(16.dp)
        ) {
            Text(
                text = "Vehicle Dashboard",
                style = MaterialTheme.typography.headlineMedium.copy(
                    fontWeight = FontWeight.Bold
                ),
                color = MaterialTheme.colorScheme.primary,
                modifier = Modifier.padding(bottom = 8.dp)
            )
            
            // Active Warnings Section
            if (viewModel.activeWarnings.isNotEmpty()) {
                ElevatedCard(
                    modifier = Modifier
                        .fillMaxWidth()
                        .animateContentSize(),
                    shape = RoundedCornerShape(24.dp),
                    colors = CardDefaults.elevatedCardColors(
                        containerColor = MaterialTheme.colorScheme.errorContainer.copy(alpha = 0.1f)
                    )
                ) {
                    WarningNotificationsPanel(
                        warnings = viewModel.activeWarnings,
                        onDismissWarning = { viewModel.dismissWarning(it) },
                        onAcknowledgeAll = { viewModel.dismissAllWarnings() },
                        modifier = Modifier.padding(16.dp)
                    )
                }
            }
            
            // Comprehensive Dashboard
            ElevatedCard(
                modifier = Modifier
                    .fillMaxWidth()
                    .animateContentSize(),
                shape = RoundedCornerShape(24.dp),
                colors = CardDefaults.elevatedCardColors(
                    containerColor = MaterialTheme.colorScheme.surface
                )
            ) {
                Column(
                    modifier = Modifier.padding(16.dp),
                    verticalArrangement = Arrangement.spacedBy(8.dp)
                ) {
                    Text(
                        text = "Live Metrics",
                        style = MaterialTheme.typography.titleMedium,
                        color = MaterialTheme.colorScheme.primary,
                        modifier = Modifier.padding(bottom = 8.dp)
                    )
                    
                    ComprehensiveDashboard(
                        rpm = viewModel.rpm,
                        engineTemperature = viewModel.engineTemperature,
                        fuelLevel = viewModel.fuelLevel,
                        fuelConsumptionRate = viewModel.fuelConsumptionRate,
                        averageConsumption = viewModel.averageFuelConsumption,
                        estimatedRange = viewModel.estimatedRange,
                        speedKmh = viewModel.speed,
                        throttlePosition = viewModel.throttlePosition,
                        brakePosition = viewModel.brakePosition,
                        currentGear = viewModel.currentGear,
                        gearPosition = viewModel.gearPosition,
                        modifier = Modifier.fillMaxWidth()
                    )
                }
            }
            
            // Driving Style Indicator
            ElevatedCard(
                modifier = Modifier
                    .fillMaxWidth()
                    .animateContentSize(),
                shape = RoundedCornerShape(24.dp),
                colors = CardDefaults.elevatedCardColors(
                    containerColor = MaterialTheme.colorScheme.secondaryContainer.copy(alpha = 0.5f)
                )
            ) {
                Column(
                    modifier = Modifier.padding(16.dp),
                    verticalArrangement = Arrangement.spacedBy(8.dp)
                ) {
                    Text(
                        text = "Driving Style Analysis",
                        style = MaterialTheme.typography.titleMedium,
                        color = MaterialTheme.colorScheme.primary,
                        modifier = Modifier.padding(bottom = 8.dp)
                    )
                    
                    StyleIndicator(
                        aggressiveScore = viewModel.aggressiveScores.last(),
                        comfortLevel = viewModel.comfortLevel,
                        accelerationScore = viewModel.accelerationScore,
                        decelerationScore = viewModel.decelerationScore,
                        corneringScore = viewModel.corneringScore,
                        engineEfficiencyScore = viewModel.engineEfficiencyScore,
                        drivingState = viewModel.drivingState,
                        modifier = Modifier.fillMaxWidth()
                    )
                }
            }
            
            // Vehicle Status Monitor
            ElevatedCard(
                modifier = Modifier
                    .fillMaxWidth()
                    .animateContentSize(),
                shape = RoundedCornerShape(24.dp),
                colors = CardDefaults.elevatedCardColors(
                    containerColor = MaterialTheme.colorScheme.tertiaryContainer.copy(alpha = 0.3f)
                )
            ) {
                Column(
                    modifier = Modifier.padding(16.dp),
                    verticalArrangement = Arrangement.spacedBy(8.dp)
                ) {
                    Text(
                        text = "Vehicle Status",
                        style = MaterialTheme.typography.titleMedium,
                        color = MaterialTheme.colorScheme.primary,
                        modifier = Modifier.padding(bottom = 8.dp)
                    )
                    
                    VehicleStatusMonitor(
                        engineStatus = viewModel.engineSystemStatus,
                        transmissionStatus = viewModel.transmissionSystemStatus,
                        fuelSystemStatus = viewModel.fuelSystemStatus,
                        emissionSystemStatus = viewModel.emissionSystemStatus,
                        brakeSystemStatus = viewModel.brakeSystemStatus,
                        batteryStatus = viewModel.batterySystemStatus,
                        engineTemperature = viewModel.engineTemperature,
                        batteryVoltage = viewModel.batteryVoltage,
                        oilPressure = viewModel.oilPressure,
                        fuelLevel = viewModel.fuelLevel,
                        engineDetails = viewModel.engineSystemDetails,
                        transmissionDetails = viewModel.transmissionSystemDetails,
                        fuelSystemDetails = viewModel.fuelSystemDetails,
                        emissionSystemDetails = viewModel.emissionSystemDetails,
                        brakeSystemDetails = viewModel.brakeSystemDetails,
                        batteryDetails = viewModel.batterySystemDetails,
                        modifier = Modifier.fillMaxWidth()
                    )
                }
            }
            
            // Debug Info Button
            FilledTonalButton(
                onClick = { showDebug = true },
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(vertical = 8.dp),
                shape = RoundedCornerShape(16.dp),
                colors = ButtonDefaults.filledTonalButtonColors(
                    containerColor = MaterialTheme.colorScheme.secondaryContainer
                )
            ) { 
                Row(
                    horizontalArrangement = Arrangement.Center,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Icon(
                        imageVector = Icons.Default.BugReport,
                        contentDescription = null,
                        modifier = Modifier.padding(end = 8.dp)
                    )
                    Text(
                        text = "Debug Information",
                        style = MaterialTheme.typography.labelLarge
                    )
                }
            }
        }
        
        if (showDebug) {
            DebugInfo(viewModel) { showDebug = false }
        }
    }
}

@Composable
fun SettingsScreen(
    viewModel: OBDViewModel,
    scenarioSimulator: DrivingScenarioSimulator
) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(16.dp)
            .verticalScroll(rememberScrollState()),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Text(
            text = "Settings",
            style = MaterialTheme.typography.headlineMedium,
            color = MaterialTheme.colorScheme.primary
        )
        
        Spacer(modifier = Modifier.height(16.dp))
        
        // Scenario Selection Panel
        ScenarioSelectionPanel(
            scenarioSimulator = scenarioSimulator,
            modifier = Modifier.fillMaxWidth()
        )
        
        Spacer(modifier = Modifier.height(16.dp))
        
        // Emulator Control Panel
        EmulatorControlPanel(
            viewModel = viewModel,
            modifier = Modifier.fillMaxWidth()
        )
        
        Spacer(modifier = Modifier.height(16.dp))
        
        // Data Recording Panel
        DataRecordingPanel(
            viewModel = viewModel,
            modifier = Modifier.fillMaxWidth()
        )
    }
}

@Composable
fun DebugInfo(viewModel: OBDViewModel, onDismiss: () -> Unit) {
    Dialog(onDismissRequest = onDismiss) {
        Surface(shape = MaterialTheme.shapes.medium) {
            Column(
                modifier = Modifier
                    .padding(16.dp)
                    .fillMaxWidth()
            ) {
                LazyColumn {
                    items(viewModel.obdData.toList()) { (key, value) ->
                        Text(
                            "$key: ${value.lastOrNull()?.value ?: "N/A"} ${value.lastOrNull()?.unit ?: ""}",
                            style = MaterialTheme.typography.bodyMedium
                        )
                        Spacer(modifier = Modifier.height(8.dp))
                    }
                }
                Spacer(modifier = Modifier.height(16.dp))
                Text(
                    text = "Comfort Level: ${viewModel.comfortLevel}",
                    style = MaterialTheme.typography.bodyMedium
                )
                Spacer(modifier = Modifier.height(16.dp))
                Text(
                    text = "Aggressive Driving State: ${viewModel.drivingState}",
                    style = MaterialTheme.typography.bodyMedium
                )
                
                Spacer(modifier = Modifier.height(16.dp))
                Button(onClick = onDismiss) {
                    Text("Close")
                }
            }
        }
    }
}

/**
 * Screen for data analysis, driving metrics, and historical data visualization
 */
@Composable
fun AnalysisScreen(viewModel: OBDViewModel) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(8.dp)
            .verticalScroll(rememberScrollState()),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        // Enhanced Driving Analysis Panel
        DrivingAnalysisPanel(
            overallScore = viewModel.overallDrivingScore,
            efficiencyScore = viewModel.efficiencyScore,
            smoothnessScore = viewModel.smoothnessScore,
            aggressivenessScore = 1.0f - viewModel.aggressiveScores.last().coerceIn(0f, 1f),
            comfortScore = viewModel.comfortLevel,
            modifier = Modifier.fillMaxWidth()
        )
        
        Spacer(modifier = Modifier.height(16.dp))
        
        // Detailed Driving Style Information
        Card(
            modifier = Modifier.fillMaxWidth(),
            shape = RoundedCornerShape(12.dp)
        ) {
            StyleIndicator(
                aggressiveScore = viewModel.aggressiveScores.last(),
                comfortLevel = viewModel.comfortLevel,
                accelerationScore = viewModel.accelerationScore,
                decelerationScore = viewModel.decelerationScore,
                corneringScore = viewModel.corneringScore,
                engineEfficiencyScore = viewModel.engineEfficiencyScore,
                drivingState = viewModel.drivingState,
                modifier = Modifier.padding(8.dp)
            )
        }
        
        Spacer(modifier = Modifier.height(24.dp))
        
        // Historical Data Display
        HistoricalDataDisplay(
            temperatureModelProducer = viewModel.temperatureModelProducer,
            fuelConsumptionModelProducer = viewModel.consumptionModelProducer,
            fuelLevelModelProducer = viewModel.fuelLevelModelProducer,
            speedModelProducer = viewModel.speedModelProducer,
            modifier = Modifier.fillMaxWidth()
        )
    }
}