package com.eb.obd2.views.plugins

import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Add
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.ExtendedFloatingActionButton
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Scaffold
import androidx.compose.material3.SnackbarHost
import androidx.compose.material3.SnackbarHostState
import androidx.compose.material3.Text
import androidx.compose.material3.TopAppBar
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.hilt.navigation.compose.hiltViewModel
import androidx.navigation.NavController
import com.eb.obd2.models.CapabilityType
import com.eb.obd2.services.PluginService
import com.eb.obd2.viewmodels.PluginViewModel
import javax.inject.Inject

/**
 * Screen that displays a dashboard of plugin visualizations
 */
@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun PluginDashboardScreen(
    navController: NavController,
    viewModel: PluginViewModel = hiltViewModel(),
    pluginService: PluginService
) {
    val snackbarHostState = remember { SnackbarHostState() }
    val scrollState = rememberScrollState()
    
    // State
    val installedPlugins by viewModel.installedPlugins.collectAsState()
    val enabledPlugins = installedPlugins.filter { it.enabled }
    val isLoading by viewModel.isLoading.collectAsState()
    val errorMessage by viewModel.errorMessage.collectAsState()
    
    // Get visualization plugins
    val visualizationPlugins = remember(enabledPlugins) {
        enabledPlugins.filter { plugin ->
            plugin.capabilities.any { it.type == CapabilityType.VISUALIZATION }
        }
    }
    
    // Display error message if needed
    errorMessage?.let {
        LaunchedEffect(it) {
            snackbarHostState.showSnackbar(message = it)
            viewModel.clearError()
        }
    }
    
    Scaffold(
        topBar = {
            TopAppBar(
                title = { Text("Visualizations Dashboard") },
                actions = {
                    IconButton(onClick = { navController.navigate("plugin_list") }) {
                        Icon(
                            imageVector = Icons.Default.Settings,
                            contentDescription = "Manage Plugins"
                        )
                    }
                }
            )
        },
        snackbarHost = { SnackbarHost(snackbarHostState) },
        floatingActionButton = {
            ExtendedFloatingActionButton(
                text = { Text("Add Plugin") },
                icon = { Icon(Icons.Default.Add, contentDescription = "Add Plugin") },
                onClick = { navController.navigate("plugin_list") }
            )
        }
    ) { paddingValues ->
        Box(
            modifier = Modifier
                .fillMaxSize()
                .padding(paddingValues)
        ) {
            if (isLoading) {
                CircularProgressIndicator(
                    modifier = Modifier.align(Alignment.Center)
                )
            } else if (visualizationPlugins.isEmpty()) {
                Column(
                    modifier = Modifier
                        .fillMaxSize()
                        .padding(32.dp),
                    horizontalAlignment = Alignment.CenterHorizontally
                ) {
                    Text(
                        text = "No visualization plugins enabled",
                        style = MaterialTheme.typography.headlineSmall,
                        textAlign = TextAlign.Center
                    )
                    Text(
                        text = "Install and enable plugins with visualization capabilities to see data visualizations.",
                        style = MaterialTheme.typography.bodyMedium,
                        textAlign = TextAlign.Center,
                        modifier = Modifier.padding(top = 8.dp)
                    )
                }
            } else {
                Column(
                    modifier = Modifier
                        .fillMaxSize()
                        .verticalScroll(scrollState)
                        .padding(8.dp)
                ) {
                    PluginVisualizationDashboard(
                        plugins = visualizationPlugins,
                        pluginService = pluginService,
                        onConfigurePlugin = { plugin ->
                            navController.navigate("plugin_details/${plugin.id}")
                        }
                    )
                }
            }
        }
    }
} 