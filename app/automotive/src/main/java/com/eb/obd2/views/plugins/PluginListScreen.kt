package com.eb.obd2.views.plugins

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Add
import androidx.compose.material.icons.filled.CloudDownload
import androidx.compose.material.icons.filled.Refresh
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.Divider
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.ExtendedFloatingActionButton
import androidx.compose.material3.FilterChip
import androidx.compose.material3.FilterChipDefaults
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Snackbar
import androidx.compose.material3.SnackbarHost
import androidx.compose.material3.SnackbarHostState
import androidx.compose.material3.Switch
import androidx.compose.material3.Tab
import androidx.compose.material3.TabRow
import androidx.compose.material3.Text
import androidx.compose.material3.TopAppBar
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.hilt.navigation.compose.hiltViewModel
import androidx.navigation.NavController
import com.eb.obd2.models.CapabilityType
import com.eb.obd2.models.Plugin
import com.eb.obd2.viewmodels.PluginViewModel
import kotlinx.coroutines.launch

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun PluginListScreen(
    navController: NavController,
    viewModel: PluginViewModel = hiltViewModel()
) {
    val scope = rememberCoroutineScope()
    val snackbarHostState = remember { SnackbarHostState() }

    // State
    val availablePlugins by viewModel.availablePlugins.collectAsState()
    val installedPlugins by viewModel.installedPlugins.collectAsState()
    val isLoading by viewModel.isLoading.collectAsState()
    val errorMessage by viewModel.errorMessage.collectAsState()
    val capabilityFilter by viewModel.capabilityFilter.collectAsState()
    
    // Tab selection (0 = Installed, 1 = Available)
    var selectedTabIndex by remember { mutableStateOf(0) }
    
    // Filter dropdown
    var showFilterDropdown by remember { mutableStateOf(false) }
    
    // Display error message if needed
    errorMessage?.let {
        LaunchedEffect(it) {
            scope.launch {
                snackbarHostState.showSnackbar(message = it)
                viewModel.clearError()
            }
        }
    }

    Scaffold(
        topBar = {
            TopAppBar(
                title = { Text("Plugin Manager") },
                actions = {
                    IconButton(
                        onClick = {
                            scope.launch {
                                viewModel.fetchAvailablePlugins()
                            }
                        }
                    ) {
                        Icon(Icons.Filled.Refresh, contentDescription = "Refresh")
                    }
                    IconButton(
                        onClick = {
                            showFilterDropdown = true
                        }
                    ) {
                        Icon(Icons.Filled.Settings, contentDescription = "Filter")
                        CapabilityFilterDropdown(
                            expanded = showFilterDropdown,
                            onDismissRequest = { showFilterDropdown = false },
                            selectedCapability = capabilityFilter,
                            onSelectCapability = { capability ->
                                viewModel.setCapabilityFilter(capability)
                                showFilterDropdown = false
                            }
                        )
                    }
                }
            )
        },
        snackbarHost = {
            SnackbarHost(hostState = snackbarHostState) { data ->
                Snackbar(
                    modifier = Modifier.padding(16.dp),
                    action = {
                        Button(onClick = { snackbarHostState.currentSnackbarData?.dismiss() }) {
                            Text("Dismiss")
                        }
                    }
                ) {
                    Text(data.visuals.message)
                }
            }
        },
        floatingActionButton = {
            if (selectedTabIndex == 1) {
                ExtendedFloatingActionButton(
                    text = { Text("Store") },
                    icon = { Icon(Icons.Filled.CloudDownload, contentDescription = "Plugin Store") },
                    onClick = { navController.navigate("plugin_store") }
                )
            }
        }
    ) { paddingValues ->
        Column(
            modifier = Modifier
                .fillMaxSize()
                .padding(paddingValues)
        ) {
            // Tab row
            TabRow(selectedTabIndex = selectedTabIndex) {
                Tab(
                    selected = selectedTabIndex == 0,
                    onClick = { selectedTabIndex = 0 },
                    text = { Text("Installed") }
                )
                Tab(
                    selected = selectedTabIndex == 1,
                    onClick = { selectedTabIndex = 1 },
                    text = { Text("Available") }
                )
            }
            
            // Filter chips
            CapabilityFilterChips(
                selectedCapability = capabilityFilter,
                onSelectCapability = { viewModel.setCapabilityFilter(it) }
            )
            
            // Loading indicator
            if (isLoading) {
                Box(
                    modifier = Modifier.fillMaxSize(),
                    contentAlignment = Alignment.Center
                ) {
                    CircularProgressIndicator()
                }
            } else {
                // Content based on selected tab
                when (selectedTabIndex) {
                    0 -> {
                        // Installed plugins
                        val filteredPlugins = if (capabilityFilter == null) {
                            installedPlugins
                        } else {
                            viewModel.getFilteredInstalledPlugins()
                        }
                        
                        if (filteredPlugins.isEmpty()) {
                            Box(
                                modifier = Modifier.fillMaxSize(),
                                contentAlignment = Alignment.Center
                            ) {
                                Text(
                                    text = "No installed plugins",
                                    style = MaterialTheme.typography.bodyLarge
                                )
                            }
                        } else {
                            InstalledPluginsList(
                                plugins = filteredPlugins,
                                onPluginClick = { plugin ->
                                    navController.navigate("plugin_details/${plugin.id}")
                                },
                                onToggleEnabled = { plugin, enabled ->
                                    viewModel.setPluginEnabled(plugin.id, enabled)
                                }
                            )
                        }
                    }
                    1 -> {
                        // Available plugins
                        val filteredPlugins = if (capabilityFilter == null) {
                            availablePlugins
                        } else {
                            viewModel.getFilteredAvailablePlugins()
                        }
                        
                        if (filteredPlugins.isEmpty()) {
                            Box(
                                modifier = Modifier.fillMaxSize(),
                                contentAlignment = Alignment.Center
                            ) {
                                Text(
                                    text = "No available plugins",
                                    style = MaterialTheme.typography.bodyLarge
                                )
                            }
                        } else {
                            AvailablePluginsList(
                                plugins = filteredPlugins,
                                installedPluginIds = installedPlugins.map { it.id },
                                onPluginClick = { plugin ->
                                    navController.navigate("plugin_details/${plugin.id}")
                                },
                                onInstallClick = { plugin ->
                                    viewModel.installPlugin(plugin.id)
                                }
                            )
                        }
                    }
                }
            }
        }
    }
}

@Composable
fun CapabilityFilterChips(
    selectedCapability: CapabilityType?,
    onSelectCapability: (CapabilityType?) -> Unit
) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(8.dp),
        horizontalArrangement = Arrangement.spacedBy(8.dp)
    ) {
        FilterChip(
            selected = selectedCapability == null,
            onClick = { onSelectCapability(null) },
            label = { Text("All") },
            colors = FilterChipDefaults.filterChipColors(
                selectedContainerColor = MaterialTheme.colorScheme.primaryContainer,
                selectedLabelColor = MaterialTheme.colorScheme.onPrimaryContainer
            )
        )
        
        CapabilityType.values().forEach { capability ->
            FilterChip(
                selected = selectedCapability == capability,
                onClick = { onSelectCapability(capability) },
                label = { Text(capability.name.replace("_", " ")) },
                colors = FilterChipDefaults.filterChipColors(
                    selectedContainerColor = MaterialTheme.colorScheme.primaryContainer,
                    selectedLabelColor = MaterialTheme.colorScheme.onPrimaryContainer
                )
            )
        }
    }
}

@Composable
fun CapabilityFilterDropdown(
    expanded: Boolean,
    onDismissRequest: () -> Unit,
    selectedCapability: CapabilityType?,
    onSelectCapability: (CapabilityType?) -> Unit
) {
    DropdownMenu(
        expanded = expanded,
        onDismissRequest = onDismissRequest
    ) {
        DropdownMenuItem(
            text = { Text("All") },
            onClick = {
                onSelectCapability(null)
                onDismissRequest()
            }
        )
        
        Divider()
        
        CapabilityType.values().forEach { capability ->
            DropdownMenuItem(
                text = { Text(capability.name.replace("_", " ")) },
                onClick = {
                    onSelectCapability(capability)
                    onDismissRequest()
                }
            )
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun InstalledPluginsList(
    plugins: List<Plugin>,
    onPluginClick: (Plugin) -> Unit,
    onToggleEnabled: (Plugin, Boolean) -> Unit
) {
    LazyColumn(
        modifier = Modifier.fillMaxSize(),
        verticalArrangement = Arrangement.spacedBy(8.dp),
        contentPadding = PaddingValues(8.dp)
    ) {
        items(plugins) { plugin ->
            Card(
                modifier = Modifier.fillMaxWidth(),
                onClick = { onPluginClick(plugin) }
            ) {
                Column(
                    modifier = Modifier
                        .fillMaxWidth()
                        .padding(16.dp)
                ) {
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Text(
                            text = plugin.name,
                            style = MaterialTheme.typography.titleMedium,
                            fontWeight = FontWeight.Bold
                        )
                        
                        Switch(
                            checked = plugin.enabled,
                            onCheckedChange = { enabled ->
                                onToggleEnabled(plugin, enabled)
                            }
                        )
                    }
                    
                    Text(
                        text = plugin.description,
                        style = MaterialTheme.typography.bodyMedium,
                        maxLines = 2,
                        modifier = Modifier.padding(top = 8.dp)
                    )
                    
                    Row(
                        modifier = Modifier
                            .fillMaxWidth()
                            .padding(top = 8.dp),
                        horizontalArrangement = Arrangement.SpaceBetween
                    ) {
                        Text(
                            text = "Version: ${plugin.version}",
                            style = MaterialTheme.typography.bodySmall
                        )
                        
                        Text(
                            text = "By: ${plugin.author}",
                            style = MaterialTheme.typography.bodySmall
                        )
                    }
                    
                    // Capability chips
                    if (plugin.capabilities.isNotEmpty()) {
                        Row(
                            modifier = Modifier
                                .fillMaxWidth()
                                .padding(top = 8.dp),
                            horizontalArrangement = Arrangement.spacedBy(4.dp)
                        ) {
                            plugin.capabilities.take(3).forEach { capability ->
                                FilterChip(
                                    selected = false,
                                    onClick = { },
                                    enabled = false,
                                    label = { 
                                        Text(
                                            text = capability.type.name.replace("_", " "),
                                            style = MaterialTheme.typography.bodySmall
                                        )
                                    }
                                )
                            }
                            
                            if (plugin.capabilities.size > 3) {
                                Text(
                                    text = "+${plugin.capabilities.size - 3} more",
                                    style = MaterialTheme.typography.bodySmall,
                                    modifier = Modifier.align(Alignment.CenterVertically)
                                )
                            }
                        }
                    }
                }
            }
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun AvailablePluginsList(
    plugins: List<Plugin>,
    installedPluginIds: List<String>,
    onPluginClick: (Plugin) -> Unit,
    onInstallClick: (Plugin) -> Unit
) {
    LazyColumn(
        modifier = Modifier.fillMaxSize(),
        verticalArrangement = Arrangement.spacedBy(8.dp),
        contentPadding = PaddingValues(8.dp)
    ) {
        items(plugins) { plugin ->
            Card(
                modifier = Modifier.fillMaxWidth(),
                onClick = { onPluginClick(plugin) }
            ) {
                Column(
                    modifier = Modifier
                        .fillMaxWidth()
                        .padding(16.dp)
                ) {
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Text(
                            text = plugin.name,
                            style = MaterialTheme.typography.titleMedium,
                            fontWeight = FontWeight.Bold
                        )
                        
                        if (plugin.id in installedPluginIds) {
                            Text(
                                text = "Installed",
                                style = MaterialTheme.typography.bodyMedium,
                                color = MaterialTheme.colorScheme.primary
                            )
                        } else {
                            IconButton(
                                onClick = { onInstallClick(plugin) }
                            ) {
                                Icon(
                                    imageVector = Icons.Filled.Add,
                                    contentDescription = "Install"
                                )
                            }
                        }
                    }
                    
                    Text(
                        text = plugin.description,
                        style = MaterialTheme.typography.bodyMedium,
                        maxLines = 2,
                        modifier = Modifier.padding(top = 8.dp)
                    )
                    
                    Row(
                        modifier = Modifier
                            .fillMaxWidth()
                            .padding(top = 8.dp),
                        horizontalArrangement = Arrangement.SpaceBetween
                    ) {
                        Text(
                            text = "Version: ${plugin.version}",
                            style = MaterialTheme.typography.bodySmall
                        )
                        
                        Text(
                            text = "By: ${plugin.author}",
                            style = MaterialTheme.typography.bodySmall
                        )
                    }
                    
                    // Capability chips
                    if (plugin.capabilities.isNotEmpty()) {
                        Row(
                            modifier = Modifier
                                .fillMaxWidth()
                                .padding(top = 8.dp),
                            horizontalArrangement = Arrangement.spacedBy(4.dp)
                        ) {
                            plugin.capabilities.take(3).forEach { capability ->
                                FilterChip(
                                    selected = false,
                                    onClick = { },
                                    enabled = false,
                                    label = { 
                                        Text(
                                            text = capability.type.name.replace("_", " "),
                                            style = MaterialTheme.typography.bodySmall
                                        )
                                    }
                                )
                            }
                            
                            if (plugin.capabilities.size > 3) {
                                Text(
                                    text = "+${plugin.capabilities.size - 3} more",
                                    style = MaterialTheme.typography.bodySmall,
                                    modifier = Modifier.align(Alignment.CenterVertically)
                                )
                            }
                        }
                    }
                }
            }
        }
    }
} 