package com.eb.obd2.views.plugins

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ArrowBack
import androidx.compose.material.icons.filled.Delete
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.Card
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.Divider
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.ExposedDropdownMenuBox
import androidx.compose.material3.ExposedDropdownMenuDefaults
import androidx.compose.material3.FilterChip
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Slider
import androidx.compose.material3.SliderDefaults
import androidx.compose.material3.SnackbarHost
import androidx.compose.material3.SnackbarHostState
import androidx.compose.material3.Surface
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
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
import androidx.compose.ui.focus.onFocusChanged
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.hilt.navigation.compose.hiltViewModel
import androidx.navigation.NavController
import com.eb.obd2.models.ConfigOptionType
import com.eb.obd2.models.Plugin
import com.eb.obd2.models.PluginConfigOption
import com.eb.obd2.viewmodels.PluginViewModel
import kotlinx.coroutines.launch

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun PluginDetailScreen(
    navController: NavController,
    pluginId: String,
    viewModel: PluginViewModel = hiltViewModel()
) {
    val scope = rememberCoroutineScope()
    val snackbarHostState = remember { SnackbarHostState() }
    
    // State
    val isLoading by viewModel.isLoading.collectAsState()
    val errorMessage by viewModel.errorMessage.collectAsState()
    
    // Find plugin in repositories
    val installedPlugins by viewModel.installedPlugins.collectAsState()
    val availablePlugins by viewModel.availablePlugins.collectAsState()
    
    val plugin = remember(pluginId, installedPlugins, availablePlugins) {
        installedPlugins.find { it.id == pluginId }
            ?: availablePlugins.find { it.id == pluginId }
    }
    
    // Uninstall confirmation dialog
    var showUninstallDialog by remember { mutableStateOf(false) }
    
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
                title = { Text("Plugin Details") },
                navigationIcon = {
                    IconButton(onClick = { navController.navigateUp() }) {
                        Icon(Icons.Default.ArrowBack, contentDescription = "Back")
                    }
                },
                actions = {
                    if (plugin != null && installedPlugins.any { it.id == pluginId }) {
                        // Show delete button only for installed plugins
                        IconButton(onClick = { showUninstallDialog = true }) {
                            Icon(Icons.Default.Delete, contentDescription = "Uninstall")
                        }
                    }
                }
            )
        },
        snackbarHost = { SnackbarHost(snackbarHostState) }
    ) { paddingValues ->
        if (isLoading) {
            Box(
                modifier = Modifier.fillMaxSize(),
                contentAlignment = Alignment.Center
            ) {
                CircularProgressIndicator()
            }
        } else if (plugin == null) {
            Box(
                modifier = Modifier
                    .fillMaxSize()
                    .padding(paddingValues),
                contentAlignment = Alignment.Center
            ) {
                Text("Plugin not found")
            }
        } else {
            PluginDetailContent(
                plugin = plugin,
                isInstalled = installedPlugins.any { it.id == pluginId },
                modifier = Modifier.padding(paddingValues),
                onInstall = {
                    viewModel.installPlugin(plugin.id)
                },
                onUninstall = {
                    showUninstallDialog = true
                },
                onToggleEnabled = { enabled ->
                    viewModel.setPluginEnabled(plugin.id, enabled)
                },
                onUpdateConfig = { optionId, value ->
                    viewModel.updatePluginConfig(plugin.id, optionId, value)
                }
            )
        }
    }
    
    // Uninstall confirmation dialog
    if (showUninstallDialog) {
        AlertDialog(
            onDismissRequest = { showUninstallDialog = false },
            title = { Text("Uninstall Plugin") },
            text = { Text("Are you sure you want to uninstall this plugin?") },
            confirmButton = {
                TextButton(
                    onClick = {
                        showUninstallDialog = false
                        viewModel.uninstallPlugin(pluginId)
                        navController.navigateUp()
                    }
                ) {
                    Text("Uninstall")
                }
            },
            dismissButton = {
                TextButton(onClick = { showUninstallDialog = false }) {
                    Text("Cancel")
                }
            }
        )
    }
}

@Composable
fun PluginDetailContent(
    plugin: Plugin,
    isInstalled: Boolean,
    modifier: Modifier = Modifier,
    onInstall: () -> Unit,
    onUninstall: () -> Unit,
    onToggleEnabled: (Boolean) -> Unit,
    onUpdateConfig: (String, String) -> Unit
) {
    val scrollState = rememberScrollState()
    
    Column(
        modifier = modifier
            .fillMaxSize()
            .verticalScroll(scrollState)
            .padding(16.dp)
    ) {
        // Header
        Text(
            text = plugin.name,
            style = MaterialTheme.typography.headlineMedium,
            fontWeight = FontWeight.Bold
        )
        
        Text(
            text = "Version ${plugin.version} by ${plugin.author}",
            style = MaterialTheme.typography.bodyMedium,
            modifier = Modifier.padding(top = 4.dp)
        )
        
        // Description
        Card(
            modifier = Modifier
                .fillMaxWidth()
                .padding(vertical = 16.dp)
        ) {
            Column(
                modifier = Modifier.padding(16.dp)
            ) {
                Text(
                    text = "Description",
                    style = MaterialTheme.typography.titleMedium,
                    fontWeight = FontWeight.Bold
                )
                
                Text(
                    text = plugin.description,
                    style = MaterialTheme.typography.bodyMedium,
                    modifier = Modifier.padding(top = 8.dp)
                )
            }
        }
        
        // Capabilities
        if (plugin.capabilities.isNotEmpty()) {
            Card(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(bottom = 16.dp)
            ) {
                Column(
                    modifier = Modifier.padding(16.dp)
                ) {
                    Text(
                        text = "Capabilities",
                        style = MaterialTheme.typography.titleMedium,
                        fontWeight = FontWeight.Bold
                    )
                    
                    Spacer(Modifier.height(8.dp))
                    
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.spacedBy(8.dp)
                    ) {
                        plugin.capabilities.forEach { capability ->
                            FilterChip(
                                selected = false,
                                onClick = { },
                                enabled = false,
                                label = { 
                                    Text(
                                        text = capability.type.name.replace("_", " "),
                                        style = MaterialTheme.typography.bodyMedium
                                    )
                                }
                            )
                        }
                    }
                    
                    Spacer(Modifier.height(8.dp))
                    
                    // Additional capability details
                    plugin.capabilities.forEach { capability ->
                        Column(
                            modifier = Modifier.padding(vertical = 4.dp)
                        ) {
                            Text(
                                text = capability.name,
                                style = MaterialTheme.typography.bodyMedium,
                                fontWeight = FontWeight.Bold
                            )
                            
                            Text(
                                text = capability.description,
                                style = MaterialTheme.typography.bodySmall
                            )
                        }
                    }
                }
            }
        }
        
        // Plugin status and actions
        Card(
            modifier = Modifier
                .fillMaxWidth()
                .padding(bottom = 16.dp)
        ) {
            Column(
                modifier = Modifier.padding(16.dp)
            ) {
                Text(
                    text = "Status",
                    style = MaterialTheme.typography.titleMedium,
                    fontWeight = FontWeight.Bold
                )
                
                Spacer(Modifier.height(8.dp))
                
                if (isInstalled) {
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween,
                        verticalAlignment = Alignment.CenterVertically
                    ) {
                        Text(
                            text = "Enabled",
                            style = MaterialTheme.typography.bodyMedium
                        )
                        
                        Switch(
                            checked = plugin.enabled,
                            onCheckedChange = onToggleEnabled
                        )
                    }
                    
                    Spacer(Modifier.height(8.dp))
                    
                    Button(
                        onClick = onUninstall,
                        modifier = Modifier.fillMaxWidth()
                    ) {
                        Text("Uninstall")
                    }
                } else {
                    Button(
                        onClick = onInstall,
                        modifier = Modifier.fillMaxWidth()
                    ) {
                        Text("Install")
                    }
                }
            }
        }
        
        // Configuration options
        if (isInstalled && plugin.configOptions.isNotEmpty()) {
            Card(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(bottom = 16.dp)
            ) {
                Column(
                    modifier = Modifier.padding(16.dp)
                ) {
                    Text(
                        text = "Configuration",
                        style = MaterialTheme.typography.titleMedium,
                        fontWeight = FontWeight.Bold
                    )
                    
                    Spacer(Modifier.height(16.dp))
                    
                    plugin.configOptions.forEach { option ->
                        ConfigOptionItem(
                            option = option,
                            onValueChange = { value ->
                                onUpdateConfig(option.id, value)
                            }
                        )
                        
                        Divider(modifier = Modifier.padding(vertical = 8.dp))
                    }
                }
            }
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun ConfigOptionItem(
    option: PluginConfigOption,
    onValueChange: (String) -> Unit
) {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 8.dp)
    ) {
        Text(
            text = option.name,
            style = MaterialTheme.typography.bodyLarge,
            fontWeight = FontWeight.Bold
        )
        
        Text(
            text = option.description,
            style = MaterialTheme.typography.bodySmall,
            modifier = Modifier.padding(bottom = 8.dp)
        )
        
        when (option.type) {
            ConfigOptionType.STRING -> {
                OutlinedTextField(
                    value = option.currentValue,
                    onValueChange = onValueChange,
                    modifier = Modifier.fillMaxWidth(),
                    singleLine = true
                )
            }
            
            ConfigOptionType.NUMBER -> {
                var textValue by remember { mutableStateOf(option.currentValue) }
                var sliderValue by remember {
                    mutableStateOf(option.currentValue.toFloatOrNull() ?: 0f)
                }
                
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Slider(
                        value = sliderValue,
                        onValueChange = { 
                            sliderValue = it
                            textValue = it.toString()
                        },
                        onValueChangeFinished = {
                            onValueChange(sliderValue.toString())
                        },
                        colors = SliderDefaults.colors(
                            thumbColor = MaterialTheme.colorScheme.primary,
                            activeTrackColor = MaterialTheme.colorScheme.primary
                        ),
                        modifier = Modifier.weight(3f)
                    )
                    
                    OutlinedTextField(
                        value = textValue,
                        onValueChange = { value ->
                            textValue = value
                            value.toFloatOrNull()?.let {
                                sliderValue = it
                            }
                        },
                        modifier = Modifier
                            .weight(1f)
                            .padding(start = 8.dp)
                            .onFocusChanged { 
                                if (!it.isFocused) {
                                    onValueChange(textValue)
                                }
                            },
                        singleLine = true
                    )
                }
            }
            
            ConfigOptionType.BOOLEAN -> {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween,
                    verticalAlignment = Alignment.CenterVertically
                ) {
                    Text(
                        text = if (option.currentValue.toBoolean()) "Enabled" else "Disabled",
                        style = MaterialTheme.typography.bodyMedium
                    )
                    
                    Switch(
                        checked = option.currentValue.toBoolean(),
                        onCheckedChange = { onValueChange(it.toString()) }
                    )
                }
            }
            
            ConfigOptionType.SELECTION -> {
                var expanded by remember { mutableStateOf(false) }
                
                ExposedDropdownMenuBox(
                    expanded = expanded,
                    onExpandedChange = { expanded = it }
                ) {
                    OutlinedTextField(
                        value = option.currentValue,
                        onValueChange = {},
                        readOnly = true,
                        trailingIcon = { ExposedDropdownMenuDefaults.TrailingIcon(expanded = expanded) },
                        modifier = Modifier
                            .fillMaxWidth()
                            .menuAnchor()
                    )
                    
                    ExposedDropdownMenu(
                        expanded = expanded,
                        onDismissRequest = { expanded = false }
                    ) {
                        option.possibleValues.forEach { value ->
                            DropdownMenuItem(
                                text = { Text(value) },
                                onClick = {
                                    onValueChange(value)
                                    expanded = false
                                }
                            )
                        }
                    }
                }
            }
            
            ConfigOptionType.MULTI_SELECTION -> {
                val selectedValues = option.currentValue.split(",").map { it.trim() }.filter { it.isNotEmpty() }
                
                Column(
                    modifier = Modifier.fillMaxWidth()
                ) {
                    Text(
                        text = "Selected: ${selectedValues.joinToString(", ")}",
                        style = MaterialTheme.typography.bodySmall,
                        modifier = Modifier.padding(bottom = 8.dp)
                    )
                    
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.spacedBy(8.dp)
                    ) {
                        option.possibleValues.forEach { value ->
                            val isSelected = value in selectedValues
                            
                            FilterChip(
                                selected = isSelected,
                                onClick = {
                                    val newSelection = if (isSelected) {
                                        selectedValues - value
                                    } else {
                                        selectedValues + value
                                    }
                                    onValueChange(newSelection.joinToString(","))
                                },
                                label = { Text(value) }
                            )
                        }
                    }
                }
            }
            
            ConfigOptionType.COLOR -> {
                var color by remember { mutableStateOf(option.currentValue) }
                
                OutlinedTextField(
                    value = color,
                    onValueChange = { 
                        color = it
                        onValueChange(it)
                    },
                    modifier = Modifier.fillMaxWidth(),
                    singleLine = true,
                    placeholder = { Text("HEX color value (#RRGGBB)") },
                    leadingIcon = {
                        Surface(
                            modifier = Modifier
                                .padding(8.dp)
                                .height(24.dp)
                                .fillMaxWidth(0.1f),
                            color = try {
                                androidx.compose.ui.graphics.Color(
                                    android.graphics.Color.parseColor(color)
                                )
                            } catch (e: Exception) {
                                MaterialTheme.colorScheme.error
                            }
                        ) { }
                    }
                )
            }
        }
    }
} 