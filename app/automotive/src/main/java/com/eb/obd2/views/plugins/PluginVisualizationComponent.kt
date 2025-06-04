package com.eb.obd2.views.plugins

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.Card
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.Divider
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import android.view.ViewGroup
import android.widget.FrameLayout
import com.eb.obd2.models.Plugin
import com.eb.obd2.services.PluginService
import com.eb.obd2.viewmodels.PluginViewModel

/**
 * Component that displays a plugin visualization
 * 
 * @param plugin The plugin to display visualization for
 * @param visualizationName The name of the visualization to display
 * @param pluginService The plugin service
 * @param onConfigureClick Callback when the configure button is clicked
 */
@Composable
fun PluginVisualizationComponent(
    plugin: Plugin,
    visualizationName: String,
    pluginService: PluginService,
    onConfigureClick: () -> Unit = {},
    modifier: Modifier = Modifier
) {
    val context = LocalContext.current
    var error by remember { mutableStateOf<String?>(null) }
    var isLoading by remember { mutableStateOf(true) }
    
    Card(
        modifier = modifier
            .fillMaxWidth()
            .padding(8.dp)
    ) {
        Column(
            modifier = Modifier.padding(16.dp)
        ) {
            // Header
            Row(
                modifier = Modifier.fillMaxWidth(),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = visualizationName,
                    style = MaterialTheme.typography.titleMedium,
                    fontWeight = FontWeight.Bold,
                    maxLines = 1,
                    overflow = TextOverflow.Ellipsis,
                    modifier = Modifier.weight(1f)
                )
                
                Spacer(modifier = Modifier.width(8.dp))
                
                Text(
                    text = plugin.name,
                    style = MaterialTheme.typography.bodySmall,
                    color = MaterialTheme.colorScheme.onSurfaceVariant,
                    maxLines = 1,
                    overflow = TextOverflow.Ellipsis
                )
                
                IconButton(
                    onClick = onConfigureClick,
                    modifier = Modifier.size(32.dp)
                ) {
                    Icon(
                        imageVector = Icons.Default.Settings,
                        contentDescription = "Configure",
                        tint = MaterialTheme.colorScheme.primary
                    )
                }
            }
            
            Divider(modifier = Modifier.padding(vertical = 8.dp))
            
            // Visualization content
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .height(200.dp)
                    .border(1.dp, MaterialTheme.colorScheme.outlineVariant, MaterialTheme.shapes.medium)
                    .background(MaterialTheme.colorScheme.surfaceVariant, MaterialTheme.shapes.medium),
                contentAlignment = Alignment.Center
            ) {
                if (isLoading) {
                    CircularProgressIndicator(
                        modifier = Modifier.size(48.dp),
                        color = MaterialTheme.colorScheme.primary
                    )
                } else if (error != null) {
                    Text(
                        text = error ?: "An error occurred",
                        style = MaterialTheme.typography.bodyMedium,
                        color = MaterialTheme.colorScheme.error,
                        textAlign = TextAlign.Center,
                        modifier = Modifier.padding(16.dp)
                    )
                } else {
                    // Get visualization class name
                    val pluginLoader = pluginService.getPluginLoader(plugin.id)
                    val visualizations = pluginLoader?.getVisualizations() ?: emptyMap()
                    val visualizationClassName = visualizations[visualizationName]
                    
                    if (visualizationClassName != null) {
                        // Display the plugin visualization view
                        PluginVisualizationView(
                            plugin = plugin,
                            visualizationClassName = visualizationClassName,
                            pluginService = pluginService,
                            onError = { error = it },
                            onLoading = { isLoading = it }
                        )
                    } else {
                        Text(
                            text = "Visualization not found: $visualizationName",
                            style = MaterialTheme.typography.bodyMedium,
                            color = MaterialTheme.colorScheme.error,
                            textAlign = TextAlign.Center,
                            modifier = Modifier.padding(16.dp)
                        )
                    }
                }
            }
        }
    }
}

/**
 * Android view wrapper for plugin visualizations
 */
@Composable
fun PluginVisualizationView(
    plugin: Plugin,
    visualizationClassName: String,
    pluginService: PluginService,
    onError: (String) -> Unit,
    onLoading: (Boolean) -> Unit
) {
    val context = LocalContext.current
    
    // Create a container for the visualization
    val container = remember { FrameLayout(context) }
    
    DisposableEffect(plugin.id, visualizationClassName) {
        onLoading(true)
        
        try {
            // Get the plugin class loader
            val pluginLoader = pluginService.getPluginLoader(plugin.id)
            
            if (pluginLoader == null) {
                onError("Plugin not loaded")
                onLoading(false)
                return@DisposableEffect onDispose {}
            }
            
            // Load the visualization class
            val visualizationClass = Class.forName(
                visualizationClassName,
                true,
                pluginLoader.javaClass.classLoader
            )
            
            // Create an instance of the visualization
            val visualization = visualizationClass.newInstance()
            
            // Check if it's a View
            if (visualization !is android.view.View) {
                onError("Visualization is not a View")
                onLoading(false)
                return@DisposableEffect onDispose {}
            }
            
            // Add the view to the container
            container.removeAllViews()
            container.addView(
                visualization,
                ViewGroup.LayoutParams(
                    ViewGroup.LayoutParams.MATCH_PARENT,
                    ViewGroup.LayoutParams.MATCH_PARENT
                )
            )
            
            onLoading(false)
            
            onDispose {
                // Cleanup
                container.removeAllViews()
                
                // Call cleanup method if it exists
                try {
                    val cleanupMethod = visualizationClass.getMethod("cleanup")
                    cleanupMethod.invoke(visualization)
                } catch (e: Exception) {
                    // Ignore if cleanup method doesn't exist
                }
            }
        } catch (e: Exception) {
            onError("Error loading visualization: ${e.message}")
            onLoading(false)
            onDispose {}
        }
    }
    
    // Render the container
    AndroidView(
        factory = { container },
        modifier = Modifier.fillMaxWidth()
    )
}

/**
 * Dashboard grid of plugin visualizations
 * 
 * @param plugins List of plugins to display visualizations for
 * @param pluginService The plugin service
 * @param onConfigurePlugin Callback when a plugin needs to be configured
 */
@Composable
fun PluginVisualizationDashboard(
    plugins: List<Plugin>,
    pluginService: PluginService,
    onConfigurePlugin: (Plugin) -> Unit = {},
    modifier: Modifier = Modifier
) {
    val visualizationMap = remember(plugins) {
        plugins.mapNotNull { plugin ->
            val loader = pluginService.getPluginLoader(plugin.id) ?: return@mapNotNull null
            val visualizations = loader.getVisualizations()
            if (visualizations.isEmpty()) return@mapNotNull null
            plugin to visualizations
        }
    }
    
    Column(modifier = modifier.fillMaxWidth()) {
        visualizationMap.forEach { (plugin, visualizations) ->
            visualizations.keys.forEach { visualizationName ->
                PluginVisualizationComponent(
                    plugin = plugin,
                    visualizationName = visualizationName,
                    pluginService = pluginService,
                    onConfigureClick = { onConfigurePlugin(plugin) }
                )
            }
        }
        
        if (visualizationMap.isEmpty()) {
            Box(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(32.dp),
                contentAlignment = Alignment.Center
            ) {
                Text(
                    text = "No visualizations available. Install and enable plugins with visualization capabilities.",
                    style = MaterialTheme.typography.bodyMedium,
                    textAlign = TextAlign.Center,
                    color = MaterialTheme.colorScheme.onSurfaceVariant
                )
            }
        }
    }
} 