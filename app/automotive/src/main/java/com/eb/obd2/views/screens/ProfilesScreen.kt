package com.eb.obd2.views.screens

import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Modifier
import androidx.hilt.navigation.compose.hiltViewModel
import com.eb.obd2.models.DrivingProfile
import com.eb.obd2.models.ProfileCategory
import com.eb.obd2.viewmodels.OBDViewModel
import com.eb.obd2.views.components.ProfileManagerScreen

/**
 * Screen for managing driving profiles
 */
@Composable
fun ProfilesScreen(
    viewModel: OBDViewModel = hiltViewModel()
) {
    // Collect state flows
    val localProfiles = viewModel.localProfiles
    val remoteProfiles = viewModel.remoteProfiles
    val filteredProfiles = viewModel.filteredProfiles
    
    Column(
        modifier = Modifier.fillMaxSize()
    ) {
        ProfileManagerScreen(
            localProfiles = localProfiles,
            remoteProfiles = remoteProfiles,
            filteredProfiles = filteredProfiles,
            activeProfile = viewModel.activeProfile,
            selectedCategory = viewModel.selectedCategory,
            isSyncing = viewModel.isSyncingProfiles,
            statusMessage = viewModel.profileStatusMessage,
            onCategorySelected = viewModel::setProfileCategoryFilter,
            onActivateProfile = viewModel::activateProfile,
            onDeleteProfile = viewModel::deleteProfile,
            onImportProfile = viewModel::importProfileFromServer,
            onUploadProfile = viewModel::uploadProfileToServer,
            onSyncWithServer = viewModel::syncProfilesWithServer,
            onCreateNewProfile = viewModel::saveCurrentProfile
        )
    }
} 