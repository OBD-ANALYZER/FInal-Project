package com.eb.obd2.modules

import android.content.Context
import com.eb.obd2.repositories.RecordRepository
import com.eb.obd2.services.OBDConnection
import com.eb.obd2.services.OBDService
import com.eb.obd2.services.TCPOBDConnection
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.android.qualifiers.ApplicationContext
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
object OBDServiceModule {

    @Provides
    @Singleton
    fun provideTCPOBDConnection(): OBDConnection {
        return TCPOBDConnection()
    }

    @Provides
    @Singleton
    fun provideOBDService(
        connection: OBDConnection,
        repository: RecordRepository
    ): OBDService {
        return OBDService(connection, repository)
    }
}