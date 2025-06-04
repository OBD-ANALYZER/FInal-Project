package com.eb.obd2.modules

import android.content.Context
import androidx.room.Room
import com.eb.obd2.repositories.source.persistent.RecordDao
import com.eb.obd2.repositories.source.persistent.SpeedDao
import com.eb.obd2.services.DatabaseService
import com.eb.obd2.services.FirebaseService
import dagger.Module
import dagger.Provides
import dagger.hilt.InstallIn
import dagger.hilt.android.qualifiers.ApplicationContext
import dagger.hilt.components.SingletonComponent
import javax.inject.Singleton

@Module
@InstallIn(SingletonComponent::class)
object DatabaseServiceModule {
    @Provides
    @Singleton
    fun provideFirebaseService(): FirebaseService {
        return FirebaseService()
    }

    @Provides
    @Singleton
    fun provideDatabase(@ApplicationContext context: Context): DatabaseService {
        return Room.databaseBuilder(
            context,
            DatabaseService::class.java,
            "database"
        ).build()
    }

    @Provides
    fun provideRecordDao(database: DatabaseService): RecordDao {
        return database.RecordDao()
    }

    @Provides
    fun provideSpeedDao(database: DatabaseService): SpeedDao {
        return database.SpeedDao()
    }
}