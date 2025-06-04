package com.eb.obd2.services

import android.content.Context
import android.content.pm.PackageManager
import com.eb.obd2.R
import java.io.InputStream
import java.io.OutputStream
import java.net.Socket

enum class OBDConnectionType {
    TCP,
    COM
}