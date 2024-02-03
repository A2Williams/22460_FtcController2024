plugins {
    id("com.android.application")
}

android {
    namespace = "com.example.robotautodrivetoapriltagtankjava"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.example.robotautodrivetoapriltagtankjava"
        minSdk = 25
        targetSdk = 34
        versionCode = 1
        versionName = "1.0"

    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(getDefaultProguardFile("proguard-android-optimize.txt"), "proguard-rules.pro")
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }
}

dependencies {

    implementation("com.google.android.gms:play-services-wearable:18.0.0")
}