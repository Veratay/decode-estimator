#include <jni.h>

#include "decode_estimator/pose_estimator.hpp"

#include <exception>
#include <string>
#include <vector>

namespace {

void throw_exception(JNIEnv* env, const char* class_name, const std::string& message) {
    if (env->ExceptionCheck())
        return;
    jclass exception_class = env->FindClass(class_name);
    if (!exception_class)
        return;
    env->ThrowNew(exception_class, message.c_str());
    env->DeleteLocalRef(exception_class);
}

void throw_illegal_argument(JNIEnv* env, const std::string& message) {
    throw_exception(env, "java/lang/IllegalArgumentException", message);
}

void throw_illegal_state(JNIEnv* env, const std::string& message) {
    throw_exception(env, "java/lang/IllegalStateException", message);
}

void throw_runtime_exception(JNIEnv* env, const std::string& message) {
    throw_exception(env, "java/lang/RuntimeException", message);
}

decode::PoseEstimator* from_handle(jlong handle) {
    return reinterpret_cast<decode::PoseEstimator*>(handle);
}

} // namespace

extern "C" JNIEXPORT jlong JNICALL
Java_com_decode_estimator_PoseEstimatorBridge_nativeCreate(JNIEnv* env, jclass) {
    try {
        auto* estimator = new decode::PoseEstimator();
        return reinterpret_cast<jlong>(estimator);
    } catch (const std::exception& e) {
        throw_runtime_exception(
            env, std::string("Failed to create PoseEstimator: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env, "Failed to create PoseEstimator: unknown error");
    }
    return 0;
}

extern "C" JNIEXPORT jlong JNICALL
Java_com_decode_estimator_PoseEstimatorBridge_nativeCreateWithConfig(
    JNIEnv* env, jclass, jdouble prior_sigma_xy, jdouble prior_sigma_theta,
    jdouble odom_sigma_xy, jdouble odom_sigma_theta, jdouble default_bearing_sigma,
    jdouble default_distance_sigma) {
    try {
        decode::EstimatorConfig config;
        config.prior_sigma_xy = prior_sigma_xy;
        config.prior_sigma_theta = prior_sigma_theta;
        config.odom_sigma_xy = odom_sigma_xy;
        config.odom_sigma_theta = odom_sigma_theta;
        config.default_bearing_sigma = default_bearing_sigma;
        config.default_distance_sigma = default_distance_sigma;

        auto* estimator = new decode::PoseEstimator(config);
        return reinterpret_cast<jlong>(estimator);
    } catch (const std::exception& e) {
        throw_runtime_exception(
            env, std::string("Failed to create PoseEstimator with config: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env, "Failed to create PoseEstimator with config: unknown error");
    }
    return 0;
}

extern "C" JNIEXPORT void JNICALL
Java_com_decode_estimator_PoseEstimatorBridge_nativeDestroy(JNIEnv* env, jclass, jlong handle) {
    if (handle == 0) {
        throw_illegal_argument(env, "PoseEstimator handle is null");
        return;
    }

    auto* estimator = from_handle(handle);
    delete estimator;
}

extern "C" JNIEXPORT void JNICALL
Java_com_decode_estimator_PoseEstimatorBridge_nativeInitialize(
    JNIEnv* env, jclass, jlong handle, jintArray tag_ids, jdoubleArray landmark_x,
    jdoubleArray landmark_y, jdouble initial_x, jdouble initial_y, jdouble initial_theta) {
    if (handle == 0) {
        throw_illegal_state(env, "PoseEstimator handle is null");
        return;
    }

    if (!tag_ids || !landmark_x || !landmark_y) {
        throw_illegal_argument(env, "Landmark arrays cannot be null");
        return;
    }

    const jsize num_landmarks = env->GetArrayLength(tag_ids);
    if (env->GetArrayLength(landmark_x) != num_landmarks ||
        env->GetArrayLength(landmark_y) != num_landmarks) {
        throw_illegal_argument(env, "Landmark array length mismatch");
        return;
    }

    auto* estimator = from_handle(handle);

    jint* ids = env->GetIntArrayElements(tag_ids, nullptr);
    jdouble* x = env->GetDoubleArrayElements(landmark_x, nullptr);
    jdouble* y = env->GetDoubleArrayElements(landmark_y, nullptr);

    if (!ids || !x || !y) {
        if (ids)
            env->ReleaseIntArrayElements(tag_ids, ids, JNI_ABORT);
        if (x)
            env->ReleaseDoubleArrayElements(landmark_x, x, JNI_ABORT);
        if (y)
            env->ReleaseDoubleArrayElements(landmark_y, y, JNI_ABORT);
        throw_runtime_exception(env, "Failed to access landmark array elements");
        return;
    }

    try {
        std::vector<decode::Landmark> landmarks;
        landmarks.reserve(num_landmarks);
        for (jsize i = 0; i < num_landmarks; ++i) {
            landmarks.push_back({static_cast<int32_t>(ids[i]), x[i], y[i]});
        }

        estimator->initialize(landmarks, initial_x, initial_y, initial_theta);

    } catch (const std::exception& e) {
        throw_runtime_exception(env, std::string("Initialize failed: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env, "Initialize failed: unknown error");
    }

    env->ReleaseIntArrayElements(tag_ids, ids, JNI_ABORT);
    env->ReleaseDoubleArrayElements(landmark_x, x, JNI_ABORT);
    env->ReleaseDoubleArrayElements(landmark_y, y, JNI_ABORT);
}

extern "C" JNIEXPORT void JNICALL
Java_com_decode_estimator_PoseEstimatorBridge_nativeReset(JNIEnv* env, jclass, jlong handle) {
    if (handle == 0) {
        throw_illegal_state(env, "PoseEstimator handle is null");
        return;
    }

    auto* estimator = from_handle(handle);
    try {
        estimator->reset();
    } catch (const std::exception& e) {
        throw_runtime_exception(env, std::string("Reset failed: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env, "Reset failed: unknown error");
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_decode_estimator_PoseEstimatorBridge_nativeProcessOdometry(
    JNIEnv* env, jclass, jlong handle, jdouble dx, jdouble dy, jdouble dtheta,
    jdouble timestamp) {
    if (handle == 0) {
        throw_illegal_state(env, "PoseEstimator handle is null");
        return;
    }

    auto* estimator = from_handle(handle);
    try {
        decode::OdometryMeasurement odom{dx, dy, dtheta, timestamp};
        estimator->processOdometry(odom);
    } catch (const std::exception& e) {
        throw_runtime_exception(env, std::string("Process odometry failed: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env, "Process odometry failed: unknown error");
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_decode_estimator_PoseEstimatorBridge_nativeAddBearingMeasurement(
    JNIEnv* env, jclass, jlong handle, jint tag_id, jdouble bearing_rad,
    jdouble uncertainty_rad, jdouble timestamp) {
    if (handle == 0) {
        throw_illegal_state(env, "PoseEstimator handle is null");
        return;
    }

    auto* estimator = from_handle(handle);
    try {
        decode::BearingMeasurement bearing{
            static_cast<int32_t>(tag_id), bearing_rad, uncertainty_rad, timestamp};
        estimator->addBearingMeasurement(bearing);
    } catch (const std::exception& e) {
        throw_runtime_exception(env,
                                std::string("Add bearing measurement failed: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env, "Add bearing measurement failed: unknown error");
    }
}

extern "C" JNIEXPORT void JNICALL
Java_com_decode_estimator_PoseEstimatorBridge_nativeAddDistanceMeasurement(
    JNIEnv* env, jclass, jlong handle, jint tag_id, jdouble distance_m,
    jdouble uncertainty_m, jdouble timestamp) {
    if (handle == 0) {
        throw_illegal_state(env, "PoseEstimator handle is null");
        return;
    }

    auto* estimator = from_handle(handle);
    try {
        decode::DistanceMeasurement distance{
            static_cast<int32_t>(tag_id), distance_m, uncertainty_m, timestamp};
        estimator->addDistanceMeasurement(distance);
    } catch (const std::exception& e) {
        throw_runtime_exception(env,
                                std::string("Add distance measurement failed: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env, "Add distance measurement failed: unknown error");
    }
}

extern "C" JNIEXPORT void JNICALL Java_com_decode_estimator_PoseEstimatorBridge_nativeUpdate(
    JNIEnv* env, jclass, jlong handle) {
    if (handle == 0) {
        throw_illegal_state(env, "PoseEstimator handle is null");
        return;
    }

    auto* estimator = from_handle(handle);
    try {
        estimator->update();
    } catch (const std::exception& e) {
        throw_runtime_exception(env, std::string("Update failed: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env, "Update failed: unknown error");
    }
}

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_com_decode_estimator_PoseEstimatorBridge_nativeGetCurrentEstimate(JNIEnv* env,
                                                                        jclass,
                                                                        jlong handle) {
    if (handle == 0) {
        throw_illegal_state(env, "PoseEstimator handle is null");
        return nullptr;
    }

    auto* estimator = from_handle(handle);
    try {
        const auto estimate = estimator->getCurrentEstimate();

        // Return [x, y, theta, timestamp]
        jdoubleArray result = env->NewDoubleArray(4);
        if (!result) {
            throw_runtime_exception(env, "Failed to allocate result array");
            return nullptr;
        }

        jdouble values[4] = {estimate.x, estimate.y, estimate.theta, estimate.timestamp};
        env->SetDoubleArrayRegion(result, 0, 4, values);

        return result;
    } catch (const std::exception& e) {
        throw_runtime_exception(env, std::string("Get current estimate failed: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env, "Get current estimate failed: unknown error");
    }
    return nullptr;
}

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_com_decode_estimator_PoseEstimatorBridge_nativeGetCurrentEstimateWithCovariance(
    JNIEnv* env, jclass, jlong handle) {
    if (handle == 0) {
        throw_illegal_state(env, "PoseEstimator handle is null");
        return nullptr;
    }

    auto* estimator = from_handle(handle);
    try {
        const auto estimate = estimator->getCurrentEstimateWithCovariance();

        // Return [x, y, theta, timestamp, cov[0], cov[1], ..., cov[8]]
        jdoubleArray result = env->NewDoubleArray(13);
        if (!result) {
            throw_runtime_exception(env, "Failed to allocate result array");
            return nullptr;
        }

        jdouble values[13];
        values[0] = estimate.x;
        values[1] = estimate.y;
        values[2] = estimate.theta;
        values[3] = estimate.timestamp;
        for (size_t i = 0; i < 9; ++i) {
            values[4 + i] = estimate.covariance[i];
        }

        env->SetDoubleArrayRegion(result, 0, 13, values);

        return result;
    } catch (const std::exception& e) {
        throw_runtime_exception(
            env, std::string("Get current estimate with covariance failed: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env,
                                "Get current estimate with covariance failed: unknown error");
    }
    return nullptr;
}

extern "C" JNIEXPORT jboolean JNICALL
Java_com_decode_estimator_PoseEstimatorBridge_nativeIsInitialized(JNIEnv* env,
                                                                   jclass,
                                                                   jlong handle) {
    if (handle == 0) {
        throw_illegal_state(env, "PoseEstimator handle is null");
        return JNI_FALSE;
    }

    auto* estimator = from_handle(handle);
    try {
        return estimator->isInitialized() ? JNI_TRUE : JNI_FALSE;
    } catch (const std::exception& e) {
        throw_runtime_exception(env, std::string("Is initialized failed: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env, "Is initialized failed: unknown error");
    }
    return JNI_FALSE;
}

extern "C" JNIEXPORT jdouble JNICALL
Java_com_decode_estimator_PoseEstimatorBridge_nativeGetLastSolveTimeMs(JNIEnv* env,
                                                                        jclass,
                                                                        jlong handle) {
    if (handle == 0) {
        throw_illegal_state(env, "PoseEstimator handle is null");
        return 0.0;
    }

    auto* estimator = from_handle(handle);
    try {
        return static_cast<jdouble>(estimator->getLastSolveTimeMs());
    } catch (const std::exception& e) {
        throw_runtime_exception(env, std::string("Get last solve time failed: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env, "Get last solve time failed: unknown error");
    }
    return 0.0;
}

extern "C" JNIEXPORT jdouble JNICALL
Java_com_decode_estimator_PoseEstimatorBridge_nativeGetAverageSolveTimeMs(JNIEnv* env,
                                                                           jclass,
                                                                           jlong handle) {
    if (handle == 0) {
        throw_illegal_state(env, "PoseEstimator handle is null");
        return 0.0;
    }

    auto* estimator = from_handle(handle);
    try {
        return static_cast<jdouble>(estimator->getAverageSolveTimeMs());
    } catch (const std::exception& e) {
        throw_runtime_exception(env, std::string("Get average solve time failed: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env, "Get average solve time failed: unknown error");
    }
    return 0.0;
}
