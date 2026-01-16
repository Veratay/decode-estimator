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

std::array<gtsam::Point3, 4> computeLandmarkCornersWorld(const decode::Landmark& lm) {
    // Compute local corners (tag frame)
    double s = lm.size / 2.0;
    std::array<gtsam::Point3, 4> corners_local = {
        gtsam::Point3(-s, -s, 0.0),  // TL (Top-Left)
        gtsam::Point3( s, -s, 0.0),  // TR (Top-Right)
        gtsam::Point3( s,  s, 0.0),  // BR (Bottom-Right)
        gtsam::Point3(-s,  s, 0.0)   // BL (Bottom-Left)
    };

    // Create landmark pose in world frame
    gtsam::Pose3 tag_pose_world(
        gtsam::Rot3::Ypr(lm.yaw, lm.pitch, lm.roll),
        gtsam::Point3(lm.x, lm.y, lm.z)
    );

    // Transform corners to world frame
    std::array<gtsam::Point3, 4> corners_world;
    for (size_t i = 0; i < 4; ++i) {
        corners_world[i] = tag_pose_world.transformFrom(corners_local[i]);
    }

    return corners_world;
}

} // namespace

extern "C" JNIEXPORT jlong JNICALL
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeCreate(JNIEnv* env, jclass) {
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
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeCreateWithConfig(
    JNIEnv* env, jclass, jdouble prior_sigma_xy, jdouble prior_sigma_theta,
    jdouble odom_sigma_xy, jdouble odom_sigma_theta, jdouble default_pixel_sigma,
    jdouble relinearize_threshold, jint relinearize_skip, jboolean enable_partial_relinearization,
    jboolean compact_odometry,
    jboolean enable_robust_tag_loss, jint robust_tag_loss, jdouble robust_tag_loss_k,
    jboolean enable_tag_gating, jdouble min_tag_area_px, jdouble max_tag_view_angle_deg,
    jboolean enable_cheirality_check, jdouble cheirality_sigma, jdouble min_tag_z_distance,
    jboolean enable_post_process, jdouble post_process_vision_gap_s,
    jdouble post_process_settle_s, jint post_process_settle_updates,
    jdouble fx, jdouble fy, jdouble cx, jdouble cy,
    jdouble k1, jdouble k2, jdouble k3, jdouble p1, jdouble p2,
    jdouble camera_offset_x, jdouble camera_offset_y, jdouble camera_offset_z,
    jdouble camera_roll, jdouble camera_pitch, jdouble camera_yaw,
    jdouble pixel_sigma_angle_k,
    jboolean enable_spatial_correlation, jdouble correlation_distance_m,
    jdouble correlation_downweight_factor, jint correlation_history_size,
    jboolean enable_bias_correction, jdouble radial_bias_k) {
    try {
        decode::EstimatorConfig config;
        config.prior_sigma_xy = prior_sigma_xy;
        config.prior_sigma_theta = prior_sigma_theta;
        config.odom_sigma_xy = odom_sigma_xy;
        config.odom_sigma_theta = odom_sigma_theta;
        config.default_pixel_sigma = default_pixel_sigma;
        config.relinearize_threshold = relinearize_threshold;
        config.relinearize_skip = static_cast<int>(relinearize_skip);
        config.enable_partial_relinearization = (enable_partial_relinearization == JNI_TRUE);
        config.compact_odometry = (compact_odometry == JNI_TRUE);
        config.enable_robust_tag_loss = (enable_robust_tag_loss == JNI_TRUE);
        config.robust_tag_loss_k = robust_tag_loss_k;
        config.enable_tag_gating = (enable_tag_gating == JNI_TRUE);
        config.min_tag_area_px = min_tag_area_px;
        config.max_tag_view_angle_deg = max_tag_view_angle_deg;
        config.enable_cheirality_check = (enable_cheirality_check == JNI_TRUE);
        config.cheirality_sigma = cheirality_sigma;
        config.min_tag_z_distance = min_tag_z_distance;
        config.enable_post_process = (enable_post_process == JNI_TRUE);
        config.post_process_vision_gap_s = post_process_vision_gap_s;
        config.post_process_settle_s = post_process_settle_s;
        config.post_process_settle_updates = static_cast<int>(post_process_settle_updates);
        config.fx = fx;
        config.fy = fy;
        config.cx = cx;
        config.cy = cy;
        config.k1 = k1;
        config.k2 = k2;
        config.k3 = k3;
        config.p1 = p1;
        config.p2 = p2;
        config.camera_offset_x = camera_offset_x;
        config.camera_offset_y = camera_offset_y;
        config.camera_offset_z = camera_offset_z;
        config.camera_roll = camera_roll;
        config.camera_pitch = camera_pitch;
        config.camera_yaw = camera_yaw;

        // Viewing angle-dependent noise
        config.pixel_sigma_angle_k = pixel_sigma_angle_k;

        // Spatial correlation downweighting
        config.enable_spatial_correlation = (enable_spatial_correlation == JNI_TRUE);
        config.correlation_distance_m = correlation_distance_m;
        config.correlation_downweight_factor = correlation_downweight_factor;
        config.correlation_history_size = static_cast<size_t>(correlation_history_size);

        // Bias correction
        config.enable_bias_correction = (enable_bias_correction == JNI_TRUE);
        config.radial_bias_k = radial_bias_k;

        switch (robust_tag_loss) {
            case 0:
                config.robust_tag_loss = decode::RobustLossType::Huber;
                break;
            case 1:
                config.robust_tag_loss = decode::RobustLossType::Tukey;
                break;
            case 2:
                config.robust_tag_loss = decode::RobustLossType::Cauchy;
                break;
            default:
                config.robust_tag_loss = decode::RobustLossType::Huber;
                break;
        }

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
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeDestroy(JNIEnv* env, jclass, jlong handle) {
    if (handle == 0) {
        throw_illegal_argument(env, "PoseEstimator handle is null");
        return;
    }

    auto* estimator = from_handle(handle);
    delete estimator;
}

extern "C" JNIEXPORT void JNICALL
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeInitialize(
    JNIEnv* env, jclass, jlong handle, jintArray tag_ids, 
    jdoubleArray lm_x, jdoubleArray lm_y, jdoubleArray lm_z,
    jdoubleArray lm_roll, jdoubleArray lm_pitch, jdoubleArray lm_yaw,
    jdoubleArray lm_size,
    jdouble initial_x, jdouble initial_y, jdouble initial_theta) {
    if (handle == 0) {
        throw_illegal_state(env, "PoseEstimator handle is null");
        return;
    }

    if (!tag_ids || !lm_x || !lm_y || !lm_z || !lm_roll || !lm_pitch || !lm_yaw || !lm_size) {
        throw_illegal_argument(env, "Landmark arrays cannot be null");
        return;
    }

    const jsize num_landmarks = env->GetArrayLength(tag_ids);
    // (Optional: Check all lengths match)

    auto* estimator = from_handle(handle);

    jint* ids = env->GetIntArrayElements(tag_ids, nullptr);
    jdouble* x = env->GetDoubleArrayElements(lm_x, nullptr);
    jdouble* y = env->GetDoubleArrayElements(lm_y, nullptr);
    jdouble* z = env->GetDoubleArrayElements(lm_z, nullptr);
    jdouble* r = env->GetDoubleArrayElements(lm_roll, nullptr);
    jdouble* p = env->GetDoubleArrayElements(lm_pitch, nullptr);
    jdouble* yw = env->GetDoubleArrayElements(lm_yaw, nullptr);
    jdouble* s = env->GetDoubleArrayElements(lm_size, nullptr);

    try {
        std::vector<decode::Landmark> landmarks;
        landmarks.reserve(num_landmarks);
        for (jsize i = 0; i < num_landmarks; ++i) {
            decode::Landmark lm;
            lm.id = static_cast<int32_t>(ids[i]);
            lm.x = x[i];
            lm.y = y[i];
            lm.z = z[i];
            lm.roll = r[i];
            lm.pitch = p[i];
            lm.yaw = yw[i];
            lm.size = s[i];
            landmarks.push_back(lm);
        }

        estimator->initialize(landmarks, initial_x, initial_y, initial_theta);

    } catch (const std::exception& e) {
        throw_runtime_exception(env, std::string("Initialize failed: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env, "Initialize failed: unknown error");
    }

    env->ReleaseIntArrayElements(tag_ids, ids, JNI_ABORT);
    env->ReleaseDoubleArrayElements(lm_x, x, JNI_ABORT);
    env->ReleaseDoubleArrayElements(lm_y, y, JNI_ABORT);
    env->ReleaseDoubleArrayElements(lm_z, z, JNI_ABORT);
    env->ReleaseDoubleArrayElements(lm_roll, r, JNI_ABORT);
    env->ReleaseDoubleArrayElements(lm_pitch, p, JNI_ABORT);
    env->ReleaseDoubleArrayElements(lm_yaw, yw, JNI_ABORT);
    env->ReleaseDoubleArrayElements(lm_size, s, JNI_ABORT);
}

extern "C" JNIEXPORT void JNICALL
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeReset(JNIEnv* env, jclass, jlong handle) {
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
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeProcessOdometry(
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
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeAddTagMeasurement(
    JNIEnv* env, jclass, jlong handle, jint tag_id, 
    jdoubleArray corners, // Expecting 8 doubles: u1,v1, u2,v2, u3,v3, u4,v4
    jdouble pixel_sigma, jdouble timestamp, jdouble turret_yaw_rad) {
    
    if (handle == 0) {
        throw_illegal_state(env, "PoseEstimator handle is null");
        return;
    }
    
    if (!corners || env->GetArrayLength(corners) != 8) {
        throw_illegal_argument(env, "Corners array must have 8 elements");
        return;
    }

    auto* estimator = from_handle(handle);
    jdouble* c = env->GetDoubleArrayElements(corners, nullptr);
    
    try {
        decode::TagMeasurement tag;
        tag.tag_id = static_cast<int32_t>(tag_id);
        tag.timestamp = timestamp;
        tag.pixel_sigma = pixel_sigma;
        tag.turret_yaw_rad = turret_yaw_rad;
        
        for(int i=0; i<4; ++i) {
            tag.corners.emplace_back(c[2*i], c[2*i+1]);
        }
        
        estimator->addTagMeasurement(tag);
    } catch (const std::exception& e) {
        throw_runtime_exception(env,
                                std::string("Add tag measurement failed: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env, "Add tag measurement failed: unknown error");
    }
    
    env->ReleaseDoubleArrayElements(corners, c, JNI_ABORT);
}

extern "C" JNIEXPORT void JNICALL Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeUpdate(
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
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeGetCurrentEstimate(JNIEnv* env,
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
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeGetCurrentEstimateWithCovariance(
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
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeIsInitialized(JNIEnv* env,
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
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeGetLastSolveTimeMs(JNIEnv* env,
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
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeGetAverageSolveTimeMs(JNIEnv* env,
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

extern "C" JNIEXPORT jlongArray JNICALL
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeGetLastMemoryUsage(JNIEnv* env,
                                                                        jclass,
                                                                        jlong handle) {
    if (handle == 0) {
        throw_illegal_state(env, "PoseEstimator handle is null");
        return nullptr;
    }

    auto* estimator = from_handle(handle);
    try {
        const decode::MemoryUsage usage = estimator->getLastMemoryUsage();

        // Return [virtual_bytes, resident_bytes, shared_bytes, data_bytes, valid]
        jlongArray result = env->NewLongArray(5);
        if (!result) {
            throw_runtime_exception(env, "Failed to allocate memory usage array");
            return nullptr;
        }

        jlong values[5] = {
            static_cast<jlong>(usage.virtual_bytes),
            static_cast<jlong>(usage.resident_bytes),
            static_cast<jlong>(usage.shared_bytes),
            static_cast<jlong>(usage.data_bytes),
            static_cast<jlong>(usage.valid ? 1 : 0)
        };
        env->SetLongArrayRegion(result, 0, 5, values);
        return result;
    } catch (const std::exception& e) {
        throw_runtime_exception(env, std::string("Get memory usage failed: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env, "Get memory usage failed: unknown error");
    }
    return nullptr;
}

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeGetDiagnosticsSnapshot(JNIEnv* env,
                                                                            jclass,
                                                                            jlong handle) {
    if (handle == 0) {
        throw_illegal_state(env, "PoseEstimator handle is null");
        return nullptr;
    }

    auto* estimator = from_handle(handle);
    try {
        const decode::DiagnosticsSnapshot snapshot = estimator->getDiagnosticsSnapshot();

        // Return [pending_tags, pending_graph_factors, pending_values, current_pose_index,
        //         horizon_capacity, last_solve_ms, avg_solve_ms, last_horizon_reset_ms,
        //         last_horizon_cov_ms, last_horizon_reset_pose_index]
        jdoubleArray result = env->NewDoubleArray(10);
        if (!result) {
            throw_runtime_exception(env, "Failed to allocate diagnostics array");
            return nullptr;
        }

        jdouble values[10] = {
            static_cast<jdouble>(snapshot.pending_tags),
            static_cast<jdouble>(snapshot.pending_graph_factors),
            static_cast<jdouble>(snapshot.pending_values),
            static_cast<jdouble>(snapshot.current_pose_index),
            static_cast<jdouble>(snapshot.horizon_capacity),
            snapshot.last_solve_ms,
            snapshot.avg_solve_ms,
            snapshot.last_horizon_reset_ms,
            snapshot.last_horizon_cov_ms,
            static_cast<jdouble>(snapshot.last_horizon_reset_pose_index)
        };
        env->SetDoubleArrayRegion(result, 0, 10, values);
        return result;
    } catch (const std::exception& e) {
        throw_runtime_exception(env, std::string("Get diagnostics failed: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env, "Get diagnostics failed: unknown error");
    }
    return nullptr;
}

extern "C" JNIEXPORT void JNICALL
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeEnableVisualization(
    JNIEnv* env, jclass, jlong handle, jboolean enabled) {
    if (handle == 0) return;
    auto* estimator = from_handle(handle);
    estimator->enableVisualization(enabled == JNI_TRUE);
}

extern "C" JNIEXPORT void JNICALL
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeConfigureVisualization(
    JNIEnv* env, jclass, jlong handle, jboolean stream, jstring urlOrPath, jstring appId) {
    if (handle == 0) return;
    
    const char* url_path_c = env->GetStringUTFChars(urlOrPath, nullptr);
    const char* app_id_c = env->GetStringUTFChars(appId, nullptr);
    
    auto* estimator = from_handle(handle);
    estimator->configureVisualization(stream == JNI_TRUE, std::string(url_path_c), std::string(app_id_c));
    
    env->ReleaseStringUTFChars(urlOrPath, url_path_c);
    env->ReleaseStringUTFChars(appId, app_id_c);
}

extern "C" JNIEXPORT void JNICALL
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeFlushVisualization(
    JNIEnv* env, jclass, jlong handle) {
    if (handle == 0) return;
    auto* estimator = from_handle(handle);
    estimator->flushVisualization();
}

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeGetPredictedCorners(
    JNIEnv* env, jclass, jlong handle, jint tagId) {
    if (handle == 0) return nullptr;
    auto* estimator = from_handle(handle);
    
    std::vector<std::pair<double, double>> corners = estimator->getPredictedCorners(tagId);
    if (corners.empty()) return nullptr;
    
    jdoubleArray result = env->NewDoubleArray(corners.size() * 2);
    jdouble* buf = env->GetDoubleArrayElements(result, nullptr);
    
    for(size_t i=0; i<corners.size(); ++i) {
        buf[2*i] = corners[i].first;
        buf[2*i+1] = corners[i].second;
    }
    
    env->ReleaseDoubleArrayElements(result, buf, 0);
    return result;
}

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeGetAllLandmarkCorners(JNIEnv* env,
                                                                            jclass,
                                                                            jlong handle) {
    if (handle == 0) {
        throw_illegal_state(env, "PoseEstimator handle is null");
        return nullptr;
    }

    auto* estimator = from_handle(handle);
    try {
        if (!estimator->isInitialized()) {
            throw_illegal_state(env, "PoseEstimator not initialized");
            return nullptr;
        }

        const auto& landmark_map = estimator->getLandmarkMap();
        const auto& all_landmarks = landmark_map.getAllLandmarks();

        // Handle empty landmark map
        if (all_landmarks.empty()) {
            return env->NewDoubleArray(0);
        }

        // Allocate result array: num_landmarks * 13 (1 tag_id + 4 corners * 3 coords)
        jdoubleArray result = env->NewDoubleArray(all_landmarks.size() * 13);
        if (!result) {
            throw_runtime_exception(env, "Failed to allocate landmark corners array");
            return nullptr;
        }

        // Compute corners for each landmark and fill array
        std::vector<jdouble> values;
        values.reserve(all_landmarks.size() * 13);

        for (const auto& [tag_id, lm] : all_landmarks) {
            // Add tag ID as double
            values.push_back(static_cast<jdouble>(tag_id));

            // Compute corners using helper function
            auto corners_world = computeLandmarkCornersWorld(lm);

            // Add 4 corners (12 values: x,y,z for each corner)
            for (const auto& corner : corners_world) {
                values.push_back(corner.x());
                values.push_back(corner.y());
                values.push_back(corner.z());
            }
        }

        env->SetDoubleArrayRegion(result, 0, values.size(), values.data());
        return result;
    } catch (const std::exception& e) {
        throw_runtime_exception(env,
                                std::string("Get all landmark corners failed: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env, "Get all landmark corners failed: unknown error");
    }
    return nullptr;
}

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeGetLandmarkCorners(JNIEnv* env,
                                                                         jclass,
                                                                         jlong handle,
                                                                         jint tag_id) {
    if (handle == 0) {
        throw_illegal_state(env, "PoseEstimator handle is null");
        return nullptr;
    }

    auto* estimator = from_handle(handle);
    try {
        if (!estimator->isInitialized()) {
            throw_illegal_state(env, "PoseEstimator not initialized");
            return nullptr;
        }

        const auto& landmark_map = estimator->getLandmarkMap();
        auto landmark_opt = landmark_map.getLandmark(static_cast<int32_t>(tag_id));

        if (!landmark_opt) {
            throw_illegal_argument(
                env, std::string("Landmark with ID ") + std::to_string(tag_id) + " not found");
            return nullptr;
        }

        // Allocate result array: 12 values (4 corners * 3 coords)
        jdoubleArray result = env->NewDoubleArray(12);
        if (!result) {
            throw_runtime_exception(env, "Failed to allocate landmark corners array");
            return nullptr;
        }

        // Compute corners using helper function
        auto corners_world = computeLandmarkCornersWorld(*landmark_opt);

        // Fill array with corner coordinates
        jdouble values[12];
        size_t idx = 0;
        for (const auto& corner : corners_world) {
            values[idx++] = corner.x();
            values[idx++] = corner.y();
            values[idx++] = corner.z();
        }

        env->SetDoubleArrayRegion(result, 0, 12, values);
        return result;
    } catch (const std::exception& e) {
        throw_runtime_exception(env,
                                std::string("Get landmark corners failed: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env, "Get landmark corners failed: unknown error");
    }
    return nullptr;
}

extern "C" JNIEXPORT jdoubleArray JNICALL
Java_sigmacorns_control_aim_PoseEstimatorBridge_nativeGetCameraUnitVectors(JNIEnv* env,
                                                                           jclass,
                                                                           jlong handle) {
    if (handle == 0) {
        throw_illegal_state(env, "PoseEstimator handle is null");
        return nullptr;
    }

    auto* estimator = from_handle(handle);
    try {
        if (!estimator->isInitialized()) {
            throw_illegal_state(env, "PoseEstimator not initialized");
            return nullptr;
        }

        // Get current camera pose in world frame
        gtsam::Pose3 camera_pose = estimator->getCurrentCameraPose();

        // Extract position
        gtsam::Point3 position = camera_pose.translation();

        // Extract rotation matrix (3x3)
        gtsam::Matrix3 rotation = camera_pose.rotation().matrix();

        // Allocate result array: 12 doubles
        jdoubleArray result = env->NewDoubleArray(12);
        if (!result) {
            throw_runtime_exception(env, "Failed to allocate camera vectors array");
            return nullptr;
        }

        // Fill array: position + right + up + forward
        jdouble values[12] = {
            // Position (x, y, z)
            position.x(), position.y(), position.z(),
            // Right vector (column 0)
            rotation(0, 0), rotation(1, 0), rotation(2, 0),
            // Up vector (column 1)
            rotation(0, 1), rotation(1, 1), rotation(2, 1),
            // Forward vector (column 2)
            rotation(0, 2), rotation(1, 2), rotation(2, 2)
        };

        env->SetDoubleArrayRegion(result, 0, 12, values);
        return result;
    } catch (const std::exception& e) {
        throw_runtime_exception(env,
                                std::string("Get camera unit vectors failed: ") + e.what());
    } catch (...) {
        throw_runtime_exception(env, "Get camera unit vectors failed: unknown error");
    }
    return nullptr;
}
