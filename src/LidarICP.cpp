/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   LidarICP.cpp
 * @brief  Simple SLAM FrontEnd for point-cloud sensors via ICP registration
 * @author Jose Luis Blanco Claraco
 * @date   Dec 17, 2018
 */

/** \defgroup mola_fe_lidar_icp_grp mola-fe-lidar-icp.
 * Simple SLAM FrontEnd for point-cloud sensors via ICP registration.
 *
 *
 */

#include <mola-fe-lidar-icp/LidarICP.h>
#include <mola-kernel/yaml_helpers.h>
#include <mrpt/core/initializer.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/system/datetime.h>
#include <yaml-cpp/yaml.h>

using namespace mola;

MRPT_INITIALIZER(do_register){MOLA_REGISTER_MODULE(LidarICP)}

LidarICP::LidarICP() = default;

void LidarICP::initialize(const std::string& cfg_block)
{
    MRPT_TRY_START

    // Default params:
    params_.mrpt_icp.maxIterations        = 50;
    params_.mrpt_icp.skip_cov_calculation = false;
    params_.mrpt_icp.thresholdDist        = 1.25;
    params_.mrpt_icp.thresholdAng         = mrpt::DEG2RAD(1.0);
    params_.mrpt_icp.ALFA                 = 0.01;

    // Load:
    auto c   = YAML::Load(cfg_block);
    auto cfg = c["params"];
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << cfg);

    YAML_LOAD_REQ(params_, min_dist_xyz_between_keyframes, double);
    YAML_LOAD_OPT(params_, min_time_between_scans, double);
    YAML_LOAD_OPT(params_, min_icp_goodness, double);
    YAML_LOAD_OPT(params_, decimate_to_point_count, unsigned int);

    YAML_LOAD_OPT(params_, mrpt_icp.maxIterations, unsigned int);
    YAML_LOAD_OPT(params_, mrpt_icp.thresholdDist, double);
    YAML_LOAD_OPT_DEG(params_, mrpt_icp.thresholdAng, double);

    MRPT_TRY_END
}
void LidarICP::spinOnce()
{
    MRPT_TRY_START

    ProfilerEntry tleg(profiler_, "spinOnce");

    //
    MRPT_TRY_END
}

void LidarICP::reset() { state_ = MethodState(); }

void LidarICP::onNewObservation(CObservation::Ptr& o)
{
    MRPT_TRY_START
    ProfilerEntry tleg(profiler_, "onNewObservation");

    // Only process "my" sensor source:
    ASSERT_(o);
    if (o->sensorLabel != raw_sensor_label_) return;

    const auto queued = worker_pool_.pendingTasks();
    if (queued > 1)
    {
        MRPT_LOG_THROTTLE_WARN(
            5.0, "Dropping observation due to worker threads too busy.");
        return;
    }
    profiler_.enter("delay_onNewObs_to_process");

    // Enqueue task:
    worker_pool_.enqueue(&LidarICP::doProcessNewObservation, this, o);

    MRPT_TRY_END
}

// here happens the main stuff:
void LidarICP::doProcessNewObservation(CObservation::Ptr& o)
{
    // All methods that are enqueued into a thread pool should have its own
    // top-level try-catch:
    try
    {
        ASSERT_(o);

        ProfilerEntry tleg(profiler_, "doProcessNewObservation");
        profiler_.leave("delay_onNewObs_to_process");

        // Only process pointclouds that are sufficiently apart in time:
        const auto this_obs_tim = o->timestamp;
        if (state_.last_obs_tim != mrpt::Clock::time_point() &&
            mrpt::system::timeDifference(state_.last_obs_tim, this_obs_tim) <
                params_.min_time_between_scans)
        {
            // Drop observation.
            return;
        }

        profiler_.enter("doProcessNewObservation.obs2pointcloud");

        auto       this_obs_points = mrpt::maps::CSimplePointsMap::Create();
        const bool have_points     = this_obs_points->insertObservationPtr(o);

        profiler_.leave("doProcessNewObservation.obs2pointcloud");

        // Store for next step:
        auto last_obs_tim   = state_.last_obs_tim;
        auto last_obs       = state_.last_obs;
        auto last_points    = state_.last_points;
        state_.last_obs     = o;
        state_.last_obs_tim = this_obs_tim;
        state_.last_points  = this_obs_points;

        // First time we cannot do ICP since we need at least two pointclouds:
        if (!last_points)
        {
            MRPT_LOG_DEBUG("First pointcloud: skipping ICP.");
            return;
        }

        if (!have_points)
        {
            MRPT_LOG_WARN_STREAM(
                "Observation of type `" << o->GetRuntimeClass()->className
                                        << "` could not be converted into a "
                                           "pointcloud. Doing nothing.");
            return;
        }

        // Register point clouds using any of the available ICP algorithms:
        mrpt::poses::CPose3DPDFGaussian initial_guess;
        // Use velocity model for the initial guess:
        double dt = .0;
        if (last_obs_tim != mrpt::Clock::time_point())
            dt = mrpt::system::timeDifference(last_obs_tim, this_obs_tim);
        initial_guess.mean.setFromValues(
            state_.last_iter_twist.vx * dt, state_.last_iter_twist.vy * dt,
            state_.last_iter_twist.vz * dt);
        MRPT_TODO("do omega_xyz part!");

        profiler_.enter("doProcessNewObservation.icp_latest");

        mrpt::slam::CICP::TReturnInfo ret_info;

        // Call ICP:
        if (params_.decimate_to_point_count > 0)
        {
            unsigned decim = static_cast<unsigned>(
                this_obs_points->size() / params_.decimate_to_point_count);
            params_.mrpt_icp.corresponding_points_decimation = decim;
        }

        state_.mrpt_icp.options = params_.mrpt_icp;
        auto rel_pose_pdf       = state_.mrpt_icp.Align3DPDF(
            last_points.get(), this_obs_points.get(), initial_guess,
            nullptr /*running_time*/, &ret_info);

        const mrpt::poses::CPose3D rel_pose = rel_pose_pdf->getMeanVal();

        profiler_.leave("doProcessNewObservation.icp_latest");

        // Update velocity model:
        state_.last_iter_twist.vx = rel_pose.x() / dt;
        state_.last_iter_twist.vy = rel_pose.y() / dt;
        state_.last_iter_twist.vz = rel_pose.z() / dt;
        MRPT_TODO("do omega_xyz part!");

        const double icp_goodness = ret_info.goodness;
        MRPT_LOG_DEBUG_FMT(
            "MRPT ICP: goodness=%.03f iters=%u", ret_info.goodness,
            ret_info.nIterations);

        MRPT_LOG_DEBUG_STREAM("ICP rel_pose=" << rel_pose.asString());
        MRPT_LOG_DEBUG_STREAM(
            "Cur point count="
            << this_obs_points->size()
            << " last point count=" << last_points->size() << " decimation="
            << params_.mrpt_icp.corresponding_points_decimation);
        MRPT_LOG_DEBUG_STREAM(
            "Est.twist=" << state_.last_iter_twist.asString());
        MRPT_LOG_DEBUG_STREAM(
            "Time since last scan=" << mrpt::system::formatTimeInterval(dt));

        // Create a new KF if the distance since the last one is large enough:
        state_.accum_since_last_kf = state_.accum_since_last_kf + rel_pose;
        const double dist_eucl_since_last = state_.accum_since_last_kf.norm();
        MRPT_TODO("Add rotation threshold");

        MRPT_LOG_DEBUG_FMT(
            "Since last KF: dist=%5.03f m", dist_eucl_since_last);

        if (icp_goodness > params_.min_icp_goodness &&
            (state_.last_kf == mola::INVALID_ID ||
             dist_eucl_since_last > params_.min_dist_xyz_between_keyframes))
        {
            // Yes: create new KF
            // 1) New KeyFrame
            BackEndBase::ProposeKF_Input kf;

            kf.timestamp = this_obs_tim;
            {
                mrpt::obs::CSensoryFrame sf;
                sf.push_back(o);
                kf.observations = std::move(sf);
            }

            std::future<BackEndBase::ProposeKF_Output> kf_out_fut;
            kf_out_fut = slam_backend_->onProposeNewKeyFrame(kf);

            // Wait until it's executed:
            auto kf_out = kf_out_fut.get();

            ASSERT_(kf_out.success);
            ASSERT_(kf_out.new_kf_id && kf_out.new_kf_id != mola::INVALID_ID);

            // 2) New SE(3) constraint between consecutive Keyframes:
            MRPT_TODO("Continue here!");

            MRPT_LOG_INFO_STREAM(
                "New KF: ID=" << *kf_out.new_kf_id << " rel_pose="
                              << state_.accum_since_last_kf.asString());

            // Reset accumulators:
            state_.accum_since_last_kf = mrpt::poses::CPose3D();
            state_.last_kf             = kf_out.new_kf_id.value();
        }
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
    }
}
