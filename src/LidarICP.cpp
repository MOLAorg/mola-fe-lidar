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
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/core/initializer.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/random.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <yaml-cpp/yaml.h>

using namespace mola;

MRPT_INITIALIZER(do_register){MOLA_REGISTER_MODULE(LidarICP)}

LidarICP::LidarICP() = default;

static void load_icp_set_of_params(
    std::vector<MultiCloudICP::Parameters>& out, YAML::Node& cfg,
    const std::string& prefix)
{
    using namespace std::string_literals;

    std::string sName = prefix + "maxIterations"s;
    ASSERTMSG_(cfg[sName], "Missing YAML required entry: `"s + sName + "`"s);
    std::string maxIterations = cfg[sName].as<std::string>("");

    sName = prefix + "thresholdDist"s;
    ASSERTMSG_(cfg[sName], "Missing YAML required entry: `"s + sName + "`"s);
    std::string thresholdDists = cfg[sName].as<std::string>("");

    sName = prefix + "thresholdAng"s;
    ASSERTMSG_(cfg[sName], "Missing YAML required entry: `"s + sName + "`"s);
    std::string thresholdAngs = cfg[sName].as<std::string>("");

    // Vector -> values:
    std::vector<std::string> maxIterations_vals;
    mrpt::system::tokenize(maxIterations, " ,", maxIterations_vals);

    std::vector<std::string> thresholdDists_vals;
    mrpt::system::tokenize(thresholdDists, " ,", thresholdDists_vals);

    std::vector<std::string> thresholdAngs_vals;
    mrpt::system::tokenize(thresholdAngs, " ,", thresholdAngs_vals);

    ASSERT_(maxIterations_vals.size() >= 1);
    ASSERT_(maxIterations_vals.size() == thresholdDists_vals.size());
    ASSERT_(maxIterations_vals.size() == thresholdAngs_vals.size());

    const size_t n = thresholdAngs_vals.size();
    out.resize(n);
    for (size_t i = 0; i < n; i++)
    {
        out[i].maxIterations = std::stoul(maxIterations_vals[i]);
        out[i].thresholdDist = std::stod(thresholdDists_vals[i]);
        out[i].thresholdAng  = mrpt::DEG2RAD(std::stod(thresholdAngs_vals[i]));
    }
}

void LidarICP::initialize(const std::string& cfg_block)
{
    MRPT_TRY_START

    // Load:
    auto c   = YAML::Load(cfg_block);
    auto cfg = c["params"];
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << cfg);

    YAML_LOAD_REQ(params_, min_dist_xyz_between_keyframes, double);
    YAML_LOAD_OPT_DEG(params_, min_rotation_between_keyframes, double);

    YAML_LOAD_OPT(params_, min_time_between_scans, double);
    YAML_LOAD_OPT(params_, min_icp_goodness, double);
    YAML_LOAD_OPT(params_, min_icp_goodness_lc, double);
    YAML_LOAD_OPT(params_, decimate_to_point_count, unsigned int);
    YAML_LOAD_OPT(params_, voxel_filter_resolution, double);
    YAML_LOAD_OPT(params_, voxel_filter_decimation, unsigned int);
    YAML_LOAD_OPT(params_, full_pointcloud_decimation, unsigned int);
    YAML_LOAD_OPT(params_, voxel_filter_max_e2_e0, float);
    YAML_LOAD_OPT(params_, voxel_filter_max_e1_e0, float);
    YAML_LOAD_OPT(params_, voxel_filter_min_e2_e0, float);
    YAML_LOAD_OPT(params_, voxel_filter_min_e1_e0, float);

    YAML_LOAD_OPT(params_, min_dist_to_matching, double);
    YAML_LOAD_OPT(params_, max_dist_to_matching, double);
    YAML_LOAD_OPT(params_, max_dist_to_loop_closure, double);
    YAML_LOAD_OPT(params_, max_nearby_align_checks, unsigned int);
    YAML_LOAD_OPT(params_, min_topo_dist_to_consider_loopclosure, unsigned int);
    YAML_LOAD_OPT(params_, loop_closure_montecarlo_samples, unsigned int);

    load_icp_set_of_params(
        params_.icp_params_with_vel, cfg, "icp_params_with_vel.");
    load_icp_set_of_params(
        params_.icp_params_without_vel, cfg, "icp_params_without_vel.");
    load_icp_set_of_params(
        params_.icp_params_loopclosure, cfg, "icp_params_loopclosure.");

    YAML_LOAD_OPT(params_, debug_save_lidar_odometry, bool);
    YAML_LOAD_OPT(params_, debug_save_extra_edges, bool);
    YAML_LOAD_OPT(params_, debug_save_loop_closures, bool);

    {
        ProfilerEntry tle(profiler_, "filterPointCloud_initialize");
        state_.filter_grid.resize(
            {-50.0, -50.0, -10.0}, {50.0, 50.0, 10.0},
            params_.voxel_filter_resolution);
    }

    // attach to world model, if present:
    auto wms = findService<WorldModel>();
    if (wms.size() == 1)
        worldmodel_ = std::dynamic_pointer_cast<WorldModel>(wms[0]);

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
    profiler_.registerUserMeasure("onNewObservation.queue_length", queued);
    if (queued > 10)
    {
        MRPT_LOG_THROTTLE_ERROR(
            1.0, "Dropping observation due to worker threads too busy.");
        profiler_.registerUserMeasure("onNewObservation.drop_observation", 1);
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
            MRPT_LOG_DEBUG(
                "doProcessNewObservation: dropping observation, for "
                "`min_time_between_scans`.");
            return;
        }

        // Extract points from observation:
        pointclouds_t this_obs_points;
        bool          have_points;
        {
            ProfilerEntry tle(
                profiler_, "doProcessNewObservation.1.obs2pointcloud");

            const auto& pc = this_obs_points.layers["original"] =
                mrpt::maps::CSimplePointsMap::Create();

            have_points = pc->insertObservationPtr(o);
        }

        // Filter: keep corner areas only:
        {
            ProfilerEntry tle(
                profiler_, "doProcessNewObservation.2.filter_pointclouds");

            filterPointCloud(this_obs_points);

            // Remove the original, full-res point cloud to save memory:
            this_obs_points.layers.erase("original");
        }

        // Store for next step:
        auto last_obs_tim   = state_.last_obs_tim;
        auto last_points    = state_.last_points;
        state_.last_obs_tim = this_obs_tim;
        state_.last_points  = this_obs_points;

        if (!have_points)
        {
            MRPT_LOG_WARN_STREAM(
                "Observation of type `" << o->GetRuntimeClass()->className
                                        << "` could not be converted into a "
                                           "pointcloud. Doing nothing.");
            return;
        }

        bool create_keyframe = false;

        // First time we cannot do ICP since we need at least two pointclouds:
        if (last_points.layers.empty())
        {
            // Skip ICP.
            MRPT_LOG_DEBUG("First pointcloud: skipping ICP.");

            // Still, create a first KF (at origin)
            create_keyframe = true;
        }
        else
        {
            // Register point clouds using ICP:
            // ------------------------------------
            mrpt::poses::CPose3DPDFGaussian initial_guess;
            // Use velocity model for the initial guess:
            double dt = .0;
            if (last_obs_tim != mrpt::Clock::time_point())
                dt = mrpt::system::timeDifference(last_obs_tim, this_obs_tim);

            ICP_Output icp_out;
            ICP_Input  icp_in;
            icp_in.init_guess_to_wrt_from = mrpt::math::TPose3D(
                state_.last_iter_twist.vx * dt, state_.last_iter_twist.vy * dt,
                state_.last_iter_twist.vz * dt, state_.last_iter_twist.wz * dt,
                0, 0);
            MRPT_TODO("do omega_xyz part!");

            icp_in.to_pc   = this_obs_points;
            icp_in.from_pc = last_points;
            icp_in.from_id = state_.last_kf;
            icp_in.to_id =
                mola::INVALID_ID;  // current data, not a new KF (yet)
            icp_in.debug_str = "lidar_odom";

            // If we don't have a valid twist estimation, use a larger ICP
            // correspondence threshold:
            icp_in.icp_params = state_.last_iter_twist_is_good
                                    ? params_.icp_params_with_vel
                                    : params_.icp_params_without_vel;

            // Run ICP:
            {
                ProfilerEntry tle(
                    profiler_, "doProcessNewObservation.3.icp_latest");

                run_one_icp(icp_in, icp_out);
            }
            const mrpt::poses::CPose3D rel_pose =
                icp_out.found_pose_to_wrt_from.getMeanVal();

            // Update velocity model:
            state_.last_iter_twist.vx = rel_pose.x() / dt;
            state_.last_iter_twist.vy = rel_pose.y() / dt;
            state_.last_iter_twist.vz = rel_pose.z() / dt;
            state_.last_iter_twist.wz = rel_pose.yaw() / dt;
            MRPT_TODO("do omega_xyz part!");

            state_.last_iter_twist_is_good = true;

            MRPT_LOG_DEBUG_STREAM(
                "Est.twist=" << state_.last_iter_twist.asString());
            MRPT_LOG_DEBUG_STREAM(
                "Time since last scan="
                << mrpt::system::formatTimeInterval(dt));

            // Create a new KF if the distance since the last one is large
            // enough:
            state_.accum_since_last_kf = state_.accum_since_last_kf + rel_pose;
            const double dist_eucl_since_last =
                state_.accum_since_last_kf.norm();
            const double rot_since_last = 0;
            MRPT_TODO("Add rotation threshold");

            MRPT_LOG_DEBUG_FMT(
                "Since last KF: dist=%5.03f m rotation=%.01f deg",
                dist_eucl_since_last, mrpt::RAD2DEG(rot_since_last));

            create_keyframe =
                (icp_out.goodness > params_.min_icp_goodness &&
                 (dist_eucl_since_last >
                      params_.min_dist_xyz_between_keyframes ||
                  rot_since_last > params_.min_rotation_between_keyframes));

        }  // end: yes, we can do ICP

        // Should we create a new KF?
        if (create_keyframe)
        {
            // Yes: create new KF
            // 1) New KeyFrame
            BackEndBase::ProposeKF_Input kf;

            kf.timestamp = this_obs_tim;
            {
                mrpt::obs::CSensoryFrame& sf = kf.observations.emplace();
                sf.push_back(o);
            }

            std::future<BackEndBase::ProposeKF_Output> kf_out_fut;
            kf_out_fut = slam_backend_->addKeyFrame(kf);

            // Wait until it's executed:
            auto kf_out = kf_out_fut.get();

            ASSERT_(kf_out.success);
            ASSERT_(
                kf_out.new_kf_id &&
                kf_out.new_kf_id.value() != mola::INVALID_ID);

            {
                std::lock_guard<std::mutex> lck(local_pose_graph_mtx);

                // Add point cloud to local graph:
                state_.local_pose_graph.pcs[kf_out.new_kf_id.value()] =
                    this_obs_points;
            }
            MRPT_LOG_INFO_STREAM("New KF: ID=" << *kf_out.new_kf_id);

            // 2) New SE(3) constraint between consecutive Keyframes:
            if (state_.last_kf != mola::INVALID_ID)
            {
                std::future<BackEndBase::AddFactor_Output> factor_out_fut;
                // Important: The "constant velocity model" factor is
                // automatically added by the SLAM module (if applicable). Here,
                // all we need to tell it is the SE(3) constraint, and the
                // KeyFrame timestamp:
                mola::FactorRelativePose3 fPose3(
                    state_.last_kf, kf_out.new_kf_id.value(),
                    state_.accum_since_last_kf.asTPose());

                mola::Factor f = std::move(fPose3);
                factor_out_fut = slam_backend_->addFactor(f);

                // Wait until it's executed:
                auto factor_out = factor_out_fut.get();
                ASSERT_(factor_out.success);
                ASSERT_(
                    factor_out.new_factor_id &&
                    factor_out.new_factor_id != mola::INVALID_FID);

                // Append to local graph as well:
                {
                    std::lock_guard<std::mutex> lck(local_pose_graph_mtx);

                    state_.local_pose_graph.graph.insertEdgeAtEnd(
                        state_.last_kf, *kf_out.new_kf_id,
                        state_.accum_since_last_kf);
                }

                MRPT_LOG_DEBUG_STREAM(
                    "New FactorRelativePose3ConstVel: #"
                    << state_.last_kf << " <=> #" << kf_out.new_kf_id.value()
                    << ". rel_pose=" << state_.accum_since_last_kf.asString());
            }

            // Reset accumulators:
            state_.accum_since_last_kf = mrpt::poses::CPose3D();
            state_.last_kf             = kf_out.new_kf_id.value();
        }  // end done add a new KF

        // In any case, publish to the SLAM BackEnd what's our **current**
        // vehicle pose, no matter if it's a keyframe or not:
        {
            ProfilerEntry tle(
                profiler_,
                "doProcessNewObservation.4.advertiseUpdatedLocalization");

            BackEndBase::AdvertiseUpdatedLocalization_Input new_loc;
            new_loc.timestamp    = this_obs_tim;
            new_loc.reference_kf = state_.last_kf;
            new_loc.pose         = state_.accum_since_last_kf.asTPose();

            std::future<void> adv_pose_fut =
                slam_backend_->advertiseUpdatedLocalization(new_loc);
        }

        // Now, let's try to align this new KF against a few past KFs as well.
        // we'll do it in separate threads, with priorities so the latest KFs
        // are always attended first:
        bool can_check_for_other_matches = true;
        {
            std::lock_guard<std::mutex> lck(local_pose_graph_mtx);
            can_check_for_other_matches =
                (state_.local_pose_graph.pcs.size() > 1);
        }

        if (can_check_for_other_matches)
        {
            ProfilerEntry tle(
                profiler_, "doProcessNewObservation.5.checkForNearbyKFs");
            checkForNearbyKFs();
        }
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
    }
}

void LidarICP::checkForNearbyKFs()
{
    using namespace std::string_literals;

    MRPT_START

    // Run Dijkstra wrt to the last KF:
    using euclidean_dist_t = double;
    std::map<
        euclidean_dist_t, std::pair<mrpt::graphs::TNodeID, topological_dist_t>>
               KF_distances;
    mola::id_t current_kf_id{mola::INVALID_ID};
    {
        std::lock_guard<std::mutex> lck(local_pose_graph_mtx);

        auto& lpg     = state_.local_pose_graph.graph;
        current_kf_id = state_.last_kf;

        // Call Dijkstra, starting from the current_kf_id as root, and build a
        // spanning-tree to estimate the relative poses, and topological
        // distances, to all other nodes:
        lpg.root = current_kf_id;
        lpg.nodes.clear();
        lpg.nodes[lpg.root] = mrpt::poses::CPose3D::Identity();
        std::map<mrpt::graphs::TNodeID, size_t> topolog_dists;

        lpg.dijkstra_nodes_estimate(std::ref(topolog_dists));

        // Sort KFs by distance
        for (const auto& kfs : lpg.nodes)
        {
            const auto it_dist = topolog_dists.find(kfs.first);
            ASSERT_(it_dist != topolog_dists.end());
            const topological_dist_t topo_d = it_dist->second;

            KF_distances[kfs.second.norm()] = std::make_pair(kfs.first, topo_d);
        }

        std::map<mrpt::graphs::TNodeID, std::set<mrpt::graphs::TNodeID>> adj;
        lpg.getAdjacencyMatrix(adj);

        // Remove too distant KFs:
        while (lpg.nodes.size() > params_.max_KFs_local_graph)
        {
            const auto id_to_remove = KF_distances.rbegin()->second.first;
            KF_distances.erase(std::prev(KF_distances.end()));

            lpg.nodes.erase(id_to_remove);
            state_.local_pose_graph.pcs.erase(id_to_remove);
            for (const auto other_id : adj[id_to_remove])
            {
                lpg.edges.erase(std::make_pair(id_to_remove, other_id));
                lpg.edges.erase(std::make_pair(other_id, id_to_remove));
            }
        }
    }

    // Pick the node at an intermediary distance and try to align
    // against it:
    auto it1 = KF_distances.lower_bound(params_.min_dist_to_matching);
    auto it2 = KF_distances.upper_bound(std::max(
        params_.max_dist_to_loop_closure, params_.max_dist_to_matching));

    // Store here all the desired checks:
    std::vector<ICP_Input::Ptr>                nearby_checks;
    std::map<euclidean_dist_t, ICP_Input::Ptr> loop_closure_checks;

    for (auto it = it1; it != it2; ++it)
    {
        const double             kf_eucl_dist        = it->first;
        const auto               kf_id               = it->second.first;
        const topological_dist_t kf_topo_d           = it->second.second;
        bool                     edge_already_exists = false;
        const bool               is_potential_loop_closure =
            (kf_topo_d >= params_.min_topo_dist_to_consider_loopclosure);

        // Only explore KFs farther than this threshold if they are LCs:
        if (!is_potential_loop_closure &&
            kf_eucl_dist > params_.max_dist_to_matching)
            continue;

        // Already sent out for checking?
        const auto pair_ids = std::make_pair(
            std::min(kf_id, current_kf_id), std::max(kf_id, current_kf_id));

        {
            std::lock_guard<std::mutex> lck(local_pose_graph_mtx);

            if (state_.local_pose_graph.checked_KF_pairs.count(pair_ids) != 0)
                edge_already_exists = true;
        }

        MRPT_TODO("Factors should have an annotation to know who created them");
        // Also check in the WorldModel if *we* created an edge already between
        // those two KFs:
        if (!edge_already_exists && worldmodel_)
        {
            worldmodel_->entities_lock_for_read();
            worldmodel_->factors_lock_for_read();

            const auto connected = worldmodel_->entity_neighbors(kf_id);
            if (connected.count(current_kf_id) != 0)
            {
                MRPT_LOG_DEBUG_STREAM(
                    "[checkForNearbyKFs] Discarding pair check since a factor "
                    "already exists between #"
                    << kf_id << " <==> #" << current_kf_id);
                edge_already_exists = false;
            }

            worldmodel_->factors_unlock_for_read();
            worldmodel_->entities_unlock_for_read();
        }

        if (!edge_already_exists)
        {
            std::lock_guard<std::mutex> lck(local_pose_graph_mtx);

            auto d     = std::make_shared<ICP_Input>();
            d->to_id   = kf_id;
            d->from_id = current_kf_id;
            d->to_pc   = state_.local_pose_graph.pcs[d->to_id];
            d->from_pc = state_.local_pose_graph.pcs[d->from_id];
            d->init_guess_to_wrt_from =
                state_.local_pose_graph.graph.nodes[kf_id].asTPose();

            // Is this an extra edge for a nearby KF, or a potential loop
            // closure?
            if (!is_potential_loop_closure)
            {
                // Regular, nearby KF-to-KF ICP check:
                d->align_kind = AlignKind::NearbyAlign;
                d->debug_str  = "extra_edge"s;
                d->icp_params = params_.icp_params_with_vel;

                nearby_checks.emplace_back(std::move(d));
            }
            else
            {
                // Attempt to close a loop:
                d->align_kind = AlignKind::LoopClosure;
                d->debug_str  = "loop_closure"s;
                d->icp_params = params_.icp_params_loopclosure;

                loop_closure_checks[kf_eucl_dist] = std::move(d);
            }
        }
    }

    // Actually send the tasks to the worker thread, firstly filtering
    // some of them to reduce the computational cost:
    // Nearby checks: send a maximum of "N"
    const size_t nNearbyChecks    = nearby_checks.size();
    const size_t nearbyCheckDecim = std::max(
        static_cast<size_t>(1U),
        nNearbyChecks / params_.max_nearby_align_checks);
    for (size_t idx = 0; idx < nNearbyChecks; idx += nearbyCheckDecim)
    {
        const auto& d = nearby_checks[idx];
        worker_pool_past_KFs_.enqueue(
            &LidarICP::doCheckForNonAdjacentKFs, this, d);

        {
            std::lock_guard<std::mutex> lck(local_pose_graph_mtx);

            // Mark as already considered for check:
            state_.local_pose_graph.checked_KF_pairs.insert(std::make_pair(
                std::min(d->to_id, d->from_id),
                std::max(d->to_id, d->from_id)));
        }
    }
    // Loop closures: just send the one with the smallest distance (in theory,
    // it *might* be the easiest one to align...)
    if (!loop_closure_checks.empty())
    {
        const auto& d = loop_closure_checks.begin()->second;

        worker_pool_past_KFs_.enqueue(
            &LidarICP::doCheckForNonAdjacentKFs, this, d);

        MRPT_LOG_WARN_STREAM(
            "Attempting to close a loop between KFs #" << d->to_id << " <==> #"
                                                       << d->from_id);
        {
            std::lock_guard<std::mutex> lck(local_pose_graph_mtx);
            // Mark as already considered for check:
            state_.local_pose_graph.checked_KF_pairs.insert(std::make_pair(
                std::min(d->to_id, d->from_id),
                std::max(d->to_id, d->from_id)));
        }
    }

    MRPT_END
}

void LidarICP::doCheckForNonAdjacentKFs(ICP_Input::Ptr d)
{
    try
    {
        ProfilerEntry tleg(profiler_, "doCheckForNonAdjacentKFs");

        // Call ICP:
        ICP_Output icp_out;

        if (d->align_kind != AlignKind::LoopClosure)
        {
            // Regular case:
            ProfilerEntry tle(profiler_, "doCheckForNonAdjacentKFs.run_icp");
            run_one_icp(*d, icp_out);
        }
        else
        {
            // Loop closure:
            ProfilerEntry tle(
                profiler_, "doCheckForNonAdjacentKFs.run_icp_loop_closure");

            // do a small montecarlo sampling and keep the best attempt:
            const double std_xyz = params_.max_dist_to_loop_closure * 0.1;
            const double std_rot = mrpt::DEG2RAD(2.0);

            const auto original_guess = d->init_guess_to_wrt_from;

            mrpt::random::CRandomGenerator rnd;

            for (size_t i = 0; i < params_.loop_closure_montecarlo_samples; i++)
            {
                d->init_guess_to_wrt_from = original_guess;
                d->init_guess_to_wrt_from.x += rnd.drawGaussian1D(0, std_xyz);
                d->init_guess_to_wrt_from.y += rnd.drawGaussian1D(0, std_xyz);
                d->init_guess_to_wrt_from.z += rnd.drawGaussian1D(0, std_xyz);
                d->init_guess_to_wrt_from.yaw += rnd.drawGaussian1D(0, std_rot);

                ICP_Output this_icp_out;
                run_one_icp(*d, this_icp_out);
                if (this_icp_out.goodness > icp_out.goodness)
                    icp_out = this_icp_out;
            }
        }

        const mrpt::poses::CPose3D rel_pose =
            icp_out.found_pose_to_wrt_from.getMeanVal();
        const double icp_goodness = icp_out.goodness;

        // Accept the new edge?
        const mrpt::poses::CPose3D init_guess(d->init_guess_to_wrt_from);
        const double pos_correction = (rel_pose - init_guess).norm();
        const double correction_percent =
            pos_correction / (init_guess.norm() + 0.01);

        MRPT_LOG_DEBUG_STREAM(
            "[doCheckForNonAdjacentKFs] Checking KFs: #"
            << d->from_id << " ==> #" << d->to_id
            << " init_guess: " << d->init_guess_to_wrt_from.asString() << "\n"
            << mrpt::format("ICP goodness=%.03f\n", icp_out.goodness)
            << "ICP rel_pose=" << rel_pose.asString() << " init_guess was "
            << init_guess.asString() << " (changes " << 100 * correction_percent
            << "%)");

        const double goodness_thres =
            (d->align_kind == AlignKind::LoopClosure
                 ? params_.min_icp_goodness_lc
                 : params_.min_icp_goodness);

        if (icp_goodness > goodness_thres &&
            (correction_percent < 0.2 ||
             d->align_kind == AlignKind::LoopClosure))
        {
            std::future<BackEndBase::AddFactor_Output> factor_out_fut;
            mola::FactorRelativePose3                  fPose3(
                d->from_id, d->to_id, rel_pose.asTPose());

            mola::Factor f = std::move(fPose3);
            factor_out_fut = slam_backend_->addFactor(f);

            // Wait until it's executed:
            auto factor_out = factor_out_fut.get();
            ASSERT_(factor_out.success);
            ASSERT_(
                factor_out.new_factor_id &&
                factor_out.new_factor_id != mola::INVALID_FID);

            // Append to local graph as well::
            {
                std::lock_guard<std::mutex> lck(local_pose_graph_mtx);
                state_.local_pose_graph.graph.insertEdgeAtEnd(
                    d->from_id, d->to_id, rel_pose);
            }

            MRPT_LOG_DEBUG_STREAM(
                "New FactorRelativePose3: #"
                << d->from_id << " <=> #" << d->to_id
                << ". rel_pose=" << rel_pose.asString());
        }
    }
    catch (const std::exception& e)
    {
        MRPT_LOG_ERROR_STREAM("Exception:\n" << mrpt::exception_to_str(e));
    }
}

void LidarICP::run_one_icp(const ICP_Input& in, ICP_Output& out)
{
    using namespace std::string_literals;

    MRPT_START

    {
        ProfilerEntry tle(profiler_, "run_one_icp");

        ASSERT_(!in.icp_params.empty());

        size_t                  largest_pc_count = 1;
        MultiCloudICP::clouds_t pcs_from, pcs_to;
        for (auto& layer : in.from_pc.layers)
        {
            pcs_from.push_back(layer.second);
            pcs_to.push_back(in.to_pc.layers.at(layer.first));

            mrpt::keep_max(largest_pc_count, layer.second->size());
        }

        unsigned int decim = 1;
        if (params_.decimate_to_point_count > 0)
            decim = static_cast<unsigned>(
                largest_pc_count / params_.decimate_to_point_count);

        mrpt::math::TPose3D current_solution = in.init_guess_to_wrt_from;

        for (unsigned int stage = 0; stage < in.icp_params.size(); stage++)
        {
            MRPT_LOG_DEBUG_STREAM(
                "MRPT ICP: max point count=" << largest_pc_count
                                             << " decimation=" << decim);

            MultiCloudICP::Parameters icp_params       = in.icp_params[stage];
            icp_params.corresponding_points_decimation = decim;

            MultiCloudICP::Results icp_result;
            MultiCloudICP::align(
                pcs_from, pcs_to, current_solution, icp_params, icp_result);

            if (icp_result.goodness > 0)
            {
                // Keep as init value for next stage:
                current_solution = icp_result.optimal_tf.mean.asTPose();
            }

            out.found_pose_to_wrt_from = icp_result.optimal_tf;
            out.goodness               = icp_result.goodness;

            MRPT_LOG_DEBUG_FMT(
                "ICP stage #%u (kind=%u): goodness=%.03f iters=%u rel_pose=%s "
                "termReason=%u",
                stage, static_cast<unsigned int>(in.align_kind), out.goodness,
                static_cast<unsigned int>(icp_result.nIterations),
                out.found_pose_to_wrt_from.getMeanVal().asString().c_str(),
                static_cast<unsigned int>(icp_result.terminationReason));
        }

        // Check quality of match:
        MRPT_TODO("Impl. finite differences based Hessian check");
    }

    // -------------------------------------------------
    MRPT_TODO("Move this to its own method");

    // Save debug files for debugging ICP quality
    bool gen_debug = (in.align_kind == AlignKind::LidarOdometry &&
                      params_.debug_save_lidar_odometry) ||
                     (in.align_kind == AlignKind::NearbyAlign &&
                      params_.debug_save_extra_edges) ||
                     (in.align_kind == AlignKind::LoopClosure &&
                      params_.debug_save_loop_closures);

    if (gen_debug)
    {
        const auto num_pc_layers = in.from_pc.layers.size();
        debug_dump_icp_file_counter++;

        for (unsigned int l = 0; l < num_pc_layers; l++)
        {
            auto fil_name_prefix = mrpt::system::fileNameStripInvalidChars(
                getModuleInstanceName() +
                mrpt::format(
                    "_debug_ICP_%s_%05u_layer%02u", in.debug_str.c_str(),
                    static_cast<unsigned int>(debug_dump_icp_file_counter), l));

            // Init:
            mrpt::opengl::COpenGLScene scene;

            auto it_from = in.from_pc.layers.begin();
            std::advance(it_from, l);
            auto it_to = in.to_pc.layers.begin();
            std::advance(it_to, l);

            scene.insert(
                mrpt::opengl::stock_objects::CornerXYZSimple(2.0f, 4.0f));
            auto gl_from = mrpt::opengl::CSetOfObjects::Create();
            it_from->second->renderOptions.color =
                mrpt::img::TColorf(.0f, .0f, 1.0f);
            it_from->second->getAs3DObject(gl_from);
            gl_from->setName("KF_from"s);
            gl_from->enableShowName();
            scene.insert(gl_from);

            auto gl_to = mrpt::opengl::CSetOfObjects::Create();
            gl_to->insert(
                mrpt::opengl::stock_objects::CornerXYZSimple(1.0f, 2.0f));
            it_to->second->renderOptions.color =
                mrpt::img::TColorf(1.0f, .0f, .0f);
            it_to->second->getAs3DObject(gl_to);
            gl_to->setName("KF_to"s);
            gl_to->enableShowName();
            gl_to->setPose(in.init_guess_to_wrt_from);
            scene.insert(gl_to);

            auto gl_info  = mrpt::opengl::CText::Create(),
                 gl_info2 = mrpt::opengl::CText::Create();
            gl_info->setLocation(0., 0., 5.);
            gl_info2->setLocation(0., 0., 4.8);
            scene.insert(gl_info);
            scene.insert(gl_info2);

            {
                std::ostringstream ss;
                ss << "to_ID     = " << in.to_id
                   << " from_ID   = " << in.from_id << " | " << in.debug_str;
                gl_info->setString(ss.str());
            }
            {
                std::ostringstream ss;
                ss << "init_pose = " << in.init_guess_to_wrt_from.asString();
                gl_info2->setString(ss.str());
            }

            const auto fil_name_init = fil_name_prefix + "_0init.3Dscene"s;
            if (scene.saveToFile(fil_name_init))
                MRPT_LOG_DEBUG_STREAM(
                    "Wrote debug init ICP scene to: " << fil_name_init);
            else
                MRPT_LOG_ERROR_STREAM(
                    "Error saving init ICP scene to :" << fil_name_init);

            // Final:
            const auto final_pose = out.found_pose_to_wrt_from.getMeanVal();
            gl_to->setPose(final_pose);

            {
                std::ostringstream ss;
                ss << "to_ID     = " << in.to_id
                   << " from_ID   = " << in.from_id;
                gl_info->setString(ss.str());
            }
            {
                std::ostringstream ss;
                ss << " final_pose = " << final_pose.asString()
                   << " goodness: " << out.goodness * 100.0;

                gl_info2->setString(ss.str());
            }

            const auto fil_name_final = fil_name_prefix + "_1final.3Dscene"s;
            if (scene.saveToFile(fil_name_final))
                MRPT_LOG_DEBUG_STREAM(
                    "Wrote debug final ICP scene to: " << fil_name_final);
            else
                MRPT_LOG_ERROR_STREAM(
                    "Error saving final ICP scene to :" << fil_name_final);

            // Also: save as Rawlog for ICP debugging in RawLogViewer app:
            {
#if 0
                mrpt::obs::CRawlog rawlog;
                {
                    auto pc = mrpt::obs::CObservationPointCloud::Create();
                    pc->pointcloud  = in.from_pc.sampled;
                    pc->sensorLabel = "from_sampled";
                    rawlog.addObservationMemoryReference(pc);
                }
                {
                    auto pc = mrpt::obs::CObservationPointCloud::Create();
                    pc->pointcloud  = in.to_pc.sampled;
                    pc->sensorLabel = "to_sampled";
                    rawlog.addObservationMemoryReference(pc);
                }
            {
                auto pc         = mrpt::obs::CObservationPointCloud::Create();
                pc->pointcloud  = in.from_pc.original;
                pc->sensorLabel = "from_original";
                rawlog.addObservationMemoryReference(pc);
            }
            {
                auto pc         = mrpt::obs::CObservationPointCloud::Create();
                pc->pointcloud  = in.to_pc.original;
                pc->sensorLabel = "to_original";
                rawlog.addObservationMemoryReference(pc);
            }
            {
                mrpt::config::CConfigFileMemory cfg;
                in.icp_params.saveToConfigFile(cfg, "ICP");

                auto comm  = mrpt::obs::CObservationComment::Create();
                comm->text = cfg.getContent();
                rawlog.addObservationMemoryReference(comm);
            }

            const auto fil_name_rawlog = fil_name_prefix + ".rawlog"s;
            if (rawlog.saveToRawLogFile(fil_name_rawlog))
                MRPT_LOG_DEBUG_STREAM(
                    "Wrote ICP debug rawlog: " << fil_name_rawlog);
#endif
            }
        }
    }

    MRPT_END
}

void LidarICP::filterPointCloud(pointclouds_t& pcs)
{
    MRPT_START

    // Get a ref to the input, full resolution point cloud:
    const auto& pcptr = pcs.layers["original"];
    ASSERTMSG_(pcptr, "Missing point cloud layer: `original`");
    const auto& pc = *pcptr;

    auto& pc_edges      = pcs.layers["edges"];
    auto& pc_planes     = pcs.layers["planes"];
    auto& pc_full_decim = pcs.layers["full_decim"];
    if (!pc_edges) pc_edges = mrpt::maps::CSimplePointsMap::Create();
    if (!pc_planes) pc_planes = mrpt::maps::CSimplePointsMap::Create();
    if (!pc_full_decim) pc_full_decim = mrpt::maps::CSimplePointsMap::Create();

    pc_edges->clear();
    pc_edges->reserve(pc.size() / 10);
    pc_planes->clear();
    pc_planes->reserve(pc.size() / 10);
    pc_full_decim->clear();
    pc_full_decim->reserve(pc.size() / 10);

    state_.filter_grid.clear();
    state_.filter_grid.processPointCloud(pc);

    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

    const float max_e20 = params_.voxel_filter_max_e2_e0;
    const float max_e10 = params_.voxel_filter_max_e1_e0;
    const float min_e20 = params_.voxel_filter_min_e2_e0;
    const float min_e10 = params_.voxel_filter_min_e1_e0;

    std::size_t nEdgeVoxels = 0, nPlaneVoxels = 0, nTotalVoxels = 0;
    for (const auto& vxl_pts : state_.filter_grid.pts_voxels)
    {
        if (!vxl_pts.indices.empty()) nTotalVoxels++;
        if (vxl_pts.indices.size() < 5) continue;

        // Analyze the voxel contents:
        mrpt::math::TPoint3Df mean{0, 0, 0};
        const float           inv_n = (1.0f / vxl_pts.indices.size());
        for (size_t i = 0; i < vxl_pts.indices.size(); i++)
        {
            const auto pt_idx = vxl_pts.indices[i];
            mean.x += xs[pt_idx];
            mean.y += ys[pt_idx];
            mean.z += zs[pt_idx];
        }
        mean.x *= inv_n;
        mean.y *= inv_n;
        mean.z *= inv_n;

        Eigen::Matrix3f mat_a;
        mat_a.setZero();
        for (size_t i = 0; i < vxl_pts.indices.size(); i++)
        {
            const auto                  pt_idx = vxl_pts.indices[i];
            const mrpt::math::TPoint3Df a(
                xs[pt_idx] - mean.x, ys[pt_idx] - mean.y, zs[pt_idx] - mean.z);
            mat_a(0, 0) += a.x * a.x;
            mat_a(1, 0) += a.x * a.y;
            mat_a(2, 0) += a.x * a.z;
            mat_a(1, 1) += a.y * a.y;
            mat_a(2, 1) += a.y * a.z;
            mat_a(2, 2) += a.z * a.z;
        }
        mat_a *= inv_n;

        // This only looks at the lower-triangular part of the cov
        // matrix (which is wrong in loam_velodyne!)
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(mat_a);

        const Eigen::Vector3f eig_vals = esolver.eigenvalues();

        const float e0 = eig_vals[0], e1 = eig_vals[1], e2 = eig_vals[2];

        mrpt::maps::CPointsMap* dest = nullptr;
        if (e2 < max_e20 * e0 && e1 < max_e10 * e0)
        {
            nEdgeVoxels++;
            dest = pc_edges.get();
        }
        else if (e2 > min_e20 * e0 && e1 > min_e10 * e0)
        {
            // Filter out horizontal planes, since their uneven density
            // makes ICP fail to converge.
            // A plane on the ground has its 0'th eigenvector like [0 0 1]
            const Eigen::Vector3f ev0 = esolver.eigenvectors().col(0);
            // || mean.x > 10.0f || mean.y > 10.0f)
            if (std::abs(ev0.z()) < 0.9f)
            {
                nPlaneVoxels++;
                dest = pc_planes.get();
            }
        }
        if (dest != nullptr)
        {
            for (size_t i = 0; i < vxl_pts.indices.size();
                 i += params_.voxel_filter_decimation)
            {
                const auto pt_idx = vxl_pts.indices[i];
                dest->insertPointFast(xs[pt_idx], ys[pt_idx], zs[pt_idx]);
            }
        }
        for (size_t i = 0; i < vxl_pts.indices.size();
             i += params_.full_pointcloud_decimation)
        {
            const auto pt_idx = vxl_pts.indices[i];
            pc_full_decim->insertPointFast(xs[pt_idx], ys[pt_idx], zs[pt_idx]);
        }
    }
    MRPT_LOG_DEBUG_STREAM(
        "[VoxelGridFilter] Voxel counts: total=" << nTotalVoxels
                                                 << " edges=" << nEdgeVoxels
                                                 << " planes=" << nPlaneVoxels);
    MRPT_END
}
