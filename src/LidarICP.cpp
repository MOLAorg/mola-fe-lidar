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
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <yaml-cpp/yaml.h>

// ------------
#include <mrpt/containers/CDynamicGrid3D.h>

class CPointCloudVoxelGrid
{
   public:
    CPointCloudVoxelGrid() = default;

    void resize(
        const mrpt::math::TPoint3D& min_corner,
        const mrpt::math::TPoint3D& max_corner, const float voxel_size)
    {
        pts_voxels.clear();
        pts_voxels.setSize(
            min_corner.x, max_corner.x, min_corner.y, max_corner.y,
            min_corner.z, max_corner.z, voxel_size, voxel_size);
    }

    void processPointCloud(const mrpt::maps::CPointsMap& p)
    {
        const auto& xs   = p.getPointsBufferRef_x();
        const auto& ys   = p.getPointsBufferRef_y();
        const auto& zs   = p.getPointsBufferRef_z();
        const auto  npts = xs.size();

        for (std::size_t i = 0; i < npts; i++)
        {
            auto* c = pts_voxels.cellByPos(xs[i], ys[i], zs[i]);
            if (c) c->indices.push_back(i);  // only if not out of grid range
        }
    }

    void clear()
    {
        for (auto& c : pts_voxels) c.indices.clear();
    }

    /** The list of point indices in each voxel */
    struct voxel_t
    {
        std::vector<std::size_t> indices;
    };
    using grid_t = mrpt::containers::CDynamicGrid3D<voxel_t, float>;

    /** The point indices in each voxel. Directly access to each desired cell,
     * use its iterator, etc. */
    grid_t pts_voxels;
};

// ------------

using namespace mola;

MRPT_INITIALIZER(do_register){MOLA_REGISTER_MODULE(LidarICP)}

LidarICP::LidarICP() = default;

void LidarICP::initialize(const std::string& cfg_block)
{
    MRPT_TRY_START

    // Load:
    auto c   = YAML::Load(cfg_block);
    auto cfg = c["params"];
    MRPT_LOG_DEBUG_STREAM("Loading these params:\n" << cfg);

    YAML_LOAD_REQ(params_, min_dist_xyz_between_keyframes, double);
    YAML_LOAD_OPT(params_, min_time_between_scans, double);
    YAML_LOAD_OPT(params_, min_icp_goodness, double);
    YAML_LOAD_OPT(params_, decimate_to_point_count, unsigned int);
    YAML_LOAD_OPT(params_, voxel_filter_resolution, double);
    YAML_LOAD_OPT(params_, voxel_filter_max_e2_e0, double);
    YAML_LOAD_OPT(params_, voxel_filter_max_e1_e0, double);

    YAML_LOAD_OPT(params_, mrpt_icp_with_vel.maxIterations, unsigned int);
    YAML_LOAD_OPT(params_, mrpt_icp_with_vel.thresholdDist, double);
    YAML_LOAD_OPT_DEG(params_, mrpt_icp_with_vel.thresholdAng, double);
    YAML_LOAD_OPT(params_, mrpt_icp_with_vel.ALFA, double);
    YAML_LOAD_OPT(params_, mrpt_icp_with_vel.smallestThresholdDist, double);

    YAML_LOAD_OPT(params_, mrpt_icp_without_vel.maxIterations, unsigned int);
    YAML_LOAD_OPT(params_, mrpt_icp_without_vel.thresholdDist, double);
    YAML_LOAD_OPT_DEG(params_, mrpt_icp_without_vel.thresholdAng, double);
    YAML_LOAD_OPT(params_, mrpt_icp_without_vel.ALFA, double);
    YAML_LOAD_OPT(params_, mrpt_icp_without_vel.smallestThresholdDist, double);

    YAML_LOAD_OPT(params_, debug_save_all_icp_results, bool);

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

        // Extract points from observation:
        mrpt::maps::CPointsMap::Ptr this_obs_points;
        bool                        have_points;
        {
            ProfilerEntry tle(
                profiler_, "doProcessNewObservation.1.obs2pointcloud");

            this_obs_points = mrpt::maps::CSimplePointsMap::Create();
            have_points     = this_obs_points->insertObservationPtr(o);
        }

        // Filter: keep corner areas only:
        {
            ProfilerEntry tle(
                profiler_, "doProcessNewObservation.2.filter_pointclouds");

            filterPointCloud(*this_obs_points);
        }

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
            state_.last_iter_twist.vz * dt, 0, 0, 0);
        MRPT_TODO("do omega_xyz part!");

        icp_in.to_pc   = this_obs_points;
        icp_in.from_pc = last_points;
        icp_in.from_id = state_.last_kf;
        icp_in.to_id   = mola::INVALID_ID;  // current data, not a new KF (yet)
        icp_in.debug_str = "lidar_odometry";

        // If we don't have a valid twist estimation, use a larger ICP
        // correspondence threshold:
        icp_in.mrpt_icp_params = state_.last_iter_twist_is_good
                                     ? params_.mrpt_icp_with_vel
                                     : params_.mrpt_icp_without_vel;

        // Run ICP:
        {
            ProfilerEntry tle(
                profiler_, "doProcessNewObservation.3.icp_latest");

            run_one_icp(icp_in, icp_out);
        }
        const mrpt::poses::CPose3D rel_pose =
            icp_out.found_pose_to_wrt_from->getMeanVal();

        // Update velocity model:
        state_.last_iter_twist.vx = rel_pose.x() / dt;
        state_.last_iter_twist.vy = rel_pose.y() / dt;
        state_.last_iter_twist.vz = rel_pose.z() / dt;
        MRPT_TODO("do omega_xyz part!");

        state_.last_iter_twist_is_good = true;

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

        // Should we create a new KF?
        if (icp_out.goodness > params_.min_icp_goodness &&
            dist_eucl_since_last > params_.min_dist_xyz_between_keyframes)
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

            // 2) New SE(3) constraint between consecutive Keyframes:
            if (state_.last_kf != mola::INVALID_ID)
            {
                std::future<BackEndBase::AddFactor_Output> factor_out_fut;
                mola::FactorRelativePose3                  fPose3(
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
            }

            // Done.
            MRPT_LOG_INFO_STREAM(
                "New KF: ID=" << *kf_out.new_kf_id << " rel_pose="
                              << state_.accum_since_last_kf.asString());

            // Reset accumulators:
            state_.accum_since_last_kf = mrpt::poses::CPose3D();
            state_.last_kf             = kf_out.new_kf_id.value();
        }  // end done add a new KF

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
                profiler_, "doProcessNewObservation.4.checkForNearbyKFs");
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
    MRPT_START

    // Run Dijkstra wrt to the last KF:
    std::map<double, mrpt::graphs::TNodeID> KF_distances;
    mola::id_t                              last_kf_id{mola::INVALID_ID};
    {
        std::lock_guard<std::mutex> lck(local_pose_graph_mtx);

        auto& lpg  = state_.local_pose_graph.graph;
        last_kf_id = state_.last_kf;

        lpg.root = last_kf_id;
        lpg.nodes.clear();
        lpg.nodes[lpg.root] = mrpt::poses::CPose3D::Identity();
        lpg.dijkstra_nodes_estimate();

        // Remove too distant KFs: they belong to "loop closure", not to
        // "lidar odometry"!
        for (const auto& kfs : lpg.nodes)
            KF_distances[kfs.second.norm()] = kfs.first;

        std::map<mrpt::graphs::TNodeID, std::set<mrpt::graphs::TNodeID>> adj;
        lpg.getAdjacencyMatrix(adj);

        while (lpg.nodes.size() > params_.max_KFs_local_graph)
        {
            const auto id_to_remove = KF_distances.rbegin()->second;
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
    const double min_dist_to_test = 5.0;
    const double max_dist_to_test = 25.0;

    auto it1 = KF_distances.lower_bound(min_dist_to_test);
    auto it2 = KF_distances.upper_bound(max_dist_to_test);

    for (auto it = it1; it != it2; ++it)
    {
        const auto kf_id               = it->second;
        bool       edge_already_exists = false;

        // Already sent out for checking?
        const auto pair_ids = std::make_pair(
            std::min(kf_id, last_kf_id), std::max(kf_id, last_kf_id));

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
            worldmodel_->entities_lock();
            worldmodel_->factors_lock();

            const auto connected = worldmodel_->entity_neighbors(kf_id);
            if (connected.count(last_kf_id) != 0)
            {
                MRPT_LOG_DEBUG_STREAM(
                    "[checkForNearbyKFs] Discarding pair check since a factor "
                    "already exists between #"
                    << kf_id << " <==> #" << last_kf_id);
                edge_already_exists = false;
            }

            worldmodel_->factors_unlock();
            worldmodel_->entities_unlock();
        }

        if (!edge_already_exists)
        {
            std::lock_guard<std::mutex> lck(local_pose_graph_mtx);

            auto d     = std::make_shared<ICP_Input>();
            d->to_id   = kf_id;
            d->from_id = last_kf_id;
            d->to_pc   = state_.local_pose_graph.pcs[d->to_id];
            d->from_pc = state_.local_pose_graph.pcs[d->from_id];
            d->init_guess_to_wrt_from =
                state_.local_pose_graph.graph.nodes[kf_id].asTPose();

            worker_pool_past_KFs_.enqueue(
                &LidarICP::doCheckForNonAdjacentKFs, this, d);

            // Mark as sent for check:
            state_.local_pose_graph.checked_KF_pairs.insert(pair_ids);
        }
    }

    MRPT_END
}

void LidarICP::doCheckForNonAdjacentKFs(std::shared_ptr<ICP_Input> d)
{
    try
    {
        ProfilerEntry tleg(profiler_, "doCheckForNonAdjacentKFs");

        // Call ICP:
        // Use current values for: state_.mrpt_icp.options
        mrpt::slam::CICP::TReturnInfo ret_info;
        ASSERT_(d->from_pc);
        ASSERT_(d->to_pc);

        ICP_Output icp_out;
        d->debug_str = "doCheckForNonAdjacentKFs";
        {
            run_one_icp(*d, icp_out);
        }
        const mrpt::poses::CPose3D rel_pose =
            icp_out.found_pose_to_wrt_from->getMeanVal();
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
            << mrpt::format(
                   "MRPT ICP: goodness=%.03f iters=%u\n", ret_info.goodness,
                   ret_info.nIterations)
            << "ICP rel_pose=" << rel_pose.asString() << " init_guess was "
            << init_guess.asString() << " (changes " << 100 * correction_percent
            << "%)");

        if (icp_goodness > params_.min_icp_goodness &&
            correction_percent < 0.20)
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

    // Call ICP:
    mrpt::slam::CICP mrpt_icp;
    mrpt_icp.options = in.mrpt_icp_params;
    if (params_.decimate_to_point_count > 0)
    {
        unsigned decim = static_cast<unsigned>(
            in.to_pc->size() / params_.decimate_to_point_count);
        mrpt_icp.options.corresponding_points_decimation = decim;
    }
    else
    {
        mrpt_icp.options.corresponding_points_decimation = 1;
    }

    // Ensure kd-trees are built.
    // kd-trees have each own mutexes to ensure well-defined behavior:
    in.from_pc->kdTreeEnsureIndexBuilt3D();
    in.to_pc->kdTreeEnsureIndexBuilt3D();

    mrpt::poses::CPose3DPDFGaussian initial_guess;
    initial_guess.mean = mrpt::poses::CPose3D(in.init_guess_to_wrt_from);
    mrpt::slam::CICP::TReturnInfo ret_info;

    out.found_pose_to_wrt_from = mrpt_icp.Align3DPDF(
        in.from_pc.get(), in.to_pc.get(), initial_guess,
        nullptr /*running_time*/, &ret_info);

    out.goodness = static_cast<double>(ret_info.goodness);
    MRPT_LOG_DEBUG_FMT(
        "MRPT ICP: goodness=%.03f iters=%u rel_pose=%s", out.goodness,
        ret_info.nIterations,
        out.found_pose_to_wrt_from->getMeanVal().asString().c_str());
    MRPT_LOG_DEBUG_STREAM(
        "MRPT ICP: `to` point count="
        << in.to_pc->size() << " `from` point count=" << in.from_pc->size()
        << " decimation=" << mrpt_icp.options.corresponding_points_decimation);

    // -------------------------------------------------
    // Save debug files for debugging ICP quality
    if (params_.debug_save_all_icp_results)
    {
        auto fil_name_prefix = mrpt::system::fileNameStripInvalidChars(
            getModuleInstanceName() + mrpt::format(
                                          "_debug_ICP_%s_%05u",
                                          in.debug_str.c_str(),
                                          debug_dump_icp_file_counter++));

        // Init:
        mrpt::opengl::COpenGLScene scene;

        scene.insert(mrpt::opengl::stock_objects::CornerXYZSimple(2.0f, 4.0f));
        auto gl_from                    = mrpt::opengl::CSetOfObjects::Create();
        in.from_pc->renderOptions.color = mrpt::img::TColorf(.0f, .0f, 1.0f);
        in.from_pc->getAs3DObject(gl_from);
        gl_from->setName("KF_from"s);
        gl_from->enableShowName();
        scene.insert(gl_from);

        auto gl_to = mrpt::opengl::CSetOfObjects::Create();
        gl_to->insert(mrpt::opengl::stock_objects::CornerXYZSimple(1.0f, 2.0f));
        in.to_pc->renderOptions.color = mrpt::img::TColorf(1.0f, .0f, .0f);
        in.to_pc->getAs3DObject(gl_to);
        gl_to->setName("KF_to"s);
        gl_to->enableShowName();
        gl_to->setPose(initial_guess.mean);
        scene.insert(gl_to);

        auto gl_info  = mrpt::opengl::CText::Create(),
             gl_info2 = mrpt::opengl::CText::Create();
        gl_info->setLocation(0., 0., 5.);
        gl_info2->setLocation(0., 0., 4.8);
        scene.insert(gl_info);
        scene.insert(gl_info2);

        {
            std::ostringstream ss;
            ss << "to_ID     = " << in.to_id << " from_ID   = " << in.from_id
               << " | " << in.debug_str;
            gl_info->setString(ss.str());
        }
        {
            std::ostringstream ss;
            ss << "init_pose = " << initial_guess.mean.asString();
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
        const auto final_pose = out.found_pose_to_wrt_from->getMeanVal();
        gl_to->setPose(final_pose);

        {
            std::ostringstream ss;
            ss << "to_ID     = " << in.to_id << " from_ID   = " << in.from_id;
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
            mrpt::obs::CRawlog rawlog;
            {
                auto pc1         = mrpt::obs::CObservationPointCloud::Create();
                pc1->pointcloud  = in.from_pc;
                pc1->sensorLabel = "from";
                rawlog.addObservationMemoryReference(pc1);
            }
            {
                auto pc2         = mrpt::obs::CObservationPointCloud::Create();
                pc2->pointcloud  = in.to_pc;
                pc2->sensorLabel = "to";
                rawlog.addObservationMemoryReference(pc2);
            }
            {
                mrpt::config::CConfigFileMemory cfg;
                in.mrpt_icp_params.saveToConfigFile(cfg, "ICP");

                auto comm  = mrpt::obs::CObservationComment::Create();
                comm->text = cfg.getContent();
                rawlog.addObservationMemoryReference(comm);
            }

            const auto fil_name_rawlog = fil_name_prefix + ".rawlog"s;
            if (rawlog.saveToRawLogFile(fil_name_rawlog))
                MRPT_LOG_DEBUG_STREAM(
                    "Wrote ICP debug rawlog: " << fil_name_rawlog);
        }
    }

    MRPT_END
}

void LidarICP::filterPointCloud(mrpt::maps::CPointsMap& pc) const
{
    MRPT_START

    static bool                 ini = false;
    static CPointCloudVoxelGrid grid;
    if (!ini)
    {
        ProfilerEntry tle(profiler_, "filterPointCloud_initialize");
        ini = true;
        grid.resize(
            {-50.0, -50.0, -10.0}, {50.0, 50.0, 10.0},
            params_.voxel_filter_resolution);
    }

    grid.clear();
    grid.processPointCloud(pc);

    const auto& xs = pc.getPointsBufferRef_x();
    const auto& ys = pc.getPointsBufferRef_y();
    const auto& zs = pc.getPointsBufferRef_z();

    mrpt::maps::CSimplePointsMap filtered_points;

    std::size_t nGoodVoxels = 0, nTotalVoxels = 0;
    for (const auto& vxl_pts : grid.pts_voxels)
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
            const mrpt::math::TPoint3Df a{
                xs[pt_idx] - mean.x, ys[pt_idx] - mean.y, zs[pt_idx] - mean.z};
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

        if (e2 < params_.voxel_filter_max_e2_e0 * e0 &&
            e1 < params_.voxel_filter_max_e1_e0 * e0)
        {
            nGoodVoxels++;
            for (size_t i = 0; i < vxl_pts.indices.size(); i++)
            {
                const auto pt_idx = vxl_pts.indices[i];
                filtered_points.insertPointFast(
                    xs[pt_idx], ys[pt_idx], zs[pt_idx]);
            }
        }
    }
    MRPT_LOG_DEBUG_STREAM(
        "VoxelGridFilter: good voxels=" << nGoodVoxels << " out of "
                                        << nTotalVoxels);

    // Replace original -> filtered pointcloud:
    MRPT_TODO("impl std::move for pointclouds");
    //*this_obs_points = std::move(filtered_points);
    pc.clear();
    pc.addFrom(filtered_points);

    MRPT_END
}
