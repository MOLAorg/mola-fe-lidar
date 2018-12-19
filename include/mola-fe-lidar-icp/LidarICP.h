/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */
/**
 * @file   LidarICP.h
 * @brief  Simple SLAM FrontEnd for point-cloud sensors via ICP registration
 * @author Jose Luis Blanco Claraco
 * @date   Dec 17, 2018
 */
#pragma once

#include <mola-kernel/FrontEndBase.h>
#include <mola-kernel/WorkerThreadsPool.h>
#include <mrpt/maps/CPointsMap.h>
#include <mrpt/slam/CICP.h>

namespace mola
{
/** A front-end for Lidar/point-cloud odometry & SLAM.
 *
 * \ingroup mola_fe_lidar_icp_grp */
class LidarICP : public FrontEndBase
{
   public:
    LidarICP();
    virtual ~LidarICP() override = default;

    // See docs in base class
    void initialize(const std::string& cfg_block) override;
    void spinOnce() override;
    void onNewObservation(CObservation::Ptr& o) override;

    /** Re-initializes the front-end */
    void reset();

    struct Parameters
    {
        /** Minimum time (seconds) between scans for  */
        double min_time_between_scans_{0.2};

        mrpt::slam::CICP::TConfigParams mrpt_icp{};
    };

    /** Algorithm parameters */
    Parameters params_;

   private:
    /** The worker thread pool with 1 thread for processing incomming scans */
    mola::WorkerThreadsPool worker_pool_{1};

    /** All variables that hold the algorithm state */
    struct MethodState
    {
        mrpt::Clock::time_point     last_obs_tim{};
        mrpt::maps::CPointsMap::Ptr last_points{};
        CObservation::Ptr           last_obs{};
        mrpt::slam::CICP            mrpt_icp;
        mrpt::math::TTwist3D        last_iter_twist;
    };

    MethodState state_;

    /** Here happens the actual processing, invoked from the worker thread pool
     * for each incomming observation */
    void doProcessNewObservation(CObservation::Ptr& o);
};

}  // namespace mola
