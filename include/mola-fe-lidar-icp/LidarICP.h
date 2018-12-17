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

namespace mola
{
/**
 * \ingroup mola_fe_lidar_icp_grp */
class LidarICP : public FrontEndBase
{
   public:
    LidarICP();

    // See docs in base class
    void initialize(const std::string& cfg_block) override;
    void spinOnce() override;
    void onNewObservation(CObservation::Ptr& o) override;

   private:
};

}  // namespace mola
