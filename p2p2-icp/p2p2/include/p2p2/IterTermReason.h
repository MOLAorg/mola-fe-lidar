/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 * Copyright (C) 2018-2019 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

namespace p2p2
{
/** Reason of iterating termination */
enum class IterTermReason
{
    Undefined = 0,
    NoPairings,
    MaxIterations,
    Stalled
};

}  // namespace p2p2
