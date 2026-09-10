#ifndef M1_LOCAL_FAST2D__RESTRICTED_MPPI_INTERNAL_HPP_
#define M1_LOCAL_FAST2D__RESTRICTED_MPPI_INTERNAL_HPP_

// Humble 1.1.20 exposes no extension point between MPPIController and its
// concrete Optimizer.  This deliberately narrow access shim is the project
// seam: it changes access declarations only while compiling the adapter and
// never copies or modifies Nav2 sources.  It is guarded at runtime by the
// installed nav2_mppi_controller package version.
#define protected public
#include "nav2_mppi_controller/controller.hpp"
#undef protected

#endif
