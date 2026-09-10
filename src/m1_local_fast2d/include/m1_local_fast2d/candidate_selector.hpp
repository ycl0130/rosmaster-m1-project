#ifndef M1_LOCAL_FAST2D__CANDIDATE_SELECTOR_HPP_
#define M1_LOCAL_FAST2D__CANDIDATE_SELECTOR_HPP_

#include <optional>
#include <vector>

#include "m1_local_fast2d/msg/candidate_risk_score_array.hpp"
#include "m1_local_fast2d/msg/local_trajectory_candidate_array.hpp"
#include "m1_local_fast2d/msg/selected_local_trajectory.hpp"

namespace m1_local_fast2d
{
struct CandidateSelectorConfig
{
  double w_mean{1.0};
  double w_max{1.0};
  double w_high{0.5};
  double baseline_preference{0.05};
  double switch_margin{0.10};
  double switch_penalty{0.10};
  double route_match_threshold{0.30};
  double minimum_coverage_ratio{0.25};
  int minimum_valid_queries{3};
};

struct CandidateSelectionDiagnostic
{
  uint32_t candidate_index{0}; bool usable{false}; bool matches_previous_route{false};
  double raw_cost{0.0}; double switch_penalty{0.0}; double final_cost{0.0};
};
struct SelectionResult
{
  uint64_t planning_result_id{0}; uint32_t selected_candidate_index{0};
  std::optional<uint32_t> previous_matched_candidate; bool switched{false};
  bool prediction_used{false}; bool selection_valid{false}; uint8_t selection_reason{0};
  double selection_cost{0.0}; std::vector<CandidateSelectionDiagnostic> diagnostics;
};

class CandidateSelector
{
public:
  static SelectionResult select(const msg::LocalTrajectoryCandidateArray & candidates,
    const msg::CandidateRiskScoreArray * scores,
    const msg::TimedTrajectory * previous, const CandidateSelectorConfig & config,
    bool scores_fresh = true, bool scores_causal = true, bool scores_frame_valid = true);
  static double routeDistance(const msg::TimedTrajectory & a, const msg::TimedTrajectory & b);
};
}  // namespace m1_local_fast2d
#endif
