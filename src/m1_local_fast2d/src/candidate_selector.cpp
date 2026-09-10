#include "m1_local_fast2d/candidate_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace m1_local_fast2d
{
namespace
{
double directed(const msg::TimedTrajectory & a, const msg::TimedTrajectory & b)
{
  if (a.points.empty() || b.points.empty()) {return std::numeric_limits<double>::infinity();}
  double total = 0.0;
  for (const auto & p : a.points) {
    double best = std::numeric_limits<double>::infinity();
    for (const auto & q : b.points) {best = std::min(best, std::hypot(p.x - q.x, p.y - q.y));}
    total += best;
  }
  return total / a.points.size();
}
const msg::LocalTrajectoryCandidate * primary(const msg::LocalTrajectoryCandidateArray & set)
{
  for (const auto & c : set.candidates) {if (c.candidate_index == 0 && c.valid) {return &c;}}
  return set.candidates.empty() ? nullptr : &set.candidates.front();
}
}  // namespace

double CandidateSelector::routeDistance(const msg::TimedTrajectory & a, const msg::TimedTrajectory & b)
{
  return 0.5 * (directed(a, b) + directed(b, a));
}

SelectionResult CandidateSelector::select(const msg::LocalTrajectoryCandidateArray & set,
  const msg::CandidateRiskScoreArray * scores, const msg::TimedTrajectory * previous,
  const CandidateSelectorConfig & cfg, bool scores_fresh, bool scores_causal, bool scores_frame_valid)
{
  SelectionResult out; out.planning_result_id = set.planning_result_id;
  const auto * fallback = primary(set);
  if (!fallback) {out.selection_reason = msg::SelectedLocalTrajectory::FALLBACK_PRIMARY; return out;}
  out.selected_candidate_index = fallback->candidate_index;
  out.selection_reason = msg::SelectedLocalTrajectory::FALLBACK_PRIMARY;
  std::unordered_map<uint32_t, const msg::CandidateRiskScore *> mapped;
  bool malformed = scores == nullptr || scores->planning_result_id != set.planning_result_id ||
    scores->scores.size() != set.candidates.size();
  if (!malformed) {
    for (const auto & score : scores->scores) {
      if (score.planning_result_id != set.planning_result_id || mapped.count(score.candidate_index)) {malformed = true; break;}
      mapped[score.candidate_index] = &score;
    }
    for (const auto & c : set.candidates) {if (!mapped.count(c.candidate_index)) {malformed = true; break;}}
  }
  if (malformed) {out.selection_reason = scores ? msg::SelectedLocalTrajectory::FALLBACK_SCORE_MISMATCH : msg::SelectedLocalTrajectory::FALLBACK_NO_SCORE; return out;}
  if (!scores_frame_valid) {out.selection_reason = msg::SelectedLocalTrajectory::FALLBACK_FRAME_MISMATCH; return out;}
  if (!scores_fresh || !scores_causal) {out.selection_reason = msg::SelectedLocalTrajectory::FALLBACK_STALE; return out;}
  bool any_usable = false;
  struct Item {const msg::LocalTrajectoryCandidate * candidate; const msg::CandidateRiskScore * score; double raw; bool matched;};
  std::vector<Item> usable;
  for (const auto & c : set.candidates) {
    const auto * score = mapped[c.candidate_index];
    const bool usable_score = c.valid && score->score_valid && score->coverage_sufficient &&
      score->coverage_ratio >= cfg.minimum_coverage_ratio && score->valid_query_count >= static_cast<uint32_t>(cfg.minimum_valid_queries);
    const bool matched = previous && routeDistance(c.trajectory, *previous) <= cfg.route_match_threshold;
    CandidateSelectionDiagnostic d; d.candidate_index = c.candidate_index; d.usable = usable_score; d.matches_previous_route = matched;
    if (usable_score) {
      d.raw_cost = cfg.w_mean * score->mean_risk + cfg.w_max * score->max_risk + cfg.w_high * score->high_risk_fraction +
        (c.candidate_index == 0 ? 0.0 : cfg.baseline_preference);
      usable.push_back({&c, score, d.raw_cost, matched}); any_usable = true;
    }
    out.diagnostics.push_back(d);
  }
  if (!any_usable) {out.selection_reason = msg::SelectedLocalTrajectory::FALLBACK_LOW_COVERAGE; return out;}
  // The previously selected physical route, if represented in this cycle, is
  // the hysteresis reference; indices deliberately have no continuity meaning.
  const Item * current = nullptr;
  for (const auto & i : usable) {if (i.matched) {current = &i; out.previous_matched_candidate = i.candidate->candidate_index; break;}}
  const Item * best = &*std::min_element(usable.begin(), usable.end(), [](const Item & a, const Item & b) {return a.raw < b.raw;});
  const bool route_unavailable = previous && !current;
  const Item * chosen = best;
  if (current && best != current) {
    const double with_switch = best->raw + cfg.switch_penalty;
    if (current->raw - with_switch <= cfg.switch_margin) {chosen = current;}
  }
  for (auto & d : out.diagnostics) {
    auto it = std::find_if(usable.begin(), usable.end(), [&d](const Item & x) {return x.candidate->candidate_index == d.candidate_index;});
    if (it != usable.end()) {d.raw_cost = it->raw; d.switch_penalty = (current && !it->matched) ? cfg.switch_penalty : 0.0; d.final_cost = d.raw_cost + d.switch_penalty;}
  }
  out.selected_candidate_index = chosen->candidate->candidate_index;
  out.selection_cost = chosen->raw + ((current && !chosen->matched) ? cfg.switch_penalty : 0.0);
  out.prediction_used = true; out.selection_valid = true;
  out.switched = previous && !chosen->matched;
  out.selection_reason = route_unavailable && out.switched ? msg::SelectedLocalTrajectory::CURRENT_ROUTE_UNAVAILABLE : msg::SelectedLocalTrajectory::SELECTED_PREDICTIVE;
  return out;
}
}  // namespace m1_local_fast2d
