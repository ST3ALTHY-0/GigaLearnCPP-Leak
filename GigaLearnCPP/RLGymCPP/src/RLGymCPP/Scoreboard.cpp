#include "Scoreboard.h"
#include <algorithm>
#include <iostream>
#include <cstdio>

namespace RLGC {

Scoreboard::Scoreboard(bool random_resets_, int tick_skip_, int max_time_seconds_, bool skip_warning)
    : random_resets(random_resets_), tick_skip(tick_skip_), max_time_seconds(max_time_seconds_) {
    std::random_device rd;
    rng.seed(rd());
    if (!skip_warning) {
        std::printf("Scoreboard: storing canonical score in GameState::score.\n");
    }
}

Scoreboard::~Scoreboard() {}

void Scoreboard::CountGoalsFromState(const GameState* s, int outGoals[2]) const {
    outGoals[0] = outGoals[1] = 0;
    const GameState* cur = s;
    while (cur) {
        if (cur->goalScored) {
            Team scoring = RS_TEAM_FROM_Y(-cur->ball.pos.y);
            outGoals[(int)scoring]++;
        }
        cur = cur->prev;
    }
}

std::pair<int,int> Scoreboard::CountGoalsPair(const GameState* s) const {
    int g[2];
    CountGoalsFromState(s, g);
    return {g[0], g[1]};
}

void Scoreboard::reset(GameState& initial_state) {
    state = &initial_state;
    int players_per_team = (int)initial_state.players.size() / 2;

    if (random_resets) {
        double gpm = GOALS_PER_MIN[std::max(0, std::min(2, players_per_team - 1))];
        double mu_full = gpm * max_time_seconds / SECONDS_PER_MINUTE;
        std::poisson_distribution<int> pois_full(mu_full);
        int full_b = pois_full(rng);
        int full_o = pois_full(rng);

        if (full_b == full_o) {
            ticks_left = std::numeric_limits<double>::infinity();
            scoreline = {full_b, full_o};
        } else {
            int max_ticks = max_time_seconds * TICKS_PER_SECOND;
            std::uniform_int_distribution<int> dist_ticks(0, std::max(0, max_ticks - 1));
            ticks_left = dist_ticks(rng);
            double seconds_spent = max_time_seconds - ticks_left / (double)TICKS_PER_SECOND;
            double mu_spent = gpm * seconds_spent / SECONDS_PER_MINUTE;
            std::poisson_distribution<int> pois_spent(mu_spent);
            int b = pois_spent(rng);
            int o = pois_spent(rng);
            scoreline = {b, o};
        }
    } else {
        scoreline = {0,0};
        ticks_left = (double)max_time_seconds * TICKS_PER_SECOND;
    }

    modify_gamestate(initial_state);
}

void Scoreboard::step(GameState& s, bool update_scores) {
    if (&s != state) {
        // Don't count during kickoffs
        if (s.ball.pos.y != 0) {
            if (std::isfinite(ticks_left)) {
                ticks_left = std::max(0.0, ticks_left - tick_skip);
            }
        }

        if (update_scores) {
            auto [b, o] = scoreline;
            bool changed = false;
            auto oldScore = scoreline;

            // Compare counted goals between current and previous stored state
            if (state) {
                auto prevScores = CountGoalsPair(state);
                auto curScores = CountGoalsPair(&s);
                if (curScores.first > prevScores.first) {
                    b += curScores.first - prevScores.first;
                    changed = true;
                }
                if (curScores.second > prevScores.second) {
                    o += curScores.second - prevScores.second;
                    changed = true;
                }
            }

            bool tied = (b == o);
            if (is_overtime()) {
                if (!tied) {
                    ticks_left = -std::numeric_limits<double>::infinity(); // finished
                }
            }

            if ((ticks_left <= 0 || !std::isfinite(ticks_left)) && (s.ball.pos.z <= 110.0f || changed)) {
                if (tied) {
                    ticks_left = std::numeric_limits<double>::infinity(); // overtime
                } else {
                    ticks_left = -std::numeric_limits<double>::infinity(); // finished
                }
            }

            scoreline = {b,o};

            // If the score changed, print a debug message with context
            if (changed || scoreline != oldScore) {
                // Print previous -> new, where (blue,orange). Include match tick info if available.
                std::fprintf(stderr, "Scoreboard DEBUG: score changed %d,%d -> %d,%d at lastTick=%llu matchTick=%llu\n",
                    oldScore.first, oldScore.second, scoreline.first, scoreline.second,
                    (unsigned long long)s.lastTickCount, (unsigned long long)s.matchTickCount);
            }
        }

        state = &s;
    }

    modify_gamestate(s);
}

// void Scoreboard::modify_gamestate(GameState& s) {
//     // Write the canonical score into the GameState.score field. We no longer
//     // encode scoreboard/time into ball.angVel.
//     s.score[0] = scoreline.first;
//     s.score[1] = scoreline.second;
// }

// bool Scoreboard::is_overtime() const {
//     return (ticks_left > 0.0) && std::isinf(ticks_left);
// }

// bool Scoreboard::is_finished() const {
//     return (ticks_left < 0.0) && std::isinf(ticks_left);
// }

double Scoreboard::win_prob() const {
    int players_per_team = 1;
    if (state) players_per_team = (int)state->players.size() / 2;
    double time_left_seconds = ticks_left;
    if (std::isfinite(ticks_left)) time_left_seconds = ticks_left / (double)TICKS_PER_SECOND;
    int diff = scoreline.first - scoreline.second;
    return win_prob_static(players_per_team, time_left_seconds, diff);
}

// --- static helper implementation ---
static double normal_cdf(double x, double mean, double stddev) {
    double z = (x - mean) / (stddev * std::sqrt(2.0));
    return 0.5 * (1.0 + std::erf(z));
}

static double normal_pmf_approx(int k, double mean, double var) {
    double stddev = std::sqrt(var);
    double left = normal_cdf(k - 0.5, mean, stddev);
    double right = normal_cdf(k + 0.5, mean, stddev);
    return std::max(0.0, right - left);
}

double Scoreboard::win_prob_static(int players_per_team, double time_left_seconds, int differential) {
    // Heuristic port of the python function using normal/skellam approx for speed
    using std::isfinite;

    double goal_floor_ratio = (CommonValues::GOAL_HEIGHT * 880.0) / (2.0 * (CommonValues::GOAL_HEIGHT * 880.0) + (4.0 * CommonValues::BACK_WALL_Y * CommonValues::SIDE_WALL_X - 1152.0 * 1152.0));

    double gpm = GOALS_PER_MIN[std::max(0, std::min(2, players_per_team - 1))];
    bool zero_seconds = (time_left_seconds <= 0.0) || !isfinite(time_left_seconds);

    double p = 0.0;

    if (!zero_seconds) {
        double mu_left = gpm * time_left_seconds / (double)SECONDS_PER_MINUTE;
        // Skellam with equal mus: mean 0, variance 2*mu_left
        double var = 2.0 * mu_left;
        // CDF at (differential - 2)
        double cdf_val = normal_cdf(differential - 2 + 0.5, 0.0, std::sqrt(var));
        p += cdf_val;

        // pmf at diff
        double pmf_diff = normal_pmf_approx(differential, 0.0, var);
        p += pmf_diff * 0.5; // tied -> win in OT with p=0.5

        // pmf at diff-1
        double pmf_d_minus_1 = normal_pmf_approx(differential - 1, 0.0, var);
        p += pmf_d_minus_1 * (1.0 - goal_floor_ratio * 0.5);

        // pmf at diff+1
        double pmf_d_plus_1 = normal_pmf_approx(differential + 1, 0.0, var);
        p += pmf_d_plus_1 * (goal_floor_ratio * 0.5);
    } else {
        // zero seconds (or infinite time marker in python): handle deterministically
        if (differential >= 2) p += 1.0;
        if (differential == 0) p += 0.5;
        if (differential == 1) p += (1.0 - goal_floor_ratio * 0.5);
        if (differential == -1) p += (goal_floor_ratio * 0.5);
    }

    if (p < 0.0) p = 0.0;
    if (p > 1.0) p = 1.0;
    return p;
}

}
