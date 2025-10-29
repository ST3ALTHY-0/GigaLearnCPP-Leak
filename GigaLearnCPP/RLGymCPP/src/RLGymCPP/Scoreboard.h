#pragma once
#include <random>
#include <cmath>
#include <limits>
#include "Gamestates/GameState.h"
#include "CommonValues.h"

namespace RLGC {

class Scoreboard {
public:
    static constexpr int TICKS_PER_SECOND = 120;
    static constexpr int SECONDS_PER_MINUTE = 60;
    // stats for goals per minute by players per team (1,2,3 players)
    constexpr static double GOALS_PER_MIN[3] = {1.0, 0.6, 0.45};

    Scoreboard(bool random_resets = true, int tick_skip = 8, int max_time_seconds = 300, bool skip_warning = false);
    ~Scoreboard();

    void reset(GameState& initial_state);
    void step(GameState& state, bool update_scores = true);
    void modify_gamestate(GameState& state);

    bool is_overtime() const;
    bool is_finished() const;

    // Returns probability (0..1) that blue team will win (heuristic)
    double win_prob() const;

private:
    bool random_resets;
    int tick_skip;
    int max_time_seconds;

    double ticks_left = 0.0; // can be +inf for overtime or -inf for finished
    std::pair<int,int> scoreline = {0,0}; // blue, orange
    const GameState* state = nullptr; // last seen state pointer

    std::mt19937 rng;

    // Helpers
    void CountGoalsFromState(const GameState* s, int outGoals[2]) const;
    std::pair<int,int> CountGoalsPair(const GameState* s) const;

    // win probability helper (static): players_per_team, time_left_seconds, differential
    static double win_prob_static(int players_per_team, double time_left_seconds, int differential);
};

}
