#include "EnvSet.h"
#include "../Rewards/ZeroSumReward.h"
#include <typeinfo>

template <bool RLGC::PlayerEventState::*DATA_VAR>
void IncPlayerCounter(Car *car, void *userInfoPtr)
{
	if (!car)
		return;

	auto userInfo = (RLGC::EnvSet::CallbackUserInfo *)userInfoPtr;

	auto &gs = userInfo->envSet->state.gameStates[userInfo->arenaIdx];
	for (auto &player : gs.players)
		if (player.carId == car->id)
			(player.eventState.*DATA_VAR) = true;
}

void _ShotEventCallback(Arena *arena, Car *shooter, Car *passer, void *userInfo)
{
	IncPlayerCounter<&RLGC::PlayerEventState::shot>(shooter, userInfo);
	IncPlayerCounter<&RLGC::PlayerEventState::shotPass>(passer, userInfo);
}

void _GoalEventCallback(Arena *arena, Car *scorer, Car *passer, void *userInfo)
{
	IncPlayerCounter<&RLGC::PlayerEventState::goal>(scorer, userInfo);
	IncPlayerCounter<&RLGC::PlayerEventState::assist>(passer, userInfo);
}

void _SaveEventCallback(Arena *arena, Car *saver, void *userInfo)
{
	IncPlayerCounter<&RLGC::PlayerEventState::save>(saver, userInfo);
}

void _BumpCallback(Arena *arena, Car *bumper, Car *victim, bool isDemo, void *userInfo)
{
	if (bumper->team == victim->team)
		return;

	IncPlayerCounter<&RLGC::PlayerEventState::bump>(bumper, userInfo);
	IncPlayerCounter<&RLGC::PlayerEventState::bumped>(victim, userInfo);

	if (isDemo)
	{
		IncPlayerCounter<&RLGC::PlayerEventState::demo>(bumper, userInfo);
		IncPlayerCounter<&RLGC::PlayerEventState::demoed>(victim, userInfo);
	}
}

/////////////////////////////

RLGC::EnvSet::EnvSet(const EnvSetConfig &config) : config(config)
{

	RG_ASSERT(config.tickSkip > 0);
	RG_ASSERT(config.actionDelay >= 0 && config.actionDelay <= config.tickSkip);

	std::mutex appendMutex = {};
	auto fnCreateArenas = [&](int idx)
	{
		auto createResult = config.envCreateFn(idx);
		auto arena = createResult.arena;

		appendMutex.lock();
		{
			arenas.push_back(arena);

			auto userInfo = new CallbackUserInfo();
			userInfo->arena = arena;
			userInfo->arenaIdx = idx;
			userInfo->envSet = this;
			eventCallbackInfos.push_back(userInfo);
			arena->SetCarBumpCallback(_BumpCallback, userInfo);

			if (arena->gameMode != GameMode::HEATSEEKER)
			{
				GameEventTracker *tracker = new GameEventTracker({});
				eventTrackers.push_back(tracker);

				tracker->SetShotCallback(_ShotEventCallback, userInfo);
				tracker->SetGoalCallback(_GoalEventCallback, userInfo);
				tracker->SetSaveCallback(_SaveEventCallback, userInfo);
			}
			else
			{
				eventTrackers.push_back(NULL);
				eventCallbackInfos.push_back(NULL);
			}

			userInfos.push_back(createResult.userInfo);

			rewards.push_back(createResult.rewards);
			terminalConditions.push_back(createResult.terminalConditions);
			obsBuilders.push_back(createResult.obsBuilder);
			actionParsers.push_back(createResult.actionParser);
			stateSetters.push_back(createResult.stateSetter);
		}
		appendMutex.unlock();
	};
	g_ThreadPool.StartBatchedJobs(fnCreateArenas, config.numArenas, false);

	state.Resize(arenas);

	// Ensure newly-created GameState entries have a valid per-match start tick so
	// match-local timing (matchTickCount) is available from the very first game.
	// Without this, the first game would have matchStartTick==0 and matchTickCount
	// would remain zero until ResetArena runs later.
	for (int i = 0; i < arenas.size(); i++)
	{
		state.gameStates[i].matchStartTick = arenas[i]->tickCount;
		state.gameStates[i].matchTickCount = 0;
	}

	// Determine obs size and action amount, initialize arrays accordingly
	{
		stateSetters[0]->ResetArena(arenas[0]);
		GameState testState = GameState(arenas[0]);
		testState.userInfo = userInfos[0];
		obsBuilders[0]->Reset(testState);
		obsSize = obsBuilders[0]->BuildObs(testState.players[0], testState).size();
		state.obs = DimList2<float>(state.numPlayers, obsSize);

		state.actionMasks = DimList2<uint8_t>(state.numPlayers, actionParsers[0]->GetActionAmount());
	}

	// Reset all arenas initially
	g_ThreadPool.StartBatchedJobs(
		std::bind(&RLGC::EnvSet::ResetArena, this, std::placeholders::_1),
		config.numArenas, false);
}

void RLGC::EnvSet::StepFirstHalf(bool async)
{

	auto fnStepArena = [&](int arenaIdx)
	{
		Arena *arena = arenas[arenaIdx];
		auto &gs = state.gameStates[arenaIdx];

		{
			// Set previous gamestates
			state.prevGameStates[arenaIdx] = gs;
		}

		gs.ResetBeforeStep();

		// Step arena with old actions
		arena->Step(config.actionDelay);
	};

	g_ThreadPool.StartBatchedJobs(fnStepArena, arenas.size(), async);
}

void RLGC::EnvSet::StepSecondHalf(const IList &actionIndices, bool async)
{

	auto fnStepArenas = [&](int arenaIdx)
	{
		Arena *arena = arenas[arenaIdx];
		auto &gs = state.gameStates[arenaIdx];
		int playerStartIdx = state.arenaPlayerStartIdx[arenaIdx];

		// Parse and set actions
		auto actions = std::vector<Action>(gs.players.size());
		auto carItr = arena->_cars.begin();
		for (int i = 0; i < gs.players.size(); i++, carItr++)
		{
			auto &player = gs.players[i];
			Car *car = *carItr;
			Action action = actionParsers[arenaIdx]->ParseAction(actionIndices[playerStartIdx + i], player, gs);
			car->controls = (CarControls)action;
			actions[i] = action;
		}

		// Step arena with new actions we got from observing the last state
		// Update the gamestate after
		{
			arena->Step(config.tickSkip - config.actionDelay);

			if (eventTrackers[arenaIdx])
				eventTrackers[arenaIdx]->Update(arena);

			GameState *gsPrev = &state.prevGameStates[arenaIdx];
			if (gsPrev->IsEmpty())
				gsPrev = NULL;

			gs.UpdateFromArena(arena, actions, gsPrev);

			// If a goal was scored this step (and wasn't already true in the previous
			// state), update the canonical GameState::score directly so other
			// systems can read the score without relying on Scoreboard.
			if (gs.goalScored)
			{
				bool prevHadGoal = (gsPrev && gsPrev->goalScored);
				if (!prevHadGoal)
				{
					Team scoring = RS_TEAM_FROM_Y(-gs.ball.pos.y);
					if (scoring == Team::BLUE)
						gs.score[0]++;
					else
						gs.score[1]++;
					// fprintf(stderr, "EnvSet DEBUG: direct score update arena %d scoring=%s newScore=(%d,%d)\n",
					// 		arenaIdx, (scoring == Team::BLUE) ? "BLUE" : "ORANGE", gs.score[0], gs.score[1]);
				}
			}
		}

		// Update terminal
		uint8_t terminalType = TerminalType::NOT_TERMINAL;
		{
			for (auto cond : terminalConditions[arenaIdx])
			{
				if (cond->IsTerminal(gs))
				{
					bool isTrunc = cond->IsTruncation();
					uint8_t curTerminalType = isTrunc ? TerminalType::TRUNCATED : TerminalType::NORMAL;
					if (terminalType == TerminalType::NOT_TERMINAL)
					{
						terminalType = curTerminalType;
					}
					else
					{
						// We already know this state is terminal
						// However, if we only know it is a truncated terminal, we should let normal terminals take priority
						// (Normal terminals are better information than truncations)
						if (curTerminalType == TerminalType::NORMAL)
							terminalType = curTerminalType;
					}

					// NOTE: We can't break since terminal conditions are guaranteed to be called once per step
				}
			}
			state.terminals[arenaIdx] = terminalType;
		}

		// If this arena hit a terminal condition, print a detailed debug snapshot
		if (state.terminals[arenaIdx] != TerminalType::NOT_TERMINAL)
		{
			const char *termStr = state.terminals[arenaIdx] == TerminalType::NORMAL ? "NORMAL" : "TRUNCATED";
			// fprintf(stderr, "EnvSet TERMINAL DEBUG: arena %d terminal=%s arenaTick=%d matchTick=%llu players=%zu\n",
			// 		arenaIdx, termStr, arena->tickCount, (unsigned long long)gs.matchTickCount, gs.players.size());

			// Which terminal conditions fired? Print their index and RTTI name if available
			for (int ci = 0; ci < terminalConditions[arenaIdx].size(); ++ci)
			{
				auto cond = terminalConditions[arenaIdx][ci];
				if (cond->IsTerminal(gs))
				{
					// fprintf(stderr, "  terminalCond[%d] = %s trunc=%d\n", ci, typeid(*cond).name(), cond->IsTruncation() ? 1 : 0);
				}
			}

			int playerStartIdxDbg = state.arenaPlayerStartIdx[arenaIdx];
			for (int i = 0; i < gs.players.size(); ++i)
			{
				auto &p = gs.players[i];
				float rew = 0.0f;
				if (playerStartIdxDbg + i < state.rewards.size())
					rew = state.rewards[playerStartIdxDbg + i];
				// fprintf(stderr, "  player[%d]: carId=%d team=%d pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f) boost=%.1f reward=%.6f touch=%d\n",
				// 		i, p.carId, (int)p.team, p.pos.x, p.pos.y, p.pos.z, p.vel.x, p.vel.y, p.vel.z, p.boost, rew, p.ballTouchedStep ? 1 : 0);
			}

			// fprintf(stderr, "  score=(%d,%d)\n", gs.score[0], gs.score[1]);
		}

		// Pre-step rewards
		{
			for (auto &weighted : rewards[arenaIdx])
				weighted.reward->PreStep(gs);
		}

		// Update rewards
		{
			FList allRewards = FList(gs.players.size(), 0);
			for (int rewardIdx = 0; rewardIdx < rewards[arenaIdx].size(); rewardIdx++)
			{
				auto &weightedReward = rewards[arenaIdx][rewardIdx];
				FList output = weightedReward.reward->GetAllRewards(gs, terminalType);
				for (int i = 0; i < gs.players.size(); i++)
					allRewards[i] += output[i] * weightedReward.weight;

				// Save the reward
				if (config.saveRewards)
				{
					int playerSampleIndex;
					if (config.shuffleRewardSampling)
					{
						playerSampleIndex = Math::RandInt(0, output.size());
					}
					else
					{
						// Find player with the lowest id
						playerSampleIndex = 0;
						int lowestID = gs.players[0].carId;
						for (int i = 1; i < gs.players.size(); i++)
						{
							auto id = gs.players[i].carId;
							if (id < lowestID)
							{
								lowestID = id;
								playerSampleIndex = i;
							}
						}
					}
					// We will only take the reward from a random player
					float rewardToSave = output[playerSampleIndex];

					// If zero-sum, use the inner reward
					if (ZeroSumReward *zeroSum = dynamic_cast<ZeroSumReward *>(weightedReward.reward))
						rewardToSave = zeroSum->_lastRewards[playerSampleIndex];

					// If needed, initialize last rewards
					if (state.lastRewards[arenaIdx].empty())
						state.lastRewards[arenaIdx].resize(rewards[arenaIdx].size());

					state.lastRewards[arenaIdx][rewardIdx] = rewardToSave;
				}
			}

			for (int i = 0; i < gs.players.size(); i++)
				state.rewards[playerStartIdx + i] = allRewards[i];
		}

		// Update observations
		{
			for (int i = 0; i < gs.players.size(); i++)
				state.obs.Set(playerStartIdx + i, obsBuilders[arenaIdx]->BuildObs(gs.players[i], gs));
		}

		// Update action masks
		{
			for (int i = 0; i < gs.players.size(); i++)
				state.actionMasks.Set(playerStartIdx + i, actionParsers[arenaIdx]->GetActionMask(gs.players[i], gs));
		}

		// Debug: print game state, rewards, players, and score only when a goal is scored
		if (gs.goalScored)
		{

			// fprintf(stderr, "EnvSet TICK DEBUG (goal): arena %d arenaTick=%d matchTick=%llu players=%zu\n",
			// 		arenaIdx, arena->tickCount, (unsigned long long)gs.matchTickCount, gs.players.size());

			for (int i = 0; i < gs.players.size(); ++i)
			{
				auto &p = gs.players[i];
				float rew = 0.0f;
				if (playerStartIdx + i < state.rewards.size())
					rew = state.rewards[playerStartIdx + i];
			// 	fprintf(stderr, "  player[%d]: carId=%d team=%d pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f) boost=%.1f reward=%.6f touch=%d\n",
			// 			i, p.carId, (int)p.team, p.pos.x, p.pos.y, p.pos.z, p.vel.x, p.vel.y, p.vel.z, p.boost, rew, p.ballTouchedStep ? 1 : 0);
			 }

			// Score stored in GameState::score (blue, orange)
			// fprintf(stderr, "  score=(%d,%d)\n", gs.score[0], gs.score[1]);
		}

		// If a goal was scored this step and the episode is not terminal for another reason,
		// reset the arena to kickoff so play continues (match does not end).
		if (gs.goalScored && state.terminals[arenaIdx] == TerminalType::NOT_TERMINAL)
		{
			this->SoftResetArena(arenaIdx);
		}
	};

	g_ThreadPool.StartBatchedJobs(fnStepArenas, arenas.size(), async);
}

void RLGC::EnvSet::SoftResetArena(int index)
{
	Arena *arena = arenas[index];

	// Preserve a copy of the current state as the 'previous' state so UpdateFromArena
	//    can maintain the prev-chain (rewards and history kept).
	state.prevGameStates[index] = state.gameStates[index];
	GameState *gsPrev = &state.prevGameStates[index];
	if (gsPrev->IsEmpty())
		gsPrev = NULL;

	// Apply the state setter to reposition the arena objects (kickoff)
	stateSetters[index]->ResetArena(arena);

	// Update the existing gamestate in-place from the arena. Use an empty actions
	//    vector (no pending actions) because we're just reflecting the reset positions.
	auto &cur = state.gameStates[index];
	std::vector<Action> actions(cur.players.size());
	cur.UpdateFromArena(arena, actions, gsPrev);

	// Rebuild observations and action masks for players to reflect the new positions.
	int playerStartIdx = state.arenaPlayerStartIdx[index];
	for (int i = 0; i < cur.players.size(); i++)
	{
		auto obs = obsBuilders[index]->BuildObs(cur.players[i], cur);
		state.obs.Set(playerStartIdx + i, obs);

		auto actionMask = actionParsers[index]->GetActionMask(cur.players[i], cur);
		state.actionMasks.Set(playerStartIdx + i, actionMask);
	}

	// Note: We intentionally do NOT reset obsBuilders, rewards, terminal conditions,
	// or eventTrackers here because we want to preserve match-level state.
}

void RLGC::EnvSet::ResetArena(int index)
{
	stateSetters[index]->ResetArena(arenas[index]);
	GameState newState = GameState(arenas[index]);
	// Initialize per-match start tick so rewards using match-relative time work correctly
	newState.userInfo = userInfos[index];
	newState.matchStartTick = arenas[index]->tickCount;
	newState.matchTickCount = 0;
	state.gameStates[index] = newState;

	// Update event tracker
	if (eventTrackers[index])
		eventTrackers[index]->ResetPersistentInfo();

	// Reset all the other stuff
	obsBuilders[index]->Reset(newState);
	for (auto &cond : terminalConditions[index])
		cond->Reset(newState);
	for (auto &weightedReward : rewards[index])
		weightedReward.reward->Reset(newState);

	int playerStartIdx = state.arenaPlayerStartIdx[index];
	for (int i = 0; i < newState.players.size(); i++)
	{

		// Update obs
		auto obs = obsBuilders[index]->BuildObs(newState.players[i], newState);
		state.obs.Set(playerStartIdx + i, obs);

		// Update action mask
		auto actionMask = actionParsers[index]->GetActionMask(newState.players[i], newState);
		state.actionMasks.Set(playerStartIdx + i, actionMask);
	}

	// Remove previous state
	state.prevGameStates[index].MakeEmpty();
}

void RLGC::EnvSet::Reset()
{
	for (int i = 0; i < arenas.size(); i++)
		if (state.terminals[i])
			g_ThreadPool.StartJobAsync(std::bind(&EnvSet::ResetArena, this, std::placeholders::_1), i);
	std::fill(state.terminals.begin(), state.terminals.end(), 0);
	g_ThreadPool.WaitUntilDone();
}