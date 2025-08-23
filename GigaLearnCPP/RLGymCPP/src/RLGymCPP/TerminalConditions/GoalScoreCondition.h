#pragma once
#include "TerminalCondition.h"
#include "../Math.h"

namespace RLGC {
	class GoalScoreCondition : public TerminalCondition {
	public:
		virtual bool IsTerminal(const GameState& currentState) {
			return currentState.goalScored;
		}

		virtual bool IsTruncation() {
			return false;
		}
	};
}