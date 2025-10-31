#pragma once
#include "TerminalCondition.h"

namespace RLGC {
    //meant to help train bots to do quick kickoff bc those bastards love not touching the ball first
	class FirstTouchCondition : public TerminalCondition {
	public:

		bool firstTouched = false; // becomes true when the ball is first touched after reset

        FirstTouchCondition() : firstTouched(false) {}

		virtual void Reset(const GameState& initialState) {
			firstTouched = false;
		};

		// Terminate immediately when any player first touches the ball.
		virtual bool IsTerminal(const GameState& currentState) {
			for (auto& player : currentState.players) {
				if (player.ballTouchedStep) {
					return true;
				}
			}
			return false;
		}

		virtual bool IsTruncation() {
			return true;
		}
	};
}