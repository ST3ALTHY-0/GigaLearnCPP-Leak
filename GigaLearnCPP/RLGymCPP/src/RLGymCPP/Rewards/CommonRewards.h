#pragma once
#include "Reward.h"
#include "../Math.h"
#include <cstdio>
#include <cstdint>

namespace RLGC {

	template<bool PlayerEventState::* VAR, bool NEGATIVE>
	class PlayerDataEventReward : public Reward {
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			bool val =  player.eventState.*VAR;

			if (NEGATIVE) {
				return -(float)val;
			} else {
				return (float)val;
			}
		}
	};

	typedef PlayerDataEventReward<&PlayerEventState::goal, false> PlayerGoalReward; // NOTE: Given only to the player who last touched the ball on the opposing team
	typedef PlayerDataEventReward<&PlayerEventState::assist, false> AssistReward;
	typedef PlayerDataEventReward<&PlayerEventState::shot, false> ShotReward;
	typedef PlayerDataEventReward<&PlayerEventState::shotPass, false> ShotPassReward;
	typedef PlayerDataEventReward<&PlayerEventState::save, false> SaveReward;
	typedef PlayerDataEventReward<&PlayerEventState::bump, false> BumpReward;
	typedef PlayerDataEventReward<&PlayerEventState::bumped, true> BumpedPenalty;
	typedef PlayerDataEventReward<&PlayerEventState::demo, false> DemoReward;
	typedef PlayerDataEventReward<&PlayerEventState::demoed, true> DemoedPenalty;





	class ControlledTouchReward : public Reward {
public:
    virtual float GetReward(const Player& p, const GameState& s, bool) {
        if (!s.prev || !p.ballTouchedStep) return 0;

        Vec preDir = s.prev->ball.vel.Normalized();
        Vec postDir = s.ball.vel.Normalized();
        float angleDiff = preDir.Dot(postDir);
        return RS_CLAMP(angleDiff, 0, 1); // Reward straight, stable touches
    }
};
/*
a for the speed it produces # On ground it gets about 0.04 just for touching, as well as some extr
                # Ball is pretty close to z=150 when on top of car, so 1 second of dribbling is 1 reward
                # Close to 20 in the limit with ball on top, but opponents should learn to challenge way before that
                avg_height = 0.5 * (car_height + ball_height)
                h0 = self._height_activation(0)
                h1 = self._height_activation(CEILING_Z)
                hx = self._height_activation(avg_height)
                height_factor = ((hx - h0) / (h1 - h0)) ** 2
                wall_dist_factor = 1 - np.exp(-self.dist_to_closest_wall(*player.car_data.position[:2]) / CAR_MAX_SPEED)
                player_rewards[i] += self.touch_height_w * height_factor * (1 + wall_dist_factor)
                if player.has_flip and not last.has_flip \
                        and player.car_data.position[2] > 3 * BALL_RADIUS \
                        and np.linalg.norm(state.ball.position - player.car_data.position) < 2 * BALL_RADIUS \
                        and cosine_similarity(state.ball.position - player.car_data.position,
                                              -player.car_data.up()) > 0.9:
                    player_rewards[i] += self.flip_reset_w
*/

class ProximityToBallReward : public Reward {
public:
	// maxDistance defines the distance at which the reward becomes zero.
	// Reward = 1.0 when player is on the ball, linearly falls to 0.0 at maxDistance.
	float maxDistance;

	ProximityToBallReward(float maxDistance_ = 6000.0f) : maxDistance(maxDistance_) {}

	virtual float GetReward(const Player& player, const GameState& state, bool /*isFinal*/) override {
		float dist = (player.pos - state.ball.pos).Length();
		if (dist >= maxDistance) return 0.0f;
		float reward = 1.0f - (dist / maxDistance);
		return RS_CLAMP(reward, 0.0f, 1.0f);
	}
};

	class DribbleReward : public Reward {
	public:
		float touchHeightWeight;
		float flipResetWeight;
		
		DribbleReward(float touchHeightWeight = 1.0f, float flipResetWeight = 5.0f) 
			: touchHeightWeight(touchHeightWeight), flipResetWeight(flipResetWeight) {}

	private:
		// Height activation function - creates smooth curve for height rewards
		float HeightActivation(float height) {
			// Exponential-like activation that gives higher rewards for greater heights
			return 1.0f - expf(-height / 1000.0f); // Adjust divisor to tune activation curve
		}

		// Calculate distance to closest wall (simplified 2D calculation)
		float DistToClosestWall(float x, float y) {
			float distToSideWalls = RS_MIN(CommonValues::SIDE_WALL_X - abs(x), CommonValues::SIDE_WALL_X + abs(x));
			float distToBackWalls = RS_MIN(CommonValues::BACK_WALL_Y - abs(y), CommonValues::BACK_WALL_Y + abs(y));
			return RS_MIN(distToSideWalls, distToBackWalls);
		}

	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			// Only reward when player touches the ball
			if (!player.ballTouchedStep)
				return 0;

			float totalReward = 0;

			// Calculate average height between car and ball
			float carHeight = player.pos.z;
			float ballHeight = state.ball.pos.z;
			float avgHeight = 0.5f * (carHeight + ballHeight);

			// Height factor calculation using activation function
			float h0 = HeightActivation(0);
			float h1 = HeightActivation(CommonValues::CEILING_Z);
			float hx = HeightActivation(avgHeight);
			float heightFactor = powf((hx - h0) / (h1 - h0), 2.0f); // Squared for more dramatic curve

			// Wall distance factor - higher reward when away from walls (more space to dribble)
			float wallDist = DistToClosestWall(player.pos.x, player.pos.y);
			float wallDistFactor = 1.0f - expf(-wallDist / CommonValues::CAR_MAX_SPEED);

			// Base touch height reward
			totalReward += touchHeightWeight * heightFactor * (1.0f + wallDistFactor);

			// // Flip reset detection and reward
			// if (state.prev) {
			// 	// Find previous player state
			// 	const Player* prevPlayer = nullptr;
			// 	for (const auto& p : state.prev->players) {
			// 		if (p.index == player.index) {
			// 			prevPlayer = &p;
			// 			break;
			// 		}
			// 	}

			// 	if (prevPlayer) {
			// 		// Check if player gained flip (flip reset)
			// 		bool gainedFlip = player.hasFlip && !prevPlayer->hasFlip;
					
			// 		// Additional conditions for valid flip reset:
			// 		// 1. Player must be above 3 ball radii height
			// 		bool aboveMinHeight = player.pos.z > 3.0f * CommonValues::BALL_RADIUS;
					
			// 		// 2. Ball must be close to player (within 2 ball radii)
			// 		float distToBall = (state.ball.pos - player.pos).Length();
			// 		bool ballCloseEnough = distToBall < 2.0f * CommonValues::BALL_RADIUS;
					
			// 		// 3. Ball should be roughly underneath the car (cosine similarity > 0.9)
			// 		Vec ballToPlayer = player.pos - state.ball.pos;
			// 		Vec playerUp = player.rotMat.up; // Car's up vector
			// 		float alignment = ballToPlayer.Normalized().Dot(playerUp);
			// 		bool ballUnderneath = alignment > 0.9f;

			// 		if (gainedFlip && aboveMinHeight && ballCloseEnough && ballUnderneath) {
			// 			totalReward += flipResetWeight;
			// 		}
			// 	}
			// }

			return totalReward;
		}
	};

// Rewards hitting the ball higher and further from walls, with an increasing
// combo multiplier for successive in-air hits by the same player (aerial dribbling).
class AerialDribbleReward : public Reward {
public:
	// weights for height and wall distance components
	float heightWeight;
	float wallWeight;
	// per-additional-touch multiplier (e.g. 0.5 => second touch = 1.5x, third = 2.0x)
	float comboBonusPerTouch;
	// maximum multiplier cap
	float maxComboMultiplier;
	// lookback window (seconds) to consider previous touches part of the combo
	float lookbackSeconds;

	AerialDribbleReward(float heightWeight_ = 1.0f, float wallWeight_ = 1.0f, float comboBonusPerTouch_ = 0.5f,
						float maxComboMultiplier_ = 3.0f, float lookbackSeconds_ = 2.0f)
		: heightWeight(heightWeight_), wallWeight(wallWeight_), comboBonusPerTouch(comboBonusPerTouch_),
		  maxComboMultiplier(maxComboMultiplier_), lookbackSeconds(lookbackSeconds_) {}

private:
	// simple distance to nearest wall (uses field extents from CommonValues)
	float DistToClosestWall(const Vec& pos) const {
		float distToSide = RS_MIN(CommonValues::SIDE_WALL_X - fabsf(pos.x), CommonValues::SIDE_WALL_X + fabsf(pos.x));
		float distToBack = RS_MIN(CommonValues::BACK_WALL_Y - fabsf(pos.y), CommonValues::BACK_WALL_Y + fabsf(pos.y));
		return RS_MIN(distToSide, distToBack);
	}

public:
	virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
		// Only evaluate when the player just touched the ball
		if (!player.ballTouchedStep)
			return 0.0f;

		// Height component: average of car and ball height normalized to ceiling
		float avgHeight = 0.5f * (player.pos.z + state.ball.pos.z);
		float heightFrac = RS_CLAMP(avgHeight / CommonValues::CEILING_Z, 0.0f, 1.0f);

		// Wall distance factor: farther from walls => closer to 1
		float wallDist = DistToClosestWall(player.pos);
		float wallFactor = 1.0f - expf(-wallDist / CommonValues::CAR_MAX_SPEED);

		float base = heightWeight * heightFrac + wallWeight * wallFactor;

		// Count successive prior touches by this player while airborne within lookback window
		int comboCount = 1; // current touch
		float tickTime = 1.0f / 120.0f;
		if (state.lastArena)
			tickTime = state.lastArena->tickTime;
		int maxSteps = (int)ceil(lookbackSeconds / tickTime);

		const GameState* gs = &state;
		for (int s = 0; s < maxSteps && gs->prev; ++s) {
			gs = gs->prev;
			// find the player snapshot in this previous state
			const Player* prevP = nullptr;
			for (const auto& pp : gs->players) {
				if (pp.index == player.index) { prevP = &pp; break; }
			}
			if (!prevP) continue;

			// Only count prior touches that were airborne (dribble in air)
			if (prevP->ballTouchedStep && !prevP->isOnGround) {
				comboCount++;
			} else if (prevP->ballTouchedStep && prevP->isOnGround) {
				// Prior touch on ground breaks aerial combo
				break;
			}
		}

		float comboMultiplier = 1.0f + comboBonusPerTouch * (float)(comboCount - 1);
		if (comboMultiplier > maxComboMultiplier)
			comboMultiplier = maxComboMultiplier;

		float reward = base * comboMultiplier;
		// Keep reward in a reasonable range
		return RS_CLAMP(reward, 0.0f, maxComboMultiplier * (heightWeight + wallWeight));
	}
};

	class FlickGoalReward : public Reward {
	public:
		float minFlickSpeed;
		float maxTimeToGoal;
		
		FlickGoalReward(float minFlickSpeedKPH = 80.0f, float maxTimeToGoalSeconds = 3.0f) 
			: minFlickSpeed(RLGC::Math::KPHToVel(minFlickSpeedKPH)), maxTimeToGoal(maxTimeToGoalSeconds) {}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			// Only check when a goal is scored
			if (!state.goalScored)
				return 0;

			// Check if this player's team scored (not own goal)
			bool playerTeamScored = (player.team != RS_TEAM_FROM_Y(state.ball.pos.y));
			if (!playerTeamScored)
				return 0;

			// Need previous state to detect flick
			if (!state.prev || !state.prev->prev)
				return 0;

			// Look back through recent states to find a flick by this player
			const GameState* checkState = &state;
			float timeChecked = 0;
			
			for (int stepsBack = 0; stepsBack < (maxTimeToGoal * 120) && checkState->prev; stepsBack++) {
				checkState = checkState->prev;
				timeChecked += 1.0f / 120.0f; // Assuming 120 ticks per second
				
				if (timeChecked > maxTimeToGoal)
					break;

				// Find the player in the previous state
				const Player* prevPlayer = nullptr;
				for (const auto& p : checkState->players) {
					if (p.index == player.index) {
						prevPlayer = &p;
						break;
					}
				}
				
				if (!prevPlayer || !prevPlayer->ballTouchedStep)
					continue;

				// Check if this was a flick (ball accelerated significantly upward and forward)
				if (checkState->prev) {
					Vec ballVelChange = checkState->ball.vel - checkState->prev->ball.vel;
					float speedIncrease = ballVelChange.Length();
					float upwardComponent = ballVelChange.z;
					
					// Flick criteria: significant speed increase with upward component
					if (speedIncrease > minFlickSpeed && upwardComponent > minFlickSpeed * 0.3f) {
						// Additional check: ball was close to player (dribbling position)
						Vec ballToPlayer = prevPlayer->pos - checkState->prev->ball.pos;
						float distance = ballToPlayer.Length();
						
						if (distance < 200.0f) { // Ball was close enough to be a flick
							// Calculate reward based on flick quality and time to goal
							float speedReward = RS_MIN(1.0f, speedIncrease / (CommonValues::BALL_MAX_SPEED * 0.5f));
							float timeReward = 1.0f - (timeChecked / maxTimeToGoal);
							float upwardReward = RS_MIN(1.0f, upwardComponent / (minFlickSpeed * 0.5f));
							
							return (speedReward + timeReward + upwardReward) / 3.0f;
						}
					}
				}
			}
			
			return 0; // No flick detected before this goal
		}
	};

	/*
	MAX_TIME_IN_AIR = 1.75 # A rough estimate of the maximum reasonable aerial time
air_time_frac = min(player.air_time, MAX_TIME_IN_AIR) / MAX_TIME_IN_AIR
height_frac = ball.position[2] / CommonValues.CEILING_Z
reward = min(air_time_frac, height_frac)
*/

	// Rewards aerial play by combining player air time with ball height
	class AerialReward : public Reward {
	public:
		float maxAirTime;
		
		AerialReward(float maxAirTimeSeconds = 1.75f) 
			: maxAirTime(maxAirTimeSeconds) {}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			// Only reward when player is in the air
			if (player.isOnGround)
				return 0;

			// Calculate air time fraction (capped at max reasonable air time)
			float airTime = player.airTime;
			float airTimeFrac = RS_MIN(airTime, maxAirTime) / maxAirTime;
			
			// Calculate ball height fraction relative to ceiling
			float heightFrac = state.ball.pos.z / CommonValues::CEILING_Z;
			
			// Reward is the minimum of both fractions (both need to be high for good aerial play)
			// This encourages aerials when ball is high and player has been airborne for reasonable time
			return RS_MIN(airTimeFrac, heightFrac);
		}
	};



	// Rewards a goal by anyone on the team
	// NOTE: Already zero-sum
	class GoalReward : public Reward {
	public:
		float concedeScale;
		GoalReward(float concedeScale = -1) : concedeScale(concedeScale) {}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			if (!state.goalScored)
				return 0;

			bool scored = (player.team != RS_TEAM_FROM_Y(state.ball.pos.y));
			return scored ? 1 : concedeScale;
		}
	};

	// https://github.com/AechPro/rocket-league-gym-sim/blob/main/rlgym_sim/utils/reward_functions/common_rewards/misc_rewards.py
	class VelocityReward : public Reward {
	public:
		bool isNegative;
		VelocityReward(bool isNegative = false) : isNegative(isNegative) {}
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			return player.vel.Length() / CommonValues::CAR_MAX_SPEED * (1 - 2 * isNegative);
		}
	};

	class AlignBallToOpponentGoalReward : public Reward {
	public:
		// Maximum lateral offset from the ball->goal line (in UU) where reward falls to zero.
		// maxForwardDistance controls how far behind the ball (away from goal) we still consider alignment useful.
		float maxLateralDistance;
		float maxForwardDistance;

		AlignBallToOpponentGoalReward(float maxLateralDistance_ = 2000.0f, float maxForwardDistance_ = 6000.0f)
			: maxLateralDistance(maxLateralDistance_), maxForwardDistance(maxForwardDistance_) {}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			// Need valid ball position
			Vec ballPos = state.ball.pos;

			// Choose opponent goal (the goal the player should be attacking)
			Vec opponentGoalBack = (player.team == Team::BLUE) ? CommonValues::ORANGE_GOAL_BACK : CommonValues::BLUE_GOAL_BACK;

			// Direction from ball toward opponent goal
			Vec ballToGoal = opponentGoalBack - ballPos;
			float goalDist = ballToGoal.Length();
			if (goalDist <= 1e-6f) return 0.0f;
			Vec dirBallToGoal = ballToGoal / goalDist;

			// Vector from ball to player
			Vec ballToPlayer = player.pos - ballPos;

			// We want the ordering: opponent goal -> ball -> player
			// That means the player should be on the side of the ball away from the goal.
			// Use the projection onto the direction away from the goal (-dirBallToGoal).
			float forwardProjection = ballToPlayer.Dot(-dirBallToGoal); // how far behind the ball (away from goal) the player is

			// Require the player to be behind the ball (relative to goal) and within a reasonable distance
			if (forwardProjection <= 0.0f) return 0.0f;

			// Lateral distance from the ideal line (ball->away-from-goal)
			Vec along = (-dirBallToGoal) * forwardProjection;
			Vec lateralVec = ballToPlayer - along;
			float lateralDist = lateralVec.Length();

			// Normalize factors
			float lateralFactor = 1.0f - RS_CLAMP(lateralDist / maxLateralDistance, 0.0f, 1.0f);

			// Reward players that are behind the ball but not too far back.
			float forwardFactor = 1.0f - RS_CLAMP(forwardProjection / maxForwardDistance, 0.0f, 1.0f);

			// Alignment of player's position relative to the ball->away-from-goal direction (1 = exactly on line)
			float dirLen = ballToPlayer.Length();
			float alignment = 0.0f;
			if (dirLen > 1e-6f) {
				Vec ballToPlayerDir = ballToPlayer / dirLen;
				alignment = RS_CLAMP((-dirBallToGoal).Dot(ballToPlayerDir), 0.0f, 1.0f);
			} else {
				// If player is essentially on the ball, consider them aligned (but they won't be behind)
				alignment = 0.0f;
			}

			// Final reward: combine alignment (angle), lateral proximity and backward positioning
			float reward = alignment * lateralFactor * forwardFactor;

			return RS_CLAMP(reward, 0.0f, 1.0f);
		}
	};

	// https://github.com/AechPro/rocket-league-gym-sim/blob/main/rlgym_sim/utils/reward_functions/common_rewards/ball_goal_rewards.py
	class VelocityBallToGoalReward : public Reward {
	public:
		bool ownGoal = false;
		VelocityBallToGoalReward(bool ownGoal = false) : ownGoal(ownGoal) {}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			bool targetOrangeGoal = player.team == Team::BLUE;
			if (ownGoal)
				targetOrangeGoal = !targetOrangeGoal;

			Vec targetPos = targetOrangeGoal ? CommonValues::ORANGE_GOAL_BACK : CommonValues::BLUE_GOAL_BACK;
			
			Vec ballDirToGoal = (targetPos - state.ball.pos).Normalized();
			return ballDirToGoal.Dot(state.ball.vel / CommonValues::BALL_MAX_SPEED);
		}
	};

	// Rewards ball movement toward goal center while penalizing movement toward corners
	class DirectionalGoalReward : public Reward {
	public:
		float cornerPenalty;
		bool ownGoal = false;
		
		DirectionalGoalReward(float cornerPenalty = 0.5f, bool ownGoal = false) 
			: cornerPenalty(cornerPenalty), ownGoal(ownGoal) {}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			bool targetOrangeGoal = player.team == Team::BLUE;
			if (ownGoal)
				targetOrangeGoal = !targetOrangeGoal;

			// Get goal center position
			Vec goalCenter = targetOrangeGoal ? CommonValues::ORANGE_GOAL_CENTER : CommonValues::BLUE_GOAL_CENTER;
			
			// Calculate direction from ball to goal center
			Vec ballToGoalCenter = (goalCenter - state.ball.pos).Normalized();
			
			// Calculate how aligned the ball velocity is with the goal direction
			Vec normalizedBallVel = state.ball.vel / CommonValues::BALL_MAX_SPEED;
			float goalAlignment = ballToGoalCenter.Dot(normalizedBallVel);
			
			// Calculate penalty for movement toward corners
			// Check if ball is moving toward the side areas of the field
			float ballX = abs(state.ball.pos.x);
			float ballY = abs(state.ball.pos.y);
			float velX = abs(normalizedBallVel.x);
			
			// Penalty increases as ball moves toward sides (high X values)
			float sidePenalty = 0;
			if (ballX > CommonValues::SIDE_WALL_X * 0.6f) { // Near the sides
				sidePenalty = velX * cornerPenalty * (ballX / CommonValues::SIDE_WALL_X);
			}
			
			// Additional penalty for movement toward back corners
			float backPenalty = 0;
			if (ballY > CommonValues::BACK_WALL_Y * 0.7f) { // Near back wall
				backPenalty = abs(normalizedBallVel.y) * cornerPenalty * 0.5f;
			}
			
			// Return goal alignment minus corner penalties
			return goalAlignment - sidePenalty - backPenalty;
		}
	};

	// Rewards defensive rotation back to goal when ball is in corners/sides
	class DefensiveRotationReward : public Reward {
	public:
		float minRotationDistance;
		float ballCornerThreshold;
		
		DefensiveRotationReward(float minRotationDistance = 1000.0f, float ballCornerThreshold = 0.6f) 
			: minRotationDistance(minRotationDistance), ballCornerThreshold(ballCornerThreshold) {}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			// Only apply when ball is in corners/sides of the field
			float ballX = abs(state.ball.pos.x);
			float ballY = abs(state.ball.pos.y);
			bool ballInCornerOrSide = (ballX > CommonValues::SIDE_WALL_X * ballCornerThreshold) || 
									  (ballY > CommonValues::BACK_WALL_Y * ballCornerThreshold);
			
			if (!ballInCornerOrSide)
				return 0;

			// Get player's goal (the one they're defending)
			Vec defendingGoal = (player.team == Team::BLUE) ? CommonValues::BLUE_GOAL_CENTER : CommonValues::ORANGE_GOAL_CENTER;
			
			// Calculate distance from player to their goal
			float distToGoal = (player.pos - defendingGoal).Length();
			
			// Only reward rotation if player is far enough from goal (not already defending)
			if (distToGoal < minRotationDistance)
				return 0;

			// Check if player recently touched the ball (has possession)
			bool recentlyTouchedBall = player.ballTouchedStep;
			if (state.prev && state.prev->prev) {
				// Check last few steps for ball touches
				const Player* prevPlayer = nullptr;
				for (const auto& p : state.prev->players) {
					if (p.index == player.index) {
						prevPlayer = &p;
						break;
					}
				}
				if (prevPlayer && prevPlayer->ballTouchedStep)
					recentlyTouchedBall = true;
			}
			
			// Don't rotate if player has possession (should chase/control instead)
			if (recentlyTouchedBall)
				return 0;

			// Calculate direction to goal and player's movement direction
			Vec dirToGoal = (defendingGoal - player.pos).Normalized();
			Vec playerVelNorm = player.vel.Normalized();
			
			// Reward movement toward goal
			float rotationAlignment = dirToGoal.Dot(playerVelNorm);
			
			// Additional reward for collecting boost on the way back
			float boostReward = 0;
			if (player.boost < 50.0f) { // Only when low on boost
				// Find the nearest boost pad in the direction of goal rotation
				// This is a simplified approach - in practice you'd check actual boost pad locations
				Vec goalDirection = dirToGoal;
				Vec playerDirection = playerVelNorm;
				
				// Reward if moving toward goal and could pick up boost
				float boostPathAlignment = goalDirection.Dot(playerDirection);
				if (boostPathAlignment > 0.5f && player.vel.Length() > 500.0f) { // Moving reasonably fast toward goal
					boostReward = 0.3f;
				}
			}
			
			// Scale reward based on how critical the rotation is
			float criticalityScale = 1.0f;
			
			// More critical if ball is moving toward player's goal
			Vec ballToPlayerGoal = (defendingGoal - state.ball.pos).Normalized();
			Vec ballVelNorm = state.ball.vel.Normalized();
			float ballThreatLevel = ballToPlayerGoal.Dot(ballVelNorm);
			if (ballThreatLevel > 0.3f) { // Ball moving toward player's goal
				criticalityScale = 1.5f;
			}
			
			// More critical if opponent is closer to ball
			float playerDistToBall = (player.pos - state.ball.pos).Length();
			bool opponentCloser = false;
			for (const auto& p : state.players) {
				if (p.team != player.team) {
					float opponentDistToBall = (p.pos - state.ball.pos).Length();
					if (opponentDistToBall < playerDistToBall) {
						opponentCloser = true;
						break;
					}
				}
			}
			if (opponentCloser) {
				criticalityScale *= 1.3f;
			}
			
			// Final reward: rotation alignment + boost bonus, scaled by criticality
			return RS_CLAMP((rotationAlignment + boostReward) * criticalityScale, 0, 2.0f);
		}
	};

	// Rewards shots that are on target (would go into the goal if unblocked)
	class ShotOnTargetReward : public Reward {
	public:
		float minShotSpeed;
		float maxProjectionTime;
		bool ownGoal = false;
		
		ShotOnTargetReward(float minShotSpeedKPH = 30.0f, float maxProjectionTimeSeconds = 2.0f, bool ownGoal = false) 
			: minShotSpeed(RLGC::Math::KPHToVel(minShotSpeedKPH)), maxProjectionTime(maxProjectionTimeSeconds), ownGoal(ownGoal) {}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			// Only reward when player touches the ball
			if (!player.ballTouchedStep)
				return 0;

			// Need previous state to calculate shot
			if (!state.prev)
				return 0;

			// Calculate ball velocity change (shot power)
			Vec ballVelChange = state.ball.vel - state.prev->ball.vel;
			float shotSpeed = ballVelChange.Length();
			
			// Only consider shots above minimum speed
			if (shotSpeed < minShotSpeed)
				return 0;

			// Determine target goal
			bool targetOrangeGoal = player.team == Team::BLUE;
			if (ownGoal)
				targetOrangeGoal = !targetOrangeGoal;

			// Get goal dimensions and position
			Vec goalCenter;
			float goalWidth = CommonValues::GOAL_WIDTH_FROM_CENTER * 2; // Total width
			float goalHeight = CommonValues::GOAL_HEIGHT;
			
			if (targetOrangeGoal) {
				goalCenter = CommonValues::ORANGE_GOAL_CENTER;
			} else {
				goalCenter = CommonValues::BLUE_GOAL_CENTER;
			}

			// Project ball trajectory to see if it hits the goal
			Vec ballPos = state.ball.pos;
			Vec ballVel = state.ball.vel;
			
			// Simple linear projection (ignoring gravity/bounces for simplicity)
			float timeToGoalPlane = 0;
			bool willReachGoal = false;
			
			if (targetOrangeGoal && ballVel.y > 0) {
				// Ball moving toward orange goal (positive Y)
				timeToGoalPlane = (CommonValues::BACK_WALL_Y - ballPos.y) / ballVel.y;
				willReachGoal = true;
			} else if (!targetOrangeGoal && ballVel.y < 0) {
				// Ball moving toward blue goal (negative Y)
				timeToGoalPlane = (-CommonValues::BACK_WALL_Y - ballPos.y) / ballVel.y;
				willReachGoal = true;
			}
			
			// Only consider shots that will reach the goal plane in reasonable time
			if (!willReachGoal || timeToGoalPlane < 0 || timeToGoalPlane > maxProjectionTime)
				return 0;

			// Calculate where ball will be when it reaches the goal plane
			Vec projectedPos = ballPos + ballVel * timeToGoalPlane;
			
			// Check if projected position is within goal bounds
			bool withinGoalX = (projectedPos.x >= -CommonValues::GOAL_WIDTH_FROM_CENTER) && (projectedPos.x <= CommonValues::GOAL_WIDTH_FROM_CENTER);
			bool withinGoalZ = (projectedPos.z >= 0) && (projectedPos.z <= CommonValues::GOAL_HEIGHT);
			
			if (!withinGoalX || !withinGoalZ)
				return 0; // Shot will miss the goal

			// Calculate reward based on shot quality
			float speedReward = RS_MIN(1.0f, shotSpeed / (CommonValues::BALL_MAX_SPEED * 0.5f));
			
			// Reward shots closer to goal center
			Vec goalCenter2D = Vec(goalCenter.x, goalCenter.y, CommonValues::GOAL_HEIGHT/2); // Center of goal face
			float distFromCenter = (projectedPos - goalCenter2D).Length();
			float maxGoalRadius = sqrtf(CommonValues::GOAL_WIDTH_FROM_CENTER * CommonValues::GOAL_WIDTH_FROM_CENTER + (CommonValues::GOAL_HEIGHT/2) * (CommonValues::GOAL_HEIGHT/2));
			float centeringReward = 1.0f - (distFromCenter / maxGoalRadius);
			
			// Reward faster shots (less time to react)
			float timeReward = 1.0f - (timeToGoalPlane / maxProjectionTime);
			
			// Combine all factors
			float totalReward = (speedReward + centeringReward + timeReward) / 3.0f;
			
			// Bonus for shots from closer range (more accurate)
			float distToGoal = (ballPos - goalCenter).Length();
			if (distToGoal < 2000.0f) { // Within 2000 units
				totalReward *= 1.2f; // 20% bonus for close shots
			}
			
			return RS_CLAMP(totalReward, 0, 1.5f);
		}
	};

	// https://github.com/AechPro/rocket-league-gym-sim/blob/main/rlgym_sim/utils/reward_functions/common_rewards/player_ball_rewards.py
	class VelocityPlayerToBallReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			Vec dirToBall = (state.ball.pos - player.pos).Normalized();
			Vec normVel = player.vel / CommonValues::CAR_MAX_SPEED;
			return dirToBall.Dot(normVel);
		}
	};

	// https://github.com/AechPro/rocket-league-gym-sim/blob/main/rlgym_sim/utils/reward_functions/common_rewards/player_ball_rewards.py
	class FaceBallReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			Vec dirToBall = (state.ball.pos - player.pos).Normalized();
			return player.rotMat.forward.Dot(dirToBall);
		}
	};

	class TouchBallReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			return player.ballTouchedStep;
		}
	};

	

	class SpeedReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			return player.vel.Length() / CommonValues::CAR_MAX_SPEED;
		}
	};

	class WavedashReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			if (!player.prev)
				return 0;

			if (player.isOnGround && (player.prev->isFlipping && !player.prev->isOnGround)) {
				return 1;
			} else {
				return 0;
			}
		}
	};

	class PickupBoostReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!player.prev)
				return 0;

			if (player.boost > player.prev->boost) {
				return sqrtf(player.boost / 100.f) - sqrtf(player.prev->boost / 100.f);
			} else {
				return 0;
			}
		}
	};

	// https://github.com/AechPro/rocket-league-gym-sim/blob/main/rlgym_sim/utils/reward_functions/common_rewards/misc_rewards.py
	class SaveBoostReward : public Reward {
	public:
		float exponent;
		SaveBoostReward(float exponent = 0.5f) : exponent(exponent) {}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) {
			return RS_CLAMP(powf(player.boost / 100, exponent), 0, 1);
		}
	};

	// Penalizes wasting boost, especially when close to the ground.
	// Returns a negative value (penalty) proportional to boost used in the last tick
	// and weighted by proximity to the ground.
	class BoostWastePenaltyReward : public Reward {
	public:
		// scale: maximum penalty magnitude (per full 100 boost used)
		// groundThreshold: altitude (UU) under which penalties scale linearly to full effect
		float scale;
		float groundThreshold;

		BoostWastePenaltyReward(float scale_ = 0.8f, float groundThreshold_ = 500.0f)
			: scale(scale_), groundThreshold(groundThreshold_) {}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			// Need previous player state to compute boost usage
			if (!player.prev)
				return 0.0f;

			float prevBoost = player.prev->boost;
			float curBoost = player.boost;
			float used = prevBoost - curBoost; // positive when boost was spent
			if (used <= 0.0f)
				return 0.0f;

			// proximity to ground: 1.0 at z=0, 0.0 at or above groundThreshold
			float h = RS_CLAMP(player.pos.z, 0.0f, groundThreshold);
			float groundFactor = 1.0f - (h / groundThreshold);

			// Slightly amplify penalty if player is actually on the ground
			float onGroundBonus = player.isOnGround ? 1.25f : 1.0f;

			float penalty = (used / 100.0f) * scale * groundFactor * onGroundBonus;
			// Return negative value to act as a penalty
			return -RS_CLAMP(penalty, 0.0f, scale * 2.0f);
		}
	};

	// Simple grounded penalty: small negative reward when the car is on the ground
	// and its height is below the ball radius (i.e., touching/close to grass).
	// This mirrors the Python snippet: if player.on_ground and car_height < BALL_RADIUS: penalty.
	class GroundedPenaltyReward : public Reward {
	public:
		// magnitude in [0,1]; returned reward will be a positive value in [0,1]
		float scale;

		GroundedPenaltyReward(float scale_ = 0.01f) : scale(RS_CLAMP(scale_, 0.0f, 1.0f)) {}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			// reward only when explicitly on ground and car height below ball radius
			if (player.isOnGround && player.pos.z < CommonValues::BALL_RADIUS) {
				return RS_CLAMP(scale, 0.0f, 1.0f);
			}
			return 0.0f;
		}
	};

	// Rewards touching the ball at higher altitudes, scaling with height and wall distance.

	


	class AirReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			return !player.isOnGround;
		}
	};

	// Simple rotation reward: normalized angular velocity while airborne.
	// Matches the Python snippet: ang_vel_norm = ||ang_vel|| / CAR_MAX_ANG_VEL
	// The returned value is in [0,1]; apply weighting externally when adding the reward.
	class RotationReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			// Only reward while airborne
			if (player.isOnGround)
				return 0.0f;

			float angMag = player.angVel.Length() / CommonValues::CAR_MAX_ANG_VEL;
			return RS_CLAMP(angMag, 0.0f, 1.0f);
		}
	};

	// Mostly based on the classic Necto rewards
	// Total reward output for speeding the ball up to MAX_REWARDED_BALL_SPEED is 1.0
	// The bot can do this slowly (putting) or quickly (shooting)
	class TouchAccelReward : public Reward {
	public:
		constexpr static float MAX_REWARDED_BALL_SPEED = RLGC::Math::KPHToVel(110);

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!state.prev)
				return 0;

			if (player.ballTouchedStep) {
				float prevSpeedFrac = RS_MIN(1, state.prev->ball.vel.Length() / MAX_REWARDED_BALL_SPEED);
				float curSpeedFrac = RS_MIN(1, state.ball.vel.Length() / MAX_REWARDED_BALL_SPEED);

				if (curSpeedFrac > prevSpeedFrac) {
					return (curSpeedFrac - prevSpeedFrac);
				} else {
					// Not speeding up the ball so we don't care
					return 0;
				}
			} else {
				return 0;
			}
		}
	};

	class StrongTouchReward : public Reward {
	public:
		float minRewardedVel, maxRewardedVel;
		StrongTouchReward(float minSpeedKPH = 20, float maxSpeedKPH = 130) {
			minRewardedVel = RLGC::Math::KPHToVel(minSpeedKPH);
			maxRewardedVel = RLGC::Math::KPHToVel(maxSpeedKPH);
		}

		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			if (!state.prev)
				return 0;

			if (player.ballTouchedStep) {
				float hitForce = (state.ball.vel - state.prev->ball.vel).Length();
				if (hitForce < minRewardedVel)
					return 0;

				return RS_MIN(1, hitForce / maxRewardedVel);
			} else {
				return 0;
			}
		}
	};


class HigherWinProbReward : public Reward {
public:
    float totalMatchSeconds;
    float scale;
    bool continuous; //allows both teams to get reward scaled to to their win prob
    bool debugPrint;
    uint64_t lastPrintedTick = (uint64_t)-1;

    HigherWinProbReward(float totalMatchSeconds_ = 300.0f, float scale_ = 1.0f, bool continuous_ = true, bool debugPrint_ = false)
        : totalMatchSeconds(totalMatchSeconds_), scale(scale_), continuous(continuous_), debugPrint(debugPrint_) {}

private:
    void CountGoals(const GameState* s, int outGoals[2]) const {
        outGoals[0] = outGoals[1] = 0;
        if (!s) return;
        outGoals[0] = s->score[0];
        outGoals[1] = s->score[1];
    }

    float EstimateTimeLeft(const GameState& state) const {
        if (state.lastArena) {
            const Arena* a = state.lastArena;
            float elapsed = 0.0f;
            if (state.matchTickCount > 0)
                elapsed = (float)state.matchTickCount * a->tickTime;
            else {
                uint64_t startTick = state.matchStartTick ? state.matchStartTick : state.lastTickCount;
                int64_t elapsedTicks = (int64_t)a->tickCount - (int64_t)startTick;
                if (elapsedTicks < 0) elapsedTicks = 0;
                elapsed = (float)elapsedTicks * a->tickTime;
            }
            return RS_MAX(0.0f, totalMatchSeconds - elapsed);
        }
        return totalMatchSeconds;
    }

    float Sigmoid(float x) const {
        return 1.0f / (1.0f + expf(-x));
    }

    float ComputeWinProb(int teamIndex, const GameState* s) const {
        int goals[2];
        CountGoals(s, goals);
        int scoreDiff = goals[teamIndex] - goals[1 - teamIndex];

        float timeLeft = s ? EstimateTimeLeft(*s) : totalMatchSeconds;
        float timeFactor = 1.0f - RS_CLAMP(timeLeft / totalMatchSeconds, 0.0f, 1.0f); // increases as match progresses
		float sensitivity = 0.5f + 3.0f * powf(timeFactor, 2.0f);

        return Sigmoid(sensitivity * (float)scoreDiff);
    }

public:
    virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
        if (!state.prev)
            return 0.0f;

        int teamIndex = (int)player.team;
        float curWP = ComputeWinProb(teamIndex, &state);
        float oppWP = ComputeWinProb(1 - teamIndex, &state);

        // if (debugPrint) {
        //     uint64_t tick = state.lastTickCount;
        //     if (tick != lastPrintedTick) {
        //         lastPrintedTick = tick;
        //         int goalsCur[2]; CountGoals(&state, goalsCur);
        //         float timeLeft = EstimateTimeLeft(state);
        //         const char* teamName = (player.team == Team::BLUE) ? "BLUE" : "ORANGE";
        //         std::printf("[HigherWinProb] tick=%llu team=%s score=%d-%d curWP=%.3f oppWP=%.3f time_left=%.2f\n",
        //             (unsigned long long)tick, teamName, goalsCur[0], goalsCur[1], curWP, oppWP, timeLeft);
        //     }
        // }

        if (continuous)
            return RS_CLAMP(scale * curWP, 0.0f, scale);
        else
            return (curWP > oppWP) ? scale : 0.0f;
    }
};

	
}