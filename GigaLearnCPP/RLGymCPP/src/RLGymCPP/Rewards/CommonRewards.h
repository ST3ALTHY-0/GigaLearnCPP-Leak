#pragma once
#include "Reward.h"
#include "../Math.h"

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
 # On ground it gets about 0.04 just for touching, as well as some extra for the speed it produces
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


	class AirReward : public Reward {
	public:
		virtual float GetReward(const Player& player, const GameState& state, bool isFinal) override {
			return !player.isOnGround;
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
}