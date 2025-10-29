#include <GigaLearnCPP/Learner.h>

#include <RLGymCPP/Rewards/CommonRewards.h>
#include <RLGymCPP/Rewards/ZeroSumReward.h>
#include <RLGymCPP/TerminalConditions/NoTouchCondition.h>
#include <RLGymCPP/TerminalConditions/GoalScoreCondition.h>
#include <RLGymCPP/TerminalConditions/EndOfMatchCondition.h>
#include <cstdlib>
#include <RLGymCPP/OBSBuilders/DefaultObs.h>
#include <RLGymCPP/OBSBuilders/AdvancedObs.h>
#include <RLGymCPP/StateSetters/KickoffState.h>
#include <RLGymCPP/StateSetters/RandomState.h>
#include <RLGymCPP/ActionParsers/DefaultAction.h>

using namespace GGL;  // GigaLearn
using namespace RLGC; // RLGymCPP

// Create the RLGymCPP environment for each of our games
EnvCreateResult EnvCreateFunc(int index)
{

	// Necto Rewards

	std::vector<WeightedReward> rewards = {

		/*
⚽ Goal & Match-Related Rewards
Reward	Weight	Purpose
goal_w = 10	Big positive reward when your team scores.
win_prob_w = 10	Encourages states where the team’s win probability increases (based on score/time left).
goal_dist_w = 10	Rewards moving the ball closer to the opponent’s goal and away from your own.
goal_speed_bonus_w = 2.5	Bonus when scoring with a fast-moving ball.
goal_dist_bonus_w = 2.5	Bonus when scoring from farther away (encourages long shots).
🚗 Player Action & Positioning Rewards
Reward	Weight	Purpose
dist_w = 0.25	Rewards being closer to the ball.
align_w = 0.25	Rewards aligning between the ball and the opponent’s goal (good attacking positioning).
boost_gain_w = 1.5	Rewards collecting boost (especially when low).
boost_lose_w = 0.8	Penalizes wasting boost, particularly when close to the ground (defensive penalty).
ang_vel_w = 0.005	Slightly rewards rotating/spinning, to encourage aerial exploration.
touch_grass_w = 0.005	Small penalty for staying grounded too long (encourages aerial play).
🏀 Ball Interaction Rewards
Reward	Weight	Purpose
touch_height_w = 3	Rewards touching the ball at higher altitudes, scaling with height and wall distance.
touch_accel_w = 0.5	Rewards increasing the ball’s speed on touch (stronger hits).
flip_reset_w = 10	Large reward for a successful flip reset (touching the ball with the bottom of the car while airborne).
💥 Demolition & Opponent Interaction
Reward	Weight	Purpose
demo_w = 8	Reward for demolishing opponents, penalty for being demoed.
opponent_punish_w = 1	Slight penalty tied to opponent team’s average reward, discouraging imbalance.

🧠 Team & Coordination
Reward	Weight	Purpose
team_spirit = 0.6	Controls how much reward is shared across teammates vs. earned individually.
High = teamwork focus, Low = individual play emphasis.
🏁 State Quality & Miscellaneous
Concept	Description
State quality (state_quality)	Captures how favorable the game state is (ball near enemy goal, win probability high).
Player quality (player_qualities)	Evaluates each player’s positioning and alignment toward useful play.
Wall distance factor	Adds bonuses for height touches away from walls (cleaner aerial control).
Height activation curve	Normalizes rewards based on height within the field bounds.
*/

		// Goal & Match-Related Rewards
	{new GoalReward(), 20},
	{new SaveReward(), 10},
	{new HigherWinProbReward(300.0f, 1.0f, true, false), 5}, // test different values, is added with every tick so careful
	{new VelocityBallToGoalReward(), 5.0f},
		// add reward for scoring harder/further shots

		// Player Action & Positioning Rewards
		{new ProximityToBallReward(), .5f},
		{new FaceBallReward(), .5f},
		{new AlignBallToOpponentGoalReward(), .5f},
		{new PickupBoostReward(), 2},
		{new SaveBoostReward(), 0.8f},
		{new RotationReward(), .005f},
		{new GroundedPenaltyReward(), .005f},
		{new TouchAccelReward(), 1},
		{new WavedashReward(), 1},
		{new AerialDribbleReward(), 3},
		{new VelocityPlayerToBallReward(), .25f},
		// add flip reset reward

		// 💥 Demolition & Opponent Interaction
		{new ZeroSumReward(new BumpReward(), 0.5f), 10},
		{new ZeroSumReward(new DemoReward(), 0.5f), 20}
};
// These are ok rewards that will produce a scoring bot in ~100m steps
// std::vector<WeightedReward>
// 	rewards = {

// 		// Movement
// 		// { new AirReward(), .1f },
// 		//{ new SpeedReward(), 0.25f },
// 		// { new WavedashReward(), 1.f },

// 		// Player-ball
// 		// { new ControlledTouchReward(), .10f },
// 		// { new FaceBallReward(), 0.2f },
// 		// { new VelocityPlayerToBallReward(), 0.2f },
// 		// { new VelocityBallToGoalReward(), 1.0f },
// 		// { new TouchBallReward(), 0.2f },

// 		// // { new DefensiveRotationReward(1000.0f, 0.6f), 6.0f }, // Rewards proper defensive rotation
// 		// { new StrongTouchReward(20, 100), .10f },

// 		// // Ball-goal
// 		// { new ZeroSumReward(new ShotOnTargetReward(30.0f, 2.0f), 1), 5.0f }, // Rewards accurate shots on goal
// 		// // { new ZeroSumReward(new DirectionalGoalReward(0.5f), 1), 4.0f }, // Encourages goal-center direction, penalizes corners

// 		// // Boost
// 		// { new PickupBoostReward(), 1.0f },
// 		// { new SaveBoostReward(), 2.0f },

// 		// // Game events
// 		// { new ZeroSumReward(new BumpReward(), 0.5f), 40 },
// 		// { new ZeroSumReward(new DemoReward(), 0.5f), 100 },
// 		// { new ZeroSumReward(new GoalReward(), 0.5f), 200 },
// 		// { new ZeroSumReward(new SaveReward(), 0.5f), 55 }

// 		// Movement
// 		{new AirReward(), 0.25f},
// 		{new AerialReward(1.75f), 1.0f}, // Rewards aerial play combining air time + ball height

// 		// Player-ball
// 		{new FaceBallReward(), 0.25f},
// 		{new VelocityPlayerToBallReward(), 4.f},
// 		{new StrongTouchReward(20, 100), 60},
// 		{new AerialReward(), 2.f},
// 		{new DribbleReward(1.0f, 5.0f), 1.0f},
// 		{new ShotOnTargetReward(30.0f, 2.0f), 5.0f},
// 		// { new ControlledTouchReward(), .25f },

// 		// Ball-goal
// 		{new ZeroSumReward(new VelocityBallToGoalReward(), 1), 2.0f},

// 		// Boost
// 		{new PickupBoostReward(), 10.f},
// 		{new SaveBoostReward(), 0.5f},

// 		// Game events
// 		{new ZeroSumReward(new BumpReward(), 0.5f), 20},
// 		{new ZeroSumReward(new DemoReward(), 0.5f), 80},
// 		{new GoalReward(), 150},
// 		{new SaveReward(), 50},

// 		// TEST
// 		{new HigherWinProbReward(300.0f, 1.0f, true, false), 1}};

std::vector<TerminalCondition *> terminalConditions = {
	new NoTouchCondition(10),
	new EndOfMatchCondition(300.0f)
	// new GoalScoreCondition()
};

// Make the arena
int playersPerTeam = 1;
auto arena = Arena::Create(GameMode::SOCCAR);
for (int i = 0; i < playersPerTeam; i++)
{
	arena->AddCar(Team::BLUE);
	arena->AddCar(Team::ORANGE);
}

EnvCreateResult result = {};
result.actionParser = new DefaultAction();
result.obsBuilder = new AdvancedObs();
result.stateSetter = new KickoffState();
result.terminalConditions = terminalConditions;
result.rewards = rewards;

result.arena = arena;

return result;
}

void StepCallback(Learner *learner, const std::vector<GameState> &states, Report &report)
{
	// To prevent expensive metrics from eating at performance, we will only run them on 1/4th of steps
	// This doesn't really matter unless you have expensive metrics (which this example doesn't)
	bool doExpensiveMetrics = (rand() % 4) == 0;

	// Add our metrics
	for (auto &state : states)
	{
		if (doExpensiveMetrics)
		{
			for (auto &player : state.players)
			{
				report.AddAvg("Player/In Air Ratio", !player.isOnGround);
				report.AddAvg("Player/Ball Touch Ratio", player.ballTouchedStep);
				report.AddAvg("Player/Demoed Ratio", player.isDemoed);

				report.AddAvg("Player/Speed", player.vel.Length());
				Vec dirToBall = (state.ball.pos - player.pos).Normalized();
				report.AddAvg("Player/Speed Towards Ball", RS_MAX(0, player.vel.Dot(dirToBall)));

				report.AddAvg("Player/Boost", player.boost);

				if (player.ballTouchedStep)
					report.AddAvg("Player/Touch Height", state.ball.pos.z);
			}
		}

		if (state.goalScored)
			report.AddAvg("Game/Goal Speed", state.ball.vel.Length());
	}
}

int main(int argc, char *argv[])
{
	// Initialize RocketSim with collision meshes
	// IMPORTANT: Change this path to point to your meshes
	RocketSim::Init("C:\\Programming\\CPP\\GigaLearn2\\rlbot\\collision_meshes");

	// Make configuration for the learner
	LearnerConfig cfg = {};

	cfg.deviceType = LearnerDeviceType::GPU_CUDA;

	cfg.tickSkip = 8;
	cfg.actionDelay = cfg.tickSkip - 1; // Normal value in other RLGym frameworks

	// Play around with this to see what the optimal is for your machine, more games will consume more RAM/CPU
	cfg.numGames = 1024;

	// Leave this empty to use a random seed each run
	// The random seed can have a strong effect on the outcome of a run
	cfg.randomSeed = -1;

	int tsPerItr = 100'000;
	cfg.ppo.tsPerItr = tsPerItr;
	cfg.ppo.batchSize = tsPerItr;
	cfg.ppo.miniBatchSize = 50000; // Matching batch size for simpler processing (make a multiple of batchSize)

	// Using 2 epochs seems pretty optimal when comparing time training to skill
	// Perhaps 1 or 3 is better for you, test and find out!
	cfg.ppo.epochs = 2;

	// This scales differently than "ent_coef" in other frameworks
	// This is the scale for normalized entropy, which means you won't have to change it if you add more actions
	cfg.ppo.entropyScale = 0.035f;

	// Rate of reward decay
	// Starting low tends to work out
	cfg.ppo.gaeGamma = 0.99;

	// Good learning rate to start
	cfg.ppo.policyLR = 1.5e-4;
	cfg.ppo.criticLR = 1.5e-4;

	// will increase model quality at price of performance, increase if you have a better GPU
	// restart training model if changed
	cfg.ppo.sharedHead.layerSizes = {1024, 1024, 512};
	cfg.ppo.policy.layerSizes = {512, 256};
	cfg.ppo.critic.layerSizes = {512, 256};

	cfg.ppo.useHalfPrecision = true;
	cfg.ppo.deterministic = false;

	auto optim = ModelOptimType::ADAM;
	cfg.ppo.policy.optimType = optim;
	cfg.ppo.critic.optimType = optim;
	cfg.ppo.sharedHead.optimType = optim;

	auto activation = ModelActivationType::RELU;
	cfg.ppo.policy.activationType = activation;
	cfg.ppo.critic.activationType = activation;
	cfg.ppo.sharedHead.activationType = activation;

	bool addLayerNorm = true;
	cfg.ppo.policy.addLayerNorm = addLayerNorm;
	cfg.ppo.critic.addLayerNorm = addLayerNorm;
	cfg.ppo.sharedHead.addLayerNorm = addLayerNorm;

	cfg.sendMetrics = true; // Send metrics
	cfg.renderMode = false;
	cfg.renderTimeScale = 2; // 2 = twice as fast

	// Set checkpoint folder to load/save models
	cfg.checkpointFolder = "checkpoints";

	// GPU/Torch env variables
	// $env:PYTORCH_CUDA_ALLOC_CONF="expandable_segments:True"
	// torch.backends.cuda.matmul.allow_tf32 = True
	// torch.backends.cudnn.benchmark = True

	// Make the learner with the environment creation function and the config we just made
	Learner *learner = new Learner(EnvCreateFunc, cfg, StepCallback);

	// Start learning!
	learner->Start();

	return EXIT_SUCCESS;
}
