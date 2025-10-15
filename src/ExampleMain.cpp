#include <GigaLearnCPP/Learner.h>

#include <RLGymCPP/Rewards/CommonRewards.h>
#include <RLGymCPP/Rewards/ZeroSumReward.h>
#include <RLGymCPP/TerminalConditions/NoTouchCondition.h>
#include <RLGymCPP/TerminalConditions/GoalScoreCondition.h>
#include <RLGymCPP/OBSBuilders/DefaultObs.h>
#include <RLGymCPP/OBSBuilders/AdvancedObs.h>
#include <RLGymCPP/StateSetters/KickoffState.h>
#include <RLGymCPP/StateSetters/RandomState.h>
#include <RLGymCPP/ActionParsers/DefaultAction.h>

using namespace GGL; // GigaLearn
using namespace RLGC; // RLGymCPP

// Create the RLGymCPP environment for each of our games
EnvCreateResult EnvCreateFunc(int index) {
	// These are ok rewards that will produce a scoring bot in ~100m steps
	std::vector<WeightedReward> rewards = {

		// Movement
		// { new AirReward(), .1f },
		//{ new SpeedReward(), 0.25f },
		// { new WavedashReward(), 1.f },


		// Player-ball
		// { new ControlledTouchReward(), .10f },
		// { new FaceBallReward(), 0.2f },
		// { new VelocityPlayerToBallReward(), 0.2f },
		// { new VelocityBallToGoalReward(), 1.0f },
		// { new TouchBallReward(), 0.2f },
		
		// // { new DefensiveRotationReward(1000.0f, 0.6f), 6.0f }, // Rewards proper defensive rotation
		// { new StrongTouchReward(20, 100), .10f },

		// // Ball-goal
		// { new ZeroSumReward(new ShotOnTargetReward(30.0f, 2.0f), 1), 5.0f }, // Rewards accurate shots on goal
		// // { new ZeroSumReward(new DirectionalGoalReward(0.5f), 1), 4.0f }, // Encourages goal-center direction, penalizes corners

		// // Boost
		// { new PickupBoostReward(), 1.0f },
		// { new SaveBoostReward(), 2.0f },

		// // Game events
		// { new ZeroSumReward(new BumpReward(), 0.5f), 40 },
		// { new ZeroSumReward(new DemoReward(), 0.5f), 100 },
		// { new ZeroSumReward(new GoalReward(), 0.5f), 200 },
		// { new ZeroSumReward(new SaveReward(), 0.5f), 55 }

		// Movement
		{ new AirReward(), 0.25f },
		{ new AerialReward(1.75f), 1.0f }, // Rewards aerial play combining air time + ball height

		// Player-ball
		{ new FaceBallReward(), 0.25f },
		{ new VelocityPlayerToBallReward(), 4.f },
		{ new StrongTouchReward(20, 100), 60 },
		{ new AerialReward(), 2.f },
		{ new DribbleReward(1.0f, 5.0f), 1.0f },
		{ new ShotOnTargetReward(30.0f, 2.0f), 5.0f },
		// { new ControlledTouchReward(), .25f },

		// Ball-goal
		{ new ZeroSumReward(new VelocityBallToGoalReward(), 1), 2.0f },

		// Boost
		{ new PickupBoostReward(), 10.f },
		{ new SaveBoostReward(), 0.2f },

		// Game events
		{ new ZeroSumReward(new BumpReward(), 0.5f), 20 },
		{ new ZeroSumReward(new DemoReward(), 0.5f), 80 },
		{ new GoalReward(), 150 },
		{ new ZeroSumReward(new SaveReward(), 1.f), 50 }

	};

	std::vector<TerminalCondition*> terminalConditions = {
		new NoTouchCondition(10),
		new GoalScoreCondition()
	};

	// Make the arena
	int playersPerTeam = 1;
	auto arena = Arena::Create(GameMode::SOCCAR);
	for (int i = 0; i < playersPerTeam; i++) {
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

void StepCallback(Learner* learner, const std::vector<GameState>& states, Report& report) {
	// To prevent expensive metrics from eating at performance, we will only run them on 1/4th of steps
	// This doesn't really matter unless you have expensive metrics (which this example doesn't)
	bool doExpensiveMetrics = (rand() % 4) == 0;

	// Add our metrics
	for (auto& state : states) {
		if (doExpensiveMetrics) {
			for (auto& player : state.players) {
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

int main(int argc, char* argv[]) {
	// Initialize RocketSim with collision meshes
	// IMPORTANT: Change this path to point to your meshes
	RocketSim::Init("C:\\Programming\\CPP\\GigaLearn2\\rlbot\\collision_meshes");

	// Make configuration for the learner
	LearnerConfig cfg = {};

	cfg.deviceType = LearnerDeviceType::GPU_CUDA;

	cfg.tickSkip = 8;
	cfg.actionDelay = cfg.tickSkip - 1; // Normal value in other RLGym frameworks

	// Play around with this to see what the optimal is for your machine, more games will consume more RAM/CPU
	cfg.numGames = 712; // Reduced from 512 for less PC load

	// Leave this empty to use a random seed each run
	// The random seed can have a strong effect on the outcome of a run
	cfg.randomSeed = 1234567;

	int tsPerItr = 100'000; // Reduced from 100k for less PC load
	cfg.ppo.tsPerItr = tsPerItr;
	cfg.ppo.batchSize = tsPerItr;
	cfg.ppo.miniBatchSize = 50000; // Matching batch size for simpler processing

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

	//will increase model quality at price of performance, increase if you have a better GPU 
	//restart training model if changed
	cfg.ppo.sharedHead.layerSizes = { 1024, 1024, 512};
	cfg.ppo.policy.layerSizes = { 512, 256};
	cfg.ppo.critic.layerSizes = {512, 256};

	/*
	cfg.ppo.sharedHead.layerSizes = { 1024, 1024, 512 };
	cfg.ppo.policy.layerSizes = { 512, 1024, 512 };
	cfg.ppo.critic.layerSizes = { 512, 1024, 1024, 512 };
	*/


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

	cfg.sendMetrics = false; // Send metrics
	cfg.renderMode = false;
	cfg.renderTimeScale = 2; //2 = twice as fast
	
	// Set checkpoint folder to load/save models
	cfg.checkpointFolder = "checkpoints";
	
	

	// Make the learner with the environment creation function and the config we just made
	Learner* learner = new Learner(EnvCreateFunc, cfg, StepCallback);

	// Start learning!
	learner->Start();

	return EXIT_SUCCESS;
}
