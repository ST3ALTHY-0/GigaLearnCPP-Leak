# GigaLearnCPP Leaked Version

-------------
# This is the version that cheaters decided to leak :(

### Forked version of GigaLearnCPP from ZealanL
I modified this for my own use, feel free to copy whatever, at this point everything important was written by ZealanL, or 
copied straight from GigaLearnCCP (whoever wrote that).

### This library was intended to be private, but apparently we can't have nice things. 

A trusted former GigaLearn user (with source code access) decided to join the cheaters and then share around this library, now it's public.
Thus, I am formally publishing this specific version for anyone to use.

-------------

GigaLearn is an even-faster C++ machine learning framework for Rocket League bots.
This is a progression far beyond my previous C++ learning framework, RLGymPPO-CPP (which I have stopped developing).

## Speed
Collection speeds are around 2x faster in GigaLearn than RLGymPPO-CPP, and around 10x faster than RLGym-PPO (on my machine).  
Consumption speeds are a bit faster than RLGymPPO-CPP, although this varies heavily.  
This speed is not at all final and I plan to make the library much faster once I finish other important features.

## Features

**Basic Features (Shared With Most Frameworks)**:
- Fast PPO implementation
- Configurable model layer sizes
- Checkpoint saving/loading
- Working example
- Return standardization and obs standardization support
- Built-in visualization support
- Easy custom metrics suppport

**Unique Learning Features**:
- Extremely fast monolithic single-process inference model
- Complete and proper action masking
- Built-in shared layers support (enabled by default)
- Built-in configurable ELO-based skill tracking system 
- Configurable model activation functions
- Configurable model optimizers
- Configurable model layer norm (recommended)
- Policy version saving system (required if using the skill tracker)
- Built-in reward logging

**Unique Environment/State Features**:
- Access to previous states (e.g. `player.prev->pos`)
- Simpler access to previous actions, final bools
- Inherented access to all `CarState` and `BallState` fields (e.g. `player.isFlipping`)
- No more duplicate state fields
- Simpler access to current state events (e.g. `if (player.eventState.shot) ...`)
- User-led setup of arenas and cars during environment creation
- RocketSim-based state setting (you are given the arena to state-set)
- Configurable action delay

***Coming Soon**:
- Training against older versions

## Installation

*There's no official installation guide yet, as I plan to rework several aspects of the library to make it easier to install.*

Here’s a current (hacky) method. Take with a grain of salt:

1. **Clone the repository**  

2. **Install prerequisites**  
   - CUDA (tested with 12.8, others may work)  
   - Visual Studio with Desktop Development for C++  
   - CMake
   - Python IMPORTANT: make sure to download 'debug binaries for VS 2017 or later' for your Python version
   - Ensure Path environment variables are set

3. **Copy Visual Studio integration files**  
   - Copy the 4 files from:  

     ```
     C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\<YourVersion>\extras\visual_studio_integration\MSBuildExtensions
     ```
     
     to:  

     ```
     C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\MSBuild\Microsoft\VC\v170
     ```

4. **Set paths**  
   - Set `Collision_meshes` path in `ExampleMain.cpp`  
   - Edit any other settings you want for the learner  

5. **Build the project**  

   ```bat
   mkdir build
   cd build
   cmake .. -G "Visual Studio 17 2022" -A x64 ^
       -DCMAKE_GENERATOR_TOOLSET="cuda=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.8" ^
       -DCMAKE_PREFIX_PATH="C:/Programming/CPP/GigaLearn2/libtorch" ^
       -DPython_EXECUTABLE="C:\Program Files\Python314\python.exe" ^
       -DPython_INCLUDE_DIR="C:/Program Files/Python314/include" ^
       -DPython_LIBRARY="C:/Program Files/Python314/libs/python314.lib"

   cmake --build . --config Release

- Change path/dir to your own

6. **Run the Project**

  - Run the .exe file in build/Release

  - Alternatively, edit the variables in build_project.bat and run_project.bat and run those to compile/build and run the project, respectively

## Bringing In Rewards/Obs Builders/Etc. from RLGymPPO_CPP
State changes:
- `PlayerData` -> `Player`
- `player.phys` -> `player.`
- `player.carState` -> `player`
- `state.ballState` -> `state.ball`

Reward changes:
- `GetReward(const PlayerData& player, const GameState& state, const Action& prevAction)` -> `GetReward(const Player& player, const GameState& state, bool isFinal) override`
- `prevAction (argument)` -> `player.prevAction`
- `GetFinalReward()` -> `isFinal (argument)`

Metrics:
- `metrics.AccumAvg(), metrics.GetAvg()` -> `report.AddAvg()`

Learner config:
- `cfg.numThreads, cfg.numGamesPerThread` -> `cfg.numGames`
- `cfg.ppo.policyLayerSizes` -> `cfg.ppo.policy.layerSizes`
- `cfg.ppo.criticLayerSizes` -> `cfg.ppo.critic.layerSizes`
- `cfg.expBufferSize` -> `(experience buffer removed)`
