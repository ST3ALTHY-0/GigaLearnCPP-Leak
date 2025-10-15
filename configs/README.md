# GigaLearn Training Configurations

This directory contains modular configuration files for training Rocket League bots.

## Directory Structure

```
configs/
├── learner/          # LearnerConfig settings (network sizes, learning rates, etc.)
├── rewards/          # Reward function configurations
├── presets/          # Complete training setups combining learner + reward configs
└── README.md         # This file
```

## Usage

### Option 1: Use Presets
Choose a complete preset from `presets/` that combines learner and reward configs:
- `beginner_bot.json` - Simple training for basic bots
- `advanced_training.json` - High-quality training with advanced mechanics
- `speed_training.json` - Optimized for maximum training speed

### Option 2: Mix and Match
Combine any learner config with any reward config:
- Learner configs: `fast_training.json`, `high_quality.json`, `debug.json`
- Reward configs: `ball_chase.json`, `advanced_mechanics.json`, `competitive.json`

## Learner Config Parameters

- **numGames**: Number of parallel games (affects CPU/RAM usage)
- **tickSkip**: Physics simulation speed (higher = faster but less precise)
- **tsPerItr**: Timesteps per training iteration
- **batchSize/miniBatchSize**: Neural network batch sizes
- **epochs**: Training epochs per iteration
- **Learning rates**: Policy and critic learning rates
- **Network sizes**: Neural network layer configurations

## Reward Config Parameters

- **type**: Reward class name (e.g., "GoalReward", "DribbleReward")
- **weight**: Reward magnitude multiplier
- **zeroSum**: Whether to make competitive (true) or cooperative (false)
- **teamSpirit**: For zero-sum rewards, how much teammates share (0-1)
- **params**: Reward-specific parameters

## Creating New Configs

1. Copy an existing config file
2. Modify the parameters as needed
3. Test with small-scale training first
4. Document your changes in the "description" field

## Performance Guidelines

- **Fast Training**: Small networks, fewer games, higher tickSkip
- **Quality Training**: Large networks, more games, lower tickSkip
- **Memory Usage**: Reduce batchSize/miniBatchSize if running out of VRAM
- **CPU Usage**: Reduce numGames if CPU is bottleneck