import sys
import os
import struct
import numpy as np
from collections import OrderedDict, namedtuple

import torch

# Remove if you dont have an rtx nvidea card, or maybe some other types look up if your card supports this
torch.backends.cuda.matmul.allow_tf32 = True 

torch.backends.cudnn.benchmark = True
torch.cuda.empty_cache()

from rlgym_ppo.ppo import DiscreteFF, ValueEstimator

def model_info_from_dict(loaded_dict):
	state_dict = OrderedDict(loaded_dict)

	bias_counts = []
	weight_counts = []
	for key, value in state_dict.items():
		if ".weight" in key:
			weight_counts.append(value.numel())
		if ".bias" in key:
			bias_counts.append(value.size(0))
			
	inputs = int(weight_counts[0] / bias_counts[0])
	outputs = bias_counts[-1]
	layer_sizes = bias_counts[:-1]
	
	return inputs, outputs, layer_sizes

def rename_model_state_dict(state_dict):
	keys = []
	for key in state_dict.keys():
		keys.append(key)
	
	for key in keys:
		if not "model." in key:
			state_dict["model." + key] = state_dict[key]
			del state_dict[key]
			
	return state_dict
	
def make_models_from_dicts(policy_state_dict, critic_state_dict):
	policy_inputs, policy_outputs, policy_sizes = model_info_from_dict(policy_state_dict)
	critic_inputs, critic_outputs, critic_sizes = model_info_from_dict(critic_state_dict)
	
	device = torch.device("cpu")
	policy = DiscreteFF(policy_inputs, policy_outputs, policy_sizes, device)
	critic = ValueEstimator(critic_inputs, critic_sizes, device)
	return policy, critic
	
def main():
	if len(sys.argv) != 3:
		sys.exit("Invalid argument count, arguments should be \"<to_cpp/to_python> <checkpoint path>\"")

	to_arg = sys.argv[1]
	if to_arg == 'to_cpp':
		to_cpp = True
	elif to_arg == 'to_python':
		to_cpp = False
	else:
		sys.exit("Invalid arguments, please specify \"-save\" or \"-load\" for the first argument.")
		
	path = sys.argv[2]
	
	device = torch.device('cpu')
	
	if to_cpp:
		print("Loading state dicts...")
		policy_state_dict = torch.load(os.path.join(path, "PPO_POLICY.pt"))
		critic_state_dict = torch.load(os.path.join(path, "PPO_VALUE_NET.pt"))

		print("Creating models...")
		policy, critic = make_models_from_dicts(policy_state_dict, critic_state_dict)
		
		print("Applying state dicts...")
		
		policy.load_state_dict(policy_state_dict)
		critic.load_state_dict(critic_state_dict)
		
		print("Saving for GigaLearnCPP...")
		policy_ts = torch.jit.script(policy.model)
		critic_ts = torch.jit.script(critic.model)
		
		output_path = "cpp_checkpoint"
		os.makedirs(output_path, exist_ok = True)
		torch.jit.save(policy_ts, output_path + "/PPO_POLICY.lt")
		torch.jit.save(critic_ts, output_path + "/PPO_CRITIC.lt")
		
	else:
		print("Loading models...")
		policy = torch.jit.load(os.path.join(path, "PPO_POLICY.lt"))
		critic = torch.jit.load(os.path.join(path, "PPO_CRITIC.lt"))
		policy_inputs, policy_outputs, policy_sizes = model_info_from_dict(policy.state_dict())
		print("Policy sizes:", policy_inputs, policy_sizes, policy_outputs)
		
		print("Creating optimizers...")
		policy_py, critic_py = make_models_from_dicts(policy.state_dict(), critic.state_dict())
		
		policy_optim = torch.optim.Adam(policy_py.parameters())
		critic_optim = torch.optim.Adam(critic_py.parameters())
		
		print("Saving for rlgym-ppo...")
		output_path = "python_checkpoint"
		os.makedirs(output_path, exist_ok = True)
		
		policy_state_dict = rename_model_state_dict(policy.state_dict())
		critic_state_dict = rename_model_state_dict(critic.state_dict())
		
		torch.save(policy_state_dict, output_path + "/PPO_POLICY.pt")
		torch.save(critic_state_dict, output_path + "/PPO_VALUE_NET.pt")
		torch.save(policy_optim.state_dict(), output_path + "/PPO_POLICY_OPTIMIZER.pt")
		torch.save(critic_optim.state_dict(), output_path + "/PPO_VALUE_NET_OPTIMIZER.pt")
		
	print(
		"Done! Partial " + ("GigaLearnCPP" if to_cpp else "rlgym-ppo") + 
		" checkpoint generated at \"" + output_path + "\"."
	)
	print(
		"NOTE: Optimizer conversion is not yet supported, so optimizers will be reset. " +
		"It may take a few iterations for learning to recover."
	)
	print("NOTE: State JSON not included (just make a new one and copy over the vars you want).")
	print("NOTE: Make sure the obs/actions/model sizes all match.")
	
if __name__=='__main__':
	main()