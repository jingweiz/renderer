# **renderer**
_a ROS package to turn your point clouds into a simulator for training Deep Reinforcement Learning agents_

(paired w/ files to interface w/ DRL agents, e.g. [pytorch-rl](https://github.com/jingweiz/pytorch-rl))
*******


## What is this?
> This repo contains a ROS package that can be used to turn your point cloud (e.g. `.pcd`'s) into a simulator, where your agent can wonder around. The `/render` service will render **RGB** or **depth** images from the current pose of your agent. Thus this repo can be helpful if you want to train some DRL agents (e.g. [pytorch-rl](https://github.com/jingweiz/pytorch-rl)) out of the point cloud data you collected, for example, of your office. This repo is developed w/ the help of [@onlytailei](https://github.com/onlytailei).

![rviz](/assets/rviz.png)

*******


## Tested system configurations:
- Ubuntu 16.04
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
*******


## How to run:
* Compile the `renderer` package:
> * clone the package into `YOUR_ROS_WORKSPACE_DIRECTORY/src/`
> * `catkin_make` in `YOUR_ROS_WORKSPACE_DIRECTORY`
* Launch node:
> `roslaunch renderer renderer.launch`
* Call service (`action_ind={3`(reset)`,0`(go straight)`,1`(turn left)`,2`(turn right)`}`):
> `rosservice call /render action_ind`
*******


## How to pair w/ [pytorch-rl](https://github.com/jingweiz/pytorch-rl)?
* Turn off the `rviz` visualizations in `./launch/renderer.launch` (cos we don't want to slow down the expensive DRL trainings :P):
> * `line2-3`: comment off to turn off `rviz`
> * `line5`: set `display` to `false`
* Copy the generated interfacing files from `YOUR_ROS_WORKSPACE_DIRECTORY` into `pytorch-rl`:
> ```cp -r YOUR_ROS_WORKSPACE_DIRECTORY/devel/lib/python.../dist-packages/renderer YOUR_pytorch-rl_DIRECTORY/```
* Copy the provided env wrapper from `YOUR_ROS_WORKSPACE_DIRECTORY` into `pytorch-rl`:
> ```cp YOUR_ROS_WORKSPACE_DIRECTORY/envs/rendererEnv.py YOUR_pytorch-rl_DIRECTORY/core/envs/```
*******


## About the point clouds:
The `pcd/office.pcd` & `pcd/chair.pcd` are provided by my awesome colleagues `Tim Caselitz (@caselitz)` and `Michael Krawez`.
If you want to use your own point clouds, you need to:
> * put those `.pcd`'s into `./data/`
> * modify the magic numbers in `line522-527` & `line549-554` in `./src/renderer.cpp` to align the clouds w/ the axis
> * finally `rosservice call /preprocess` to save the aligned clouds into `./pcd/`

*******


## The following paper might be interesting to take a look:)
> [VR Goggles for Robots: Real-to-sim Domain Adaptation for Visual Control](https://arxiv.org/abs/1802.00265): This paper deals with the _**reality gap**_ from a novel perspective, targeting transferring Deep Reinforcement Learning (DRL) policies learned in simulated environments to the real-world domain for visual control tasks. Instead of adopting the common solutions to the problem by increasing the visual fidelity of synthetic images output from simulators during the training phase, this paper seeks to tackle the problem by translating the real-world image streams back to the synthetic domain during the deployment phase, to _**make the robot feel at home**_.
We propose this as a lightweight, flexible, and efficient solution for visual control, as 1) no extra transfer steps are required during the expensive training of DRL agents in simulation; 2) the trained DRL agents will not be constrained to being deployable in only one specific real-world environment; 3) the policy training and the transfer operations are decoupled, and can be conducted in parallel.
Besides this, we propose a conceptually simple yet very effective _**shift loss**_ to constrain the consistency between subsequent frames, eliminating the need for optical flow.
We validate the _**shift loss**_ for _**artistic style transfer for videos**_ and _**domain adaptation**_, and validate our visual control approach in real-world robot experiments. A video of our results is available at:
\url{https://goo.gl/b1xz1s}.
.

```
@article{zhang2018vr,
  title={Vr goggles for robots: Real-to-sim domain adaptation for visual control},
  author={Zhang, Jingwei and Tai, Lei and Xiong, Yufeng and Liu, Ming and Boedecker, Joschka and Burgard, Wolfram},
  journal={arXiv preprint arXiv:1802.00265},
  year={2018}
}
```
