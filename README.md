# HandyController
Manual controller using your **Hand pose** and **Keyboard** input. Hand pose is recognized via web-cam. \
RL demonstration can be easily obtained by this especially for multi dimensional continuous control environment.

# Demo
**LunarLanderContinuous-v2** \
(index and middle finger) \
![lunar](https://user-images.githubusercontent.com/70328564/212521612-68cd147c-7a7a-4131-b2c9-e821b8404d6c.gif)


**RobogymDactyl** \
(all fingers and F1~4 keys)\
![robogym](https://user-images.githubusercontent.com/70328564/212521615-abb8579a-b548-441e-92ec-716847ef88a2.gif)


# Instruction

Install this package and dependencies.
```
git clone https://github.com/NM512/HandyController.git
cd HandyController
pip install -e .
```

#### Run LunarLanderContinuous-v2 environment.
```
python wrappers/lunarlander.py
```

#### Run Robogym dactyl environment.
Please follow the [offcial guide](https://github.com/openai/robogym) to install Robogym environment.\
Don't forget to install Mujoco.
```
python wrappers/robogym_dactyl.py
```

#### Use inside your custom environment
Please make a wrapper for your own environment with reference to scripts inside **wrappers** folder. \
Basically, only thing you have to consider is a correspondence between action space and input interface. \
Currentry hand pose and keyboard input are supported as input interface.
