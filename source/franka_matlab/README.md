#franka_matlab 0.2.1
-------------------
 
## Dev Environment
 
 - Tested with **MATLAB2019a**, till **MATLAB2021a**
 - Tested in **Ubuntu Linux 18.04, 20.04** with **PREEMPT_RT kernel patch** and with **Intel** CPU
 - libfranka version support **0.9.0**(System version 4.2.1)
 
## !! Important !! 
 
 - franka_matlab project is based on [libfranka](https://frankaemika.github.io/docs/). It is highly recommended, that in order to evaluate the configuration and
   most importantly the quality of the Ethernet communication, to try and first build and run the C++ demos of libfranka before any attempt with Simulink.
   Please ensure that the quality of the Ethernet communication is proper ([recommended tests](https://frankaemika.github.io/docs/troubleshooting.html#network-bandwidth-delay-and-jitter-test))
   and please pay !!special attention!! to [disabling any possible throttling of your host CPU.](https://frankaemika.github.io/docs/troubleshooting.html)
 - If the C++ demos of libfranka are performing nominally but at the same time the Simulink demos equivalent don't, please report that back to us!
 - For utilization of the library with external mode we recommend using the franka\_emika\_panda.tlc or franka\_emika\_panda\_shrlib.tlc that contain a version of ExtMode capable of properly dealing with the real time 1ms loop.
 
## Instructions
 
 - Make sure that you follow all the [installation instructions for libfranka](https://frankaemika.github.io/docs/installation_linux.html) for properly setting up your environment.
 - Add path of franka_matlab to your project with `>> addpath(franka_matlab path);`
 - Initialize the project with `>> init_franka_matlab();` (recommended) or if you have already installed libfranka and want to utilize that initialize with `>> init_franka_matlab(libfranka_folder path);` In the later case libfranka should have been built with position independent option ON!(-fPIC)
 - (Optional) By default the system target for code gen is the custom **'franka_emika_panda.tlc'**. You can also set either 'ert.tlc' or 'grt.tlc' with e.g. `>> set_code_gen_target('ert')`; Just be aware
   that **in case of ert and grt the external mode could disrupt the real time process**. **'franka_emika_panda.tlc' is specifically designed to avoid that.**
 - Mexing Simulink and Matlab libraries is necessary only first time with `>> mex_franka_simulink_library();` and `>> mex_franka_matlab_library();`
 
## Running the demos
 
In the demos folder you'll find a set of libfranka equivalent examples, implemented in Simulink with the Franka Simulink Library. Feel free
to experiment and adjust them and expand them to fit your project's needs.

Please note that the demos will generate code in the 'work' folder automatically. If you continue working on another project the code generated will still be generated in the work folder. 
You need to change that manually if that's you intention. We provide the helper script `set_code_gen_target_path(<your desired new code gen path>)` for that.
 
 - In the file demos_common_configs.m the is a set of default parameters that are necessary to build and run the model. They are automatically loaded
   when the model is opened. Necessary in that case is the definition of the **robot_ip**. We recommend defining the robot_ip directly in the demos_common_configs.m file.
 - Building the model to a standalone executable can happen with the 1st white UIButton in the bottom of the model.
 - After successfully building you can run the executable through a terminal(recommended in order to be able to inspect any potential exceptions). 
   Make sure you've exported the libfranka.so first `$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<place libfranka build folder path>`
 - The executable now should wait for a connection with External Mode. You can now connect with external mode through the Simulink Model.
 - Alternatively for executing the standalone executable and connecting with external mode you can double click the green UIButton in the bottom of the model.
   In case of this automated proceedure the exceptions thrown can not be directly observed as the executable is running in the background.
 - The "play' button should now have turned green in the Simulink model. Make sure that the brakes are open, the user button is released and the robot is 
   in idle state.
 - You can now press the play button! The model should execute.
 
## Minimum Simulink Project with franka_matlab from scratch
 
 After making sure that all installation instructions are followed through, franka_matlab is in the path `>> addpath(franka_matlab path);` 
 and properly initialized `>> init_franka_matlab();` (these two steps need to happen every time Matlab is instantiated. We suggest that 
 these two commands are added in a startup script.
 
 - Open a Blank Simulink Model.
 - In Simulink Library browser, select Franka Emika Library and add the "Apply Control" block to your model. Exactly one instance of this
   block is needed for every project.
 - Double click on the block to explore and modify the settings. For this example we choose the Control Mode **"Joint Positions - Internal Controller Mode"**.
   For the rest of the parameters we suggest defining the variables in the workspace. Running the the script `>> demos_common_configs();` will automatically define a set of standard default parameters.
 - Define the robot ip `>> robot_ip = 'place your robot IP here'`
 - You can feed the input **q_d** of the block with the q_d initial state of the robot from the block Read Initial Robot State.
 - You can also add the block **Read Robot State** and some scopes and displays for monitoring desired signals from the Robot.
 - Select **"franka_emika_panda.tlc** as System Target File in Simulink Model Configurations->Code Generation.
 - You can now build the model with either `>> rtwbuild('name of your model');` or with the build button in Simulink. If the code generation artifacts and the final executable
   are not visible to your current project folder take a look in the work folder of franka_matlab. You can change the target folder for code generation with the cmd
   `>> set_code_gen_target_path(<string with your model's name>, <string with target folder>)`.
 - After the standalone executable is generated you can execute it from a terminal. Make sure you export the environment variable LD_LIBRARY_PATH first.
   `$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<place the local libfranka build folder path>`
 - The executable should now wait for the external mode connection before starting. You can now connect with external mode. 
 - Make sure at this point that the brakes of the robot are open and the robot is in idle state. User button should be released.
 - Hitting the play button in Simulink should allow the model to get executed!
 
## Example, programmatically generate Cartesian velocity demo, without external mode.
 
Make sure that you have first **changed the robot ip in demos_common_configs.m**. Make also sure that the parameters are 
properly loaded in the workspace after the set_up_franka_simulink_model() command has been executed.

```matlab
>> set_up_franka_simulink_model('generate_cartesian_velocity_motion');
>> build_standalone('generate_cartesian_velocity_motion');
```
 - in the terminal do: `$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<place the local libfranka build folder path>`
 - execute executable that lies in the work folder!
 
## Example, programmatically generate Cartesian velocity demo, with external mode.
 
Make sure that you have first **changed the robot ip in demos_common_configs.m**. Make also sure that the parameters are 
properly loaded in the workspace after the set_up_franka_simulink_model() command has been executed.

```matlab
>> set_up_franka_simulink_model('generate_cartesian_velocity_motion', 'external_mode_on');
>> build_standalone('generate_cartesian_velocity_motion', '-DON_TARGET_WAIT_FOR_START=1', 'your target path for generated code');
```
 - in a terminal export the libfranka path for dynamic linking: `$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<place the libfranka folder build path>`
 - execute the executable that is generated for your model and lies in the target path you provided!
 - connect with ext mode through your Simulink model. Click play to start control!
 
## Notice
 
 - **Multiple instances** for the "Apply Control" block are **NOT allowed** in the same system (multiple instances can inserted in corresponding enabled subsystems).
 - **Multiple instances** of the blocks in Franka Simulink library **ARE allowed**. One and only one "Apply Control" block is necessary for all the other blocks to work.
 - For setting up the parameters it is strongly advised to refer to the Matlab file demos_common_configs.m
 - Experimental limited support for Matlab functionality through Matlab functions. Experiment with the `>>automatic_error_recovery(robot_ip)` command to get out of error states automatically.
 
 For all troubleshooting please refer to [Franka control interface troubleshooting page](https://frankaemika.github.io/docs/troubleshooting.html)
 Be especially aware of the **Motion stopped due to discontinuities or communication_constraints_violation** problem! See corresponding section in troubleshooting.
 

