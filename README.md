# This repository show how to use a robot with isaac sim and the curobo librairie cumotion

The application is sparated in two part :

1. The simulation with isaac sim omniverse (trajectory generation with rmpflow) -> [click here](with_rmpflow)
2. The trajectory generation with Curobo and the library Cumotion (with motion_gen) -> [click here](with_motion_gen)

> [!Note]  
> **Documentation :**  
> [Official IsaacSim tutorial](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_adding_new_manipulator.html)  
>[Official IsaacSim core documentation](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html?highlight=basetask#)  
>[Official Curobo documentation](https://curobo.org/get_started/2a_python_examples.html)  







> [!WARNING] 
> **If you want to use a robot imported from the urdf importer of IsaacSim you need to make this modification to your .USD file.**


1. Open the **.usd** file of your robot with isaac sim

    ![Alt text](/img/usd_before%20modification.png "usd file before modification")

2. Place the frame of your and effector here **xarmlink_tcp** under the root joint

    ![Alt text](/img/frame_tree.png "usd file after modification")

> [!Note]
> **I adressed the problem to the Nvidia team you can check this discussion if there are any updates.** [Discussion on the Nvidia forum](https://forums.developer.nvidia.com/t/adding-a-new-manipulator-example-doesnt-work/319273/6)