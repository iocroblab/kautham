The tampconfig task files contain an intial Kautham problem file, e.g. tiago_mobile_counterA_counterB.xml
Then, the controls that it contain may be changed according to the <Cont> tags that apeerar in each action, e.g. in the PICK:
    <Cont>controls/tiago_simple_only_arm_gripper_counterA.cntr</Cont>
Since the max time allowed to the planner cannot be changed, it will be the one set in the initial Kautham problem.
Then, it is IMPORTANT to set this time to the highest required time, according to the next problems that may be encountered in the TAMP manipulation task.

"KauthamName" is used to define multiple obstacles using the same URDF file.