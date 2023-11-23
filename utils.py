import numpy as np

from pydrake.visualization import ModelVisualizer
from pydrake.systems.framework import DiagramBuilder

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    CompositeTrajectory,
	Parser,
    PathParameterizedTrajectory,
    PiecewisePolynomial,
	RigidTransform,
    RollPitchYaw,
)

tree_model_name = "tree"
your_model_filename = "/Users/udayan/Documents/SEAS/6.4212/6.4212Project/TreeGeneration/tree.sdf" # Write the absolute path to your file here

def visualize_model(meshcat):
    visualizer = ModelVisualizer(meshcat=meshcat)
    visualizer.parser().AddModels(your_model_filename)
    visualizer.Run(loop_once=True)

def make_scenario_data(tree_model_name, iiwa_R, iiwa_P, iiwa_Y, iiwa_x, iiwa_y, iiwa_z, q_default):
    scenario_data = f"""
directives:
- add_model:
    name: {tree_model_name}
    file: file://{your_model_filename}
    default_free_body_pose:
        branch_a: # Change here!
            translation: [0, 0, 1]
            rotation: !Rpy {{ deg: [0, 0, 0] }}
- add_frame:
    name: tree_on_world
    X_PF:
        base_frame: world
        translation: [0, 0, 0.5]
        rotation: !Rpy {{ deg: [0, 0, 0] }}
- add_weld:
    parent: tree_on_world
    child: {tree_model_name}::branch_a
- add_model:
      name: iiwa
      file: package://drake/manipulation/models/iiwa_description/iiwa7/iiwa7_no_collision.sdf
      default_joint_positions:
        iiwa_joint_1: [{ q_default[0] }]
        iiwa_joint_2: [{ q_default[1] }]
        iiwa_joint_3: [{ q_default[2] }]
        iiwa_joint_4: [{ q_default[3] }]
        iiwa_joint_5: [{ q_default[4] }]
        iiwa_joint_6: [{ q_default[5] }]
        iiwa_joint_7: [{ q_default[6] }]
- add_frame:
    name: iiwa_on_world
    X_PF:
      base_frame: world
      translation: [{iiwa_x}, {iiwa_y}, {iiwa_z}]
      rotation: !Rpy {{ deg: [{iiwa_R}, {iiwa_P}, {iiwa_Y}] }}
- add_weld:
    parent: iiwa_on_world
    child: iiwa::iiwa_link_0
- add_model:
    name: wsg
    file: package://drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_with_tip.sdf
    default_joint_positions:
      left_finger_sliding_joint: [-0.02]
      right_finger_sliding_joint: [0.02]
- add_frame:
    name: wsg_on_iiwa
    X_PF:
      base_frame: iiwa_link_7
      translation: [0, 0, 0.114]
      rotation: !Rpy {{ deg: [90, 0, 90] }}
- add_weld:
    parent: wsg_on_iiwa
    child: wsg::body
"""
    return scenario_data

def CreateIiwaControllerPlant(q_default, iiwa_R, iiwa_P, iiwa_Y, iiwa_x, iiwa_y, iiwa_z, visualize=False):
    """creates plant that includes only the robot and gripper, used for controllers."""
    robot_sdf_path = "package://drake/manipulation/models/iiwa_description/iiwa7/iiwa7_no_collision.sdf"
    #gripper_sdf_path = "package://drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_no_tip.sdf"
    gripper_sdf_path = "package://drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_welded_fingers.sdf"
    sim_timestep = 1e-3
    #plant_robot = MultibodyPlant(sim_timestep)
    builder = DiagramBuilder()
    plant_robot, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_timestep)
    parser = Parser(plant=plant_robot)
    parser.AddModelsFromUrl(robot_sdf_path)
    parser.AddModelsFromUrl(gripper_sdf_path)
    # NOTE FOR ME-  CAN USE ADD MODELS FROM STRING
    plant_robot.WeldFrames(
        frame_on_parent_F=plant_robot.world_frame(),
        frame_on_child_M=plant_robot.GetFrameByName("iiwa_link_0"),
        X_FM=RigidTransform(
            RollPitchYaw(iiwa_R / 180 * np.pi, iiwa_P / 180 * np.pi, iiwa_Y / 180 * np.pi), np.array([iiwa_x, iiwa_y, iiwa_z])
        ),
    )
    plant_robot.WeldFrames(
        frame_on_parent_F=plant_robot.GetFrameByName("iiwa_link_7"),
        frame_on_child_M=plant_robot.GetFrameByName("body"),
        X_FM=RigidTransform(
            RollPitchYaw(np.pi / 2, 0, np.pi / 2), np.array([0, 0, 0.114])
        ),
    )
    plant_robot.mutable_gravity_field().set_gravity_vector([0, 0, 0])

    if visualize:
        #meshcat.Delete()
        visualizer = MeshcatVisualizer.AddToBuilder(
            builder,
            scene_graph,
            meshcat,
            MeshcatVisualizerParams(role=Role.kIllustration),
        )
    else:
        visualizer = None

    plant_robot.Finalize()
    diagram = builder.Build()

    plant_robot.SetDefaultPositions(q_default)

    return plant_robot, visualizer, diagram

def concatenate_trajectories(initial_trajectory, trajectories):
    if len(trajectories) == 0:
        return initial_trajectory

    adjusted_trajectories = [initial_trajectory]
    previous_end = initial_trajectory.end_time()
    intervals = [[0, previous_end]]

    for trajectory in trajectories:
        trajectory_end = trajectory.end_time()
        new_end = previous_end + trajectory_end
        print([previous_end, new_end])
        intervals.append([previous_end, new_end])
        time_shift = PiecewisePolynomial.FirstOrderHold([previous_end, new_end], np.array([[0], [trajectory_end]]).T)
        adjusted_trajectory = PathParameterizedTrajectory(trajectory, time_shift)
        adjusted_trajectories.append(adjusted_trajectory)
        previous_end = new_end

    return CompositeTrajectory(adjusted_trajectories), intervals

def generate_gripper_trajectories(initial_gripper_traj, grasp_states, intervals):

    if len(grasp_states) == 0:
        return initial_gripper_traj

    composite_trajectories = [initial_gripper_traj]
    for interval, grasp_state in zip(intervals[1:], grasp_states):
        if grasp_state == False:
            grasp_val = 0.1
        else:
            grasp_val = -0.05
        gripper_trajectory = PiecewisePolynomial.FirstOrderHold(interval, np.array([[grasp_val], [grasp_val]]).T)
        composite_trajectories.append(gripper_trajectory)
    return CompositeTrajectory(composite_trajectories)
