import numpy as np

from pydrake.visualization import ModelVisualizer
from pydrake.systems.framework import DiagramBuilder
from manipulation.utils import ConfigureParser

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    CompositeTrajectory,
	FixedOffsetFrame,
	MakeRenderEngineVtk,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
	Parser,
    PathParameterizedTrajectory,
    PiecewisePolynomial,
	RigidTransform,
    Role,
    RollPitchYaw,

	# cloned from MakeManipulationStation
    AbstractValue,
    Adder,
    AddMultibodyPlantSceneGraph,
    BallRpyJoint,
    BaseField,
    Box,
    CameraInfo,
    Capsule,
    ClippingRange,
    CoulombFriction,
    Cylinder,
    Demultiplexer,
    DepthImageToPointCloud,
    DepthRange,
    DepthRenderCamera,
    DiagramBuilder,
    DifferentialInverseKinematicsIntegrator,
    DifferentialInverseKinematicsParameters,
    GeometryInstance,
    InverseDynamicsController,
    LeafSystem,
    MakeMultibodyStateToWsgStateSystem,
    MakePhongIllustrationProperties,
    MakeRenderEngineVtk,
    ModelInstanceIndex,
    MultibodyPlant,
    Parser,
    PassThrough,
    PrismaticJoint,
    RenderCameraCore,
    RenderEngineVtkParams,
    RevoluteJoint,
    Rgba,
    RgbdSensor,
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
    SchunkWsgPositionController,
    SpatialInertia,
    Sphere,
    StateInterpolatorWithDiscreteDerivative,
    UnitInertia,
)

tree_model_name = "tree"
your_model_filename = "/Users/udayan/Documents/SEAS/6.4212/6.4212Project/TreeGeneration/tree.sdf" # Write the absolute path to your file here

iiwa_x, iiwa_y, iiwa_z = [-0.3, 0.6, 0.5]
iiwa_R, iiwa_P, iiwa_Y = [0, 0, 90]
q_default = np.array([-0.2, 0.79, 0.32, -1.76, -0.36, 0.64, -0.73])

iiwa_name = "iiwa"
wsg_name = "wsg"

iiwa_exact_collisions_filename = "/Users/udayan/Documents/SEAS/6.4212/6.4212Project/iiwa7_exact_collision.sdf"
# file: package://drake/manipulation/models/iiwa_description/iiwa7/iiwa7_with_box_collision.sdf

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
      name: { iiwa_name }
      file: file://{iiwa_exact_collisions_filename}
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
    name: { wsg_name }
    file: package://drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_with_tip.sdf
    default_joint_positions:
      left_finger_sliding_joint: [-0.02]
      right_finger_sliding_joint: [0.02]
- add_frame:
    name: wsg_on_iiwa
    X_PF:
      base_frame: iiwa_link_7
      translation: [0, 0, 0.09]
      rotation: !Rpy {{ deg: [90, 0, 90] }}
- add_weld:
    parent: wsg_on_iiwa
    child: wsg::body
"""
    return scenario_data


def CreateIiwaControllerPlantCollisions(q_default, iiwa_R, iiwa_P, iiwa_Y, iiwa_x, iiwa_y, iiwa_z, meshcat=None):

    scenario_string = make_scenario_data(tree_model_name, iiwa_R, iiwa_P, iiwa_Y, iiwa_x, iiwa_y, iiwa_z, q_default)

    sim_timestep = 1e-3
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_timestep)
    parser = Parser(plant=plant)
    parser.AddModelsFromString(scenario_string, "dmd.yaml")

    #plant.mutable_gravity_field().set_gravity_vector([0, 0, 0])

    if meshcat is not None:
        visualizer = MeshcatVisualizer.AddToBuilder(
            builder,
            scene_graph,
            meshcat,
            MeshcatVisualizerParams(role=Role.kIllustration),
        )
        collision_visualizer = MeshcatVisualizer.AddToBuilder(
       	    builder,
            scene_graph,
            meshcat,
            MeshcatVisualizerParams(
                prefix="collision", role=Role.kProximity, visible_by_default=False
            ),
    	)
    else:
        visualizer = None
        collision_visualizer = None

    plant.Finalize()
    diagram = builder.Build()

    iiwa_idx = plant.GetModelInstanceByName(iiwa_name)
    plant.SetDefaultPositions(iiwa_idx, q_default)

    return plant, diagram, scene_graph, visualizer, collision_visualizer

def CreateIiwaControllerPlant(q_default, iiwa_R, iiwa_P, iiwa_Y, iiwa_x, iiwa_y, iiwa_z, relevant_contacts=None, visualize=False):
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
    #plant_robot.mutable_gravity_field().set_gravity_vector([0, 0, 0])

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

    #######
	# contact stuff
    if relevant_contacts is not None:
        for contact in relevant_contacts:
            body_on_iiwa_name, body_on_tree_name, X_iiwaContact, contact_force = contact
            body_on_iiwa = plant_robot.GetBodyByName(body_on_iiwa_name)
            contact_frame_name = f"contact_{body_on_iiwa_name}_{body_on_tree_name}"
            contact_frame = FixedOffsetFrame(contact_frame_name, body_on_iiwa, X_iiwaContact)
            plant_robot.AddFrame(contact_frame)

	######

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


class ExtractBodyPoseWithOffset(LeafSystem):
    def __init__(self, body_poses_output_port, body_index, offset):
        LeafSystem.__init__(self)
        self.body_index = body_index
        self.DeclareAbstractInputPort(
            "poses",
            body_poses_output_port.Allocate(),
        )
        self.DeclareAbstractOutputPort(
            "pose",
            lambda: AbstractValue.Make(RigidTransform()),
            self.CalcOutput,
        )
        self._offset = offset

    def CalcOutput(self, context, output):
        poses = self.EvalAbstractInput(context, 0).get_value()
        pose = poses[int(self.body_index)]
        #pose = pose @ self._offset
        pose = self._offset
        output.get_mutable_value().set(pose.rotation(), pose.translation())

def AddRgbdSensorWithPointCloudSystem(
    builder,
    plant,
    scene_graph,
    parent_idx,
    transform,
    name
):

    renderer = "my_renderer"
    if not scene_graph.HasRenderer(renderer):
        scene_graph.AddRenderer(
            renderer, MakeRenderEngineVtk(RenderEngineVtkParams())
        )

    depth_camera = DepthRenderCamera(
        RenderCameraCore(
            renderer,
            CameraInfo(width=40, height=40, fov_y=np.pi / 2.0),
            ClippingRange(near=0.005, far=10.0),
            RigidTransform(),
        ),
        DepthRange(0.005, 10.0),
    )

    rgbd = builder.AddSystem(
        RgbdSensor(
            parent_id=plant.GetBodyFrameIdOrThrow(parent_idx),
            X_PB=transform,
            depth_camera=depth_camera,
            show_window=False,
        )
    )
    rgbd.set_name(name)

    builder.Connect(
        scene_graph.get_query_output_port(),
        rgbd.query_object_input_port(),
    )

    # Export the camera outputs
    builder.ExportOutput(
        rgbd.color_image_output_port(), f"{name}_rgb_image"
    )
    builder.ExportOutput(
        rgbd.depth_image_32F_output_port(), f"{name}_depth_image"
    )
    builder.ExportOutput(
        rgbd.label_image_output_port(), f"{name}_label_image"
    )

    # Add a system to convert the camera output into a point cloud
    to_point_cloud = builder.AddSystem(
        DepthImageToPointCloud(
            camera_info=rgbd.depth_camera_info(),
            fields=BaseField.kXYZs | BaseField.kRGBs,
        )
    )
    builder.Connect(
        rgbd.depth_image_32F_output_port(),
        to_point_cloud.depth_image_input_port(),
    )
    builder.Connect(
        rgbd.color_image_output_port(),
        to_point_cloud.color_image_input_port(),
    )
    camera_pose = builder.AddSystem(
        ExtractBodyPoseWithOffset(
            plant.get_body_poses_output_port(), parent_idx, transform
        )
    )
    builder.Connect(
        plant.get_body_poses_output_port(),
        camera_pose.get_input_port(),
    )
    builder.Connect(
        camera_pose.get_output_port(),
        to_point_cloud.GetInputPort("camera_pose"),
    )
    # Export the point cloud output.
    builder.ExportOutput(
        to_point_cloud.point_cloud_output_port(),
        f"{name}_point_cloud",
    )
    return to_point_cloud, depth_camera



def AddIiwa(plant, collision_model="no_collision"):
    parser = Parser(plant)
    iiwa = parser.AddModelsFromUrl(
        f"package://drake/manipulation/models/iiwa_description/iiwa7/iiwa7_{collision_model}.sdf"
    )[0]
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("iiwa_link_0"))

    # Set default positions:
    q0 = [0.0, 0.1, 0, -1.2, 0, 1.6, 0]
    index = 0
    for joint_index in plant.GetJointIndices(iiwa):
        joint = plant.get_mutable_joint(joint_index)
        if isinstance(joint, RevoluteJoint):
            joint.set_default_angle(q0[index])
            index += 1

    return iiwa

def AddWsg(
    plant, iiwa_model_instance, roll=np.pi / 2.0, welded=False, sphere=False
):
    parser = Parser(plant)
    ConfigureParser(parser)
    if welded:
        if sphere:
            file = "package://manipulation/schunk_wsg_50_welded_fingers_sphere.sdf"
        else:
            file = "package://manipulation/schunk_wsg_50_welded_fingers.sdf"
    else:
        file = "package://drake/manipulation/models/wsg_50_description/sdf/schunk_wsg_50_with_tip.sdf"

    directives = f"""
directives:
- add_model:
    name: gripper
    file: {file}
"""
    gripper = parser.AddModelsFromString(directives, ".dmd.yaml")[0]

    X_7G = RigidTransform(RollPitchYaw(np.pi / 2.0, 0, roll), [0, 0, 0.09])
    plant.WeldFrames(
        plant.GetFrameByName("iiwa_link_7", iiwa_model_instance),
        plant.GetFrameByName("body", gripper),
        X_7G,
    )
    return gripper


def MakeManipulationStation(
    model_directives=None,
    filename=None,
    time_step=0.002,
    iiwa_prefix="iiwa",
    wsg_prefix="wsg",
    camera_prefix="camera",
    prefinalize_callback=None,
    package_xmls=[],
):
    """
    Creates a manipulation station system, which is a sub-diagram containing:
      - A MultibodyPlant with populated via the Parser from the
        `model_directives` argument AND the `filename` argument.
      - A SceneGraph
      - For each model instance starting with `iiwa_prefix`, we add an
        additional iiwa controller system
      - For each model instance starting with `wsg_prefix`, we add an
        additional schunk controller system
      - For each body starting with `camera_prefix`, we add a RgbdSensor

    Args:
        builder: a DiagramBuilder

        model_directives: a string containing any model directives to be parsed

        filename: a string containing the name of an sdf, urdf, mujoco xml, or
        model directives yaml file.

        time_step: the standard MultibodyPlant time step.

        iiwa_prefix: Any model instances starting with `iiwa_prefix` will get
        an inverse dynamics controller, etc attached

        wsg_prefix: Any model instance starting with `wsg_prefix` will get a
        schunk controller

        camera_prefix: Any bodies in the plant (created during the
        plant_setup_callback) starting with this prefix will get a camera
        attached.

        prefinalize_callback: A function, setup(plant), that will be called
        with the multibody plant before calling finalize.  This can be useful
        for e.g. adding additional bodies/models to the simulation.

        package_xmls: A list of filenames to be passed to
        PackageMap.AddPackageXml().  This is useful if you need to add more
        models to your path (e.g. from your current working directory).
    """
    builder = DiagramBuilder()

    # Add (only) the iiwa, WSG, and cameras to the scene.
    plant, scene_graph = AddMultibodyPlantSceneGraph(
        builder, time_step=time_step
    )
    parser = Parser(plant)
    for p in package_xmls:
        parser.package_map().AddPackageXml(p)
    ConfigureParser(parser)
    if model_directives:
        parser.AddModelsFromString(model_directives, ".dmd.yaml")
    if filename:
        parser.AddModelsFromUrl(filename)
    if prefinalize_callback:
        prefinalize_callback(plant)
    plant.Finalize()

    for i in range(plant.num_model_instances()):
        model_instance = ModelInstanceIndex(i)
        model_instance_name = plant.GetModelInstanceName(model_instance)

        if model_instance_name.startswith(iiwa_prefix):
            num_iiwa_positions = plant.num_positions(model_instance)

            # I need a PassThrough system so that I can export the input port.
            iiwa_position = builder.AddSystem(PassThrough(num_iiwa_positions))
            builder.ExportInput(
                iiwa_position.get_input_port(),
                model_instance_name + "_position",
            )
            builder.ExportOutput(
                iiwa_position.get_output_port(),
                model_instance_name + "_position_commanded",
            )

            # Export the iiwa "state" outputs.
            demux = builder.AddSystem(
                Demultiplexer(2 * num_iiwa_positions, num_iiwa_positions)
            )
            builder.Connect(
                plant.get_state_output_port(model_instance),
                demux.get_input_port(),
            )
            builder.ExportOutput(
                demux.get_output_port(0),
                model_instance_name + "_position_measured",
            )
            builder.ExportOutput(
                demux.get_output_port(1),
                model_instance_name + "_velocity_estimated",
            )
            builder.ExportOutput(
                plant.get_state_output_port(model_instance),
                model_instance_name + "_state_estimated",
            )

            # Make the plant for the iiwa controller to use.
            controller_plant = MultibodyPlant(time_step=time_step)
            # TODO: Add the correct IIWA model (introspected from MBP)
            if plant.num_positions(model_instance) == 3:
                controller_iiwa = AddPlanarIiwa(controller_plant)
            else:
                controller_iiwa = AddIiwa(controller_plant)
            AddWsg(controller_plant, controller_iiwa, welded=True)
            controller_plant.Finalize()

            # Add the iiwa controller
            kp = 100
            iiwa_controller = builder.AddSystem(
                InverseDynamicsController(
                    controller_plant,
                    kp=[kp] * num_iiwa_positions,
                    ki=[1] * num_iiwa_positions,
                    kd=[20] * num_iiwa_positions,
                    has_reference_acceleration=False,
                )
            )
            print(f"kp: {kp}")
            iiwa_controller.set_name(model_instance_name + "_controller")
            builder.Connect(
                plant.get_state_output_port(model_instance),
                iiwa_controller.get_input_port_estimated_state(),
            )

            # Add in the feed-forward torque
            adder = builder.AddSystem(Adder(2, num_iiwa_positions))
            builder.Connect(
                iiwa_controller.get_output_port_control(),
                adder.get_input_port(0),
            )
            # Use a PassThrough to make the port optional (it will provide zero
            # values if not connected).
            torque_passthrough = builder.AddSystem(
                PassThrough([0] * num_iiwa_positions)
            )
            builder.Connect(
                torque_passthrough.get_output_port(), adder.get_input_port(1)
            )
            builder.ExportInput(
                torque_passthrough.get_input_port(),
                model_instance_name + "_feedforward_torque",
            )
            builder.Connect(
                adder.get_output_port(),
                plant.get_actuation_input_port(model_instance),
            )

            # Add discrete derivative to command velocities.
            desired_state_from_position = builder.AddSystem(
                StateInterpolatorWithDiscreteDerivative(
                    num_iiwa_positions,
                    time_step,
                    suppress_initial_transient=True,
                )
            )
            desired_state_from_position.set_name(
                model_instance_name + "_desired_state_from_position"
            )
            builder.Connect(
                desired_state_from_position.get_output_port(),
                iiwa_controller.get_input_port_desired_state(),
            )
            builder.Connect(
                iiwa_position.get_output_port(),
                desired_state_from_position.get_input_port(),
            )

            # Export commanded torques.
            builder.ExportOutput(
                adder.get_output_port(),
                model_instance_name + "_torque_commanded",
            )
            builder.ExportOutput(
                adder.get_output_port(),
                model_instance_name + "_torque_measured",
            )

            builder.ExportOutput(
                plant.get_generalized_contact_forces_output_port(
                    model_instance
                ),
                model_instance_name + "_torque_external",
            )

        elif model_instance_name.startswith(wsg_prefix):
            # Wsg controller.
            wsg_controller = builder.AddSystem(SchunkWsgPositionController())
            wsg_controller.set_name(model_instance_name + "_controller")
            builder.Connect(
                wsg_controller.get_generalized_force_output_port(),
                plant.get_actuation_input_port(model_instance),
            )
            builder.Connect(
                plant.get_state_output_port(model_instance),
                wsg_controller.get_state_input_port(),
            )
            builder.ExportInput(
                wsg_controller.get_desired_position_input_port(),
                model_instance_name + "_position",
            )
            builder.ExportInput(
                wsg_controller.get_force_limit_input_port(),
                model_instance_name + "_force_limit",
            )
            wsg_mbp_state_to_wsg_state = builder.AddSystem(
                MakeMultibodyStateToWsgStateSystem()
            )
            builder.Connect(
                plant.get_state_output_port(model_instance),
                wsg_mbp_state_to_wsg_state.get_input_port(),
            )
            builder.ExportOutput(
                wsg_mbp_state_to_wsg_state.get_output_port(),
                model_instance_name + "_state_measured",
            )
            builder.ExportOutput(
                wsg_controller.get_grip_force_output_port(),
                model_instance_name + "_force_measured",
            )

    # Export "cheat" ports.
    builder.ExportOutput(scene_graph.get_query_output_port(), "query_object")
    builder.ExportOutput(
        plant.get_contact_results_output_port(), "contact_results"
    )
    builder.ExportOutput(
        plant.get_state_output_port(), "plant_continuous_state"
    )
    builder.ExportOutput(plant.get_body_poses_output_port(), "body_poses")

    return builder, plant, scene_graph


