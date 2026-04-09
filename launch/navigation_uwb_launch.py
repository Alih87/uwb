# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from collections.abc import Generator
import tempfile
from typing import Optional, TypeAlias, Union

import launch
import yaml
from launch import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node, PushRosNamespace, SetParameter
from launch_ros.descriptions import ComposableNode, ParameterFile
#from nav2_common.launch import LaunchConfigAsBool, RewrittenYaml


YamlValue: TypeAlias = Union[str, int, float, bool]


class DictItemReference:

    def __init__(self, dictionary: dict[str, YamlValue], key: str):
        self.dictionary = dictionary
        self.dictKey = key

    def key(self) -> str:
        return self.dictKey

    def setValue(self, value: YamlValue) -> None:
        self.dictionary[self.dictKey] = value


class RewrittenYaml(launch.Substitution):
    """
    Substitution that modifies the given YAML file.

    Used in launch system
    """

    def __init__(
        self,
        source_file: launch.SomeSubstitutionsType,
        param_rewrites: dict[str, launch.SomeSubstitutionsType],
        root_key: Optional[launch.SomeSubstitutionsType] = None,
        key_rewrites: Optional[dict[str, launch.SomeSubstitutionsType]] = None,
        value_rewrites: Optional[dict[str, launch.SomeSubstitutionsType]] = None,
        convert_types: bool = False,
        out_dir: Optional[launch.SomeSubstitutionsType] = None,
    ) -> None:
        super().__init__()
        """
        Construct the substitution

        :param: source_file the original YAML file to modify
        :param: param_rewrites mappings to replace
        :param: root_key if provided, the contents are placed under this key
        :param: key_rewrites keys of mappings to replace
        :param: value_rewrites values to replace
        :param: convert_types whether to attempt converting the string to a number or boolean
        :param: out_dir if provided, the directory where the temporary YAML file will be created
        """

        # import here to avoid loop
        from launch.utilities import normalize_to_list_of_substitutions

        self.__source_file: list[launch.Substitution] = \
            normalize_to_list_of_substitutions(source_file)
        self.__param_rewrites = {}
        self.__key_rewrites = {}
        self.__value_rewrites = {}
        self.__convert_types = convert_types
        self.__root_key = None
        self.__out_dir = None

        for key in param_rewrites:
            self.__param_rewrites[key] = normalize_to_list_of_substitutions(
                param_rewrites[key]
            )
        if key_rewrites is not None:
            for key in key_rewrites:
                self.__key_rewrites[key] = normalize_to_list_of_substitutions(
                    key_rewrites[key]
                )
        if value_rewrites is not None:
            for value in value_rewrites:
                self.__value_rewrites[value] = normalize_to_list_of_substitutions(
                    value_rewrites[value]
                )
        if root_key is not None:
            self.__root_key = normalize_to_list_of_substitutions(root_key)

        if out_dir is not None:
            self.__out_dir = normalize_to_list_of_substitutions(out_dir)

    @property
    def name(self) -> list[launch.Substitution]:
        """Getter for name."""
        return self.__source_file

    def describe(self) -> str:
        """Return a description of this substitution as a string."""
        return ''

    def perform(self, context: launch.LaunchContext) -> str:
        yaml_filename = launch.utilities.perform_substitutions(context, self.name)

        out_dir = None
        if self.__out_dir is not None:
            out_dir = launch.utilities.perform_substitutions(context, self.__out_dir)

        rewritten_yaml = tempfile.NamedTemporaryFile(mode='w', delete=False, dir=out_dir)
        param_rewrites, keys_rewrites, value_rewrites = self.resolve_rewrites(context)

        with open(yaml_filename, 'r') as yaml_file:
            data = yaml.safe_load(yaml_file)

        self.substitute_params(data, param_rewrites)
        self.add_params(data, param_rewrites)
        self.substitute_keys(data, keys_rewrites)
        self.substitute_values(data, value_rewrites)
        if self.__root_key is not None:
            root_key = launch.utilities.perform_substitutions(context, self.__root_key)
            if root_key:
                data = {root_key: data}
        yaml.dump(data, rewritten_yaml)
        rewritten_yaml.close()
        return rewritten_yaml.name

    def resolve_rewrites(self, context: launch.LaunchContext) -> \
            tuple[dict[str, str], dict[str, str], dict[str, str]]:
        resolved_params = {}
        for key in self.__param_rewrites:
            resolved_params[key] = launch.utilities.perform_substitutions(
                context, self.__param_rewrites[key]
            )
        resolved_keys = {}
        for key in self.__key_rewrites:
            resolved_keys[key] = launch.utilities.perform_substitutions(
                context, self.__key_rewrites[key]
            )
        resolved_values = {}
        for value in self.__value_rewrites:
            resolved_values[value] = launch.utilities.perform_substitutions(
                context, self.__value_rewrites[value]
            )
        return resolved_params, resolved_keys, resolved_values

    def substitute_params(self, yaml: dict[str, YamlValue],
                          param_rewrites: dict[str, str]) -> None:
        # substitute leaf-only parameters
        for key in self.getYamlLeafKeys(yaml):
            if key.key() in param_rewrites:
                raw_value = param_rewrites[key.key()]
                key.setValue(self.convert(raw_value))

        # substitute total path parameters
        yaml_paths = self.pathify(yaml)
        for path in yaml_paths:
            if path in param_rewrites:
                # this is an absolute path (ex. 'key.keyA.keyB.val')
                rewrite_val = self.convert(param_rewrites[path])
                yaml_keys = path.split('.')
                yaml = self.updateYamlPathVals(yaml, yaml_keys, rewrite_val)

    def add_params(self, yaml: dict[str, YamlValue],
                   param_rewrites: dict[str, str]) -> None:
        # add new total path parameters
        yaml_paths = self.pathify(yaml)
        for path in param_rewrites:
            if not path in yaml_paths:  # noqa: E713
                new_val = self.convert(param_rewrites[path])
                yaml_keys = path.split('.')
                if 'ros__parameters' in yaml_keys:
                    yaml = self.updateYamlPathVals(yaml, yaml_keys, new_val)

    def substitute_values(
            self, yaml: dict[str, YamlValue],
            value_rewrites: dict[str, str]) -> None:

        def process_value(value: YamlValue) -> YamlValue:
            if isinstance(value, dict):
                for k, v in list(value.items()):
                    value[k] = process_value(v)
                return value
            elif isinstance(value, list):
                return [process_value(v) for v in value]
            elif str(value) in value_rewrites:
                return self.convert(value_rewrites[str(value)])
            return value

        for key in list(yaml.keys()):
            yaml[key] = process_value(yaml[key])

    def updateYamlPathVals(
            self, yaml: dict[str, YamlValue],
            yaml_key_list: list[str], rewrite_val: YamlValue) -> dict[str, YamlValue]:

        for key in yaml_key_list:
            if key == yaml_key_list[-1]:
                yaml[key] = rewrite_val
                break
            key = yaml_key_list.pop(0)
            if isinstance(yaml, list):
                yaml[int(key)] = self.updateYamlPathVals(
                    yaml[int(key)], yaml_key_list, rewrite_val
                )
            else:
                yaml[key] = self.updateYamlPathVals(  # type: ignore[assignment]
                    yaml.get(key, {}),  # type: ignore[arg-type]
                    yaml_key_list,
                    rewrite_val
                )
        return yaml

    def substitute_keys(
            self, yaml: dict[str, YamlValue], key_rewrites: dict[str, str]) -> None:
        if len(key_rewrites) != 0:
            for key in list(yaml.keys()):
                val = yaml[key]
                if key in key_rewrites:
                    new_key = key_rewrites[key]
                    yaml[new_key] = yaml[key]
                    del yaml[key]
                if isinstance(val, dict):
                    self.substitute_keys(val, key_rewrites)

    def getYamlLeafKeys(self, yamlData: dict[str, YamlValue]) -> \
            Generator[DictItemReference, None, None]:
        if not isinstance(yamlData, dict):
            return

        for key in yamlData.keys():
            child = yamlData[key]

            if isinstance(child, dict):
                # Recursively process nested dictionaries
                yield from self.getYamlLeafKeys(child)

            yield DictItemReference(yamlData, key)

    def pathify(
            self, d: Union[dict[str, YamlValue], list[YamlValue], YamlValue],
            p: Optional[str] = None,
            paths: Optional[dict[str, YamlValue]] = None,
            joinchar: str = '.') -> dict[str, YamlValue]:
        if p is None:
            paths = {}
            self.pathify(d, '', paths, joinchar=joinchar)
            return paths

        assert paths is not None
        pn = p
        if p != '':
            pn += joinchar
        if isinstance(d, dict):
            for k in d:
                v = d[k]
                self.pathify(v, str(pn) + str(k), paths, joinchar=joinchar)
        elif isinstance(d, list):
            for idx, e in enumerate(d):
                self.pathify(e, pn + str(idx), paths, joinchar=joinchar)
        else:
            paths[p] = d
        return paths

    def convert(self, text_value: str) -> YamlValue:
        if self.__convert_types:
            # try converting to int or float
            try:
                return float(text_value) if '.' in text_value else int(text_value)
            except ValueError:
                pass

        # try converting to bool
        if text_value.lower() == 'true':
            return True
        if text_value.lower() == 'false':
            return False

        # nothing else worked so fall through and return text
        return text_value

class LaunchConfigAsBool(launch.Substitution):
    """
    Converts a LaunchConfiguration value into a normalized boolean string: 'true' or 'false'.

    Allows CLI arguments like 'True', 'true', '1', 'yes' and 'False', 'false', '0', 'no'.
    Returns a string 'true' or 'false' for use in PythonExpression and IfCondition contexts.
    """

    def __init__(self, name: str) -> None:
        super().__init__()
        self._config = LaunchConfiguration(name)

    def perform(self, context: LaunchContext) -> str:
        value = perform_substitutions(context, [self._config])
        if value.strip().lower() in ['true', '1', 'yes', 'on']:
            return 'True'
        return 'False'

    def describe(self) -> str:
        return f'LaunchConfigAsBool({self._config.describe()})'


def generate_launch_description() -> LaunchDescription:
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfigAsBool('use_sim_time')
    autostart = LaunchConfigAsBool('autostart')
    graph_filepath = LaunchConfiguration('graph')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfigAsBool('use_composition')
    use_intra_process_comms = LaunchConfigAsBool('use_intra_process_comms')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfigAsBool('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_keepout_zones = LaunchConfigAsBool('use_keepout_zones')
    use_speed_zones = LaunchConfigAsBool('use_speed_zones')

    lifecycle_nodes = [
        'controller_server',
        #'smoother_server',
        'planner_server',
        #'route_server',
        'behavior_server',
        #'velocity_smoother',
        #'collision_monitor',
        'bt_navigator',
        #'waypoint_follower',
        #'docking_server',
        #'following_server',
    ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {'autostart': autostart}

    yaml_substitutions = {
        'KEEPOUT_ZONE_ENABLED': use_keepout_zones,
        'SPEED_ZONE_ENABLED': use_speed_zones,
    }

    # RewrittenYaml: Adds namespace to the parameters file as a root key
    # Note: Make sure that all frames are correctly namespaced in the parameters file
    # Do not add namespace to topics in the parameters file, as they will be remapped
    # by the root key only if they are not prefixed with a forward slash.
    # e.g. 'map' will be remapped to '/<namespace>/map', but '/map' will not be remapped.
    # IMPORTANT: to make your yaml file dynamic you can refer to humble branch under
    # nav2_bringup/launch/bringup_launch.py to see how the parameters file is configured
    # using ReplaceString <robot_namespace>
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            value_rewrites=yaml_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_graph_file_cmd = DeclareLaunchArgument(
        'graph',
        default_value='', description='Path to the graph file to load'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True',
    )

    declare_use_intra_process_comms_cmd = DeclareLaunchArgument(
        'use_intra_process_comms',
        default_value='False',
        description='Whether to use intra process communication',
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of container that nodes will load in if use composition',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    declare_use_keepout_zones_cmd = DeclareLaunchArgument(
        'use_keepout_zones', default_value='True',
        description='Whether to enable keepout zones or not'
    )

    declare_use_speed_zones_cmd = DeclareLaunchArgument(
        'use_speed_zones', default_value='True',
        description='Whether to enable speed zones or not'
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            PushRosNamespace(namespace=namespace),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                #remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                remappings=remappings,
            ),
            #Node(
            #    package='nav2_smoother',
            #    executable='smoother_server',
            #    name='smoother_server',
            #    output='screen',
            #    respawn=use_respawn,
            #    respawn_delay=2.0,
            #    parameters=[configured_params],
            #    arguments=['--ros-args', '--log-level', log_level],
            #    remappings=remappings,
            #),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            #Node(
            #    package='nav2_route',
            #    executable='route_server',
            #    name='route_server',
            #    output='screen',
            #    respawn=use_respawn,
            #    respawn_delay=2.0,
            #    parameters=[configured_params, {'graph_filepath': graph_filepath}],
            #    arguments=['--ros-args', '--log-level', log_level],
            #    remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                #remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                remappings=remappings,
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            #Node(
            #    package='nav2_waypoint_follower',
            #    executable='waypoint_follower',
            #    name='waypoint_follower',
            #    output='screen',
            #    respawn=use_respawn,
            #    respawn_delay=2.0,
            #    parameters=[configured_params],
            #    arguments=['--ros-args', '--log-level', log_level],
            #    remappings=remappings,
            #),
            #Node(
            #    package='nav2_velocity_smoother',
            #    executable='velocity_smoother',
            #    name='velocity_smoother',
            #    output='screen',
            #    respawn=use_respawn,
            #    respawn_delay=2.0,
            #    parameters=[configured_params],
            #    arguments=['--ros-args', '--log-level', log_level],
            #    remappings=remappings
            #    + [('cmd_vel', 'cmd_vel_nav')],
            #),
            #Node(
            #    package='nav2_collision_monitor',
            #    executable='collision_monitor',
            #    name='collision_monitor',
            #    output='screen',
            #    respawn=use_respawn,
            #    respawn_delay=2.0,
            #    parameters=[configured_params],
            #    arguments=['--ros-args', '--log-level', log_level],
            #    remappings=remappings,
            #),
            #Node(
            #    package='opennav_docking',
            #   executable='opennav_docking',
            #    name='docking_server',
            #    output='screen',
            #    respawn=use_respawn,
            #    respawn_delay=2.0,
            #    parameters=[configured_params],
            #    arguments=['--ros-args', '--log-level', log_level],
            #    remappings=remappings,
            #),
            #Node(
            #    package='opennav_following',
            #    executable='opennav_following',
            #    name='following_server',
            #    output='screen',
            #    respawn=use_respawn,
            #    respawn_delay=2.0,
            #    parameters=[configured_params],
            #    arguments=['--ros-args', '--log-level', log_level],
            #    remappings=remappings,
            #),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    configured_params,
                    {'autostart': autostart}, {'node_names': lifecycle_nodes}
                ],
            ),
        ],
    )

    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            PushRosNamespace(namespace=namespace),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[configured_params],
                        #remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                        remappings=remappings,
                        extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    ),
                    #ComposableNode(
                    #    package='nav2_smoother',
                    #    plugin='nav2_smoother::SmootherServer',
                    #    name='smoother_server',
                    #    parameters=[configured_params],
                    #    remappings=remappings,
                    #    extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    #),
                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=[configured_params],
                        remappings=remappings,
                        extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    ),
                    #ComposableNode(
                    #    package='nav2_route',
                    #    plugin='nav2_route::RouteServer',
                    #    name='route_server',
                    #    parameters=[configured_params, {'graph_filepath': graph_filepath}],
                    #    remappings=remappings,
                    #    extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    #),
                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='behavior_server::BehaviorServer',
                        name='behavior_server',
                        parameters=[configured_params],
                        #remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                        remappings=remappings,
                        extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    ),
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[configured_params],
                        remappings=remappings,
                        extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    ),
                    #ComposableNode(
                    #    package='nav2_waypoint_follower',
                    #    plugin='nav2_waypoint_follower::WaypointFollower',
                    #    name='waypoint_follower',
                    #    parameters=[configured_params],
                    #    remappings=remappings,
                    #    extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    #),
                    #ComposableNode(
                    #    package='nav2_velocity_smoother',
                    #    plugin='nav2_velocity_smoother::VelocitySmoother',
                    #    name='velocity_smoother',
                    #    parameters=[configured_params],
                    #    remappings=remappings
                    #    + [('cmd_vel', 'cmd_vel_nav')],
                    #    extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    #),
                    #ComposableNode(
                    #    package='nav2_collision_monitor',
                    #    plugin='nav2_collision_monitor::CollisionMonitor',
                    #    name='collision_monitor',
                    #    parameters=[configured_params],
                    #    remappings=remappings,
                    #    extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    #),
                    #ComposableNode(
                    #    package='opennav_docking',
                    #    plugin='opennav_docking::DockingServer',
                    #    name='docking_server',
                    #    parameters=[configured_params],
                    #    remappings=remappings,
                    #    extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    #),
                    #ComposableNode(
                    #    package='opennav_following',
                    #    plugin='opennav_following::FollowingServer',
                    #    name='following_server',
                    #    parameters=[configured_params],
                    #    remappings=remappings,
                    #    extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    #),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
                        parameters=[
                            configured_params,
                            {'autostart': autostart, 'node_names': lifecycle_nodes}
                        ],
                        extra_arguments=[{'use_intra_process_comms': use_intra_process_comms}],
                    ),
                ],
            ),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_graph_file_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_intra_process_comms_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_keepout_zones_cmd)
    ld.add_action(declare_use_speed_zones_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
