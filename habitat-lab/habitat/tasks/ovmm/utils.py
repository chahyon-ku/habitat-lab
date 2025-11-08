import habitat_sim
import traceback

from matplotlib import pyplot as plt


def find_closest_goal_index_within_distance(
    sim, goals, episode_id, use_all_viewpoints=False, max_dist=-1
):
    """Returns the index of the goal that has the closest viewpoint"""
    if use_all_viewpoints:
        goal_view_points = [
            v.agent_state.position for goal in goals for v in goal.view_points
        ]
        goal_indices = [
            i for i, goal in enumerate(goals) for v in goal.view_points
        ]
    else:
        goal_indices = list(range(len(goals)))
        goal_view_points = [
            g.view_points[0].agent_state.position for g in goals
        ]
    path = habitat_sim.MultiGoalShortestPath()
    path.requested_start = sim.articulated_agent.base_pos
    path.requested_ends = goal_view_points
    # input(path.requested_ends)
    # what we really need is to snap all goal_view_points
    # island = sim.pathfinder.get_island(path.requested_start)
    # path.requested_start = sim.pathfinder.snap_point(
    #     path.requested_start, island
    # )
    # path.requested_ends = [
    #     sim.pathfinder.snap_point(p, island)
    #     for p in path.requested_ends
    # ]
    # print(path.requested_start, island)
    # print(path.requested_ends, [
    #     sim.pathfinder.get_island(p) for p in path.requested_ends
    # ])
    # print(min_xyz)
    sim.pathfinder.find_path(path)
    if path.closest_end_point_index == -1:
        raise RuntimeError('habitat could not find path to goal')
        topdown_island_view = sim.pathfinder.get_topdown_island_view(
            0.01,
            0.1
        )
        min_xyz, max_xyz = sim.pathfinder.get_bounds()
        plt.imshow(topdown_island_view)
        plt.scatter(
            x=((path.requested_start[0] - min_xyz[0]) // 0.01,),
            y=((path.requested_start[2] - min_xyz[2]) // 0.01,),
            c='red',
        )
        plt.scatter(
            x=[(p[0] - min_xyz[0]) // 0.01 for p in path.requested_ends],
            y=[(p[2] - min_xyz[2]) // 0.01 for p in path.requested_ends],
            c='blue',
        )
        plt.show()
    assert (
        path.closest_end_point_index != -1
    ), f"None of the goals are reachable from current position for episode {episode_id}"
    if max_dist != -1 and path.geodesic_distance > max_dist:
        return -1
    # RotDist to closest goal
    return goal_indices[path.closest_end_point_index]
