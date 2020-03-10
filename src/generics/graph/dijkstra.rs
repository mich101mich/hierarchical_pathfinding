use super::super::{ordered_insert, Cost, Path};
use crate::{node_id::*, NodeID};

/// Searches a Graph using [Dijkstra's Algorithm](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm) in a Node Graph with [`NodeID`]s.
///
/// This function can be used to search for several Goals and will try to calculate a Path for every provided Goal.
/// It stops as soon as it has the shortest Path to every Goal, or when all reachable Nodes have been expanded.
///
/// ## Arguments
/// - `get_all_neighbors` - a Function that takes a Node and returns all other Nodes reachable from that Node.
///     The returned value is a Tuple of the `NodeID` of the neighbor and the Cost to get there.
/// - `start` - the starting Node
/// - `goals` - the Goals that this function is supposed to search for
///
/// ## Returns
/// a HashMap with all reachable Goal's NodeIDs as the Key and the shortest Path to reach that Goal as Value.
/// The first Node in the Path is always the `start` and the last is the corresponding Goal
pub fn dijkstra_search<NeighborIter: Iterator<Item = (NodeID, Cost)>>(
	mut get_all_neighbors: impl FnMut(NodeID) -> NeighborIter,
	mut is_walkable: impl FnMut(NodeID) -> bool,
	start: NodeID,
	goals: &[NodeID],
) -> NodeIDMap<Path<NodeID>> {
	let mut visited = node_id_map();
	let mut next = vec![(start, 0)];
	visited.insert(start, (0, start));

	let mut remaining_goals = goals.to_vec();

	let mut goal_costs = node_id_map_with_cap(goals.len());

	while let Some((current_id, _)) = next.pop() {
		let cost = visited[&current_id].0;

		let mut found_one = false;
		for &goal_id in remaining_goals.iter() {
			if current_id == goal_id {
				goal_costs.insert(goal_id, cost);
				found_one = true;
			}
		}
		if found_one {
			remaining_goals.retain(|&id| id != current_id);
			if remaining_goals.is_empty() {
				break;
			}
		}

		for (other_id, delta_cost) in get_all_neighbors(current_id) {
			let other_cost = cost + delta_cost;

			if !is_walkable(other_id) {
				let mut is_goal = false;
				for &goal_id in remaining_goals.iter() {
					if other_id == goal_id {
						is_goal = true;
					}
				}
				if !is_goal {
					continue;
				}
			}

			if let Some(&(prev_cost, _)) = visited.get(&other_id) {
				if prev_cost > other_cost {
					next.retain(|&(id, _)| id != other_id);
				}
			}

			if !visited.contains_key(&other_id) || visited[&other_id].0 > other_cost {
				ordered_insert(&mut next, (other_id, other_cost), |&(_, cost)| cost);
				visited.insert(other_id, (other_cost, current_id));
			}
		}
	}

	let mut goal_data = node_id_map_with_cap(goal_costs.len());

	for (&goal, &cost) in goal_costs.iter() {
		let steps = {
			let mut steps = vec![];
			let mut current = goal;

			while current != start {
				steps.push(current);
				let (_, prev) = visited[&current];
				current = prev;
			}
			steps.push(start);
			steps.reverse();
			steps
		};
		goal_data.insert(goal, Path::new(steps, cost));
	}

	goal_data
}
