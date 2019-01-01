use super::NodeMap;
use crate::{neighbors::Neighborhood, NodeID, Point};
use std::collections::{HashMap, HashSet};

#[derive(Clone, Debug)]
pub struct Chunk {
	pub pos: Point,
	pub size: Point,
	pub nodes: HashSet<NodeID>,
}

impl Chunk {
	pub fn new(
		pos: Point,
		size: (usize, usize),
		get_cost: impl Fn(Point) -> isize,
		neighborhood: &impl Neighborhood,
		all_nodes: &mut NodeMap,
	) -> Chunk {
		let mut nodes = HashSet::new();

		let mut candidates = HashSet::new();

		// always have corners if possible
		candidates.insert(pos);
		candidates.insert((pos.0, pos.1 + size.1 - 1));
		candidates.insert((pos.0 + size.0 - 1, pos.1));
		candidates.insert((pos.0 + size.0 - 1, pos.1 + size.1 - 1));

		for dir in 0..4 {
			Chunk::calculate_side_nodes(dir, pos, size, &get_cost, &mut candidates);
		}
		for p in candidates.into_iter().filter(|p| get_cost(*p) >= 0) {
			nodes.insert(all_nodes.add_node(p));
		}

		let chunk = Chunk { pos, size, nodes };

		let mut to_visit = chunk.nodes.iter().cloned().collect::<Vec<_>>();
		let mut points = to_visit
			.iter()
			.map(|id| all_nodes[id].pos)
			.collect::<Vec<_>>();

		// connect every Node to every other Node
		while let Some(id) = to_visit.pop() {
			if to_visit.is_empty() {
				break;
			}
			let point = points.pop().unwrap();
			let paths = chunk.find_paths(point, &points, &get_cost, neighborhood);
			for (other, path) in paths {
				let other_id = *to_visit
					.iter()
					.find(|id| all_nodes[id].pos == other)
					.unwrap();
				let other_path = path.reversed(
					get_cost(path[0]) as usize,
					get_cost(*path.last().unwrap()) as usize,
				);

				let node = all_nodes.get_mut(&id).unwrap();
				node.edges.insert(other_id, path);

				let node = all_nodes.get_mut(&other_id).unwrap();
				node.edges.insert(id, other_path);
			}
		}

		chunk
	}

	fn calculate_side_nodes(
		dir: usize,
		base_pos: Point,
		size: (usize, usize),
		get_cost: impl Fn(Point) -> isize,
		candidates: &mut HashSet<Point>,
	) {
		let mut current = [
			(base_pos.0, base_pos.1),
			(base_pos.0 + size.0 - 1, base_pos.1 + 1),
			(base_pos.0, base_pos.1 + size.1 - 1),
			(base_pos.0, base_pos.1 + 1),
		][dir];
		let (next_dir, length) = if dir % 2 == 0 {
			(1, size.0)
		} else {
			(2, size.1 - 2) // - 2 because otherwise corners would be considered twice
		};
		// 0 == up: start at top-left, go right
		// 1 == right: start at top-right, go down
		// 2 == down: start at bottom-left, go right
		// 3 == left: start at top-left, go down

		if get_in_dir(current, dir, base_pos, size).is_none() {
			return;
		}

		let opposite = |p: Point| get_in_dir(p, dir, base_pos, size).unwrap();
		let total_cost = |p: Point| get_cost(p) + get_cost(opposite(p));

		let mut has_gap = false;
		let mut gap_start = 0;
		let mut gap_start_pos = current;
		let mut previous = current;

		for i in 0..length {
			let is_last = i == length - 1;
			let solid = get_cost(current) < 0 || get_cost(opposite(current)) < 0;
			if !solid && !has_gap {
				has_gap = true;
				gap_start = i;
				gap_start_pos = current;
			}
			if (solid || is_last) && has_gap {
				has_gap = false;
				let (gap_end, gap_end_pos) = if solid {
					(i - 1, previous)
				} else {
					(i, current)
				};

				let gap_len = gap_end - gap_start + 1;

				candidates.insert(gap_start_pos);
				candidates.insert(gap_end_pos);

				if gap_len > 2 {
					let mut min = total_cost(gap_start_pos).min(total_cost(gap_end_pos));
					let mut p = gap_start_pos;
					for _ in (gap_start + 1)..gap_end {
						p = get_in_dir(p, next_dir, base_pos, size).unwrap();
						let cost = total_cost(p);
						if cost < min {
							candidates.insert(p);
							min = cost;
						}
					}
				}

				if gap_len > 6 {
					let mid = (
						(gap_start_pos.0 + gap_end_pos.0) / 2,
						(gap_start_pos.1 + gap_end_pos.1) / 2,
					);
					candidates.insert(mid);
				}
			}

			if !is_last {
				previous = current;
				current = get_in_dir(current, next_dir, base_pos, size).unwrap();
			}
		}
	}

	#[allow(dead_code)]
	pub fn find_paths(
		&self,
		start: Point,
		goals: &[Point],
		get_cost: impl Fn(Point) -> isize,
		neighborhood: &impl Neighborhood,
	) -> HashMap<Point, crate::generics::Path<Point>> {
		crate::generics::dijkstra_search(
			|p| {
				let mut n = neighborhood.get_all_neighbors(p);
				n.retain(|p| self.in_chunk(*p));
				n
			},
			|p, _| get_cost(p) as usize,
			|p| get_cost(p) >= 0,
			start,
			goals,
		)
	}

	#[allow(dead_code)]
	pub fn find_path(
		&self,
		start: Point,
		goal: Point,
		get_cost: impl Fn(Point) -> isize,
		neighborhood: &impl Neighborhood,
	) -> Option<crate::generics::Path<Point>> {
		crate::generics::a_star_search(
			|p| {
				let mut n = neighborhood.get_all_neighbors(p);
				n.retain(|p| self.in_chunk(*p));
				n
			},
			|p, _| get_cost(p) as usize,
			|p| get_cost(p) >= 0,
			start,
			goal,
			|p| neighborhood.heuristic(p, goal),
		)
	}

	pub fn in_chunk(&self, point: Point) -> bool {
		point.0 >= self.pos.0
			&& point.0 < self.pos.0 + self.size.0
			&& point.1 >= self.pos.1
			&& point.1 < self.pos.1 + self.size.1
	}
}

fn get_in_dir(pos: Point, dir: usize, base: Point, (w, h): (usize, usize)) -> Option<Point> {
	const UNIT_CIRCLE: [(isize, isize); 4] = [(0, -1), (1, 0), (0, 1), (-1, 0)];

	let diff = UNIT_CIRCLE[dir];
	if (pos.0 == base.0 && diff.0 < 0)
		|| (pos.1 == base.1 && diff.1 < 0)
		|| (pos.0 == base.0 + w - 1 && diff.0 > 0)
		|| (pos.1 == base.1 + h - 1 && diff.1 > 0)
	{
		None
	} else {
		Some((
			(pos.0 as isize + diff.0) as usize,
			(pos.1 as isize + diff.1) as usize,
		))
	}
}
