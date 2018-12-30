use super::NodeMap;
use crate::{NodeID, Point};
use std::collections::HashSet;

#[derive(Clone, Debug)]
pub struct Chunk {
	pub pos: Point,
	pub nodes: HashSet<NodeID>,
}

impl Chunk {
	pub fn new(
		pos: Point,
		size: (usize, usize),
		get_cost: impl Fn(Point) -> isize,
		all_nodes: &mut NodeMap,
	) -> Chunk {
		let mut nodes = HashSet::new();

		for dir in 0..4 {
			let side = Chunk::calculate_side_nodes(dir, pos, size, &get_cost, all_nodes);
			for id in side {
				nodes.insert(id);
			}
		}

		// TODO: connect nodes

		Chunk { pos, nodes }
	}

	fn calculate_side_nodes(
		dir: usize,
		pos: Point,
		size: (usize, usize),
		get_cost: impl Fn(Point) -> isize,
		all_nodes: &mut NodeMap,
	) -> Vec<NodeID> {
		let mut current = [
			(pos.0, pos.1),
			(pos.0 + size.0 - 1, pos.1 + 1),
			(pos.0, pos.1 + size.1 - 1),
			(pos.0, pos.1 + 1),
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

		if get_in_dir(current, dir, size).is_none() {
			return vec![];
		}

		let opposite = |pos: Point| get_in_dir(pos, dir, size).unwrap();
		let total_cost = |pos: Point| get_cost(pos) + get_cost(opposite(pos));

		let mut has_gap = false;
		let mut gap_start = 0;
		let mut gap_start_pos = current;

		let mut my_nodes = vec![];

		for i in 0..length {
			if get_cost(current) >= 0 && get_cost(opposite(current)) >= 0 {
				if !has_gap {
					has_gap = true;
					gap_start = i;
					gap_start_pos = current;
				}
			} else if has_gap {
				has_gap = false;
				let gap_end = i;
				let gap_end_pos = current;

				let gap_len = gap_end - gap_start;
				let mut nodes = vec![];

				match gap_len {
					0 => panic!("how??"),
					1 => nodes.push(gap_start_pos),
					2 => {
						if total_cost(gap_start_pos) < total_cost(gap_end_pos) {
							nodes.push(gap_start_pos);
						} else {
							nodes.push(gap_end_pos);
						}
					}
					_ => {
						nodes.push(gap_start_pos);
						nodes.push(gap_end_pos);
						let mut min = total_cost(gap_start_pos).min(total_cost(gap_end_pos));
						let mut pos = gap_start_pos;
						for _ in (gap_start + 1)..gap_end {
							pos = get_in_dir(pos, next_dir, size).unwrap();
							let cost = total_cost(pos);
							if cost < min {
								nodes.push(pos);
								min = cost;
							}
						}
					}
				}

				if gap_len > 6 {
					let mid = (
						(gap_start_pos.0 + gap_end_pos.0) / 2,
						(gap_start_pos.1 + gap_end_pos.1) / 2,
					);
					if !nodes.contains(&mid) {
						nodes.push(mid);
					}
				}

				for p in nodes {
					my_nodes.push(all_nodes.add_node(p));
				}
			}

			current = get_in_dir(current, next_dir, size).unwrap();
		}
		my_nodes
	}
}

fn get_in_dir(pos: Point, dir: usize, (w, h): (usize, usize)) -> Option<Point> {
	const UNIT_CIRCLE: [(isize, isize); 4] = [(0, -1), (1, 0), (0, 1), (-1, 0)];

	let diff = UNIT_CIRCLE[dir];
	if pos.0 == 0 && diff.0 < 0
		|| pos.1 == 0 && diff.1 < 0
		|| pos.0 == w - 1 && diff.0 > 0
		|| pos.1 == h - 1 && diff.1 > 0
	{
		None
	} else {
		Some((
			(pos.0 as isize + diff.0) as usize,
			(pos.1 as isize + diff.1) as usize,
		))
	}
}
