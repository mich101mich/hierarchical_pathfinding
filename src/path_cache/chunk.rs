use super::{path_segment::PathSegment, utils::*, NodeMap};
use crate::{neighbors::Neighborhood, NodeID, PathCacheConfig, Point};
use std::collections::{HashMap, HashSet};

#[derive(Clone, Debug)]
pub struct Chunk {
	pub pos: Point,
	pub size: Point,
	pub nodes: HashSet<NodeID>,
}

impl Chunk {
	pub fn new<N: Neighborhood>(
		pos: Point,
		size: (usize, usize),
		total_size: (usize, usize),
		get_cost: impl Fn(Point) -> isize,
		neighborhood: &N,
		all_nodes: &mut NodeMap,
		config: &PathCacheConfig,
	) -> Chunk {
		let mut chunk = Chunk {
			pos,
			size,
			nodes: HashSet::new(),
		};

		let mut candidates = HashSet::new();

		for dir in Dir::all() {
			if dir == UP && chunk.top() == 0
				|| dir == RIGHT && chunk.right() == total_size.0
				|| dir == DOWN && chunk.bottom() == total_size.1
				|| dir == LEFT && chunk.left() == 0
			{
				continue;
			}
			Chunk::calculate_side_nodes(dir, pos, size, total_size, &get_cost, &mut candidates);
		}

		let nodes: Vec<NodeID> = candidates
			.into_iter()
			.map(|p| all_nodes.add_node(p, get_cost(p)))
			.collect();

		chunk.add_nodes(nodes, &get_cost, neighborhood, all_nodes, config);

		chunk
	}

	pub fn calculate_side_nodes(
		dir: Dir,
		base_pos: Point,
		size: (usize, usize),
		total_size: (usize, usize),
		get_cost: impl Fn(Point) -> isize,
		candidates: &mut HashSet<Point>,
	) {
		let mut current = [
			(base_pos.0, base_pos.1),
			(base_pos.0 + size.0 - 1, base_pos.1),
			(base_pos.0, base_pos.1 + size.1 - 1),
			(base_pos.0, base_pos.1),
		][dir.num()];
		let (next_dir, length) = if dir.is_vertical() {
			(RIGHT, size.0)
		} else {
			(DOWN, size.1)
		};
		// 0 == up: start at top-left, go right
		// 1 == right: start at top-right, go down
		// 2 == down: start at bottom-left, go right
		// 3 == left: start at top-left, go down
		if get_in_dir(current, dir, (0, 0), total_size).is_none() {
			return;
		}

		let opposite = |p: Point| {
			get_in_dir(p, dir, (0, 0), total_size)
				.expect("Internal Error #1 in Chunk. Please report this")
		};
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
						p = get_in_dir(p, next_dir, (0, 0), total_size)
							.expect("Internal Error #2 in Chunk. Please report this");
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
				current = get_in_dir(current, next_dir, base_pos, size)
					.expect("Internal Error #3 in Chunk. Please report this");
			}
		}
	}

	pub fn add_nodes<N: Neighborhood>(
		&mut self,
		mut to_visit: Vec<NodeID>,
		get_cost: &Fn(Point) -> isize,
		neighborhood: &N,
		all_nodes: &mut NodeMap,
		config: &PathCacheConfig,
	) {
		let mut points = self
			.nodes
			.iter()
			.chain(to_visit.iter()) // results in to_visit points at the end => enables pop()
			.map(|id| all_nodes[id].pos)
			.to_vec();

		for &id in to_visit.iter() {
			self.nodes.insert(id);
		}

		// connect every Node to every other Node
		while let Some(id) = to_visit.pop() {
			if to_visit.is_empty() {
				break;
			}
			let point = points
				.pop()
				.expect("Internal Error #4 in Chunk. Please report this");

			let paths = self.find_paths(point, &points, &get_cost, neighborhood);

			for (other_pos, path) in paths {
				let other_id = *to_visit
					.iter()
					.find(|id| all_nodes[id].pos == other_pos)
					.expect("Internal Error #5 in Chunk. Please report this");

				all_nodes.add_edge(id, other_id, PathSegment::new(path, config.cache_paths));
			}
		}
	}

	#[allow(dead_code)]
	pub fn find_paths<N: Neighborhood>(
		&self,
		start: Point,
		goals: &[Point],
		get_cost: &Fn(Point) -> isize,
		neighborhood: &N,
	) -> HashMap<Point, crate::generics::Path<Point>> {
		crate::generics::dijkstra_search(
			|p| {
				let cost = get_cost(p) as usize;
				neighborhood
					.get_all_neighbors(p)
					.filter(|n| self.in_chunk(*n))
					.map(move |n| (n, cost))
			},
			|p| get_cost(p) >= 0,
			start,
			goals,
		)
	}

	#[allow(dead_code)]
	pub fn find_path<N: Neighborhood>(
		&self,
		start: Point,
		goal: Point,
		get_cost: &Fn(Point) -> isize,
		neighborhood: &N,
	) -> Option<crate::generics::Path<Point>> {
		crate::generics::a_star_search(
			|p| {
				let cost = get_cost(p) as usize;
				neighborhood
					.get_all_neighbors(p)
					.filter(|n| self.in_chunk(*n))
					.map(move |n| (n, cost))
			},
			|p| get_cost(p) >= 0,
			start,
			goal,
			|p| neighborhood.heuristic(p, goal),
		)
	}

	pub fn in_chunk(&self, point: Point) -> bool {
		point.0 >= self.left()
			&& point.0 < self.right()
			&& point.1 >= self.top()
			&& point.1 < self.bottom()
	}

	pub fn at_side(&self, point: Point, side: Dir) -> bool {
		match side {
			UP => point.1 == self.top(),
			RIGHT => point.0 == self.right(),
			DOWN => point.1 == self.bottom(),
			LEFT => point.0 == self.left(),
		}
	}

	pub fn at_any_side(&self, point: Point) -> bool {
		Dir::all().any(|dir| self.at_side(point, dir))
	}

	pub fn top(&self) -> usize {
		self.pos.1
	}
	pub fn right(&self) -> usize {
		self.pos.0 + self.size.0
	}
	pub fn bottom(&self) -> usize {
		self.pos.1 + self.size.1
	}
	pub fn left(&self) -> usize {
		self.pos.0
	}
}
