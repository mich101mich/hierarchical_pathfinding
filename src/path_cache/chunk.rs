use crate::{
    graph::*,
    neighbors::Neighborhood,
    path::{Path, PathSegment},
    *,
};

#[derive(Clone, Debug)]
pub struct Chunk {
    pub pos: Point,
    pub size: Point,
    pub nodes: NodeIDSet,
    pub sides: [bool; 4],
}

impl Chunk {
    pub fn new<N: Neighborhood>(
        pos: Point,
        size: (usize, usize),
        total_size: (usize, usize),
        mut get_cost: impl FnMut(Point) -> isize,
        neighborhood: &N,
        all_nodes: &mut NodeList,
        config: PathCacheConfig,
    ) -> Chunk {
        let mut chunk = Chunk {
            pos,
            size,
            nodes: NodeIDSet::default(),
            sides: [false; 4],
        };

        let mut candidates = PointSet::default();

        for dir in Dir::all() {
            if dir == UP && chunk.top() == 0
                || dir == RIGHT && chunk.right() == total_size.0
                || dir == DOWN && chunk.bottom() == total_size.1
                || dir == LEFT && chunk.left() == 0
            {
                continue;
            }
            chunk.sides[dir.num()] = true;

            chunk.calculate_side_nodes(dir, total_size, &mut get_cost, config, &mut candidates);
        }

        let nodes: Vec<NodeID> = candidates
            .into_iter()
            .map(|p| all_nodes.add_node(p, get_cost(p) as usize))
            .to_vec();

        chunk.add_nodes(&nodes, &mut get_cost, neighborhood, all_nodes, &config);

        chunk
    }

    pub fn calculate_side_nodes(
        &self,
        dir: Dir,
        total_size: (usize, usize),
        mut get_cost: impl FnMut(Point) -> isize,
        config: PathCacheConfig,
        candidates: &mut PointSet,
    ) {
        let mut current = [
            (self.pos.0, self.pos.1),
            (self.pos.0 + self.size.0 - 1, self.pos.1),
            (self.pos.0, self.pos.1 + self.size.1 - 1),
            (self.pos.0, self.pos.1),
        ][dir.num()];
        let (next_dir, length) = if dir.is_vertical() {
            (RIGHT, self.size.0)
        } else {
            (DOWN, self.size.1)
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

        let costs = (0..length)
            .map(|i| {
                jump_in_dir(current, next_dir, i, self.pos, self.size)
                    .expect("Internal Error #3 in Chunk. Please report this")
            })
            .map(|p| (get_cost(p), get_cost(opposite(p))))
            .to_vec();

        let solid = |i: usize| {
            let (c1, c2) = &costs[i];
            *c1 < 0 || *c2 < 0
        };
        let total_cost = |i: usize| {
            let (c1, c2) = &costs[i];
            *c1 + *c2
        };

        let mut has_gap = false;
        let mut gap_start = 0;
        let mut gap_start_pos = current;
        let mut previous = current;

        for i in 0..length {
            let is_last = i == length - 1;
            let solid = solid(i);

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

                if config.perfect_paths {
                    let mut p = gap_start_pos;
                    for _ in (gap_start + 1)..gap_end {
                        p = get_in_dir(p, next_dir, (0, 0), total_size)
                            .expect("Internal Error #6 in Chunk. Please report this");
                        candidates.insert(p);
                    }
                } else {
                    if gap_len > 2 {
                        let mut min = total_cost(gap_start).min(total_cost(gap_end));
                        let mut p = gap_start_pos;
                        for gi in (gap_start + 1)..gap_end {
                            p = get_in_dir(p, next_dir, (0, 0), total_size)
                                .expect("Internal Error #2 in Chunk. Please report this");
                            let cost = total_cost(gi);
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
            }

            if !is_last {
                previous = current;
                current = get_in_dir(current, next_dir, self.pos, self.size)
                    .expect("Internal Error #3 in Chunk. Please report this");
            }
        }
    }

    pub fn add_nodes<N: Neighborhood>(
        &mut self,
        to_visit: &[NodeID],
        mut get_cost: impl FnMut(Point) -> isize,
        neighborhood: &N,
        all_nodes: &mut NodeList,
        config: &PathCacheConfig,
    ) {
        // first to_visit, then the rest => slicing works the same on both lists
        let points = to_visit
            .iter()
            .chain(self.nodes.iter())
            .map(|id| all_nodes[*id].pos)
            .to_vec();

        for &id in to_visit.iter() {
            self.nodes.insert(id);
        }

        for (i, &id) in to_visit.iter().enumerate() {
            let point = points[i];
            let remaining = &points[(i + 1)..];
            let paths = self.find_paths(point, remaining, &mut get_cost, neighborhood);
            for (other_pos, path) in paths {
                let other_id = all_nodes
                    .id_at(other_pos)
                    .expect("Internal Error #5 in Chunk. Please report this");

                all_nodes.add_edge(id, other_id, PathSegment::new(path, config.cache_paths));
            }
        }
    }

    #[cfg(feature = "parallel")]
    pub fn connect_nodes_parallel<N: Neighborhood + Sync, F1: Fn(Point) -> isize + Sync>(
        &self,
        get_cost: F1,
        neighborhood: &N,
        all_nodes: &NodeList,
        cache_paths: bool,
    ) -> Vec<(NodeID, NodeID, PathSegment)> {
        use rayon::prelude::*;

        let mut ids = Vec::with_capacity(self.nodes.len());
        let mut points = Vec::with_capacity(self.nodes.len());
        for (i, &id) in self.nodes.iter().enumerate() {
            ids.push((i, id));
            points.push(all_nodes[id].pos);
        }

        // connect every Node to every other Node
        ids.par_iter()
            .flat_map(|&(i, id)| {
                let point = points[i];
                let remaining = &points[(i + 1)..];
                self.find_paths(point, remaining, &get_cost, neighborhood)
                    .into_par_iter()
                    .map(move |(other_pos, path)| {
                        let other_id = all_nodes
                            .id_at(other_pos)
                            .expect("Internal Error #5 in Chunk. Please report this");

                        (id, other_id, PathSegment::new(path, cache_paths))
                    })
            })
            .collect()
    }

    pub fn find_paths<N: Neighborhood>(
        &self,
        start: Point,
        goals: &[Point],
        get_cost: impl FnMut(Point) -> isize,
        neighborhood: &N,
    ) -> PointMap<Path<Point>> {
        if !self.in_chunk(start) {
            return PointMap::default();
        }
        let heuristic = goals
            .iter()
            .map(|goal| neighborhood.heuristic(start, *goal))
            .max()
            .unwrap_or(0);
        let max_heuristic = neighborhood.heuristic((0, 0), (self.size.0 - 1, self.size.1 - 1));
        let max_size = self.size.0 * self.size.1;
        let size_hint = heuristic as f32 / max_heuristic as f32 * max_size as f32;
        grid::dijkstra_search(
            neighborhood,
            |p| self.in_chunk(p),
            get_cost,
            start,
            goals,
            false,
            size_hint as usize,
        )
    }

    pub fn nearest_node<N: Neighborhood>(
        &self,
        all_nodes: &NodeList,
        start: Point,
        mut get_cost: impl FnMut(Point) -> isize,
        neighborhood: &N,
        reverse: bool,
    ) -> Option<(NodeID, Path<Point>)> {
        let start_cost = get_cost(start);
        if start_cost < 0 {
            if !reverse {
                return None;
            }
            self.nodes.iter().copied().find_map(|id| {
                self.find_path(all_nodes[id].pos, start, &mut get_cost, neighborhood)
                    .map(|path| (id, path))
            })
        } else {
            let mut points = Vec::with_capacity(self.nodes.len());
            let mut map = PointMap::default();
            let max_heuristic = neighborhood.heuristic((0, 0), (self.size.0 - 1, self.size.1 - 1));
            let mut min_heuristic = max_heuristic;
            for id in self.nodes.iter() {
                let node = &all_nodes[*id];
                let point = node.pos;
                points.push(point);
                map.insert(point, (*id, node.walk_cost));
                min_heuristic = min_heuristic.min(neighborhood.heuristic(start, point));
            }

            let max_size = self.size.0 * self.size.1;
            let size_hint = min_heuristic as f32 / max_heuristic as f32 * max_size as f32;
            grid::dijkstra_search(
                neighborhood,
                |p| self.in_chunk(p),
                get_cost,
                start,
                &points,
                true,
                size_hint as usize,
            )
            .into_iter()
            .next()
            .map(|(point, path)| {
                let (id, node_cost) = map[&point];
                (
                    id,
                    if reverse {
                        path.reversed(start_cost as usize, node_cost)
                    } else {
                        path
                    },
                )
            })
        }
    }
    pub fn find_path<N: Neighborhood>(
        &self,
        start: Point,
        goal: Point,
        get_cost: impl FnMut(Point) -> isize,
        neighborhood: &N,
    ) -> Option<Path<Point>> {
        if !self.in_chunk(start) || !self.in_chunk(goal) {
            return None;
        }
        let heuristic = neighborhood.heuristic(start, goal);
        let max_heuristic = neighborhood.heuristic((0, 0), (self.size.0 - 1, self.size.1 - 1));
        let max_size = self.size.0 * self.size.1;
        let size_hint = heuristic as f32 / max_heuristic as f32 * max_size as f32;

        grid::a_star_search(
            neighborhood,
            |p| self.in_chunk(p),
            get_cost,
            start,
            goal,
            size_hint as usize,
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
            RIGHT => point.0 == self.right() - 1,
            DOWN => point.1 == self.bottom() - 1,
            LEFT => point.0 == self.left(),
        }
    }

    pub fn is_corner(&self, point: Point) -> bool {
        Dir::all()
            .filter(|&dir| self.sides[dir.num()] && self.at_side(point, dir))
            .count()
            == 2
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
