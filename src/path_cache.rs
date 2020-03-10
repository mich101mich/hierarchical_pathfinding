use crate::{
	generics::{self, graph, grid},
	neighbors::Neighborhood,
	NodeID, Point, PointMap, PointSet,
};

mod chunk;
use self::chunk::Chunk;

mod node;
use self::node::Node;

mod cache_config;
pub use self::cache_config::PathCacheConfig;

mod abstract_path;
pub use self::abstract_path::AbstractPath;

#[macro_use]
mod node_map;
use self::node_map::NodeMap;

mod path_segment;
use self::path_segment::PathSegment;

mod utils;
use self::utils::*;

/// A struct to store the Hierarchical Pathfinding information.
#[derive(Clone, Debug)]
pub struct PathCache<N: Neighborhood> {
	width: usize,
	height: usize,
	chunks: Vec<Vec<Chunk>>,
	nodes: NodeMap,
	neighborhood: N,
	config: PathCacheConfig,
}

// this is a macro so that it only borrows self.chunks instead of self
macro_rules! get_chunk {
	($obj: ident, $point: ident) => {
		&$obj.chunks[$point.0 / $obj.config.chunk_size][$point.1 / $obj.config.chunk_size]
	};
}
macro_rules! get_chunk_mut {
	($obj: ident, $point: ident) => {
		&mut $obj.chunks[$point.0 / $obj.config.chunk_size][$point.1 / $obj.config.chunk_size]
	};
}

impl<N: Neighborhood> PathCache<N> {
	/// Creates a new PathCache
	///
	/// ## Arguments
	/// - `(width, height)` - the size of the Grid
	/// - `get_cost` - get the cost for walking over a Tile. (Cost < 0 => solid Tile)
	/// - `neighborhood` - the Neighborhood to use. (See [`Neighborhood`])
	/// - `config` - optional config for creating the cache. (See [`PathCacheConfig`])
	///
	/// ## Examples
	/// Basic usage:
	/// ```
	/// use hierarchical_pathfinding::{prelude::*, Point};
	///
	/// // create and initialize Grid
	/// // 0 = empty, 1 = swamp, 2 = wall
	/// let mut grid = [
	///     [0, 2, 0, 0, 0],
	///     [0, 2, 2, 2, 0],
	///     [0, 1, 0, 0, 0],
	///     [0, 1, 0, 2, 0],
	///     [0, 0, 0, 2, 0],
	/// ];
	/// let (width, height) = (grid.len(), grid[0].len());
	///
	/// const COST_MAP: [isize; 3] = [1, 10, -1];
	///
	/// fn cost_fn<'a>(grid: &'a [[usize; 5]; 5]) -> impl 'a + FnMut(Point) -> isize {
	///     move |(x, y)| COST_MAP[grid[y][x]]
	/// }
	///
	/// let mut pathfinding = PathCache::new(
	///     (width, height), // the size of the Grid
	///     cost_fn(&grid), // get the cost for walking over a Tile
	///     ManhattanNeighborhood::new(width, height), // the Neighborhood
	///     PathCacheConfig { chunk_size: 3, ..Default::default() }, // config
	/// );
	/// ```
	pub fn new(
		(width, height): (usize, usize),
		mut get_cost: impl FnMut(Point) -> isize,
		neighborhood: N,
		config: PathCacheConfig,
	) -> PathCache<N> {
		// calculate chunk size
		let chunk_hor = {
			let mut w = width / config.chunk_size;
			if w * config.chunk_size < width {
				w += 1;
			}
			w
		};
		let chunk_vert = {
			let mut h = height / config.chunk_size;
			if h * config.chunk_size < height {
				h += 1;
			}
			h
		};

		let mut nodes = NodeMap::new();

		// create chunks
		let mut chunks = Vec::with_capacity(chunk_hor);
		for x in 0..chunk_hor {
			let mut row = Vec::with_capacity(chunk_vert);
			let w = if x == chunk_hor - 1 {
				width - (chunk_hor - 1) * config.chunk_size
			} else {
				config.chunk_size
			};

			for y in 0..chunk_vert {
				let h = if y == chunk_vert - 1 {
					height - (chunk_vert - 1) * config.chunk_size
				} else {
					config.chunk_size
				};
				row.push(Chunk::new(
					(x * config.chunk_size, y * config.chunk_size),
					(w, h),
					(width, height),
					&mut get_cost,
					&neighborhood,
					&mut nodes,
					config,
				))
			}

			chunks.push(row);
		}

		let mut cache = PathCache {
			width,
			height,
			chunks,
			nodes,
			config,
			neighborhood,
		};

		// connect neighboring Nodes across Chunk borders
		cache.connect_nodes(&mut get_cost);

		cache
	}

	/// Calculates the Path from `start` to `goal` on the Grid.
	///
	/// If no Path could be found, `None` is returned.
	///
	/// This function takes a mutable reference of self, because `start` and `goal` need to be
	/// inserted into the Abstract Graph in order for the algorithm to work. They are removed at
	/// the end unless [`config.keep_insertions`](PathCacheConfig::keep_insertions) was set to
	/// `true` (default) when creating the PathCache.
	///
	/// ## Arguments
	/// - `start`  the Point where the search starts
	/// - `goal`  the Point to search for. This may be a solid Tile.
	/// - `get_cost`  get the cost for walking over a Tile. (Cost < 0 => solid Tile)
	///
	/// ## Examples
	/// Basic usage:
	/// ```
	/// # use hierarchical_pathfinding::{prelude::*, Point};
	/// #
	/// # // create and initialize Grid
	/// # // 0 = empty, 1 = swamp, 2 = wall
	/// # let mut grid = [
	/// #     [0, 2, 0, 0, 0],
	/// #     [0, 2, 2, 2, 2],
	/// #     [0, 1, 0, 0, 0],
	/// #     [0, 1, 0, 2, 0],
	/// #     [0, 0, 0, 2, 0],
	/// # ];
	/// # let (width, height) = (grid.len(), grid[0].len());
	/// #
	/// # const COST_MAP: [isize; 3] = [1, 10, -1];
	/// #
	/// # fn cost_fn<'a>(grid: &'a [[usize; 5]; 5]) -> impl 'a + FnMut(Point) -> isize {
	/// #     move |(x, y)| COST_MAP[grid[y][x]]
	/// # }
	/// #
	/// # let mut pathfinding = PathCache::new(
	/// #     (width, height),
	/// #     cost_fn(&grid),
	/// #     ManhattanNeighborhood::new(width, height),
	/// #     PathCacheConfig { chunk_size: 3, ..Default::default() },
	/// # );
	/// #
	/// let start = (0, 0);
	/// let goal = (4, 4);
	///
	/// // find_path returns Some(Path) on success
	/// let path = pathfinding.find_path(
	///     start,
	///     goal,
	///     cost_fn(&grid),
	/// );
	///
	/// assert!(path.is_some());
	/// let path = path.unwrap();
	///
	/// assert_eq!(path.cost(), 12);
	/// ```
	///
	/// The return Value gives the total Cost of the Path using `cost()` and allows to iterate over
	/// the Points in the Path.
	///
	/// **Note**: Setting [`config.cache_paths`](PathCacheConfig::cache_paths) to `false` means
	/// that the Paths need to be recalculated as needed. This means that for any sections of the
	/// Path that are not present, [`safe_next`](AbstractPath::safe_next) needs to be called to supply the Cost function.
	/// Calling `next` in that scenario would lead to a Panic.
	///
	/// Using the Path:
	/// ```
	/// # use hierarchical_pathfinding::{prelude::*, Point};
	/// #
	/// # // create and initialize Grid
	/// # // 0 = empty, 1 = swamp, 2 = wall
	/// # let mut grid = [
	/// #     [0, 2, 0, 0, 0],
	/// #     [0, 2, 2, 2, 2],
	/// #     [0, 1, 0, 0, 0],
	/// #     [0, 1, 0, 2, 0],
	/// #     [0, 0, 0, 2, 0],
	/// # ];
	/// # let (width, height) = (grid.len(), grid[0].len());
	/// #
	/// # const COST_MAP: [isize; 3] = [1, 10, -1];
	/// #
	/// # fn cost_fn<'a>(grid: &'a [[usize; 5]; 5]) -> impl 'a + FnMut(Point) -> isize {
	/// #     move |(x, y)| COST_MAP[grid[y][x]]
	/// # }
	/// #
	/// # let mut pathfinding = PathCache::new(
	/// #     (width, height),
	/// #     cost_fn(&grid),
	/// #     ManhattanNeighborhood::new(width, height),
	/// #     PathCacheConfig { chunk_size: 3, ..Default::default() },
	/// # );
	/// # struct Player{ pos: (usize, usize) }
	/// # impl Player {
	/// #     pub fn move_to(&mut self, pos: (usize, usize)) {
	/// #         self.pos = pos;
	/// #     }
	/// # }
	/// #
	/// let mut player = Player {
	///     pos: (0, 0),
	///     //...
	/// };
	/// let goal = (4, 4);
	///
	/// let mut path = pathfinding.find_path(
	///     player.pos,
	///     goal,
	///     cost_fn(&grid),
	/// ).unwrap();
	///
	/// player.move_to(path.next().unwrap());
	/// assert_eq!(player.pos, (0, 1));
	///
	/// // wait for next turn or whatever
	///
	/// player.move_to(path.next().unwrap());
	/// assert_eq!(player.pos, (0, 2));
	/// ```
	/// If the Grid changes, any Path objects still in use may become invalid. You can still
	/// use them if you are certain that nothing in relation to that Path changed, but it is
	/// discouraged and can lead to undefined behavior or panics.
	///
	/// Obtaining the entire Path:
	/// ```
	/// # use hierarchical_pathfinding::{prelude::*, Point};
	/// #
	/// # // create and initialize Grid
	/// # // 0 = empty, 1 = swamp, 2 = wall
	/// # let mut grid = [
	/// #     [0, 2, 0, 0, 0],
	/// #     [0, 2, 2, 2, 2],
	/// #     [0, 1, 0, 0, 0],
	/// #     [0, 1, 0, 2, 0],
	/// #     [0, 0, 0, 2, 0],
	/// # ];
	/// # let (width, height) = (grid.len(), grid[0].len());
	/// #
	/// # const COST_MAP: [isize; 3] = [1, 10, -1];
	/// #
	/// # fn cost_fn<'a>(grid: &'a [[usize; 5]; 5]) -> impl 'a + FnMut(Point) -> isize {
	/// #     move |(x, y)| COST_MAP[grid[y][x]]
	/// # }
	/// #
	/// # let mut pathfinding = PathCache::new(
	/// #     (width, height),
	/// #     cost_fn(&grid),
	/// #     ManhattanNeighborhood::new(width, height),
	/// #     PathCacheConfig { chunk_size: 3, ..Default::default() },
	/// # );
	/// # struct Player{ pos: (usize, usize) }
	/// # impl Player {
	/// #     pub fn move_to(&mut self, pos: (usize, usize)) {
	/// #         self.pos = pos;
	/// #     }
	/// # }
	/// #
	/// # let mut player = Player {
	/// #     pos: (0, 0),
	/// #     //...
	/// # };
	/// # let goal = (4, 4);
	/// #
	/// # let path = pathfinding.find_path(
	/// #     player.pos,
	/// #     goal,
	/// #     cost_fn(&grid),
	/// # );
	/// // ...
	/// let path = path.unwrap();
	///
	/// let points: Vec<(usize, usize)> = path.collect();
	/// assert_eq!(
	///     points,
	///     vec![(0, 1),  (0, 2),  (0, 3),  (0, 4),  (1, 4),  (2, 4),
	///          (2, 3),  (2, 2),  (3, 2),  (4, 2),  (4, 3),  (4, 4)],
	/// );
	/// ```
	pub fn find_path(
		&mut self,
		start: Point,
		goal: Point,
		mut get_cost: impl FnMut(Point) -> isize,
	) -> Option<AbstractPath<N>> {
		let mut inserted_start = false;
		let mut inserted_goal = false;

		let start_id = self.get_node_id(start).unwrap_or_else(|| {
			inserted_start = true;
			self.add_node(start, &mut get_cost)
		});

		let goal_id = self.get_node_id(goal).unwrap_or_else(|| {
			inserted_goal = true;
			self.add_node(goal, &mut get_cost)
		});

		let path = graph::a_star_search(
			|id| {
				self.nodes[id]
					.edges
					.iter()
					.map(|(id, path)| (*id, path.cost()))
			},
			|id| self.nodes[id].walk_cost >= 0,
			start_id,
			goal_id,
			|id| self.neighborhood.heuristic(self.nodes[id].pos, goal),
		);

		let final_path = if let Some(path) = path {
			let length: usize = path
				.iter()
				.zip(path.iter().skip(1))
				.map(|(a, b)| self.nodes[*a].edges[&b].len())
				.sum();

			if self.config.a_star_fallback && length < 2 * self.config.chunk_size {
				let path = grid::a_star_search(
					|p| self.neighborhood.get_all_neighbors(p),
					get_cost,
					start,
					goal,
					|p| self.neighborhood.heuristic(p, goal),
				)
				.expect("Internal Error #1 in PathCache. Please report this");

				Some(AbstractPath::<N>::from_known_path(
					self.neighborhood.clone(),
					path,
				))
			} else {
				let mut ret = AbstractPath::<N>::new(self.neighborhood.clone(), start);
				for (a, b) in path.iter().zip(path.iter().skip(1)) {
					let path = &self.nodes[*a].edges[&b];
					ret.add_path_segment(path.clone());
				}

				Some(ret)
			}
		} else {
			None
		};

		if !self.config.keep_insertions {
			if inserted_start {
				self.nodes.remove_node(start_id);
			}
			if inserted_goal {
				self.nodes.remove_node(goal_id);
			}
		}

		final_path
	}

	/// Calculates the Paths from one `start` to several `goals` on the Grid.
	///
	/// This is equivalent to [`find_path`](PathCache::find_path), except that it is optimized to handle multiple Goals
	/// at once. However, it is slower for very few goals, since it does not use a heuristic like
	/// [`find_path`](PathCache::find_path) does.
	///
	/// Instead of returning a single Option, it returns a Hashmap, where the position of the Goal
	/// is the key, and the Value is a Tuple of the Path and the Cost of that Path.
	///
	/// See [`find_path`](PathCache::find_path) for more details on how to use the returned Path.
	///
	/// ## Arguments
	/// - `start`  the Point where the search starts
	/// - `goals`  the Points to search for. They may be a solid Tiles.
	/// - `get_cost`  get the cost for walking over a Tile. (Cost < 0 => solid Tile)
	///
	/// ## Examples
	/// Basic usage:
	/// ```
	/// # use hierarchical_pathfinding::{prelude::*, Point};
	/// #
	/// # // create and initialize Grid
	/// # // 0 = empty, 1 = swamp, 2 = wall
	/// # let mut grid = [
	/// #     [0, 2, 0, 0, 0],
	/// #     [0, 2, 2, 2, 2],
	/// #     [0, 1, 0, 0, 0],
	/// #     [0, 1, 0, 2, 0],
	/// #     [0, 0, 0, 2, 0],
	/// # ];
	/// # let (width, height) = (grid.len(), grid[0].len());
	/// #
	/// # const COST_MAP: [isize; 3] = [1, 10, -1];
	/// #
	/// # fn cost_fn<'a>(grid: &'a [[usize; 5]; 5]) -> impl 'a + FnMut(Point) -> isize {
	/// #     move |(x, y)| COST_MAP[grid[y][x]]
	/// # }
	/// #
	/// # let mut pathfinding = PathCache::new(
	/// #     (width, height),
	/// #     cost_fn(&grid),
	/// #     ManhattanNeighborhood::new(width, height),
	/// #     PathCacheConfig { chunk_size: 3, ..Default::default() },
	/// # );
	/// #
	/// let start = (0, 0);
	/// let goals = [(4, 4), (2, 0)];
	///
	/// // find_paths returns a HashMap<goal, Path> for all successes
	/// let paths = pathfinding.find_paths(
	///     start,
	///     &goals,
	///     cost_fn(&grid),
	/// );
	///
	/// // (4, 4) is reachable
	/// assert!(paths.contains_key(&goals[0]));
	///
	/// // (2, 0) is not reachable
	/// assert!(!paths.contains_key(&goals[1]));
	/// ```
	///
	/// The returned Path is always equivalent to the one returned by [`find_path`](PathCache::find_path):
	/// ```
	/// # use hierarchical_pathfinding::{prelude::*, Point};
	/// #
	/// # // create and initialize Grid
	/// # // 0 = empty, 1 = swamp, 2 = wall
	/// # let mut grid = [
	/// #     [0, 2, 0, 0, 0],
	/// #     [0, 2, 2, 2, 2],
	/// #     [0, 1, 0, 0, 0],
	/// #     [0, 1, 0, 2, 0],
	/// #     [0, 0, 0, 2, 0],
	/// # ];
	/// # let (width, height) = (grid.len(), grid[0].len());
	/// #
	/// # const COST_MAP: [isize; 3] = [1, 10, -1];
	/// #
	/// # fn cost_fn<'a>(grid: &'a [[usize; 5]; 5]) -> impl 'a + FnMut(Point) -> isize {
	/// #     move |(x, y)| COST_MAP[grid[y][x]]
	/// # }
	/// #
	/// # let mut pathfinding = PathCache::new(
	/// #     (width, height),
	/// #     cost_fn(&grid),
	/// #     ManhattanNeighborhood::new(width, height),
	/// #     PathCacheConfig { chunk_size: 3, ..Default::default() },
	/// # );
	/// #
	/// let start = (0, 0);
	/// let goal = (4, 4);
	///
	/// let paths = pathfinding.find_paths(
	///     start,
	///     &[goal],
	///     cost_fn(&grid),
	/// );
	/// let dijkstra_path: Vec<Point> = paths[&goal].clone().collect();
	///
	/// let a_star_path: Vec<Point> = pathfinding.find_path(
	///     start,
	///     goal,
	///     cost_fn(&grid),
	/// ).unwrap().collect();
	///
	/// assert_eq!(dijkstra_path, a_star_path);
	/// ```
	pub fn find_paths(
		&mut self,
		start: Point,
		goals: &[Point],
		mut get_cost: impl FnMut(Point) -> isize,
	) -> PointMap<AbstractPath<N>> {
		let start_id = self
			.get_node_id(start)
			.unwrap_or_else(|| self.add_node(start, &mut get_cost));

		let goal_ids = goals
			.iter()
			.map(|&goal| {
				self.get_node_id(goal)
					.unwrap_or_else(|| self.add_node(goal, &mut get_cost))
			})
			.to_vec();

		let paths = graph::dijkstra_search(
			|id| {
				self.nodes[id]
					.edges
					.iter()
					.map(|(id, path)| (*id, path.cost()))
			},
			|id| self.nodes[id].walk_cost >= 0,
			start_id,
			&goal_ids,
		);

		let mut ret = PointMap::default();

		for (&goal, id) in goals.iter().zip(goal_ids) {
			if let Some(path) = paths.get(&id) {
				let mut ret_path = AbstractPath::<N>::new(self.neighborhood.clone(), start);
				for (a, b) in path.iter().zip(path.iter().skip(1)) {
					let path = &self.nodes[*a].edges[&b];
					ret_path.add_path_segment(path.clone());
				}
				ret.insert(goal, ret_path);
			}
		}

		ret
	}

	/// Notifies the PathCache that the Grid changed.
	///
	/// This Method updates any internal Paths that might have changed when the Grid changed. This
	/// is an expensive operation and should only be performed if the change affected the walking
	/// cost of a tile and the PathCache is needed again. If possible, try to bundle as many
	/// changes as possible into a single call to `tiles_changed` to avoid unnecessary
	/// recalculations.
	///
	/// Side note: if anybody has a way to improve this method, open a GitHub Issue / Pull Request.
	///
	/// ## Examples
	/// Basic usage:
	/// ```
	/// # use hierarchical_pathfinding::{prelude::*, Point};
	/// #
	/// # // create and initialize Grid
	/// # // 0 = empty, 1 = swamp, 2 = wall
	/// # let mut grid = [
	/// #     [0, 2, 0, 0, 0],
	/// #     [0, 2, 2, 2, 2],
	/// #     [0, 1, 0, 0, 0],
	/// #     [0, 1, 0, 2, 0],
	/// #     [0, 0, 0, 2, 0],
	/// # ];
	/// # let (width, height) = (grid.len(), grid[0].len());
	/// #
	/// # const COST_MAP: [isize; 3] = [1, 10, -1];
	/// #
	/// # fn cost_fn<'a>(grid: &'a [[usize; 5]; 5]) -> impl 'a + FnMut(Point) -> isize {
	/// #     move |(x, y)| COST_MAP[grid[y][x]]
	/// # }
	/// #
	/// # let mut pathfinding = PathCache::new(
	/// #     (width, height),
	/// #     cost_fn(&grid),
	/// #     ManhattanNeighborhood::new(width, height),
	/// #     PathCacheConfig { chunk_size: 3, ..Default::default() },
	/// # );
	/// #
	/// let (start, goal) = ((0, 0), (2, 0));
	///
	/// let path = pathfinding.find_path(start, goal, cost_fn(&grid));
	/// assert!(path.is_none());
	///
	/// grid[1][2] = 0;
	/// grid[3][2] = 2;
	///
	/// assert_eq!(grid, [
	///     [0, 2, 0, 0, 0],
	///     [0, 2, 0, 2, 2],
	///     [0, 1, 0, 0, 0],
	///     [0, 1, 2, 2, 0],
	///     [0, 0, 0, 2, 0],
	/// ]);
	///
	/// pathfinding.tiles_changed(
	///     &[(2, 1), (2, 3)],
	///     cost_fn(&grid),
	/// );
	///
	/// let path = pathfinding.find_path(start, goal, cost_fn(&grid));
	/// assert!(path.is_some());
	/// ```
	pub fn tiles_changed(&mut self, tiles: &[Point], mut get_cost: impl FnMut(Point) -> isize) {
		let size = self.config.chunk_size;

		let mut dirty = PointMap::default();
		for &p in tiles {
			let chunk_pos = self.get_chunk_pos(p);
			dirty.entry(chunk_pos).or_insert_with(Vec::new).push(p);
		}

		// map of chunk_pos => array: [bool; 4] where array[side] == true if chunk[side] needs to be renewed
		let mut renew = PointMap::default();

		for (&cp, positions) in dirty.iter() {
			let chunk = get_chunk!(self, cp);
			// for every changed tile in the chunk
			for &p in positions {
				// check every side that this tile is on
				for dir in Dir::all().filter(|dir| chunk.sides[dir.num()] && chunk.at_side(p, *dir))
				{
					// if there is a chunk in that direction
					let other_pos = jump_in_dir(cp, dir, size, (0, 0), (self.width, self.height))
						.expect("Internal Error #2 in PathCache. Please report this");

					// remove the current and other sides
					renew.entry(cp).or_insert([false; 4])[dir.num()] = true;
					renew.entry(other_pos).or_insert([false; 4])[dir.opposite().num()] = true;
				}
			}

			// remove all non-border nodes
			let to_remove = chunk
				.nodes
				.iter()
				.filter(|id| {
					let pos = self.nodes[**id].pos;
					!chunk.at_any_side(pos)
				})
				.copied()
				.to_vec();

			let chunk = get_chunk_mut!(self, cp);
			for id in to_remove {
				chunk.nodes.remove(&id);
				self.nodes.remove_node(id);
			}
		}

		// remove all nodes of sides in renew
		for (&cp, sides) in renew.iter() {
			let removed = {
				let chunk = get_chunk!(self, cp);
				chunk
					.nodes
					.iter()
					.filter(|id| {
						let pos = self.nodes[**id].pos;
						Dir::all().any(|dir| sides[dir.num()] && chunk.at_side(pos, dir))
					})
					.copied()
					.to_vec()
			};

			let chunk = get_chunk_mut!(self, cp);

			for id in removed {
				chunk.nodes.remove(&id);
				self.nodes.remove_node(id);
			}
		}

		// remove all Paths in changed chunks
		for cp in dirty.keys() {
			let chunk = get_chunk!(self, cp);
			for id in chunk.nodes.iter() {
				self.nodes[*id].edges.clear();
			}
		}

		// recreate sides in renew
		for (&cp, sides) in renew.iter() {
			let mut candidates = PointSet::default();
			let chunk = get_chunk_mut!(self, cp);

			for dir in Dir::all() {
				if sides[dir.num()] {
					chunk.calculate_side_nodes(
						dir,
						(self.width, self.height),
						&mut get_cost,
						self.config,
						&mut candidates,
					);
				}
			}

			let all_nodes = &self.nodes;
			candidates.retain(|&p| all_nodes.values().find(|node| node.pos == p).is_none());

			if candidates.is_empty() {
				continue;
			}

			let all_nodes = &mut self.nodes;
			let nodes = candidates
				.into_iter()
				.map(|p| all_nodes.add_node(p, get_cost(p)))
				.to_vec();

			if !dirty.contains_key(&cp) {
				chunk.add_nodes(
					nodes,
					&mut get_cost,
					&self.neighborhood,
					&mut self.nodes,
					self.config,
				);
			} else {
				for id in nodes {
					chunk.nodes.insert(id);
				}
			}
		}

		// recreate Paths
		for cp in dirty.keys() {
			let chunk = get_chunk_mut!(self, cp);
			let nodes = chunk.nodes.iter().copied().to_vec();
			chunk.nodes.clear();

			chunk.add_nodes(
				nodes,
				&mut get_cost,
				&self.neighborhood,
				&mut self.nodes,
				self.config,
			);
		}

		// re-establish cross-chunk connections
		self.connect_nodes(&mut get_cost);
	}

	/// Allows for debugging and visualizing the PathCache
	///
	/// The returned object gives read-only access to the current state of the PathCache, mainly the
	/// Nodes and how they are connected to each other
	///
	/// ## Examples
	/// Basic usage:
	/// ```
	/// # use hierarchical_pathfinding::{prelude::*, Point};
	/// # use std::collections::HashSet;
	/// #
	/// # // create and initialize Grid
	/// # // 0 = empty, 1 = swamp, 2 = wall
	/// # let mut grid = [
	/// #     [0, 2, 0, 0, 0],
	/// #     [0, 2, 2, 2, 2],
	/// #     [0, 1, 0, 0, 0],
	/// #     [0, 1, 0, 2, 0],
	/// #     [0, 0, 0, 2, 0],
	/// # ];
	/// # let (width, height) = (grid.len(), grid[0].len());
	/// #
	/// # const COST_MAP: [isize; 3] = [1, 10, -1];
	/// #
	/// # fn cost_fn<'a>(grid: &'a [[usize; 5]; 5]) -> impl 'a + FnMut(Point) -> isize {
	/// #     move |(x, y)| COST_MAP[grid[y][x]]
	/// # }
	/// #
	/// # let mut pathfinding = PathCache::new(
	/// #     (width, height),
	/// #     cost_fn(&grid),
	/// #     ManhattanNeighborhood::new(width, height),
	/// #     PathCacheConfig { chunk_size: 3, ..Default::default() },
	/// # );
	/// #
	/// // create PathCache
	///
	/// // only draw the connections between Nodes once
	/// let mut visited = HashSet::new();
	///
	/// for node in pathfinding.inspect_nodes() {
	///     let pos = node.pos();
	///     // draw Node at x: pos.0, y: pos.1
	///
	///     visited.insert(node.id());
	///     
	///     for neighbor in node.connected().filter(|n| !visited.contains(&n.id())) {
	///         let other_pos = neighbor.pos();
	///         // draw Line from pos to other_pos
	///     }
	/// }
	/// ```
	pub fn inspect_nodes(&self) -> CacheInspector<N> {
		CacheInspector::new(self)
	}

	#[allow(dead_code)]
	fn print_nodes(&self) {
		for node in self.inspect_nodes() {
			print!("{} at {:?}: ", node.id(), node.pos());

			for neighbor in node.connected() {
				print!("{:?}, ", neighbor.pos());
			}

			println!();
		}
	}

	#[allow(dead_code)]
	fn get_chunk_pos(&self, point: Point) -> Point {
		let size = self.config.chunk_size;
		((point.0 / size) * size, (point.1 / size) * size)
	}

	#[allow(dead_code)]
	fn get_node_id(&self, pos: Point) -> Option<NodeID> {
		self.nodes.id_at(pos)
	}

	/// Returns the config used to create this PathCache
	pub fn config(&self) -> PathCacheConfig {
		self.config
	}

	fn add_node(&mut self, pos: Point, mut get_cost: impl FnMut(Point) -> isize) -> NodeID {
		let cost = get_cost(pos);
		let id = self.nodes.add_node(pos, cost);

		let chunk = get_chunk!(self, pos);

		if cost < 0 {
			for &other_id in chunk.nodes.iter() {
				let other_node = &self.nodes[other_id];

				if other_node.walk_cost < 0 {
					continue;
				}
				let other_pos = other_node.pos;
				if let Some(path) =
					chunk.find_path(other_pos, pos, &mut get_cost, &self.neighborhood)
				{
					self.nodes.add_edge(
						other_id,
						id,
						PathSegment::new(path, self.config.cache_paths),
					);
				}
			}

			let chunk = get_chunk_mut!(self, pos);
			chunk.nodes.insert(id);
		} else {
			let chunk = get_chunk_mut!(self, pos);
			chunk.add_nodes(
				vec![id],
				&mut get_cost,
				&self.neighborhood,
				&mut self.nodes,
				self.config,
			);
		}

		// add any direct neighbors
		let possible = self.neighborhood.get_all_neighbors(pos).to_vec();
		let neighbors = self
			.nodes
			.values()
			.filter(|node| possible.contains(&node.pos)) // any Node next to me
			.filter(|node| !node.edges.contains_key(&id)) // that is not already connected
			.map(|node| (node.id, node.pos))
			.to_vec();

		for (other_id, other_pos) in neighbors {
			if cost >= 0 {
				let path = generics::Path::new(vec![pos, other_pos], get_cost(pos) as usize);
				self.nodes[id]
					.edges
					.insert(other_id, PathSegment::new(path, self.config.cache_paths));
			}
			if get_cost(other_pos) >= 0 {
				let other_path =
					generics::Path::new(vec![other_pos, pos], get_cost(other_pos) as usize);

				self.nodes[other_id]
					.edges
					.insert(id, PathSegment::new(other_path, self.config.cache_paths));
			}
		}

		id
	}

	fn connect_nodes(&mut self, mut get_cost: impl FnMut(Point) -> isize) {
		let ids = self.nodes.keys().to_vec();
		for id in ids {
			let pos = self.nodes[id].pos;
			let possible = self.neighborhood.get_all_neighbors(pos).to_vec();
			let neighbors = self
				.nodes
				.values()
				.filter(|node| possible.contains(&node.pos)) // any Node next to me
				.filter(|node| !node.edges.contains_key(&id)) // that is not already connected
				.map(|node| (node.id, node.pos))
				.to_vec();

			for (other_id, other_pos) in neighbors {
				let path = generics::Path::new(vec![pos, other_pos], get_cost(pos) as usize);
				let other_path =
					generics::Path::new(vec![other_pos, pos], get_cost(other_pos) as usize);

				self.nodes[id]
					.edges
					.insert(other_id, PathSegment::new(path, self.config.cache_paths));

				self.nodes[other_id]
					.edges
					.insert(id, PathSegment::new(other_path, self.config.cache_paths));
			}
		}
	}

	#[allow(dead_code)]
	fn top(&self) -> usize {
		0
	}
	#[allow(dead_code)]
	fn right(&self) -> usize {
		self.width
	}
	#[allow(dead_code)]
	fn bottom(&self) -> usize {
		self.height
	}
	#[allow(dead_code)]
	fn left(&self) -> usize {
		0
	}
}

#[derive(Debug)]
pub struct CacheInspector<'a, N: Neighborhood> {
	src: &'a PathCache<N>,
	nodes: Vec<NodeID>,
	current_index: usize,
}

impl<'a, N: Neighborhood> CacheInspector<'a, N> {
	pub fn new(src: &'a PathCache<N>) -> Self {
		CacheInspector {
			src,
			nodes: src.nodes.keys().to_vec(),
			current_index: 0,
		}
	}

	pub fn get_node(&self, id: NodeID) -> NodeInspector<N> {
		NodeInspector::new(self.src, id)
	}
}

impl<'a, N: Neighborhood> Iterator for CacheInspector<'a, N> {
	type Item = NodeInspector<'a, N>;
	fn next(&mut self) -> Option<Self::Item> {
		let ret = self
			.nodes
			.get(self.current_index)
			.map(|id| NodeInspector::new(self.src, *id));

		if self.current_index < self.nodes.len() {
			self.current_index += 1;
		}
		ret
	}
}

#[derive(Debug)]
pub struct NodeInspector<'a, N: Neighborhood> {
	src: &'a PathCache<N>,
	node: &'a Node,
}

impl<'a, N: Neighborhood> NodeInspector<'a, N> {
	pub fn new(src: &'a PathCache<N>, id: NodeID) -> Self {
		NodeInspector {
			src,
			node: &src.nodes[id],
		}
	}

	pub fn pos(&self) -> Point {
		self.node.pos
	}

	pub fn id(&self) -> NodeID {
		self.node.id
	}

	pub fn connected(&'a self) -> impl Iterator<Item = NodeInspector<'a, N>> + 'a {
		self.node
			.edges
			.keys()
			.map(move |id| NodeInspector::new(self.src, *id))
	}
}
