use crate::{generics, neighbors::Neighborhood, NodeID, Point};

mod chunk;
use self::chunk::Chunk;

mod node;
use self::node::Node;

mod cache_config;
pub use self::cache_config::PathCacheConfig;

mod abstract_path;
pub use self::abstract_path::AbstractPath;
use self::abstract_path::AbstractPathImpl;

mod node_map;
use self::node_map::NodeMap;

use std::collections::HashMap;
use std::fmt::Debug;

/// A struct to store the Hierarchical Pathfinding information.
#[derive(Clone, Debug)]
pub struct PathCache<N: Neighborhood + Debug> {
	width: usize,
	height: usize,
	chunks: Vec<Vec<Chunk>>,
	nodes: NodeMap,
	neighborhood: N,
	config: PathCacheConfig,
}

impl<N: Neighborhood + Debug> PathCache<N> {
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
	/// use hierarchical_pathfinding::prelude::*;
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
	/// let cost_map = [
	///     1,  // empty
	///     10, // swamp
	///     -1, // wall = solid
	/// ];
	///
	/// let mut pathfinding = PathCache::new(
	///     (width, height), // the size of the Grid
	///     |(x, y)| cost_map[grid[y][x]], // get the cost for walking over a Tile
	///     ManhattanNeighborhood::new(width, height), // the Neighborhood
	///     PathCacheConfig { chunk_size: 3, ..Default::default() }, // config
	/// );
	/// ```
	pub fn new(
		(width, height): (usize, usize),
		get_cost: impl Fn(Point) -> isize,
		neighborhood: N,
		config: PathCacheConfig,
	) -> PathCache<N> {
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

		let mut chunks = Vec::with_capacity(chunk_hor);
		for x in 0..chunk_hor {
			let mut row = Vec::with_capacity(chunk_vert);
			let w = if x == chunk_hor - 1 {
				width % config.chunk_size
			} else {
				config.chunk_size
			};

			for y in 0..chunk_vert {
				let h = if y == chunk_vert - 1 {
					height % config.chunk_size
				} else {
					config.chunk_size
				};
				row.push(Chunk::new(
					(x * config.chunk_size, y * config.chunk_size),
					(w, h),
					&get_cost,
					&neighborhood,
					&mut nodes,
				))
			}

			chunks.push(row);
		}

		// connect neighboring Nodes across Chunk borders
		let ids = nodes.keys().cloned().collect::<Vec<_>>();
		for id in ids {
			let pos = nodes[&id].pos;
			let possible = neighborhood.get_all_neighbors(pos);
			let neighbors = nodes
				.values()
				.filter(|node| possible.contains(&node.pos)) // any Node next to me
				.filter(|node| !node.edges.contains_key(&id)) // that is not already connected
				.map(|node| (node.id, node.pos))
				.collect::<Vec<_>>();

			for (other_id, other_pos) in neighbors {
				let path = generics::Path::new(vec![pos, other_pos], get_cost(pos) as usize);
				let other_path =
					generics::Path::new(vec![other_pos, pos], get_cost(other_pos) as usize);

				let node = nodes.get_mut(&id).unwrap();
				node.edges.insert(other_id, path);

				let node = nodes.get_mut(&other_id).unwrap();
				node.edges.insert(id, other_path);
			}
		}

		PathCache {
			width,
			height,
			chunks,
			nodes,
			config,
			neighborhood,
		}
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
	/// # use hierarchical_pathfinding::prelude::*;
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
	/// # let cost_map = [
	/// #     1,  // empty
	/// #     10, // swamp
	/// #     -1, // wall = solid
	/// # ];
	/// #
	/// # let mut pathfinding = PathCache::new(
	/// #     (width, height), // the size of the Grid
	/// #     |(x, y)| cost_map[grid[y][x]], // get the cost for walking over a Tile
	/// #     ManhattanNeighborhood::new(width, height), // the Neighborhood
	/// #     PathCacheConfig { chunk_size: 3, ..Default::default() }, // config
	/// # );
	/// #
	/// let start = (0, 0);
	/// let goal = (4, 4);
	///
	/// // find_path returns Some(Path) on success
	/// let path = pathfinding.find_path(
	///     start,
	///     goal,
	///     |(x, y)| cost_map[grid[y][x]], // cost function
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
	/// # use hierarchical_pathfinding::prelude::*;
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
	/// # let cost_map = [
	/// #     1,  // empty
	/// #     10, // swamp
	/// #     -1, // wall = solid
	/// # ];
	/// #
	/// # let mut pathfinding = PathCache::new(
	/// #     (width, height), // the size of the Grid
	/// #     |(x, y)| cost_map[grid[y][x]], // get the cost for walking over a Tile
	/// #     ManhattanNeighborhood::new(width, height), // the Neighborhood
	/// #     PathCacheConfig { chunk_size: 3, ..Default::default() }, // config
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
	/// let path = pathfinding.find_path(
	///     player.pos,
	///     goal,
	///     |(x, y)| cost_map[grid[y][x]], // cost function
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
	/// # use hierarchical_pathfinding::prelude::*;
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
	/// # let cost_map = [
	/// #     1,  // empty
	/// #     10, // swamp
	/// #     -1, // wall = solid
	/// # ];
	/// #
	/// # let mut pathfinding = PathCache::new(
	/// #     (width, height), // the size of the Grid
	/// #     |(x, y)| cost_map[grid[y][x]], // get the cost for walking over a Tile
	/// #     ManhattanNeighborhood::new(width, height), // the Neighborhood
	/// #     PathCacheConfig { chunk_size: 3, ..Default::default() }, // config
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
	/// #     |(x, y)| cost_map[grid[y][x]], // cost function
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
		get_cost: impl Fn(Point) -> isize,
	) -> Option<impl AbstractPath> {
		let start_id = self
			.get_node_id(start)
			.unwrap_or_else(|| self.add_node(start, &get_cost));

		let goal_id = self
			.get_node_id(goal)
			.unwrap_or_else(|| self.add_node(goal, &get_cost));

		let path = generics::a_star_search(
			|id| self.nodes[&id].edges.keys().cloned().collect(),
			|a, b| self.nodes[&a].edges[&b].cost,
			|id| self.nodes[&id].walk_cost >= 0,
			start_id,
			goal_id,
			|id| self.neighborhood.heuristic(self.nodes[&id].pos, goal),
		);

		if let Some(path) = path {
			let length = if self.config.cache_paths {
				path.windows(2)
					.map(|ids| self.nodes[&ids[0]].edges[&ids[1]].len())
					.sum()
			} else {
				path.cost
			};

			if self.config.a_star_fallback && length < 2 * self.config.chunk_size {
				let path = generics::a_star_search(
					|p| self.neighborhood.get_all_neighbors(p),
					|p, _| get_cost(p) as usize,
					|p| get_cost(p) >= 0,
					start,
					goal,
					|p| self.neighborhood.heuristic(p, goal),
				)
				.expect("Internal Error in PathCache. Please report this");

				Some(AbstractPathImpl::<N>::from_known_path(path))
			} else {
				let mut ret = AbstractPathImpl::<N>::new(start);
				for ids in path.windows(2) {
					let path = &self.nodes[&ids[0]].edges[&ids[1]];
					if self.config.cache_paths {
						ret.add_path(path.clone());
					} else {
						ret.add_node(self.nodes[&ids[1]].pos, path.cost);
					}
				}
				Some(ret)
			}
		} else {
			None
		}
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
	/// # use hierarchical_pathfinding::prelude::*;
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
	/// # let cost_map = [
	/// #     1,  // empty
	/// #     10, // swamp
	/// #     -1, // wall = solid
	/// # ];
	/// #
	/// # let mut pathfinding = PathCache::new(
	/// #     (width, height), // the size of the Grid
	/// #     |(x, y)| cost_map[grid[y][x]], // get the cost for walking over a Tile
	/// #     ManhattanNeighborhood::new(width, height), // the Neighborhood
	/// #     PathCacheConfig { chunk_size: 3, ..Default::default() }, // config
	/// # );
	/// #
	/// let start = (0, 0);
	/// let goals = [(4, 4), (0, 2)];
	///
	/// // find_paths returns a HashMap<goal, Path> for all successes
	/// let paths = pathfinding.find_paths(
	///     start,
	///     &goals,
	///     |(x, y)| cost_map[grid[y][x]], // cost function
	/// );
	///
	/// assert!(paths.contains_key(&goals[0]));
	///
	/// assert!(!paths.contains_key(&goals[1]));
	/// ```
	///
	/// The returned Path is always equivalent to the one returned by [`find_path`](PathCache::find_path):
	/// ```
	/// # use hierarchical_pathfinding::prelude::*;
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
	/// # let cost_map = [
	/// #     1,  // empty
	/// #     10, // swamp
	/// #     -1, // wall = solid
	/// # ];
	/// #
	/// # let mut pathfinding = PathCache::new(
	/// #     (width, height), // the size of the Grid
	/// #     |(x, y)| cost_map[grid[y][x]], // get the cost for walking over a Tile
	/// #     ManhattanNeighborhood::new(width, height), // the Neighborhood
	/// #     PathCacheConfig { chunk_size: 3, ..Default::default() }, // config
	/// # );
	/// #
	/// let start = (0, 0);
	/// let goal = (4, 4);
	///
	/// let paths = pathfinding.find_paths(
	///     start,
	///     &[goal],
	///     |(x, y)| cost_map[grid[y][x]], // cost function
	/// );
	/// let dijkstra_path: Vec<Point> = paths[&goal].collect();
	///
	/// let a_star_path = pathfinding.find_path(
	///     start,
	///     goal,
	///     |(x, y)| cost_map[grid[y][x]], // cost function
	/// ).unwrap().collect();
	///
	/// assert_eq!(dijkstra_path, a_star_path);
	/// ```
	pub fn find_paths(
		&mut self,
		_start: Point,
		_goals: &[Point],
		_get_cost: impl 'static + Fn(Point) -> isize,
	) -> HashMap<Point, impl AbstractPath> {
		let ret: HashMap<Point, AbstractPathImpl<N>> = HashMap::new();
		ret
	}

	#[allow(dead_code)]
	fn get_chunk_pos(&self, point: Point) -> Point {
		let size = self.config.chunk_size;
		((point.0 / size) * size, (point.1 / size) * size)
	}
	#[allow(dead_code)]
	fn get_chunk(&self, point: Point) -> &Chunk {
		let size = self.config.chunk_size;
		&self.chunks[point.0 / size][point.1 / size]
	}
	#[allow(dead_code)]
	fn get_chunk_mut(&mut self, point: Point) -> &mut Chunk {
		let size = self.config.chunk_size;
		&mut self.chunks[point.0 / size][point.1 / size]
	}

	#[allow(dead_code)]
	fn get_node_id(&self, pos: Point) -> Option<NodeID> {
		self.nodes
			.iter()
			.find(|(_, node)| node.pos == pos)
			.map(|(id, _)| *id)
	}

	fn add_node(&mut self, pos: Point, get_cost: &Fn(Point) -> isize) -> NodeID {
		let cost = get_cost(pos);
		let id = self.nodes.add_node(pos, cost);

		let size = self.config.chunk_size;
		let chunk = &self.chunks[pos.0 / size][pos.1 / size];
		let nodes: Vec<NodeID> = chunk.nodes.iter().cloned().collect();

		if cost < 0 {
			for other_id in nodes {
				let other_node = &self.nodes[&other_id];

				if other_node.walk_cost < 0 {
					continue;
				}
				let other_pos = other_node.pos;
				if let Some(path) = chunk.find_path(other_pos, pos, &get_cost, &self.neighborhood) {
					self.nodes.add_edge(other_id, id, path);
				}
			}
			return id;
		}

		let positions = chunk
			.nodes
			.iter()
			.map(|id| self.nodes[id].pos)
			.collect::<Vec<_>>();

		let mut paths = chunk.find_paths(pos, &positions, &get_cost, &self.neighborhood);

		for (other_id, pos) in chunk.nodes.iter().zip(positions.iter()) {
			if let Some(path) = paths.remove(pos) {
				self.nodes.add_edge(id, *other_id, path);
			}
		}

		id
	}
}
