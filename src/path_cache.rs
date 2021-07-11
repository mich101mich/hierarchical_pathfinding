use crate::{
    graph::{self, Node, NodeID, NodeIDMap, NodeIDSet, NodeMap},
    neighbors::Neighborhood,
    path::{AbstractPath, Cost, Path, PathSegment},
    *,
};

use log::trace;
macro_rules! re_trace {
    ($msg: literal, $timer: ident) => {
        let now = std::time::Instant::now();
        trace!(concat!("time to ", $msg, ": {:?}"), now - $timer);
        #[allow(unused_assignments)]
        {
            $timer = now;
        }
    };
}

mod cache_config;
pub use cache_config::PathCacheConfig;

mod chunk;
use chunk::Chunk;

enum CostFnWrapper<F1, F2>
where
    F1: Fn(Point) -> isize,
    F2: FnMut(Point) -> isize,
{
    #[allow(unused)]
    Parallel(F1),
    Sequential(F2),
}

/// A struct to store the Hierarchical Pathfinding information.
#[derive(Clone, Debug)]
pub struct PathCache<N: Neighborhood> {
    width: usize,
    height: usize,
    chunks: Vec<Chunk>,
    num_chunks: (usize, usize),
    nodes: NodeMap,
    neighborhood: N,
    config: PathCacheConfig,
}

impl<N: Neighborhood + Sync> PathCache<N> {
    /// Creates a new PathCache
    ///
    /// ## Arguments
    /// - `(width, height)` - the size of the Grid
    /// - `get_cost` - get the cost for walking over a Tile. (Cost < 0 means solid Tile)
    /// - `neighborhood` - the Neighborhood to use. (See [`Neighborhood`])
    /// - `config` - optional config for creating the cache. (See [`PathCacheConfig`])
    ///
    /// `get_cost((x, y))` should return the cost for walking over the Tile at (x, y).
    /// Costs below 0 are solid Tiles.
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
    /// type Grid = [[usize; 5]; 5];
    ///
    /// const COST_MAP: [isize; 3] = [1, 10, -1];
    ///
    /// fn cost_fn(grid: &Grid) -> impl '_ + Sync + Fn((usize, usize)) -> isize {
    ///     move |(x, y)| COST_MAP[grid[y][x]]
    /// }
    ///
    /// let mut pathfinding = PathCache::new(
    ///     (width, height), // the size of the Grid
    ///     cost_fn(&grid), // get the cost for walking over a Tile
    ///     ManhattanNeighborhood::new(width, height), // the Neighborhood
    ///     PathCacheConfig::with_chunk_size(3), // config
    /// );
    /// ```
    pub fn new<F: Sync + Fn(Point) -> isize>(
        (width, height): (usize, usize),
        get_cost: F,
        neighborhood: N,
        config: PathCacheConfig,
    ) -> PathCache<N> {
        #[cfg(feature = "parallel")]
        {
            PathCache::new_internal::<F, fn(Point) -> isize>(
                (width, height),
                CostFnWrapper::Parallel(get_cost),
                neighborhood,
                config,
            )
        }
        #[cfg(not(feature = "parallel"))]
        {
            PathCache::new_internal::<fn(Point) -> isize, F>(
                (width, height),
                CostFnWrapper::Sequential(get_cost),
                neighborhood,
                config,
            )
        }
    }

    /// Same as [`new`](PathCache::new), but doesn't use threads to allow [`FnMut`].
    ///
    /// Equivalent to `new` if `parallel` feature is disabled.
    ///
    /// Note that this is _**way**_ slower than `new` with `parallel`.
    pub fn new_with_fn_mut<F: FnMut(Point) -> isize>(
        (width, height): (usize, usize),
        get_cost: F,
        neighborhood: N,
        config: PathCacheConfig,
    ) -> PathCache<N> {
        PathCache::new_internal::<fn(Point) -> isize, F>(
            (width, height),
            CostFnWrapper::Sequential(get_cost),
            neighborhood,
            config,
        )
    }

    fn new_internal<F1, F2>(
        (width, height): (usize, usize),
        get_cost: CostFnWrapper<F1, F2>,
        neighborhood: N,
        config: PathCacheConfig,
    ) -> PathCache<N>
    where
        F1: Sync + Fn(Point) -> isize,
        F2: FnMut(Point) -> isize,
    {
        // calculate chunk size
        let (num_chunks_w, last_width) = {
            let w = width / config.chunk_size;
            let remain = width - w * config.chunk_size;
            if remain > 0 {
                (w + 1, remain)
            } else {
                (w, config.chunk_size)
            }
        };
        let (num_chunks_h, last_height) = {
            let h = height / config.chunk_size;
            let remain = height - h * config.chunk_size;
            if remain > 0 {
                (h + 1, remain)
            } else {
                (h, config.chunk_size)
            }
        };

        let mut nodes = NodeMap::new();

        // create chunks
        let chunks = match get_cost {
            CostFnWrapper::Sequential(mut get_cost) => {
                let mut chunks: Vec<Chunk> = Vec::with_capacity(num_chunks_w * num_chunks_h);
                for y in 0..num_chunks_h {
                    let h = if y == num_chunks_h - 1 {
                        last_height
                    } else {
                        config.chunk_size
                    };

                    for x in 0..num_chunks_w {
                        let w = if x == num_chunks_w - 1 {
                            last_width
                        } else {
                            config.chunk_size
                        };

                        chunks.push(Chunk::new(
                            (x * config.chunk_size, y * config.chunk_size),
                            (w, h),
                            (width, height),
                            &mut get_cost,
                            &neighborhood,
                            &mut nodes,
                            config,
                        ));
                    }
                }
                chunks
            }
            #[cfg(not(feature = "parallel"))]
            _ => panic!("Created a Parallel CostFnWrapper in a non-parallel environment"),
            #[cfg(feature = "parallel")]
            CostFnWrapper::Parallel(get_cost) => {
                use rayon::prelude::*;

                let raw_chunks: Vec<(Chunk, NodeMap)> = (0..num_chunks_h * num_chunks_w)
                    .into_par_iter()
                    .map(|index| {
                        let x = index % num_chunks_w;
                        let y = index / num_chunks_w;

                        let w = if x == num_chunks_w - 1 {
                            last_width
                        } else {
                            config.chunk_size
                        };

                        let h = if y == num_chunks_h - 1 {
                            last_height
                        } else {
                            config.chunk_size
                        };

                        let mut node_map = NodeMap::new();

                        let chunk = Chunk::new(
                            (x * config.chunk_size, y * config.chunk_size),
                            (w, h),
                            (width, height),
                            &get_cost,
                            &neighborhood,
                            &mut node_map,
                            config,
                        );

                        (chunk, node_map)
                    })
                    .collect();

                raw_chunks
                    .into_iter()
                    .map(|(mut chunk, new_nodes)| {
                        chunk.nodes = nodes.absorb(new_nodes);
                        chunk
                    })
                    .to_vec()
            }
        };

        let mut cache = PathCache {
            width,
            height,
            chunks,
            num_chunks: (num_chunks_w, num_chunks_h),
            nodes,
            neighborhood,
            config,
        };

        // connect neighboring Nodes across Chunk borders
        cache.connect_nodes(None);

        cache
    }

    /// Calculates the Path from `start` to `goal` on the Grid.
    ///
    /// If no Path could be found, `None` is returned.
    ///
    /// `get_cost((x, y))` should return the cost for walking over the Tile at (x, y).
    /// Costs below 0 are solid Tiles.
    ///
    /// ## Examples
    /// Basic usage:
    /// ```
    /// # use hierarchical_pathfinding::prelude::*;
    /// # let mut grid = [
    /// #     [0, 2, 0, 0, 0],
    /// #     [0, 2, 2, 2, 2],
    /// #     [0, 1, 0, 0, 0],
    /// #     [0, 1, 0, 2, 0],
    /// #     [0, 0, 0, 2, 0],
    /// # ];
    /// # let (width, height) = (grid.len(), grid[0].len());
    /// # fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + Sync + Fn((usize, usize)) -> isize {
    /// #     move |(x, y)| [1, 10, -1][grid[y][x]]
    /// # }
    /// let pathfinding: PathCache<_> = // ...
    /// # PathCache::new(
    /// #     (width, height),
    /// #     cost_fn(&grid),
    /// #     ManhattanNeighborhood::new(width, height),
    /// #     PathCacheConfig::with_chunk_size(3),
    /// # );
    ///
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
    /// # use hierarchical_pathfinding::prelude::*;
    /// # let mut grid = [
    /// #     [0, 2, 0, 0, 0],
    /// #     [0, 2, 2, 2, 2],
    /// #     [0, 1, 0, 0, 0],
    /// #     [0, 1, 0, 2, 0],
    /// #     [0, 0, 0, 2, 0],
    /// # ];
    /// # let (width, height) = (grid.len(), grid[0].len());
    /// # fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + Sync + Fn((usize, usize)) -> isize {
    /// #     move |(x, y)| [1, 10, -1][grid[y][x]]
    /// # }
    /// # let pathfinding = PathCache::new(
    /// #     (width, height),
    /// #     cost_fn(&grid),
    /// #     ManhattanNeighborhood::new(width, height),
    /// #     PathCacheConfig::with_chunk_size(3),
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
    /// # use hierarchical_pathfinding::prelude::*;
    /// # let mut grid = [
    /// #     [0, 2, 0, 0, 0],
    /// #     [0, 2, 2, 2, 2],
    /// #     [0, 1, 0, 0, 0],
    /// #     [0, 1, 0, 2, 0],
    /// #     [0, 0, 0, 2, 0],
    /// # ];
    /// # let (width, height) = (grid.len(), grid[0].len());
    /// # fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + Sync + Fn((usize, usize)) -> isize {
    /// #     move |(x, y)| [1, 10, -1][grid[y][x]]
    /// # }
    /// # let pathfinding = PathCache::new(
    /// #     (width, height),
    /// #     cost_fn(&grid),
    /// #     ManhattanNeighborhood::new(width, height),
    /// #     PathCacheConfig::with_chunk_size(3),
    /// # );
    /// # let start = (0, 0);
    /// # let goal = (4, 4);
    /// # let path = pathfinding.find_path(
    /// #     start,
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
        &self,
        start: Point,
        goal: Point,
        mut get_cost: impl FnMut(Point) -> isize,
    ) -> Option<AbstractPath<N>> {
        let outer_timer = std::time::Instant::now();
        let mut timer = outer_timer;

        if get_cost(start) < 0 {
            // cannot start on a wall
            return None;
        }

        let neighborhood = self.neighborhood.clone();

        if start == goal {
            return Some(AbstractPath::from_known_path(
                neighborhood,
                Path::from_slice(&[start, start], 0),
            ));
        }

        let (start_id, start_path) =
            if let Some(s) = self.find_nearest_node(start, &mut get_cost, false) {
                s
            } else {
                // no path from start to any Node => start is in cave within chunk
                // => hope that goal is in the same cave
                return self
                    .get_chunk(start)
                    .find_path(start, goal, get_cost, &neighborhood)
                    .map(|path| AbstractPath::from_known_path(neighborhood, path));
            };

        // try-operator: see above, but we know that start is not in a cave
        let (goal_id, goal_path) = self.find_nearest_node(goal, &mut get_cost, true)?;

        re_trace!("find nodes", timer);

        // size hint for number of visited nodes in graph::a_star_search:
        //     percentage of total area visited (heuristic / max_heuristic)
        //     as the percentage of nodes visited ( * self.nodes.len())
        let heuristic = neighborhood.heuristic(start, goal);
        let max_heuristic = neighborhood.heuristic((0, 0), (self.width - 1, self.height - 1));
        let max_size = self.nodes.len();
        let size_hint = heuristic as f32 / max_heuristic as f32 * max_size as f32;

        let path = graph::a_star_search(
            &self.nodes,
            start_id,
            goal_id,
            &neighborhood,
            size_hint as usize,
        )?;

        re_trace!("graph::a_star_search", timer);

        if path.len() == 2 || (self.config.a_star_fallback && path.len() <= 4) {
            // 2: start_id == goal_id
            // <= 4: start_id X X goal_id
            let res = self
                .grid_a_star(start, goal, get_cost)
                .map(|path| AbstractPath::from_known_path(neighborhood, path));

            re_trace!("A* fallback", timer);
            trace!("total time: {:?}", std::time::Instant::now() - outer_timer);

            return res;
        }

        let mut paths = NodeIDMap::default();
        paths.insert(goal_id, path);

        let res = self
            .resolve_paths(
                start,
                start_path,
                &[(goal, goal_id, goal_path)],
                &paths,
                get_cost,
            )
            .into_iter()
            .next()
            .map(|(_, path)| path);

        re_trace!("resolve_paths", timer);
        trace!("total time: {:?}", std::time::Instant::now() - outer_timer);

        res
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
    /// `get_cost((x, y))` should return the cost for walking over the Tile at (x, y).
    /// Costs below 0 are solid Tiles.
    ///
    /// See [`find_path`](PathCache::find_path) for more details on how to use the returned Paths.
    ///
    /// ## Examples
    /// Basic usage:
    /// ```
    /// # use hierarchical_pathfinding::prelude::*;
    /// # let mut grid = [
    /// #     [0, 2, 0, 0, 0],
    /// #     [0, 2, 2, 2, 2],
    /// #     [0, 1, 0, 0, 0],
    /// #     [0, 1, 0, 2, 0],
    /// #     [0, 0, 0, 2, 0],
    /// # ];
    /// # let (width, height) = (grid.len(), grid[0].len());
    /// # fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + Sync + Fn((usize, usize)) -> isize {
    /// #     move |(x, y)| [1, 10, -1][grid[y][x]]
    /// # }
    /// let pathfinding: PathCache<_> = // ...
    /// # PathCache::new(
    /// #     (width, height),
    /// #     cost_fn(&grid),
    /// #     ManhattanNeighborhood::new(width, height),
    /// #     PathCacheConfig::with_chunk_size(3),
    /// # );
    ///
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
    /// # use hierarchical_pathfinding::prelude::*;
    /// # let mut grid = [
    /// #     [0, 2, 0, 0, 0],
    /// #     [0, 2, 2, 2, 2],
    /// #     [0, 1, 0, 0, 0],
    /// #     [0, 1, 0, 2, 0],
    /// #     [0, 0, 0, 2, 0],
    /// # ];
    /// # let (width, height) = (grid.len(), grid[0].len());
    /// # fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + Sync + Fn((usize, usize)) -> isize {
    /// #     move |(x, y)| [1, 10, -1][grid[y][x]]
    /// # }
    /// # let pathfinding = PathCache::new(
    /// #     (width, height),
    /// #     cost_fn(&grid),
    /// #     ManhattanNeighborhood::new(width, height),
    /// #     PathCacheConfig::with_chunk_size(3),
    /// # );
    /// let start = (0, 0);
    /// let goal = (4, 4);
    ///
    /// let paths = pathfinding.find_paths(
    ///     start,
    ///     &[goal],
    ///     cost_fn(&grid),
    /// );
    /// let dijkstra_path: Vec<_> = paths[&goal].clone().collect();
    ///
    /// let a_star_path: Vec<_> = pathfinding.find_path(
    ///     start,
    ///     goal,
    ///     cost_fn(&grid),
    /// ).unwrap().collect();
    ///
    /// assert_eq!(dijkstra_path, a_star_path);
    /// ```
    pub fn find_paths(
        &self,
        start: Point,
        goals: &[Point],
        get_cost: impl FnMut(Point) -> isize,
    ) -> PointMap<AbstractPath<N>> {
        self.find_paths_internal(start, goals, get_cost, false)
    }

    /// Finds the closest from a list of goals.
    ///
    /// Returns a tuple of the goal and the Path to that goal, or `None` if none of the goals are
    /// reachable.
    ///
    /// Similar to [`find_paths`](PathCache::find_paths) in performance and search strategy, but
    /// stops after the first goal is found.
    ///
    /// ## Examples
    /// Basic usage:
    /// ```
    /// # use hierarchical_pathfinding::prelude::*;
    /// # let mut grid = [
    /// #     [0, 2, 0, 0, 0],
    /// #     [0, 2, 2, 2, 2],
    /// #     [0, 1, 0, 0, 0],
    /// #     [0, 1, 0, 2, 0],
    /// #     [0, 0, 0, 2, 0],
    /// # ];
    /// # let (width, height) = (grid.len(), grid[0].len());
    /// # fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + Sync + Fn((usize, usize)) -> isize {
    /// #     move |(x, y)| [1, 10, -1][grid[y][x]]
    /// # }
    /// let pathfinding: PathCache<_> = // ...
    /// # PathCache::new(
    /// #     (width, height),
    /// #     cost_fn(&grid),
    /// #     ManhattanNeighborhood::new(width, height),
    /// #     PathCacheConfig::with_chunk_size(3),
    /// # );
    ///
    /// let start = (0, 0);
    /// let goals = [(4, 4), (2, 0), (2, 2)];
    ///
    /// // find_closest_goal returns Some((goal, Path)) on success
    /// let (goal, path) = pathfinding.find_closest_goal(
    ///     start,
    ///     &goals,
    ///     cost_fn(&grid),
    /// ).unwrap();
    ///
    /// assert_eq!(goal, goals[2]);
    ///
    /// let naive_closest = pathfinding
    ///     .find_paths(start, &goals, cost_fn(&grid))
    ///     .into_iter()
    ///     .min_by_key(|(_, path)| path.cost())
    ///     .unwrap();
    ///
    /// assert_eq!(goal, naive_closest.0);
    ///
    /// let path: Vec<_> = path.collect();
    /// let naive_path: Vec<_> = naive_closest.1.collect();
    /// assert_eq!(path, naive_path);
    /// ```
    /// Comparison with [`find_paths`](PathCache::find_paths):
    /// ```
    /// # use hierarchical_pathfinding::prelude::*;
    /// # let mut grid = [
    /// #     [0, 2, 0, 0, 0],
    /// #     [0, 2, 2, 2, 2],
    /// #     [0, 1, 0, 0, 0],
    /// #     [0, 1, 0, 2, 0],
    /// #     [0, 0, 0, 2, 0],
    /// # ];
    /// # let (width, height) = (grid.len(), grid[0].len());
    /// # fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + Sync + Fn((usize, usize)) -> isize {
    /// #     move |(x, y)| [1, 10, -1][grid[y][x]]
    /// # }
    /// # let pathfinding = PathCache::new(
    /// #     (width, height),
    /// #     cost_fn(&grid),
    /// #     ManhattanNeighborhood::new(width, height),
    /// #     PathCacheConfig::with_chunk_size(3),
    /// # );
    /// # let start = (0, 0);
    /// # let goals = [(4, 4), (2, 0), (2, 2)];
    /// let (goal, path) = pathfinding.find_closest_goal(
    ///     start,
    ///     &goals,
    ///     cost_fn(&grid),
    /// ).unwrap();
    ///
    /// let naive_closest = pathfinding
    ///     .find_paths(start, &goals, cost_fn(&grid))
    ///     .into_iter()
    ///     .min_by_key(|(_, path)| path.cost())
    ///     .unwrap();
    ///
    /// assert_eq!(goal, naive_closest.0);
    ///
    /// let path: Vec<_> = path.collect();
    /// let naive_path: Vec<_> = naive_closest.1.collect();
    /// assert_eq!(path, naive_path);
    /// ```
    pub fn find_closest_goal(
        &self,
        start: Point,
        goals: &[Point],
        get_cost: impl FnMut(Point) -> isize,
    ) -> Option<(Point, AbstractPath<N>)> {
        self.find_paths_internal(start, goals, get_cost, true)
            .into_iter()
            .next()
    }

    fn find_paths_internal(
        &self,
        start: Point,
        goals: &[Point],
        mut get_cost: impl FnMut(Point) -> isize,
        only_closest_goal: bool,
    ) -> PointMap<AbstractPath<N>> {
        if get_cost(start) < 0 || goals.is_empty() {
            return PointMap::default();
        }

        if goals.len() == 1 {
            let goal = goals[0];
            return self
                .find_path(start, goal, get_cost)
                .map(|path| (goal, path))
                .into_iter()
                .collect();
        }

        let neighborhood = self.neighborhood.clone();

        let (start_id, start_path) =
            if let Some(s) = self.find_nearest_node(start, &mut get_cost, false) {
                s
            } else {
                // no path from start to any Node => start is in cave within chunk
                // => find all goals in the same cave
                return self
                    .get_chunk(start)
                    .find_paths(start, goals, get_cost, &neighborhood)
                    .into_iter()
                    .map(|(goal, path)| {
                        (
                            goal,
                            AbstractPath::from_known_path(neighborhood.clone(), path),
                        )
                    })
                    .collect();
            };

        let mut goal_data = Vec::with_capacity(goals.len());
        let mut goal_ids = Vec::with_capacity(goals.len());

        let mut ret = PointMap::default();
        let mut heuristic = 0;

        for goal in goals.iter().copied() {
            if goal == start {
                let path = AbstractPath::from_known_path(
                    self.neighborhood.clone(),
                    Path::from_slice(&[start, start], 0),
                );
                ret.insert(goal, path);
                continue;
            }

            let (goal_id, goal_path) =
                if let Some(g) = self.find_nearest_node(goal, &mut get_cost, true) {
                    g
                } else {
                    continue;
                };

            goal_data.push((goal, goal_id, goal_path));
            goal_ids.push(goal_id);
            if only_closest_goal {
                heuristic = heuristic.min(self.neighborhood.heuristic(start, goal));
            } else {
                heuristic = heuristic.max(self.neighborhood.heuristic(start, goal));
            }
        }

        let max_heuristic = neighborhood.heuristic((0, 0), (self.width - 1, self.height - 1));
        let max_size = self.nodes.len();
        let size_hint = heuristic as f32 / max_heuristic as f32 * max_size as f32;

        let paths = graph::dijkstra_search(
            &self.nodes,
            start_id,
            &goal_ids,
            only_closest_goal,
            size_hint as usize,
        );

        self.resolve_paths(start, start_path, &goal_data, &paths, get_cost)
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
    /// # use hierarchical_pathfinding::prelude::*;
    /// # let mut grid = [
    /// #     [0, 2, 0, 0, 0],
    /// #     [0, 2, 2, 2, 2],
    /// #     [0, 1, 0, 0, 0],
    /// #     [0, 1, 0, 2, 0],
    /// #     [0, 0, 0, 2, 0],
    /// # ];
    /// # let (width, height) = (grid.len(), grid[0].len());
    /// # fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + Sync + Fn((usize, usize)) -> isize {
    /// #     move |(x, y)| [1, 10, -1][grid[y][x]]
    /// # }
    /// let mut pathfinding: PathCache<_> = // ...
    /// # PathCache::new(
    /// #     (width, height),
    /// #     cost_fn(&grid),
    /// #     ManhattanNeighborhood::new(width, height),
    /// #     PathCacheConfig::with_chunk_size(3),
    /// # );
    ///
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
    pub fn tiles_changed<F: Sync + Fn(Point) -> isize>(&mut self, tiles: &[Point], get_cost: F) {
        #[cfg(feature = "parallel")]
        {
            self.tiles_changed_internal::<F, fn(Point) -> isize>(
                tiles,
                CostFnWrapper::Parallel(get_cost),
            )
        }
        #[cfg(not(feature = "parallel"))]
        {
            self.tiles_changed_internal::<fn(Point) -> isize, F>(
                tiles,
                CostFnWrapper::Sequential(get_cost),
            )
        }
    }

    /// Same as [`tiles_changed`](PathCache::tiles_changed), but doesn't use threads to allow [`FnMut`].
    ///
    /// Equivalent to `tiles_changed` if `parallel` feature is disabled.
    ///
    /// Note that this is _**way**_ slower than `tiles_changed` with `parallel`.
    pub fn tiles_changed_with_fn_mut<F: FnMut(Point) -> isize>(
        &mut self,
        tiles: &[Point],
        get_cost: F,
    ) {
        self.tiles_changed_internal::<fn(Point) -> isize, F>(
            tiles,
            CostFnWrapper::Sequential(get_cost),
        )
    }

    fn tiles_changed_internal<F1, F2>(
        &mut self,
        tiles: &[Point],
        mut get_cost: CostFnWrapper<F1, F2>,
    ) where
        F1: Sync + Fn(Point) -> isize,
        F2: FnMut(Point) -> isize,
    {
        let size = self.config.chunk_size;

        let outer_timer = std::time::Instant::now();
        let mut timer = outer_timer;

        let mut dirty = PointMap::default();
        for &p in tiles {
            let chunk_pos = self.get_chunk_pos(p);
            dirty.entry(chunk_pos).or_insert_with(Vec::new).push(p);
        }

        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        enum Renew {
            No,
            Inner,
            Corner(Point),
            All,
        }

        // map of chunk_pos => array: [Renew; 4] where array[side] says if chunk[side] needs to be renewed
        let mut renew = PointMap::default();

        for (&cp, positions) in dirty.iter() {
            let chunk = self.get_chunk(cp);
            // for every changed tile in the chunk
            for &p in positions {
                // check every side that this tile is on
                for dir in Dir::all().filter(|dir| chunk.sides[dir.num()] && chunk.at_side(p, *dir))
                {
                    // if there is a chunk in that direction
                    let other_pos = jump_in_dir(cp, dir, size, (0, 0), (self.width, self.height))
                        .expect("Internal Error #2 in PathCache. Please report this");

                    // mark the current and other side
                    let own = &mut renew.entry(cp).or_insert([Renew::No; 4])[dir.num()];
                    let old = *own;
                    if chunk.is_corner(p) {
                        if old == Renew::No || old == Renew::Inner {
                            *own = Renew::Corner(p);
                        } else if let Renew::Corner(p2) = old {
                            if p2 != p {
                                *own = Renew::All;
                            }
                        } else if old != Renew::All {
                            *own = Renew::Corner(p)
                        }
                    } else {
                        // All > Corner > Inner > No, and we don't want to override anything greater than Inner
                        if *own == Renew::No {
                            *own = Renew::Inner;
                        }
                    }
                    let other =
                        &mut renew.entry(other_pos).or_insert([Renew::No; 4])[dir.opposite().num()];
                    if *other == Renew::No {
                        *other = Renew::Inner;
                    }
                }
            }
        }

        re_trace!("establish renew", timer);

        // remove all nodes of sides in renew

        for (&cp, sides) in renew.iter() {
            let chunk_index = self.get_chunk_index(cp);
            let chunk = &self.chunks[chunk_index];
            let removed = chunk
                .nodes
                .iter()
                .filter(|id| {
                    let pos = self.nodes[**id].pos;
                    let corner = chunk.is_corner(pos);
                    Dir::all().any(|dir| match sides[dir.num()] {
                            Renew::No => false,
                            Renew::Inner => !corner,
                            Renew::Corner(c) => !corner || c == pos,
                            Renew::All => true,
                        } && chunk.at_side(pos, dir))
                })
                .copied()
                .to_vec();

            let chunk = &mut self.chunks[chunk_index];

            for id in removed {
                chunk.nodes.remove(&id);
                self.nodes.remove_node(id);
            }
        }

        re_trace!("remove nodes of sides in renew", timer);

        let mut changed_nodes = NodeIDSet::default();

        // remove all Paths in changed chunks
        for cp in dirty.keys() {
            let chunk_index = self.get_chunk_index(*cp);
            let chunk = &self.chunks[chunk_index];
            for id in chunk.nodes.iter() {
                self.nodes[*id].edges.clear();
            }
        }

        {
            let mut get_cost: &mut dyn FnMut(Point) -> isize = match &mut get_cost {
                CostFnWrapper::Sequential(get_cost) => get_cost,
                #[cfg(feature = "parallel")]
                CostFnWrapper::Parallel(get_cost) => get_cost,
                #[cfg(not(feature = "parallel"))]
                _ => panic!("Created a Parallel CostFnWrapper in a non-parallel environment"),
            };

            // recreate sides in renew
            for (&cp, sides) in renew.iter() {
                let mut candidates = PointSet::default();
                let chunk_index = self.get_chunk_index(cp);
                let chunk = &self.chunks[chunk_index];

                for dir in Dir::all() {
                    if sides[dir.num()] != Renew::No {
                        chunk.calculate_side_nodes(
                            dir,
                            (self.width, self.height),
                            &mut get_cost,
                            self.config,
                            &mut candidates,
                        );
                    }
                }

                // Only include nodes that aren't already part of the map
                candidates.retain(|&pos| self.nodes.id_at(pos).is_none());

                if candidates.is_empty() {
                    continue;
                }

                let all_nodes = &mut self.nodes;
                let nodes = candidates
                    .into_iter()
                    .map(|p| all_nodes.add_node(p, get_cost(p) as usize))
                    .to_vec();

                let chunk = &mut self.chunks[chunk_index];
                if !dirty.contains_key(&cp) {
                    for node in nodes.iter() {
                        changed_nodes.insert(*node);
                    }
                    chunk.add_nodes(
                        &nodes,
                        &mut get_cost,
                        &self.neighborhood,
                        &mut self.nodes,
                        &self.config,
                    );
                } else {
                    for id in nodes {
                        chunk.nodes.insert(id);
                    }
                }
            }
        }

        re_trace!("recreates sides in renew", timer);

        match get_cost {
            CostFnWrapper::Sequential(mut get_cost) => {
                for cp in dirty.keys() {
                    let chunk_index = self.get_chunk_index(*cp);
                    let chunk = &mut self.chunks[chunk_index];
                    let nodes = chunk.nodes.iter().copied().to_vec();

                    for node in nodes.iter() {
                        changed_nodes.insert(*node);
                    }

                    chunk.nodes.clear();
                    chunk.add_nodes(
                        &nodes,
                        &mut get_cost,
                        &self.neighborhood,
                        &mut self.nodes,
                        &self.config,
                    );
                }
                re_trace!("recreate Paths", timer);
            }
            #[cfg(not(feature = "parallel"))]
            _ => panic!("Created a Parallel CostFnWrapper in a non-parallel environment"),
            #[cfg(feature = "parallel")]
            CostFnWrapper::Parallel(get_cost) => {
                use rayon::prelude::*;
                let dirty_indices: hashbrown::HashSet<usize> = dirty
                    .keys()
                    .map(|(x, y)| self.get_chunk_index((*x, *y)))
                    .collect();

                let paths: Vec<_> = {
                    let neighborhood = &self.neighborhood;
                    let all_nodes = &self.nodes;
                    let cache_paths = self.config.cache_paths;

                    self.chunks
                        .par_iter()
                        .enumerate()
                        .filter(|(chunk_index, _)| dirty_indices.contains(chunk_index))
                        .map(|(_, chunk)| {
                            chunk.connect_nodes_parallel(
                                &get_cost,
                                neighborhood,
                                all_nodes,
                                cache_paths,
                            )
                        })
                        .collect()
                };

                re_trace!("get paths", timer);

                for (id, other_id, path) in paths.into_iter().flatten() {
                    self.nodes.add_edge(id, other_id, path);
                }

                for chunk_index in dirty_indices.iter() {
                    for node in self.chunks[*chunk_index].nodes.iter() {
                        changed_nodes.insert(*node);
                    }
                }

                re_trace!("update edges", timer);
            }
        }

        // re-establish cross-chunk connections
        self.connect_nodes(Some(changed_nodes));

        re_trace!("connect nodes", timer);
        trace!("total time: {:?}", std::time::Instant::now() - outer_timer);
    }

    /// Allows for debugging and visualizing the PathCache
    ///
    /// The returned object gives read-only access to the current state of the PathCache, mainly the
    /// Nodes and how they are connected to each other
    ///
    /// ## Examples
    /// Basic usage:
    /// ```
    /// # use hierarchical_pathfinding::prelude::*;
    /// # let mut grid = [
    /// #     [0, 2, 0, 0, 0],
    /// #     [0, 2, 2, 2, 2],
    /// #     [0, 1, 0, 0, 0],
    /// #     [0, 1, 0, 2, 0],
    /// #     [0, 0, 0, 2, 0],
    /// # ];
    /// # let (width, height) = (grid.len(), grid[0].len());
    /// # fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + Sync + Fn((usize, usize)) -> isize {
    /// #     move |(x, y)| [1, 10, -1][grid[y][x]]
    /// # }
    /// let pathfinding: PathCache<_> = // ...
    /// # PathCache::new(
    /// #     (width, height),
    /// #     cost_fn(&grid),
    /// #     ManhattanNeighborhood::new(width, height),
    /// #     PathCacheConfig::with_chunk_size(3),
    /// # );
    ///
    /// // only draw the connections between Nodes once
    /// # use std::collections::HashSet;
    /// let mut visited = HashSet::new();
    ///
    /// for node in pathfinding.inspect_nodes() {
    ///     let pos = node.pos();
    ///     // draw Node at x: pos.0, y: pos.1
    ///
    ///     visited.insert(node.id());
    ///     
    ///     for (neighbor, cost) in node.connected().filter(|(n, _)| !visited.contains(&n.id())) {
    ///         let other_pos = neighbor.pos();
    ///         // draw Line from pos to other_pos, colored by cost
    ///     }
    /// }
    /// ```
    pub fn inspect_nodes(&self) -> CacheInspector<N> {
        CacheInspector::new(self)
    }

    /// Prints all Nodes
    #[allow(dead_code)]
    fn print_nodes(&self) {
        for node in self.inspect_nodes() {
            print!("{} at {:?}: ", node.id(), node.pos());

            for (neighbor, cost) in node.connected() {
                print!("{:?}({}), ", neighbor.pos(), cost);
            }

            println!();
        }
    }

    fn get_chunk_pos(&self, point: Point) -> Point {
        let size = self.config.chunk_size;
        ((point.0 / size) * size, (point.1 / size) * size)
    }

    fn get_chunk(&self, point: Point) -> &Chunk {
        let index = self.get_chunk_index(point);
        &self.chunks[index]
    }

    fn get_chunk_index(&self, point: Point) -> usize {
        let size = self.config.chunk_size;
        let (x, y) = ((point.0 / size), (point.1 / size));
        y * self.num_chunks.0 + x
    }

    fn same_chunk(&self, a: Point, b: Point) -> bool {
        let size = self.config.chunk_size;
        a.0 / size == b.0 / size && a.1 / size == b.1 / size
    }

    fn node_at(&self, pos: Point) -> Option<NodeID> {
        self.nodes.id_at(pos)
    }

    /// Returns the config used to create this PathCache
    pub fn config(&self) -> &PathCacheConfig {
        &self.config
    }

    fn find_nearest_node(
        &self,
        pos: Point,
        get_cost: impl FnMut(Point) -> isize,
        reverse: bool,
    ) -> Option<(NodeID, Option<Path<Point>>)> {
        if let Some(id) = self.node_at(pos) {
            return Some((id, None));
        }
        self.get_chunk(pos)
            .nearest_node(&self.nodes, pos, get_cost, &self.neighborhood, reverse)
            .map(|(id, path)| (id, Some(path)))
    }

    fn grid_a_star(
        &self,
        start: Point,
        goal: Point,
        get_cost: impl FnMut(Point) -> isize,
    ) -> Option<Path<Point>> {
        let heuristic = self.neighborhood.heuristic(start, goal);
        let max_heuristic = self
            .neighborhood
            .heuristic((0, 0), (self.width - 1, self.height - 1));
        let max_size = self.width * self.height;
        let size_hint = heuristic as f32 / max_heuristic as f32 * max_size as f32;

        grid::a_star_search(
            &self.neighborhood,
            |_| true,
            get_cost,
            start,
            goal,
            size_hint as usize,
        )
    }

    fn resolve_paths(
        &self,
        start: Point,
        start_path: Option<Path<Point>>,
        goal_data: &[(Point, NodeID, Option<Path<Point>>)],
        paths: &NodeIDMap<Path<NodeID>>,
        mut get_cost: impl FnMut(Point) -> isize,
    ) -> PointMap<AbstractPath<N>> {
        let mut start_path_map = PointMap::default();
        let mut ret = PointMap::default();

        for (goal, goal_id, goal_path) in goal_data {
            let path = if let Some(path) = paths.get(goal_id) {
                path
            } else {
                continue;
            };

            let mut start_path = start_path.as_ref();
            let mut skip_first = false;
            let mut skip_last = false;
            if start_path.is_some() {
                let after_start = self.nodes[path[1]].pos;
                if self.same_chunk(start, after_start) {
                    start_path = Some(start_path_map.entry(after_start).or_insert_with(|| {
                        // this is contained within a chunk, because start_path is contained and
                        // (start_id, after_start) must be contained:
                        // Direct paths between nodes are only added in chunk::(connect/add)_nodes,
                        // or in the cross-chunk connect_nodes
                        self.get_chunk(start)
                            .find_path(start, after_start, &mut get_cost, &self.neighborhood)
                            .expect("Inconsistency in Pathfinding")
                    }));
                    skip_first = true;
                }
            }

            // path: ... -> before_goal (len-2) -> goal_id (len-1) (-> actual goal (would be next))
            // check if direct connection of before_goal -> actual goal is feasible
            let before_goal = self.nodes[path[path.len() - 2]].pos;
            if goal_path.is_some() && self.same_chunk(*goal, before_goal) {
                skip_last = true;
            }

            let mut final_path = if let Some(path) = start_path {
                AbstractPath::from_known_path(self.neighborhood.clone(), path.clone())
            } else {
                AbstractPath::new(self.neighborhood.clone(), start)
            };

            for (i, (a, b)) in path.iter().zip(path.iter().skip(1)).enumerate() {
                if (skip_first && i == 0) || (skip_last && i == path.len() - 2) {
                    // len() - 2 because skip(1) already removes one
                    continue;
                }
                final_path.add_path_segment(self.nodes[*a].edges[&b].clone());
            }

            if skip_last {
                final_path.add_path(
                    // reasoning for chunk containment: see start_path equivalent
                    self.get_chunk(before_goal)
                        .find_path(before_goal, *goal, &mut get_cost, &self.neighborhood)
                        .expect("Inconsistency in Pathfinding"),
                );
            } else if let Some(path) = goal_path {
                final_path.add_path(path.clone());
            }
            ret.insert(*goal, final_path);
        }
        ret
    }

    fn connect_nodes(&mut self, ids: Option<NodeIDSet>) {
        let ids = ids.unwrap_or_else(|| self.nodes.keys().collect());
        let mut target = vec![];
        for id in ids {
            let (pos, cost) = {
                let node = &self.nodes[id];
                (node.pos, node.walk_cost)
            };
            target.clear();
            self.neighborhood.get_all_neighbors(pos, &mut target);
            for &other_pos in target.iter() {
                if let Some(other_id) = self.node_at(other_pos) {
                    self.nodes.add_edge(
                        id,
                        other_id,
                        PathSegment::new(
                            Path::from_slice(&[pos, other_pos], cost),
                            self.config.cache_paths,
                        ),
                    );
                }
            }
        }
    }
}

/// Allows for debugging and visualizing a PathCache.
///
/// See [`inspect_nodes`](PathCache::inspect_nodes) for details and an example.
///
/// Allows iteration over all Nodes and specific lookup with [`get_node`](CacheInspector::get_node)
#[derive(Debug)]
pub struct CacheInspector<'a, N: Neighborhood> {
    src: &'a PathCache<N>,
    inner: std::vec::IntoIter<NodeID>,
}

impl<'a, N: Neighborhood> CacheInspector<'a, N> {
    /// Creates a new CacheInspector
    ///
    /// Same as calling [`.inspect_nodes()`](PathCache::inspect_nodes) on the cache
    pub fn new(src: &'a PathCache<N>) -> Self {
        CacheInspector {
            src,
            inner: src.nodes.keys().to_vec().into_iter(),
        }
    }

    /// Provides the handle to a specific Node.
    ///
    /// It is recommended to use the `Iterator` implementation instead
    pub fn get_node(&self, id: NodeID) -> NodeInspector<N> {
        NodeInspector::new(self.src, id)
    }
}

impl<'a, N: Neighborhood> Iterator for CacheInspector<'a, N> {
    type Item = NodeInspector<'a, N>;
    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next().map(|id| NodeInspector::new(self.src, id))
    }
}

/// Allows for debugging and visualizing a Node
///
/// See [`inspect_nodes`](PathCache::inspect_nodes) for details and an example.
///
/// Can be obtained by iterating over a [`CacheInspector`] or from [`get_node`](CacheInspector::get_node).
///
/// Gives basic info about the Node and an Iterator over all connected Nodes
#[derive(Debug, Clone, Copy)]
pub struct NodeInspector<'a, N: Neighborhood> {
    src: &'a PathCache<N>,
    node: &'a Node,
}

impl<'a, N: Neighborhood> NodeInspector<'a, N> {
    fn new(src: &'a PathCache<N>, id: NodeID) -> Self {
        NodeInspector {
            src,
            node: &src.nodes[id],
        }
    }

    /// The position of the Node on the Grid
    pub fn pos(&self) -> (usize, usize) {
        self.node.pos
    }

    /// The internal unique ID
    ///
    /// IDs are unique at any point in time, but may be reused if Nodes are deleted.
    pub fn id(&self) -> NodeID {
        self.node.id
    }

    /// Provides an iterator over all connected Nodes with the Cost of the Path to that Node
    pub fn connected(&'a self) -> impl Iterator<Item = (NodeInspector<'a, N>, Cost)> + 'a {
        self.node
            .edges
            .iter()
            .map(move |(id, path)| (NodeInspector::new(self.src, *id), path.cost()))
    }
}

#[cfg(test)]
mod tests {
    use crate::prelude::*;

    #[test]
    fn new() {
        let grid = [
            [0, 2, 0, 0, 0],
            [0, 2, 2, 2, 2],
            [0, 1, 0, 0, 0],
            [0, 1, 0, 2, 0],
            [0, 0, 0, 2, 0],
        ];
        let (width, height) = (grid.len(), grid[0].len());
        fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + Fn((usize, usize)) -> isize {
            move |(x, y)| [1, 10, -1][grid[y][x]]
        }
        let pathfinding = PathCache::new(
            (width, height),
            cost_fn(&grid),
            ManhattanNeighborhood::new(width, height),
            PathCacheConfig::with_chunk_size(3),
        );
        let start = (0, 0);
        let goal = (4, 4);
        let path = pathfinding.find_path(start, goal, cost_fn(&grid));
        let path = path.unwrap();
        let points: Vec<(usize, usize)> = path.collect();
        #[rustfmt::skip]
        assert_eq!(
            points,
            vec![(0, 1), (0, 2), (0, 3), (0, 4), (1, 4), (2, 4), (2, 3), (2, 2), (3, 2), (4, 2), (4, 3), (4, 4)],
        );
    }

    #[test]
    fn get_chunk_index() {
        let grid = [
            [0, 2, 0, 0, 0],
            [0, 2, 2, 2, 2],
            [0, 1, 0, 0, 0],
            [0, 1, 0, 2, 0],
            [0, 0, 0, 2, 0],
        ];
        let (width, height) = (grid.len(), grid[0].len());
        fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + Fn((usize, usize)) -> isize {
            move |(x, y)| [1, 10, -1][grid[y][x]]
        }
        let pathfinding = PathCache::new(
            (width, height),
            cost_fn(&grid),
            ManhattanNeighborhood::new(width, height),
            PathCacheConfig::with_chunk_size(3),
        );

        let point = (0, 0);
        assert_eq!(pathfinding.get_chunk_index(point), 0);

        let point = (4, 0);
        assert_eq!(pathfinding.get_chunk_index(point), 1);

        let point = (3, 2);
        assert_eq!(pathfinding.get_chunk_index(point), 1);

        let point = (4, 4);
        assert_eq!(pathfinding.get_chunk_index(point), 3);

        let point = (0, 4);
        assert_eq!(pathfinding.get_chunk_index(point), 2);
    }

    #[test]
    fn update_path() {
        let mut grid = [
            [0, 2, 0, 0, 0],
            [0, 2, 2, 2, 2],
            [0, 1, 0, 0, 0],
            [0, 1, 0, 2, 0],
            [0, 0, 0, 2, 0],
        ];
        let (width, height) = (grid.len(), grid[0].len());
        fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + Fn((usize, usize)) -> isize {
            move |(x, y)| [1, 10, -1][grid[y][x]]
        }

        let mut pathfinding = PathCache::new(
            (width, height),
            cost_fn(&grid),
            MooreNeighborhood::new(width, height),
            PathCacheConfig::with_chunk_size(3),
        );

        let start = (0, 0);
        let goal = (4, 4);
        let path = pathfinding.find_path(start, goal, cost_fn(&grid));
        let path = path.unwrap();
        let points: Vec<(usize, usize)> = path.collect();
        #[rustfmt::skip]
        assert_eq!(
            points,
            vec![(0, 1), (0, 2), (0, 3), (1, 4), (2, 3), (3, 2), (4, 3), (4, 4)],
        );

        grid[2][1] = 0;
        let changed_tiles = [(1, 1)];

        pathfinding.tiles_changed(&changed_tiles, cost_fn(&grid));

        let start = (0, 0);
        let goal = (4, 4);
        let path = pathfinding.find_path(start, goal, cost_fn(&grid));
        let path = path.unwrap();
        let points: Vec<(usize, usize)> = path.collect();
        #[rustfmt::skip]
        assert_eq!(
            points,
            vec![(0, 1), (1, 2), (2, 2), (3, 2), (4, 3), (4, 4)],
        );

        // Add a wall down the center
        let start = (0, 0);
        let goal = (3, 2);
        let path = pathfinding.find_path(start, goal, cost_fn(&grid));
        assert!(path.is_some());

        let changed_tiles: Vec<_> = (0..grid.len()).map(|y| (2, y)).collect();
        grid.iter_mut().for_each(|row| row[2] = 2);
        pathfinding.tiles_changed(&changed_tiles, cost_fn(&grid));

        let path = pathfinding.find_path(start, goal, cost_fn(&grid));
        assert!(path.is_none());
    }

    #[allow(unused)]
    // #[test]
    #[cfg(feature = "parallel")]
    fn random_test() {
        use rand::prelude::*;
        use rayon::prelude::*;
        let mut rng = StdRng::from_entropy();

        for &size in [8, 128, 1024].iter() {
            let mut grid = vec![vec![0; size]; size];
            grid.par_iter_mut().for_each(|row| {
                let mut rng = StdRng::from_entropy();
                row.fill_with(|| rng.gen_range(-2..7));
            });
            let cost_fn = |(x, y): (usize, usize)| grid[y][x];
            for &chunk_size in [8, 16, 64].iter() {
                let pathfinding = PathCache::new(
                    (size, size),
                    cost_fn,
                    ManhattanNeighborhood::new(size, size),
                    PathCacheConfig::with_chunk_size(chunk_size),
                );
                for _ in 0..100 {
                    let start = (rng.gen_range(0..size), rng.gen_range(0..size));
                    let goal = (rng.gen_range(0..size), rng.gen_range(0..size));
                    let a_star_path = pathfinding.grid_a_star(start, goal, cost_fn);
                    let path = pathfinding.find_path(start, goal, cost_fn);
                    if a_star_path.is_some() != path.is_some() {
                        use std::io::Write;
                        let mut out = std::fs::File::create("cache.txt").unwrap();
                        writeln!(out, "start: {:?},  goal: {:?}", start, goal).unwrap();
                        writeln!(out, "a_star_path: {:?}", a_star_path).unwrap();
                        writeln!(out, "our path: {:?}", path).unwrap();
                        writeln!(out, "{:#?}", pathfinding).unwrap();

                        let mut out = std::fs::File::create("grid.txt").unwrap();
                        for row in grid.iter() {
                            for cell in row.iter() {
                                write!(out, "{}", if *cell < 0 { '#' } else { ' ' }).unwrap();
                            }
                            writeln!(out).unwrap();
                        }
                        panic!("Failed");
                    }
                }
            }
        }
    }
}
