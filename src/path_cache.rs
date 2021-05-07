use crate::{
    graph::{self, Node, NodeID, NodeIDMap, NodeMap},
    neighbors::Neighborhood,
    path::{AbstractPath, Cost, Path, PathSegment},
    *,
};

mod cache_config;
pub use cache_config::PathCacheConfig;

mod chunk;
use chunk::Chunk;

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
    /// fn cost_fn(grid: &Grid) -> impl '_ + FnMut((usize, usize)) -> isize {
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
    /// # fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + FnMut((usize, usize)) -> isize {
    /// #     move |(x, y)| [1, 10, -1][grid[y][x]]
    /// # }
    /// let pathfinding: PathCache<_> = // ...
    /// # PathCache::new(
    /// #     (width, height),
    /// #     cost_fn(&grid),
    /// #     ManhattanNeighborhood::new(width, height),
    /// #     PathCacheConfig { chunk_size: 3, ..Default::default() },
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
    /// # fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + FnMut((usize, usize)) -> isize {
    /// #     move |(x, y)| [1, 10, -1][grid[y][x]]
    /// # }
    /// # let pathfinding = PathCache::new(
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
    /// # use hierarchical_pathfinding::prelude::*;
    /// # let mut grid = [
    /// #     [0, 2, 0, 0, 0],
    /// #     [0, 2, 2, 2, 2],
    /// #     [0, 1, 0, 0, 0],
    /// #     [0, 1, 0, 2, 0],
    /// #     [0, 0, 0, 2, 0],
    /// # ];
    /// # let (width, height) = (grid.len(), grid[0].len());
    /// # fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + FnMut((usize, usize)) -> isize {
    /// #     move |(x, y)| [1, 10, -1][grid[y][x]]
    /// # }
    /// # let pathfinding = PathCache::new(
    /// #     (width, height),
    /// #     cost_fn(&grid),
    /// #     ManhattanNeighborhood::new(width, height),
    /// #     PathCacheConfig { chunk_size: 3, ..Default::default() },
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

        let path = graph::a_star_search(&self.nodes, start_id, goal_id, &neighborhood)?;

        if path.len() == 2 || (self.config.a_star_fallback && path.len() <= 4) {
            // 2: start_id == goal_id
            // <= 4: start_id X X goal_id
            return self
                .grid_a_star(start, goal, get_cost)
                .map(|path| AbstractPath::from_known_path(neighborhood, path));
        }

        let mut paths = NodeIDMap::default();
        paths.insert(goal_id, path);

        self.resolve_paths(
            start,
            start_path,
            &[(goal, goal_id, goal_path)],
            &paths,
            get_cost,
        )
        .into_iter()
        .next()
        .map(|(_, path)| path)
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
    /// # fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + FnMut((usize, usize)) -> isize {
    /// #     move |(x, y)| [1, 10, -1][grid[y][x]]
    /// # }
    /// let pathfinding: PathCache<_> = // ...
    /// # PathCache::new(
    /// #     (width, height),
    /// #     cost_fn(&grid),
    /// #     ManhattanNeighborhood::new(width, height),
    /// #     PathCacheConfig { chunk_size: 3, ..Default::default() },
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
    /// # fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + FnMut((usize, usize)) -> isize {
    /// #     move |(x, y)| [1, 10, -1][grid[y][x]]
    /// # }
    /// # let pathfinding = PathCache::new(
    /// #     (width, height),
    /// #     cost_fn(&grid),
    /// #     ManhattanNeighborhood::new(width, height),
    /// #     PathCacheConfig { chunk_size: 3, ..Default::default() },
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
    /// # fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + FnMut((usize, usize)) -> isize {
    /// #     move |(x, y)| [1, 10, -1][grid[y][x]]
    /// # }
    /// let pathfinding: PathCache<_> = // ...
    /// # PathCache::new(
    /// #     (width, height),
    /// #     cost_fn(&grid),
    /// #     ManhattanNeighborhood::new(width, height),
    /// #     PathCacheConfig { chunk_size: 3, ..Default::default() },
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
    /// # fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + FnMut((usize, usize)) -> isize {
    /// #     move |(x, y)| [1, 10, -1][grid[y][x]]
    /// # }
    /// # let pathfinding = PathCache::new(
    /// #     (width, height),
    /// #     cost_fn(&grid),
    /// #     ManhattanNeighborhood::new(width, height),
    /// #     PathCacheConfig { chunk_size: 3, ..Default::default() },
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
        }

        let paths = graph::dijkstra_search(&self.nodes, start_id, &goal_ids, only_closest_goal);

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
    /// # fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + FnMut((usize, usize)) -> isize {
    /// #     move |(x, y)| [1, 10, -1][grid[y][x]]
    /// # }
    /// let mut pathfinding: PathCache<_> = // ...
    /// # PathCache::new(
    /// #     (width, height),
    /// #     cost_fn(&grid),
    /// #     ManhattanNeighborhood::new(width, height),
    /// #     PathCacheConfig { chunk_size: 3, ..Default::default() },
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
    pub fn tiles_changed(&mut self, tiles: &[Point], mut get_cost: impl FnMut(Point) -> isize) {
        let size = self.config.chunk_size;

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
                let chunk = self.get_chunk(cp);
                chunk
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

            let all_nodes = &self.nodes;
            candidates.retain(|&p| all_nodes.values().find(|node| node.pos == p).is_none());

            if candidates.is_empty() {
                continue;
            }

            let all_nodes = &mut self.nodes;
            let nodes = candidates
                .into_iter()
                .map(|p| all_nodes.add_node(p, get_cost(p) as usize))
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
    /// # use hierarchical_pathfinding::prelude::*;
    /// # let mut grid = [
    /// #     [0, 2, 0, 0, 0],
    /// #     [0, 2, 2, 2, 2],
    /// #     [0, 1, 0, 0, 0],
    /// #     [0, 1, 0, 2, 0],
    /// #     [0, 0, 0, 2, 0],
    /// # ];
    /// # let (width, height) = (grid.len(), grid[0].len());
    /// # fn cost_fn(grid: &[[usize; 5]; 5]) -> impl '_ + FnMut((usize, usize)) -> isize {
    /// #     move |(x, y)| [1, 10, -1][grid[y][x]]
    /// # }
    /// let pathfinding: PathCache<_> = // ...
    /// # PathCache::new(
    /// #     (width, height),
    /// #     cost_fn(&grid),
    /// #     ManhattanNeighborhood::new(width, height),
    /// #     PathCacheConfig { chunk_size: 3, ..Default::default() },
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
        get_chunk!(self, point)
    }

    fn same_chunk(&self, a: Point, b: Point) -> bool {
        let size = self.config.chunk_size;
        a.0 / size == b.0 / size && a.1 / size == b.1 / size
    }

    fn get_node_id(&self, pos: Point) -> Option<NodeID> {
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
        if let Some(id) = self.get_node_id(pos) {
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
        grid::a_star_search(
            |p| self.neighborhood.get_all_neighbors(p),
            get_cost,
            start,
            goal,
            |p| self.neighborhood.heuristic(p, goal),
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
                        self.grid_a_star(start, after_start, &mut get_cost)
                            .expect("Inconsistency in Pathfinding")
                    }));
                    skip_first = true;
                }
            }
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
                if (skip_first && i == 0) || (skip_last && i == path.len() - 1) {
                    continue;
                }
                final_path.add_path_segment(self.nodes[*a].edges[&b].clone());
            }

            if skip_last {
                final_path.add_path(
                    self.grid_a_star(before_goal, *goal, &mut get_cost)
                        .expect("Inconsistency in Pathfinding"),
                );
            } else if let Some(path) = goal_path {
                final_path.add_path(path.clone());
            }
            ret.insert(*goal, final_path);
        }
        ret
    }

    fn connect_nodes(&mut self, mut get_cost: impl FnMut(Point) -> isize) {
        let ids = self.nodes.keys().to_vec();
        for id in ids {
            let pos = self.nodes[id].pos;
            let possible: PointSet = self.neighborhood.get_all_neighbors(pos).collect();
            let neighbors = self
                .nodes
                .values()
                .filter(|node| possible.contains(&node.pos)) // any Node next to me
                .filter(|node| !node.edges.contains_key(&id)) // that is not already connected
                .map(|node| (node.id, node.pos))
                .to_vec();

            for (other_id, other_pos) in neighbors {
                let path = Path::from_slice(&[pos, other_pos], get_cost(pos) as usize);
                let other_path = Path::from_slice(&[other_pos, pos], get_cost(other_pos) as usize);

                self.nodes[id]
                    .edges
                    .insert(other_id, PathSegment::new(path, self.config.cache_paths));

                self.nodes[other_id]
                    .edges
                    .insert(id, PathSegment::new(other_path, self.config.cache_paths));
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
