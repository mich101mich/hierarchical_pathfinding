#![deny(
    missing_docs,
    // missing_doc_code_examples,
    missing_debug_implementations,
    missing_copy_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unsafe_code,
    unstable_features,
    unused_import_braces,
    unused_qualifications
)]

//! A crate to quickly approximate Paths on a Grid.
//!
//! ## Introduction
//! Finding Paths on a Grid using regular A* and Dijkstra is usually a rather expensive Operation,
//! since every Tile on the Grid is considered a Node, leading to those algorithms having to
//! store and visit hundreds of Nodes for a fairly short Path. Combined with the fact that Grids
//! usually have several Paths with the exact same Cost makes a naive implementation of regular
//! Pathfinding Algorithms rather inefficient.
//!
//! The idea behind Hierarchical Pathfinding is to improve that Process by collecting sections of
//! the Grid into Chunks and pre-calculating and caching the cost (and possibly the Paths) of
//! walking between the different entrances of the Chunk. The final process of finding a Path
//! between two points can then use that knowledge by treating the Grid as a Graph with Entrances
//! as Nodes and the cached costs (and Paths) as Edges, resulting in a much smaller Graph that
//! can be easily searched.
//!
//! Since the Graph is usually not an exact representation of the Grid, **the resulting Paths will
//! be slightly worse than the actual best Path** (unless [`config.perfect_paths`](PathCacheConfig::perfect_paths)
//! is set to `true`). This is usually not a problem, since the purpose of Hierarchical Pathfinding
//! is to quickly find the next direction to go in or a Heuristic for the total Cost of a Path or
//! to determine weather or not a Goal is reachable. All of these are not affected by the exact
//! Cost or Path. The only time where the actual best Path would noticeably differ from this
//! implementation's result is in the case of short Paths of roughly `Length < 2 * chunk_size`.
//! That is why this implementation calls the regular A* search after HPA* confirmed the Path to
//! be short. (This behavior can be turned of using the Config).
//!
//! This crate provides an implementation of a Hierarchical Pathfinding Algorithm for any generic Grid.
//! Paths can be searched using either A* for a Path to a single Tile, or Dijkstra for searching
//! multiple Targets. It handles solid walls in the Grid and actually finding a Path that ends near
//! a wall.
//!
//! ## Examples
//! Creating the Cache:
//! ```
//! use hierarchical_pathfinding::{prelude::*, Point};
//!
//! // create and initialize Grid
//! // 0 = empty, 1 = swamp, 2 = wall
//! let mut grid = [
//!     [0, 2, 0, 0, 0],
//!     [0, 2, 2, 2, 0],
//!     [0, 1, 0, 0, 0],
//!     [0, 1, 0, 2, 0],
//!     [0, 0, 0, 2, 0],
//! ];
//! let (width, height) = (grid.len(), grid[0].len());
//!
//! let cost_map = [
//!     1,  // empty
//!     10, // swamp
//!     -1, // wall = solid
//! ];
//!
//! let mut pathfinding = PathCache::new(
//!     (width, height), // the size of the Grid
//!     |(x, y)| cost_map[grid[y][x]], // get the cost for walking over a Tile
//!     ManhattanNeighborhood::new(width, height), // the Neighborhood
//!     PathCacheConfig { chunk_size: 3, ..Default::default() }, // config
//! );
//! ```
//! Note that the PathCache never actually asks for the Grid itself. This allows the user to
//! store the Grid in any format they want (Array, Vec, HashMap, kd-tree, ...),
//! as long as they are somehow able to access a specific (x, y) on the Grid when asked.
//!
//! The provided function takes a Position on the Grid as parameter and returns, how "expensive"
//! it is to walk across the Tile at that Position. This Cost is what will be used for calculating
//! the Cost of a Path to find the most optimal one. A negative Cost implies that the Tile cannot
//! be walked across.
//!
//! Unfortunately, it is necessary to provide this function to every method of PathCache, since
//! storing it would make the Grid immutable. See also [Updating the PathCache](#updating-the-pathcache).
//!
//! **Note**: If copying the Cost function everywhere would create too much Code / less readable
//! code, [currying](https://en.wikipedia.org/wiki/Currying) may be used:
//! ```
//! # use hierarchical_pathfinding::{prelude::*, Point};
//! #
//! # // create and initialize Grid
//! # // 0 = empty, 1 = swamp, 2 = wall
//! # let mut grid = [
//! #     [0, 2, 0, 0, 0],
//! #     [0, 2, 2, 2, 0],
//! #     [0, 1, 0, 0, 0],
//! #     [0, 1, 0, 2, 0],
//! #     [0, 0, 0, 2, 0],
//! # ];
//! # let (width, height) = (grid.len(), grid[0].len());
//! #
//! const COST_MAP: [isize; 3] = [1, 10, -1];
//!
//! // only references the Grid when called
//! fn cost_fn<'a>(grid: &'a [[usize; 5]; 5]) -> impl 'a + FnMut(Point) -> isize {
//!     move |(x, y)| COST_MAP[grid[y][x]]
//! }
//!
//! let mut pathfinding = PathCache::new(
//!     (width, height), // the size of the Grid
//!
//!     // simply call the creator function to take a reference of the Grid
//!     cost_fn(&grid),
//!
//!     // ...
//! #     ManhattanNeighborhood::new(width, height), // the Neighborhood
//! #     PathCacheConfig { chunk_size: 3, ..Default::default() }, // config
//! );
//!
//! # let start = (0, 0);
//! # let goal = (4, 4);
//! // ...
//!
//! let path = pathfinding.find_path(
//!     start, goal,
//!
//!     // function can be reused at any time
//!     cost_fn(&grid),
//!
//! );
//! ```
//!
//! ### Pathfinding
//! Finding the Path to a single Goal:
//! ```
//! # use hierarchical_pathfinding::{prelude::*, Point};
//! #
//! # // create and initialize Grid
//! # // 0 = empty, 1 = swamp, 2 = wall
//! # let mut grid = [
//! #     [0, 2, 0, 0, 0],
//! #     [0, 2, 2, 2, 2],
//! #     [0, 1, 0, 0, 0],
//! #     [0, 1, 0, 2, 0],
//! #     [0, 0, 0, 2, 0],
//! # ];
//! # let (width, height) = (grid.len(), grid[0].len());
//! #
//! # const COST_MAP: [isize; 3] = [1, 10, -1];
//! #
//! # fn cost_fn<'a>(grid: &'a [[usize; 5]; 5]) -> impl 'a + FnMut(Point) -> isize {
//! #     move |(x, y)| COST_MAP[grid[y][x]]
//! # }
//! #
//! # let mut pathfinding = PathCache::new(
//! #     (width, height),
//! #     cost_fn(&grid),
//! #     ManhattanNeighborhood::new(width, height),
//! #     PathCacheConfig { chunk_size: 3, ..Default::default() },
//! # );
//! #
//! let start = (0, 0);
//! let goal = (4, 4);
//!
//! // find_path returns Some(Path) on success
//! let path = pathfinding.find_path(
//!     start,
//!     goal,
//!     cost_fn(&grid),
//! );
//!
//! assert!(path.is_some());
//! let mut path = path.unwrap();
//!
//! assert_eq!(path.cost(), 12);
//! ```
//! For more information, see [`find_path`](PathCache::find_path).
//!
//! Finding multiple Goals:
//! ```
//! # use hierarchical_pathfinding::{prelude::*, Point};
//! #
//! # // create and initialize Grid
//! # // 0 = empty, 1 = swamp, 2 = wall
//! # let mut grid = [
//! #     [0, 2, 0, 0, 0],
//! #     [0, 2, 2, 2, 2],
//! #     [0, 1, 0, 0, 0],
//! #     [0, 1, 0, 2, 0],
//! #     [0, 0, 0, 2, 0],
//! # ];
//! # let (width, height) = (grid.len(), grid[0].len());
//! #
//! # const COST_MAP: [isize; 3] = [1, 10, -1];
//! #
//! # fn cost_fn<'a>(grid: &'a [[usize; 5]; 5]) -> impl 'a + FnMut(Point) -> isize {
//! #     move |(x, y)| COST_MAP[grid[y][x]]
//! # }
//! #
//! # let mut pathfinding = PathCache::new(
//! #     (width, height),
//! #     cost_fn(&grid),
//! #     ManhattanNeighborhood::new(width, height),
//! #     PathCacheConfig { chunk_size: 3, ..Default::default() },
//! # );
//! #
//! let start = (0, 0);
//! let goals = [(4, 4), (2, 0)];
//!
//! // find_paths returns a HashMap<goal, Path> for all successes
//! let paths = pathfinding.find_paths(
//!     start,
//!     &goals,
//!     cost_fn(&grid),
//! );
//!
//! // (4, 4) is reachable
//! assert!(paths.contains_key(&goals[0]));
//!
//! // (2, 0) is not reachable
//! assert!(!paths.contains_key(&goals[1]));
//! ```
//! For more information, see [`find_paths`](PathCache::find_paths).
//!
//! ### Using a Path
//! The easiest information obtainable from a Path is its existence. Despite being an
//! approximation of an optimal Path, HPA* is 100% correct when it comes to the existence
//! of a Path. Meaning that if HPA* cannot find a Path, no one can, and if HPA* returns a Path,
//! it is valid, given correct Neighborhood and Cost functions.
//!
//! The next step is to obtain information about the Path itself. The part that is always
//! available is the total Cost of the Path. Once again, it is just an approximation. However,
//! it gives a pretty good estimate of the actual Cost, with only minimal deviations.
//!
//! As for following the Path, HPA* was designed to allow Units to immediately start moving
//! and minimize lost time when the surroundings change in a way that alters the Path.
//! That is why it does not calculate the full Path immediately. It does, however, generate
//! the first steps of the Path without too much overhead. That is why it is advised to
//! mostly use the `next()` method of the returned Path for a few steps.
//!
//! ```
//! # use hierarchical_pathfinding::{prelude::*, Point};
//! #
//! # // create and initialize Grid
//! # // 0 = empty, 1 = swamp, 2 = wall
//! # let mut grid = [
//! #     [0, 2, 0, 0, 0],
//! #     [0, 2, 2, 2, 2],
//! #     [0, 1, 0, 0, 0],
//! #     [0, 1, 0, 2, 0],
//! #     [0, 0, 0, 2, 0],
//! # ];
//! # let (width, height) = (grid.len(), grid[0].len());
//! #
//! # const COST_MAP: [isize; 3] = [1, 10, -1];
//! #
//! # fn cost_fn<'a>(grid: &'a [[usize; 5]; 5]) -> impl 'a + FnMut(Point) -> isize {
//! #     move |(x, y)| COST_MAP[grid[y][x]]
//! # }
//! #
//! # let mut pathfinding = PathCache::new(
//! #     (width, height),
//! #     cost_fn(&grid),
//! #     ManhattanNeighborhood::new(width, height),
//! #     PathCacheConfig { chunk_size: 3, ..Default::default() },
//! # );
//! # struct Player{ pos: (usize, usize) }
//! # impl Player {
//! #     pub fn move_to(&mut self, pos: (usize, usize)) {
//! #         self.pos = pos;
//! #     }
//! # }
//! #
//! let mut player = Player {
//!     pos: (0, 0),
//!     //...
//! };
//! let goal = (4, 4);
//!
//! let mut path = pathfinding.find_path(
//!     player.pos,
//!     goal,
//!     cost_fn(&grid),
//! ).unwrap();
//!
//! player.move_to(path.next().unwrap());
//! assert_eq!(player.pos, (0, 1));
//!
//! // wait for next turn or whatever
//!
//! player.move_to(path.next().unwrap());
//! assert_eq!(player.pos, (0, 2));
//! ```
//!
//! ### Updating the PathCache
//! The PathCache does not contain a copy or reference of the Grid for mutability and Ownership reasons.
//! This means however, that the user is responsible for storing and maintaining both the Grid and the PathCache.
//! It is also necessary to update the PathCache when the Grid has changed to keep it consistent:
//! ```
//! # use hierarchical_pathfinding::{prelude::*, Point};
//! #
//! # // create and initialize Grid
//! # // 0 = empty, 1 = swamp, 2 = wall
//! # let mut grid = [
//! #     [0, 2, 0, 0, 0],
//! #     [0, 2, 2, 2, 2],
//! #     [0, 1, 0, 0, 0],
//! #     [0, 1, 0, 2, 0],
//! #     [0, 0, 0, 2, 0],
//! # ];
//! # let (width, height) = (grid.len(), grid[0].len());
//! #
//! # const COST_MAP: [isize; 3] = [1, 10, -1];
//! #
//! # fn cost_fn<'a>(grid: &'a [[usize; 5]; 5]) -> impl 'a + FnMut(Point) -> isize {
//! #     move |(x, y)| COST_MAP[grid[y][x]]
//! # }
//! #
//! # let mut pathfinding = PathCache::new(
//! #     (width, height),
//! #     cost_fn(&grid),
//! #     ManhattanNeighborhood::new(width, height),
//! #     PathCacheConfig { chunk_size: 3, ..Default::default() },
//! # );
//! #
//! let (start, goal) = ((0, 0), (2, 0));
//!
//! let path = pathfinding.find_path(start, goal, cost_fn(&grid));
//! assert!(path.is_none());
//!
//! grid[0][1] = 0;
//! grid[4][4] = 2;
//!
//! assert_eq!(grid, [
//!     [0, 0, 0, 0, 0],
//!     [0, 2, 2, 2, 2],
//!     [0, 1, 0, 0, 0],
//!     [0, 1, 0, 2, 0],
//!     [0, 0, 0, 2, 2],
//! ]);
//!
//! pathfinding.tiles_changed(
//!     &[(1, 0), (4, 4)],
//!     cost_fn(&grid),
//! );
//!
//! let path = pathfinding.find_path(start, goal, cost_fn(&grid));
//! assert!(path.is_some());
//! ```
//!
//! ### Configuration
//! The last parameter for PathCache::new is a [`PathCacheConfig`] object with different options to have more control over the generated PathCache.
//! These options are mostly used to adjust the balance between Performance and Memory Usage, with the default values aiming more at Performance.
//! The PathCacheConfig struct also provides defaults for low Memory Usage [`PathCacheConfig::LOW_MEM`]
//! or best Performance [`PathCacheConfig::HIGH_PERFORMANCE`]
//! ```
//! # use hierarchical_pathfinding::{prelude::*, Point};
//! #
//! # // create and initialize Grid
//! # // 0 = empty, 1 = swamp, 2 = wall
//! # let mut grid = [
//! #     [0, 2, 0, 0, 0],
//! #     [0, 2, 2, 2, 2],
//! #     [0, 1, 0, 0, 0],
//! #     [0, 1, 0, 2, 0],
//! #     [0, 0, 0, 2, 0],
//! # ];
//! # let (width, height) = (grid.len(), grid[0].len());
//! #
//! # const COST_MAP: [isize; 3] = [1, 10, -1];
//! #
//! # fn cost_fn<'a>(grid: &'a [[usize; 5]; 5]) -> impl 'a + FnMut(Point) -> isize {
//! #     move |(x, y)| COST_MAP[grid[y][x]]
//! # }
//!
//! let mut pathfinding = PathCache::new(
//!     (width, height), // the size of the Grid
//!     cost_fn(&grid), // get the cost for walking over a Tile
//!     ManhattanNeighborhood::new(width, height), // the Neighborhood
//!     PathCacheConfig {
//!         chunk_size: 3,
//!         ..PathCacheConfig::LOW_MEM
//!     }
//! );
//!
//! assert_eq!(pathfinding.config().chunk_size, 3);
//! ```

/// The Type used to reference a Node in the abstracted Graph
pub type NodeID = u32;

/// A shorthand for Points on the grid
pub type Point = (usize, usize);

type PointMap<V> = fnv::FnvHashMap<Point, V>;
type PointSet = fnv::FnvHashSet<Point>;

mod path_cache;
pub use self::path_cache::{AbstractPath, PathCache, PathCacheConfig};

pub mod neighbors;

pub mod generics;

pub mod node_id;

/// The prelude for this crate.
///
/// Note: Even though most examples use the internal type-definition [`Point`]
/// (aka `(usize, usize)`), it is not included in the prelude since most users probably have
/// another implementation with the same name in scope.
pub mod prelude {
    pub use crate::{
        neighbors::{ManhattanNeighborhood, MooreNeighborhood, Neighborhood},
        AbstractPath, PathCache, PathCacheConfig,
    };
}
