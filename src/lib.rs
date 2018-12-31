#![warn(
	missing_docs,
	missing_doc_code_examples,
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
//! be slightly worse than the actual best Path**. This is usually not a problem, since the
//! purpose of Hierarchical Pathfinding is to quickly find the next direction to go in or a
//! Heuristic for the total Cost of a Path or to determine weather or not a Goal is reachable.
//! All of these are not affected by the exact Cost or Path. The only time where the actual best
//! Path would noticeably differ from this crates result is in the case of small Paths of
//! ```Length < 2 * chunk_size```. That is why this implementation calls the regular A* search
//! after HPA* confirmed the Path to be short. (This behavior can be turned of using the Config).
//!
//! This crate provides an implementation of a Hierarchical Pathfinding Algorithm for any generic Grid.
//! Paths can be searched using either A* for a Path to a single Tile, or Dijkstra for searching multiple Targets.
//!
//! ## Examples
//! Creating the Cache:
//! ```
//! use hierarchical_pathfinding::{PathCache, neighbors::ManhattanNeighborhood};
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
//!     5,  // swamp
//!     -1, // wall = solid
//! ];
//!
//! let mut pathfinding = PathCache::new(
//!     (width, height), // the size of the Grid
//!     |(x, y)| cost_map[grid[x][y]], // get the cost for walking over a tile
//!     ManhattanNeighborhood::new(width, height), // the Neighborhood
//!     Default::default(), // other options for creating the cache
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
//! ### Pathfinding
//! Finding the Path to a single Goal:
//! ```
//! # use hierarchical_pathfinding::{PathCache, neighbors::ManhattanNeighborhood};
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
//! # let cost_map = [
//! #     1,  // empty
//! #     5,  // swamp
//! #     -1, // wall = solid
//! # ];
//! #
//! # let mut pathfinding = PathCache::new(
//! #     (width, height), // the size of the Grid
//! #     |(x, y)| cost_map[grid[x][y]], // get the cost for walking over a tile
//! #     ManhattanNeighborhood::new(width, height), // the Neighborhood
//! #     Default::default(), // other options for creating the cache
//! # );
//! #
//! let start = (0, 0);
//! let goal = (4, 4);
//!
//! // find_path returns Some(Path) on success
//! let path = pathfinding.find_path(
//!     start,
//!     goal,
//!     |(x, y)| cost_map[grid[x][y]], // cost function
//! );
//!
//! assert!(path.is_some());
//! ```
//! An explanation of the Parameters can be found in [find_path](struct.pathcache.html#method.find_path)
//! as well as the [neighbors crate](neighbors/index.html).
//!
//! Finding multiple Goals:
//! ```
//! # use hierarchical_pathfinding::{PathCache, neighbors::ManhattanNeighborhood};
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
//! # let cost_map = [
//! #     1,  // empty
//! #     5,  // swamp
//! #     -1, // wall = solid
//! # ];
//! #
//! # let mut pathfinding = PathCache::new(
//! #     (width, height), // the size of the Grid
//! #     |(x, y)| cost_map[grid[x][y]], // get the cost for walking over a tile
//! #     ManhattanNeighborhood::new(width, height), // the Neighborhood
//! #     Default::default(), // other options for creating the cache
//! # );
//! #
//! let start = (0, 0);
//! let goals = [(4, 4), (0, 2)];
//!
//! // find_paths returns a HashMap<goal, Path> for all successes
//! let paths = pathfinding.find_paths(
//!     start,
//!     goals,
//!     |(x, y)| cost_map[grid[x][y]], // cost function
//! );
//!
//! assert!(paths.contains_key(&goals[0]));
//!
//! assert!(!paths.contains_key(&goals[1]));
//! ```
//!
//! ### Updating the PathCache
//! The PathCache does not contain a copy or reference of the Grid for mutability and Ownership reasons.
//! This means however, that the user is responsible for storing and maintaining both the Grid and the PathCache.
//! It is also necessary to update the PathCache when the Grid has changed to keep it consistent:
//! ```
//! # use hierarchical_pathfinding::{PathCache, neighbors::ManhattanNeighborhood};
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
//! # let cost_map = [
//! #     1,  // empty
//! #     5,  // swamp
//! #     -1, // wall = solid
//! # ];
//! #
//! # let mut pathfinding = PathCache::new(
//! #     (width, height), // the size of the Grid
//! #     |(x, y)| cost_map[grid[x][y]], // get the cost for walking over a tile
//! #     ManhattanNeighborhood::new(width, height), // the Neighborhood
//! #     Default::default(), // other options for creating the cache
//! # );
//! #
//! grid[3][1] = 0;
//! grid[4][4] = 2;
//!
//! pathfinding.tiles_changed(
//!     &[(3, 1), (4, 4)],
//!     |(x, y)| cost_map[grid[x][y]], // cost function
//! );
//! ```
//!
//! ### Configuration
//! The last parameter for PathCache::new is a [PathCacheConfig](struct.PathCacheConfig.html) object with different options to have more control over the generated PathCache.
//! These options are mostly used to adjust the balance between Performance and Memory Usage, with the default values aiming more at Performance.
//! The PathCacheConfig struct also provides defaults for low Memory Usage [PathCacheConfig::LOW_MEM](struct.PathCacheConfig.html#associatedconstant.LOW_MEM)
//! or best Performance [PathCacheConfig::HIGH_PERFORMANCE](struct.PathCacheConfig.html#associatedconstant.HIGH_PERFORMANCE)
//! ```
//! use hierarchical_pathfinding::{PathCache, PathCacheConfig, neighbors::ManhattanNeighborhood};
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
//! # let cost_map = [
//! #     1,  // empty
//! #     5,  // swamp
//! #     -1, // wall = solid
//! # ];
//!
//! let mut pathfinding = PathCache::new(
//!     (width, height), // the size of the Grid
//!     |(x, y)| cost_map[grid[x][y]], // get the cost for walking over a tile
//!     ManhattanNeighborhood::new(width, height), // the Neighborhood
//!     PathCacheConfig {
//!         chunk_size: 5,
//!         ..PathCacheConfig::LOW_MEM
//!     }
//! );
//!
//! assert_eq!(pathfinding.get_config().chunk_size, 5);
//! ```

/// The Type used to reference a Node in the abstracted Graph
pub type NodeID = u32;

/// A shorthand for Points on the grid
pub type Point = (usize, usize);

mod path_cache;
pub use self::path_cache::{AbstractPath, PathCache, PathCacheConfig};

pub mod neighbors;

pub mod generics;
