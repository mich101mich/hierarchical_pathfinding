/// Options for configuring the [`PathCache`](crate::PathCache)
///
/// Default options:
/// ```
/// # use hierarchical_pathfinding::PathCacheConfig;
/// assert_eq!(
/// 	PathCacheConfig {
/// 		chunk_size: 8,
/// 		cache_paths: true,
/// 		keep_insertions: true,
/// 		a_star_fallback: true,
/// 		perfect_paths: false,
/// 	},
/// 	Default::default()
/// );
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PathCacheConfig {
	/// The size of the individual Chunks (defaults to `8`)
	pub chunk_size: usize,
	/// `true` (default): store the Paths inside each Chunk.
	///
	/// `false`: only store the Cost of the Path.
	///
	/// If set to false, calculating the full Path between two Points takes significantly more time.
	/// See [`AbstractPath`](crate::AbstractPath) for more details.
	pub cache_paths: bool,
	/// `true` (default): any Points inserted when calling find_path or find_paths are left in the Graph.
	///
	/// `false`: the Points are deleted at the end of those functions.
	///
	/// Keeping Points slightly increases future search times and memory usage, but makes searches
	/// to or from those same Points a lot faster.
	pub keep_insertions: bool,
	/// `true` (default): When a Path is short (`Length < 2 * chunk_size`), a regular A* search
	/// is performed on the Grid **after** HPA* calculated a Path to confirmed the existence and
	/// length.
	///
	/// `false`: The Paths are left as they are.
	///
	/// Setting this option to false will improve performance a bit, but the Paths will take
	/// seemingly random detours that makes them longer than they should be.
	pub a_star_fallback: bool,
	/// `true`: Nodes are placed on every open entrance of a Chunk. This means that the resulting
	/// Paths are always the most optimal ones, but both Memory Usage and Performance are worse.
	///
	/// `false` (default): Nodes are placed on only some chunk entrances.
	///
	/// The exact effect depends greatly on the Grid and how the Chunks and their entrances align.
	pub perfect_paths: bool,
}

impl PathCacheConfig {
	/// an example PathCacheConfig with options set to reduce Memory Usage
	///
	/// Values:
	/// ```
	/// # use hierarchical_pathfinding::PathCacheConfig;
	/// assert_eq!(
	/// 	PathCacheConfig {
	/// 		chunk_size: 16,
	/// 		cache_paths: false,
	/// 		keep_insertions: false,
	/// 		a_star_fallback: true,
	/// 		perfect_paths: false,
	/// 	},
	/// 	PathCacheConfig::LOW_MEM
	/// );
	/// ```
	pub const LOW_MEM: PathCacheConfig = PathCacheConfig {
		chunk_size: 16,
		cache_paths: false,
		keep_insertions: false,
		a_star_fallback: true,
		perfect_paths: false,
	};
	/// an example PathCacheConfig with options set to improve Performance
	///
	/// Values:
	/// ```
	/// # use hierarchical_pathfinding::PathCacheConfig;
	/// assert_eq!(
	/// 	PathCacheConfig {
	/// 		chunk_size: 8,
	/// 		cache_paths: true,
	/// 		keep_insertions: true,
	/// 		a_star_fallback: false,
	/// 		perfect_paths: false,
	/// 	},
	/// 	PathCacheConfig::HIGH_PERFORMANCE
	/// );
	/// ```
	pub const HIGH_PERFORMANCE: PathCacheConfig = PathCacheConfig {
		chunk_size: 8,
		cache_paths: true,
		keep_insertions: true,
		a_star_fallback: false,
		perfect_paths: false,
	};
}

impl Default for PathCacheConfig {
	fn default() -> PathCacheConfig {
		PathCacheConfig {
			chunk_size: 8,
			cache_paths: true,
			keep_insertions: true,
			a_star_fallback: true,
			perfect_paths: false,
		}
	}
}
