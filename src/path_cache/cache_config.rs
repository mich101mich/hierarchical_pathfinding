/// Options for configuring the [`PathCache`](crate::PathCache)
///
/// Default options:
/// ```
/// # use hierarchical_pathfinding::PathCacheConfig;
/// assert_eq!(
///     PathCacheConfig {
///         chunk_size: 8,
///         cache_paths: true,
///         a_star_fallback: true,
///         perfect_paths: false,
///     },
///     Default::default()
/// );
/// ```
///
/// ### Performance
/// For testing, a set of different Grids were used:
/// - "empty" is simply an Grid where all costs are `1`.
/// - "snake" has a single long winding corridor
/// - "random" is completely random costs and walls
///
/// Each Grid was tested as 16x16, 128x128 and 1024x1024. The 16x16 Grid, however, produced only
/// durations less than 100 μs, which are too inaccurate to be measured.
///
/// The full List can be seen on [GitHub](https://github.com/mich101mich/hierarchical_pathfinding/blob/benchmark/benchmark/)
/// in the `output_*.txt` files.
///
/// Grid   | 128x128 | 1024x1024
/// -------|:-------:|:---------:
/// empty  | ![](https://github.com/mich101mich/hierarchical_pathfinding/blob/benchmark/img/plot/empty128.png?raw=true) | ![](https://github.com/mich101mich/hierarchical_pathfinding/blob/benchmark/img/plot/empty1024.png?raw=true)
/// snake  | ![](https://github.com/mich101mich/hierarchical_pathfinding/blob/benchmark/img/plot/snake128.png?raw=true) | ![](https://github.com/mich101mich/hierarchical_pathfinding/blob/benchmark/img/plot/snake1024.png?raw=true)
/// random | ![](https://github.com/mich101mich/hierarchical_pathfinding/blob/benchmark/img/plot/random128.png?raw=true)| ![](https://github.com/mich101mich/hierarchical_pathfinding/blob/benchmark/img/plot/random1024.png?raw=true)
///
/// Conclusions:
/// - Bigger Chunks are usually better
/// - Chunks that take up the entire Grid are useless (see 128x128)
///
/// ### Memory
/// for 1024x1024: 100MB - 1000MB, depending on Node density on the Grid.
///
/// Can be drastically reduced by setting `cache_paths` to `false`, at the expense of repeated
/// calculations when using a Path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PathCacheConfig {
    /// The size of the individual Chunks (defaults to `8`)
    ///
    /// tl;dr: Depends highly on the size of your Grid and Lengths of your Paths;
    /// requires Experimentation if you care.
    ///
    /// Rough guide: Scale with a bigger Grid and longer desired Paths.
    /// _(don't quote me on this)_
    ///
    /// This has many different effects on the Performance and Memory:
    ///
    /// smaller chunks make calculations within a Chunk faster
    /// - => decreased update time in tiles_changed
    /// - => decreased time to find start and end Nodes
    ///
    /// bigger chunks lead to fewer Chunks and Nodes
    /// - => decreased time of actual search (especially for unreachable goals)
    /// - => longer Paths can be found a lot faster
    ///
    /// |Use Case|Recommendation|
    /// |--------|--------------|
    /// |Frequent Updates|Smaller Chunks|
    /// |Longer Paths required|Larger Chunks|
    /// |Larger Grid|Larger Chunks|
    /// |"No Path found" is common|Larger Chunks|
    /// |Grid consists of small, windy corridors|Smaller Chunks|
    pub chunk_size: usize,
    /// `true` (default): store the Paths inside each Chunk.
    ///
    /// `false`: only store the Cost of the Path.
    ///
    /// This will not affect the calculations or the time it takes to calculate the initial Path.
    /// It only makes a difference when iterating over the returned Path, because that requires
    /// re-calculating the next segment (see [`safe_next`](crate::internals::AbstractPath::safe_next) on [`AbstractPath`](crate::internals::AbstractPath)).
    ///
    /// Drastically reduces Memory usage.
    pub cache_paths: bool,
    /// `true` (default): When a Path is short (roughly `Length < 2 * chunk_size`), a regular
    /// A* search is performed on the Grid **after** HPA* calculated a Path to confirm the
    /// existence and length.
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
    /// It is questionable weather or not you should use Hierarchical Pathfinding if you enable
    /// this...
    pub perfect_paths: bool,
}

impl PathCacheConfig {
    /// Creates a new PathCacheConfig with the given `chunk_size`.
    /// ```
    /// # use hierarchical_pathfinding::PathCacheConfig;
    /// let chunk_size = 123;
    /// assert_eq!(
    ///     PathCacheConfig::with_chunk_size(chunk_size),
    ///     PathCacheConfig {
    ///         chunk_size,
    ///         ..Default::default()
    ///     }
    /// );
    /// ```
    pub fn with_chunk_size(chunk_size: usize) -> Self {
        Self {
            chunk_size,
            ..Default::default()
        }
    }

    /// an example PathCacheConfig with options set to reduce Memory Usage
    ///
    /// Values:
    /// ```
    /// # use hierarchical_pathfinding::PathCacheConfig;
    /// assert_eq!(
    ///     PathCacheConfig {
    ///         chunk_size: 64,
    ///         cache_paths: false,
    ///         a_star_fallback: true,
    ///         perfect_paths: false,
    ///     },
    ///     PathCacheConfig::LOW_MEM
    /// );
    /// ```
    pub const LOW_MEM: PathCacheConfig = PathCacheConfig {
        chunk_size: 64,
        cache_paths: false,
        a_star_fallback: true,
        perfect_paths: false,
    };
    /// an example PathCacheConfig with options set to improve Performance
    ///
    /// Values:
    /// ```
    /// # use hierarchical_pathfinding::PathCacheConfig;
    /// assert_eq!(
    ///     PathCacheConfig {
    ///         chunk_size: 16,
    ///         cache_paths: true,
    ///         a_star_fallback: false,
    ///         perfect_paths: false,
    ///     },
    ///     PathCacheConfig::HIGH_PERFORMANCE
    /// );
    /// ```
    pub const HIGH_PERFORMANCE: PathCacheConfig = PathCacheConfig {
        chunk_size: 16,
        cache_paths: true,
        a_star_fallback: false,
        perfect_paths: false,
    };
}

impl Default for PathCacheConfig {
    fn default() -> PathCacheConfig {
        PathCacheConfig {
            chunk_size: 8,
            cache_paths: true,
            a_star_fallback: true,
            perfect_paths: false,
        }
    }
}
