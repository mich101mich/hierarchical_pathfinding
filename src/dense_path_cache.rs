#![allow(unused, reason = "still in development")]
// #![warn(clippy::pedantic)]
#![allow(clippy::wildcard_imports)]

use std::sync::Arc;

use hashbrown::{HashMap, HashSet};

mod construct;
mod find;
mod path;
mod update;
mod utils;

pub use path::{PathSegment, PathSegmentIter};

use utils::*;

type Point = (usize, usize);
type Cost = Option<usize>;
const CHUNK_SIZE: usize = 8;

#[derive(Debug, Clone, Copy)]
pub struct GridCell<'a, T> {
    pub value: &'a T,
    pub pos: Point,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Dir {
    Up = 0,
    Right = 1,
    Down = 2,
    Left = 3,
}

impl Dir {
    pub const fn opposite(self) -> Dir {
        match self {
            Dir::Up => Dir::Down,
            Dir::Right => Dir::Left,
            Dir::Down => Dir::Up,
            Dir::Left => Dir::Right,
        }
    }
    pub const fn is_horizontal(self) -> bool {
        matches!(self, Dir::Left | Dir::Right)
    }
    pub const fn is_vertical(self) -> bool {
        matches!(self, Dir::Up | Dir::Down)
    }
}

type InputCallback<T> = dyn Fn(GridCell<'_, T>, Dir, GridCell<'_, T>) -> Cost + Send + Sync;
type Callback<T> = dyn Fn(Point, Dir, &'_ [Vec<T>]) -> Cost + Send + Sync;

/// Hierarchical path cache for a dense 2D grid with a fixed size.
pub struct DensePathCache<T> {
    grid: Vec<Vec<T>>,
    chunks: Vec<Vec<Chunk>>,
    cost_fn: Box<Callback<T>>,

    dirty_chunks: HashSet<Point>,
}

struct Chunk {
    top: [Option<Exit>; CHUNK_SIZE],
    bottom: [Option<Exit>; CHUNK_SIZE],
    left: [Option<Exit>; CHUNK_SIZE - 2], // excluding the corners, they are in top/bottom
    right: [Option<Exit>; CHUNK_SIZE - 2],
}

/// An exit of a chunk. May be a corner.
#[derive(Default)]
struct Exit {
    /// Paths within the chunk to other exits, arranged by layer.
    internal_paths: Vec<HashMap<Point, Arc<PathSegment>>>,
    /// The sides of the chunk that this exit is on. Indexed by [`Dir`]
    walk_costs: [Cost; 4],
}

#[derive(Debug)]
pub struct Entry<'a, T> {
    cell: &'a mut T,
    dirty_entry: Option<(Point, &'a mut HashSet<Point>)>,
}

impl<T> Entry<'_, T> {
    /// Returns a mutable reference to the underlying cell **without** marking the chunk as dirty. Use with caution.
    ///
    /// This method may only be used to modify state that is entirely unrelated to the pathfinding. Any changes that
    /// would affect costs etc. can cause sporadic panics and possibly undefined behavior!
    pub unsafe fn get_mut_untracked(&mut self) -> &mut T {
        &mut *self.cell
    }

    /// Explicitly marks the entry as dirty, ensuring that the corresponding chunk will be updated.
    ///
    /// This is usually tracked automatically using the `DerefMut` implementation of the entry. This method is only
    /// useful when those checks were skipped through [`get_mut_untracked`](Self::get_mut_untracked), but then an
    /// update was still deemed necessary.
    pub fn mark_dirty(&mut self) {
        if let Some((point, dirty_chunks)) = self.dirty_entry.take() {
            dirty_chunks.insert(point);
        }
    }
}

impl<T> std::ops::Deref for Entry<'_, T> {
    type Target = T;
    fn deref(&self) -> &Self::Target {
        self.cell
    }
}

impl<T> std::ops::DerefMut for Entry<'_, T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.mark_dirty();
        self.cell
    }
}

// Thoughts on finding the neighbors of a given position:
// - We want to use PathSegments to skip as much distance as possible.
// - We need to take individual steps within the start/end chunk, since the start/end points are (most likely) not
//   aligned with PathSegments.
// - The naive approach is to always add the direct neighbors and the endpoints of any PathSegment that starts at the
//   current position to the list of neighbors.
//   - This does not work though, because the cost of those segments is guaranteed to be at least as high as the
//     heuristic cost of the direct neighbor, so the algorithm would, in almost all cases, prefer the direct neighbor
//     over the PathSegment.
// - To fix this, we will prevent the algorithm from adding the direct neighbors in chunks that are not the start or
//   end chunk, and only use the PathSegments for navigation within those chunks.
//   - Proof that this is correct:
//     - Given a chunk C. Our initialization/update procedure will identify all exits of C and find and cache the
//       shortest paths between every pair of exits.
//     - Given an optimal path P from start to end.
//     - If the path steps through C, but C is neither the starting nor ending chunk of P, then it will need to both
//       enter and exit C.
//     - This is only possible through one of the exits of C that we have identified.
//     - The PathSegments are guaranteed to be the shortest paths between those exits, so it is impossible for P to
//       contain a shorter path through C than the PathSegment that we have computed.
//     All of this is, of course, assuming that the user-provided cost function is correct and deterministic and that
//     the cache is up to date.
// - Since we don't just have chunks, but a full hierarchy of superchunks, we will need to adapt this logic based on
//   the current position within the hierarchy.
//
// Example:
// Chunks level 1:             Chunks level 2:             Chunks level 3:
// +0----+1----+2----+3----+   +0----------+1----------+   +0----------------------+
// 0P    |P    |*    |*    |   0<          |P          |   0<                      |
// |     |     |     |     |   |           |           |   |                       |
// +-----+-----+-----+-----+   |           |           |   |                       |
// 1P    |< s  |*    |*    |   |        s  |           |   |        s              |
// |     |     |     |     |   |           |           |   |                       |
// +-----+-----+-----+-----+   +-----------+-----------+   |                       |
// 2*    |*    |P    |P    |   1P          |<          |   |                       |
// |     |     |     |     |   |           |           |   |                       |
// +-----+-----+-----+-----+   |           |           |   |                       |
// 3*    |*    |<    |P    |   |           |           |   |                       |
// |     |     |  e  |     |   |           | e         |   |            e          |
// +-----+-----+-----+-----+   +-----------+-----------+   +-----------------------+
// Legend:
// - s = start position, e = end position
// - < = For positions in this chunk, recurse to the layer below
// - P = Traverse this chunk using PathSegments
// - * = This chunk is never visited because the layer above did not recurse into it
//
// Explanation:
// - Level 3:
//   - Chunk (0,0) is both start and end chunk, so no PathSegments will be used from this level.
//     (Note that normally, a top-level chunk like this would not be generated. This example assumes that the grid is
//     larger than shown here)
// - Level 2:
//   - Chunks (0,0) and (1,1) are the start and end chunks, so no PathSegments will be used here.
//   - Chunks (1,0) and (0,1) are neither start nor end chunks, so they will only use PathSegments for navigation.
// - Level 1:
//   - Chunks (1,1) and (2,3) are the start and end chunks, so no PathSegments will be used here.
//   - The chunks aligning with Level 2 chunks (1,0) and (0,1) are never used, since the Level 2 chunks are used
//     instead. These are: (2,0), (3,0), (2,1), (3,1), (0,2), (1,2), (0,3), (1,3).
//   - The remaining chunks use PathSegments for navigation: (0,0), (1,0), (0,1), (2,2), (3,2), (3,3).
//
// Effective "cells" of this search:
// +-----+-----+-----------+
// |     |     |           |
// |     |     |           |
// +-----+-----+           |
// |     |OOsOO|           |
// |     |OOOOO|           |
// +-----+-----+-----+-----+
// |           |     |     |
// |           |     |     |
// |           +-----+-----+
// |           |OOOOO|     |
// |           |OOeOO|     |
// +-----------+-----+-----+ // Nice and hierarchical 👍
//
// Optimizations:
// - Exits at the edge of the grid are never created, since they would never be used anyway.
// - Since every exit has a PathSegment to every other exit in the same chunk, we would be checking every exit again
//   for each of those exits. To avoid this, we first check the predecessor that was used to get here. If it is in the
//   same chunk, skip looking at exits and only look at the partners of this exit in the neighboring chunks.

// just the public methods, internal methods are in the modules
impl<T: Send + Sync + 'static> DensePathCache<T> {
    /// Creates a new cache with the given grid and cost function.
    ///
    /// ### Panics
    ///
    /// Panics if the grid is empty
    #[must_use]
    pub fn new(grid: Vec<Vec<T>>, cost_fn: Box<InputCallback<T>>) -> Self {
        assert!(
            !grid.is_empty() && !grid[0].is_empty(),
            "Grid cannot be empty"
        );
        Self::new_impl(grid, cost_fn)
    }

    /// Returns the width of the grid.
    #[must_use]
    pub fn width(&self) -> usize {
        self.grid[0].len()
    }
    /// Returns the height of the grid.
    #[must_use]
    pub fn height(&self) -> usize {
        self.grid.len()
    }

    /// Returns a reference to the value at the given position, or `None` if out of bounds.
    #[must_use]
    pub fn get(&self, (x, y): Point) -> Option<&T> {
        self.grid.get(y).and_then(|row| row.get(x))
    }

    /// Returns a mutable reference to the value at the given position, or `None` if out of bounds.
    ///
    /// Note that this will mark the chunk containing the position as dirty, which will trigger a cache update the next
    /// time that a path is requested (see [cache updates](TODO) in the crate documentation). This means that this
    /// method should only be called when you intend to make changes. If you need to check the value first, call
    /// [`get`](Self::get) first or use [`entry`](Self::entry).
    #[must_use]
    pub fn get_mut(&mut self, (x, y): Point) -> Option<&mut T> {
        let ret = self.grid.get_mut(y)?.get_mut(x)?;

        self.dirty_chunks.insert(to_chunk_pos((x, y)));

        Some(ret)
    }

    /// Returns an entry for the value at the given position, or `None` if out of bounds.
    ///
    /// The entry allows you to access and modify the value, and will automatically mark the containing chunk as dirty,
    /// but only when the value is actually modified.
    ///
    /// Note that, since the `DensePathCache` uses direct indexing into the grid, it is not that much more efficient
    /// than calling [`get`](Self::get) followed by [`get_mut`](Self::get_mut). This method saves one indexing check
    /// and one `Option` check on your end.
    ///
    /// `Entry`s also allow modifying the value directly without marking the chunk as dirty, though usage of that is
    /// highly discouraged.
    #[must_use]
    pub fn entry(&mut self, (x, y): Point) -> Option<Entry<'_, T>> {
        let cell = self.grid.get_mut(y)?.get_mut(x)?;

        let dirty_entry = Some((to_chunk_pos((x, y)), &mut self.dirty_chunks));

        Some(Entry { cell, dirty_entry })
    }

    /// Sets the value at the given position, returning the old value. Panics if out of bounds.
    ///
    /// Shorthand for `std::mem::replace(cache.get_mut(pos).unwrap(), value)`.
    ///
    /// ### Panics
    ///
    /// Panics if the given position is out of bounds.
    #[track_caller]
    pub fn set(&mut self, (x, y): Point, value: T) -> T {
        let Some(cell) = self.get_mut((x, y)) else {
            panic!(
                "Called DensePathCache::set with coordinates ({x}, {y}), but size is {}x{}",
                self.width(),
                self.height()
            );
        };

        std::mem::replace(cell, value)
    }

    /// Returns whether the cache needs to be updated. If this returns `true`, the next path request will trigger a
    /// cache update, and calls to `*_no_update` methods will panic.
    #[must_use = "This method only checks whether the cache needs to be updated"]
    pub fn needs_update(&self) -> bool {
        !self.dirty_chunks.is_empty()
    }

    /// Updates the cache with any changes made to the grid.
    ///
    /// It is usually not necessary to call this method directly, as it will be called automatically when a path is
    /// requested. However, if you want to ensure that the path requests are as fast as possible, you can call this
    /// method after making changes to the grid.
    ///
    /// This is an expensive operation, so it is recommended to only call this method when necessary. If you are
    /// making multiple changes to the grid, it is recommended to only call this method after all changes have been
    /// made.
    pub fn update_cache(&mut self) {
        if self.dirty_chunks.is_empty() {
            return;
        }

        self.update_cache_impl();

        self.dirty_chunks.clear();
    }

    /// Finds the shortest path between two positions in the grid.
    pub fn find_path(&mut self, start: Point, end: Point) -> Option<PathSegment> {
        self.update_cache();
        self.find_path_no_update(start, end)
    }

    /// Finds the shortest path between two positions in the grid, without updating the cache.
    ///
    /// This method is identical to [`find_path`](Self::find_path), except that it takes an immutable self reference
    /// and does not update the cache. Instead, it will panic if the cache is dirty. Call
    /// [`update_cache`](Self::update_cache) first if the cache might be dirty.
    ///
    /// This method is useful for contexts where keeping a mutable reference to the cache is inconvenient, such as in
    /// a multi-threaded context.
    ///
    /// ### Panics
    ///
    /// Panics if the cache is dirty. Call [`update_cache`](Self::update_cache) first if the cache might be dirty.
    #[track_caller]
    #[must_use]
    pub fn find_path_no_update(&self, start: Point, end: Point) -> Option<PathSegment> {
        assert!(
            self.dirty_chunks.is_empty(),
            "Called find_path_no_update with dirty chunks. Call update_cache first or use find_path."
        );
        self.find_path_impl(start, end)
    }

    /// Finds the shortest path from a start position to multiple end positions in the grid.
    #[must_use]
    pub fn find_all_paths(&mut self, start: Point, ends: &[Point]) -> Vec<Option<PathSegment>> {
        self.update_cache();
        self.find_all_paths_no_update(start, ends)
    }

    /// Finds the shortest path from a start position to multiple end positions in the grid, without updating the cache.
    ///
    /// This method is identical to [`find_all_paths`](Self::find_all_paths), except that it takes an immutable self reference
    /// and does not update the cache. Instead, it will panic if the cache is dirty. Call
    /// [`update_cache`](Self::update_cache) first if the cache might be dirty.
    ///
    /// This method is useful for contexts where keeping a mutable reference to the cache is inconvenient, such as in
    /// a multi-threaded context.
    ///
    /// ### Panics
    ///
    /// Panics if the cache is dirty. Call [`update_cache`](Self::update_cache) first if the cache might be dirty.
    #[track_caller]
    #[must_use]
    pub fn find_all_paths_no_update(
        &self,
        start: Point,
        ends: &[Point],
    ) -> Vec<Option<PathSegment>> {
        assert!(
            self.dirty_chunks.is_empty(),
            "Called find_all_paths_no_update with dirty chunks. Call update_cache first or use find_all_paths."
        );

        self.find_all_paths_impl(start, ends)
    }

    /// Finds the shortest path from a start position to any of multiple end positions in the grid.
    ///
    /// This method will return the shortest path to the first end position that is reachable from the start position.
    /// If none of the end positions are reachable, it will return `None`.
    #[must_use]
    pub fn find_any_path(&mut self, start: Point, ends: &[Point]) -> Option<PathSegment> {
        self.update_cache();
        self.find_any_path_no_update(start, ends)
    }

    /// Finds the shortest path from a start position to any of multiple end positions in the grid, without updating the cache.
    ///
    /// This method is identical to [`find_any_path`](Self::find_any_path), except that it takes an immutable self reference
    /// and does not update the cache. Instead, it will panic if the cache is dirty. Call
    /// [`update_cache`](Self::update_cache) first if the cache might be dirty.
    ///
    /// This method is useful for contexts where keeping a mutable reference to the cache is inconvenient, such as in
    /// a multi-threaded context.
    ///
    /// ### Panics
    ///
    /// Panics if the cache is dirty. Call [`update_cache`](Self::update_cache) first if the cache might be dirty.
    #[track_caller]
    #[must_use]
    pub fn find_any_path_no_update(&self, start: Point, ends: &[Point]) -> Option<PathSegment> {
        assert!(
            self.dirty_chunks.is_empty(),
            "Called find_any_path_no_update with dirty chunks. Call update_cache first or use find_any_path."
        );

        self.find_any_path_impl(start, ends)
    }
}

impl<T: Send + Sync + 'static> std::fmt::Debug for DensePathCache<T> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DensePathCache")
            .field(
                "grid",
                &format_args!("[{}x{}]", self.width(), self.height()),
            )
            .field("cost_fn", &format_args!("<Opaque Callback>"))
            .field("dirty_chunks", &self.dirty_chunks)
            .finish_non_exhaustive()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_send_sync() {
        fn assert_send_sync<T: Send + Sync + 'static>() {}
        assert_send_sync::<DensePathCache<i32>>();
        assert_send_sync::<PathSegment>();
    }

    #[test]
    fn check_can_multithread_finds() {
        let cache = Arc::new(DensePathCache::new(
            vec![vec![0; 16]; 16],
            Box::new(|_, _, _| Some(1)),
        ));
        let cache2 = Arc::clone(&cache);

        let t1 = std::thread::spawn(move || cache.find_path_no_update((0, 0), (15, 15)).unwrap());
        let t2 = std::thread::spawn(move || cache2.find_path_no_update((0, 0), (0, 15)).unwrap());

        t1.join().unwrap();
        t2.join().unwrap();
    }
}
