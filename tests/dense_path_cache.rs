use hierarchical_pathfinding::DensePathCache;

#[test]
fn new_dimensions() {
    let grid = vec![vec![0; 20]; 10];
    let cache = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));
    assert_eq!(cache.width(), 20);
    assert_eq!(cache.height(), 10);
}

#[test]
#[should_panic(expected = "Grid cannot be empty")]
fn new_empty_grid_panics() {
    let grid: Vec<Vec<i32>> = Vec::new();
    let _ = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));
}

#[test]
#[should_panic(expected = "Grid cannot be empty")]
fn new_empty_row_panics() {
    let grid: Vec<Vec<i32>> = vec![Vec::new()];
    let _ = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));
}

#[test]
fn get_in_bounds_and_out_of_bounds() {
    let grid = vec![vec![1, 2, 3], vec![4, 5, 6]];
    let cache = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));

    assert_eq!(cache.get((0, 0)), Some(&1));
    assert_eq!(cache.get((2, 0)), Some(&3));
    assert_eq!(cache.get((1, 1)), Some(&5));
    assert_eq!(cache.get((2, 1)), Some(&6));

    assert_eq!(cache.get((3, 0)), None);
    assert_eq!(cache.get((0, 2)), None);
    assert_eq!(cache.get((10, 10)), None);
}

#[test]
fn get_mut_modifies_value_and_marks_dirty() {
    let grid = vec![vec![0; 16]; 16];
    let mut cache = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));

    assert!(!cache.needs_update());

    // Out of bounds get_mut returns None and does not mark dirty
    assert_eq!(cache.get_mut((20, 20)), None);
    assert!(!cache.needs_update());

    if let Some(val) = cache.get_mut((5, 5)) {
        *val = 42;
    }

    assert_eq!(cache.get((5, 5)), Some(&42));
    assert!(cache.needs_update());
}

#[test]
fn entry_marks_dirty_only_on_modification() {
    let grid = vec![vec![0; 16]; 16];
    let mut cache = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));

    assert!(!cache.needs_update());

    {
        let entry = cache.entry((5, 5)).unwrap();
        assert_eq!(*entry, 0); // read-only access
    }
    assert!(!cache.needs_update()); // no modification, so should not be dirty

    {
        let mut entry = cache.entry((5, 5)).unwrap();
        *entry = 99; // modify the value
    }
    assert_eq!(cache.get((5, 5)), Some(&99));
    assert!(cache.needs_update()); // modification should mark dirty
}

#[test]
fn set_replaces_value_and_marks_dirty() {
    let grid = vec![vec![10; 16]; 16];
    let mut cache = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));

    assert!(!cache.needs_update());

    let old_val = cache.set((3, 4), 99);
    assert_eq!(old_val, 10);
    assert_eq!(cache.get((3, 4)), Some(&99));
    assert!(cache.needs_update());
}

#[test]
#[should_panic(expected = "Called DensePathCache::set with coordinates (20, 5)")]
fn set_out_of_bounds_panics() {
    let grid = vec![vec![0; 16]; 16];
    let mut cache = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));
    cache.set((20, 5), 1);
}

#[test]
fn update_cache_clears_dirty_flag() {
    let grid = vec![vec![0; 16]; 16];
    let mut cache = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));

    cache.set((4, 4), 1);
    assert!(cache.needs_update());

    cache.update_cache();
    assert!(!cache.needs_update());

    // Calling update_cache again when not dirty is a no-op
    cache.update_cache();
    assert!(!cache.needs_update());
}

#[test]
#[should_panic(expected = "Called find_path_no_update with dirty chunks")]
fn find_path_no_update_panics_when_dirty() {
    let grid = vec![vec![0; 16]; 16];
    let mut cache = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));
    cache.set((0, 0), 1);
    assert!(cache.needs_update());
    let _ = cache.find_path_no_update((0, 0), (5, 5));
}

#[test]
#[should_panic(expected = "Called find_all_paths_no_update with dirty chunks")]
fn find_all_paths_no_update_panics_when_dirty() {
    let grid = vec![vec![0; 16]; 16];
    let mut cache = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));
    cache.set((0, 0), 1);
    assert!(cache.needs_update());
    let _ = cache.find_all_paths_no_update((0, 0), &[(5, 5)]);
}

#[test]
#[should_panic(expected = "Called find_any_path_no_update with dirty chunks")]
fn find_any_path_no_update_panics_when_dirty() {
    let grid = vec![vec![0; 16]; 16];
    let mut cache = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));
    cache.set((0, 0), 1);
    assert!(cache.needs_update());
    let _ = cache.find_any_path_no_update((0, 0), &[(5, 5)]);
}

// ---------------------------------------------------------------------------
// Single-path finding tests (`find_path` and `find_path_no_update`)
// ---------------------------------------------------------------------------

#[test]
fn path_same_start_and_end() {
    let grid = vec![vec![0; 16]; 16];
    let mut cache = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));

    let path = cache.find_path((3, 3), (3, 3));
    assert!(path.is_some());
}

#[test]
fn path_adjacent_cells() {
    let grid = vec![vec![0; 16]; 16];
    let mut cache = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));

    let path = cache.find_path((0, 0), (1, 0));
    assert!(path.is_some());

    let path_vert = cache.find_path((0, 0), (0, 1));
    assert!(path_vert.is_some());
}

#[test]
fn path_within_single_chunk() {
    // Single 8x8 chunk (CHUNK_SIZE = 8)
    let grid = vec![vec![0; 8]; 8];
    let mut cache = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));

    let path = cache.find_path((0, 0), (7, 7));
    assert!(path.is_some());
}

#[test]
fn path_across_multiple_chunks() {
    // 32x32 grid spanning 4x4 chunks of size 8
    let grid = vec![vec![0; 32]; 32];
    let mut cache = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));

    let path = cache.find_path((0, 0), (31, 31));
    assert!(path.is_some());

    let path_rev = cache.find_path((31, 31), (0, 0));
    assert!(path_rev.is_some());
}

#[test]
fn path_across_chunk_boundaries() {
    // Path immediately crossing chunk boundary (7, 7) -> (8, 8)
    let grid = vec![vec![0; 16]; 16];
    let mut cache = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));

    let path = cache
        .find_path((7, 7), (8, 8))
        .expect("Path across boundary should exist");
    assert_eq!(path.cost(), 2);
    assert_eq!(path.length(), 2);
    let points: Vec<(usize, usize)> = path.iter().collect();
    assert_eq!(points.len(), 3);
    assert_eq!(points.first(), Some(&(7, 7)));
    assert_eq!(points.last(), Some(&(8, 8)));
}

#[test]
fn path_around_obstacles() {
    // 0 = empty, 1 = wall
    // Create a vertical wall at x=8 from y=0..15, leaving (8, 15) open
    let mut grid = vec![vec![0; 16]; 16];
    for row in grid.iter_mut().take(15) {
        row[8] = 1;
    }

    let mut cache = DensePathCache::new(
        grid,
        Box::new(|_from, _dir, to| if *to.value == 1 { None } else { Some(1) }),
    );

    let path = cache
        .find_path((0, 0), (15, 0))
        .expect("Path around opening at (8, 15) should exist");
    let points: Vec<(usize, usize)> = path.iter().collect();

    // Must navigate down through (8, 15) and back up
    assert!(points.contains(&(8, 15)));
    assert_eq!(points.first(), Some(&(0, 0)));
    assert_eq!(points.last(), Some(&(15, 0)));
    for &p in &points {
        assert!(
            p.0 != 8 || p.1 == 15,
            "Path cannot step through wall at ({}, {})",
            p.0,
            p.1
        );
    }
}

#[test]
fn path_blocked_by_wall_returns_none() {
    // Complete vertical wall at x=8
    let mut grid = vec![vec![0; 16]; 16];
    for row in &mut grid {
        row[8] = 1;
    }

    let mut cache = DensePathCache::new(
        grid,
        Box::new(|_from, _dir, to| if *to.value == 1 { None } else { Some(1) }),
    );

    let path = cache.find_path((0, 0), (15, 0));
    assert!(path.is_none());
}

#[test]
fn path_start_or_goal_is_wall_returns_none() {
    let mut grid = vec![vec![0; 16]; 16];
    grid[0][0] = 1; // start is wall
    grid[10][10] = 1; // goal is wall

    let mut cache = DensePathCache::new(
        grid,
        Box::new(|_from, _dir, to| if *to.value == 1 { None } else { Some(1) }),
    );

    assert!(cache.find_path((0, 0), (5, 5)).is_none());
    assert!(cache.find_path((5, 5), (10, 10)).is_none());
}

#[test]
fn path_with_variable_costs_prefers_cheaper_route() {
    // Grid with two possible routes:
    // Route 1 through y=0 has high cost swamp (cost 10)
    // Route 2 through y=2 has low cost road (cost 1)
    let mut grid = vec![vec![10; 5]; 3];
    grid[2].fill(1); // cheap road along y=2
    grid[0][0] = 1; // start
    grid[1][0] = 1; // connector
    grid[0][4] = 1; // end
    grid[1][4] = 1; // connector

    let mut cache = DensePathCache::new(grid, Box::new(|_from, _dir, to| Some(*to.value)));

    let path = cache.find_path((0, 0), (4, 0)).expect("Path should exist");
    let points: Vec<(usize, usize)> = path.iter().collect();

    // Optimal route goes down to the road at y=2: (0,0)->(0,1)->(0,2)->(1,2)->(2,2)->(3,2)->(4,2)->(4,1)->(4,0)
    // Total cost = 1+1+1+1+1+1+1+1 = 8 vs staying at y=0 which would cost 10*4 = 40
    assert!(
        points.contains(&(2, 2)),
        "Expected path to use the cheaper road at y=2"
    );
}

// ---------------------------------------------------------------------------
// Cache update and grid mutation effects on pathfinding
// ---------------------------------------------------------------------------

#[test]
fn path_updates_when_grid_is_modified() {
    // Start with a completely blocked wall at x=8
    let mut grid = vec![vec![0; 16]; 16];
    for row in &mut grid {
        row[8] = 1;
    }

    let mut cache = DensePathCache::new(
        grid,
        Box::new(|_from, _dir, to| if *to.value == 1 { None } else { Some(1) }),
    );

    assert!(cache.find_path((0, 0), (15, 0)).is_none());

    // Open a hole in the wall at (8, 0)
    cache.set((8, 0), 0);
    assert!(cache.needs_update());

    // find_path should automatically update the cache and find the path
    let path = cache.find_path((0, 0), (15, 0));
    assert!(path.is_some());
    assert!(!cache.needs_update());

    // Close the hole again via get_mut
    if let Some(cell) = cache.get_mut((8, 0)) {
        *cell = 1;
    }
    assert!(cache.needs_update());

    let path_blocked = cache.find_path((0, 0), (15, 0));
    assert!(path_blocked.is_none());
}

#[test]
fn path_no_update_works_when_cache_is_clean() {
    let grid = vec![vec![0; 16]; 16];
    let mut cache = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));

    // Explicitly update cache to ensure it is clean
    cache.update_cache();
    assert!(!cache.needs_update());

    let path = cache
        .find_path_no_update((0, 0), (5, 5))
        .expect("Path should exist");
    assert_eq!(path.cost(), 10);
}

// ---------------------------------------------------------------------------
// Multi-goal pathfinding tests (`find_all_paths` and `find_any_path`)
// ---------------------------------------------------------------------------

#[test]
fn find_all_paths_multiple_destinations() {
    // Wall dividing bottom half from top half on right side
    let mut grid = vec![vec![0; 16]; 16];
    for row in grid.iter_mut().skip(8) {
        row[8] = 1;
    }

    let mut cache = DensePathCache::new(
        grid,
        Box::new(|_from, _dir, to| if *to.value == 1 { None } else { Some(1) }),
    );

    let goals = [(2, 2), (12, 2), (0, 0)];
    let paths = cache.find_all_paths((0, 0), &goals);

    assert_eq!(paths.len(), 3);
    assert!(paths[0].is_some());
    assert!(paths[1].is_some());
    assert!(paths[2].is_some());

    let path_to_self = paths[2].as_ref().unwrap();
    assert_eq!(path_to_self.cost(), 0);
    assert_eq!(path_to_self.length(), 0);
}

#[test]
fn find_all_paths_with_unreachable_goals() {
    // Enclose (15, 15) in an impenetrable box
    let mut grid = vec![vec![0; 16]; 16];
    grid[14][14] = 1;
    grid[14][15] = 1;
    grid[15][14] = 1;
    grid[15][15] = 1;

    let mut cache = DensePathCache::new(
        grid,
        Box::new(|_from, _dir, to| if *to.value == 1 { None } else { Some(1) }),
    );

    let goals = [(5, 5), (15, 15)];
    let paths = cache.find_all_paths((0, 0), &goals);

    assert_eq!(paths.len(), 2);
    assert!(paths[0].is_some());
    assert!(paths[1].is_none());
}

#[test]
fn find_all_paths_empty_goals_list() {
    let grid = vec![vec![0; 16]; 16];
    let mut cache = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));

    let paths = cache.find_all_paths((0, 0), &[]);
    assert!(paths.is_empty());

    cache.update_cache();
    let paths_no_update = cache.find_all_paths_no_update((0, 0), &[]);
    assert!(paths_no_update.is_empty());
}

#[test]
fn find_any_path_picks_reachable_goal() {
    // (15, 15) is completely surrounded by walls, (5, 5) is open
    let mut grid = vec![vec![0; 16]; 16];
    grid[14][14] = 1;
    grid[14][15] = 1;
    grid[15][14] = 1;
    grid[15][15] = 1;

    let mut cache = DensePathCache::new(
        grid,
        Box::new(|_from, _dir, to| if *to.value == 1 { None } else { Some(1) }),
    );

    let goals = [(15, 15), (5, 5)];
    let path = cache
        .find_any_path((0, 0), &goals)
        .expect("Should find path to (5, 5)");
    let points: Vec<(usize, usize)> = path.iter().collect();
    assert_eq!(points.last(), Some(&(5, 5)));
}

#[test]
fn find_any_path_all_unreachable_returns_none() {
    // Complete wall across grid
    let mut grid = vec![vec![0; 16]; 16];
    for row in &mut grid {
        row[8] = 1;
    }

    let mut cache = DensePathCache::new(
        grid,
        Box::new(|_from, _dir, to| if *to.value == 1 { None } else { Some(1) }),
    );

    let goals = [(10, 0), (12, 5), (15, 15)];
    let path = cache.find_any_path((0, 0), &goals);
    assert!(path.is_none());
}

#[test]
fn find_any_path_empty_goals_returns_none() {
    let grid = vec![vec![0; 16]; 16];
    let mut cache = DensePathCache::new(grid, Box::new(|_, _, _| Some(1)));

    assert!(cache.find_any_path((0, 0), &[]).is_none());

    cache.update_cache();
    assert!(cache.find_any_path_no_update((0, 0), &[]).is_none());
}

// ---------------------------------------------------------------------------
// Non-square and non-chunk-aligned grids
// ---------------------------------------------------------------------------

#[test]
fn non_chunk_aligned_grid_sizes() {
    let grid_small = vec![vec![0; 1]; 1];
    let mut cache_small = DensePathCache::new(grid_small, Box::new(|_, _, _| Some(1)));
    assert_eq!(cache_small.width(), 1);
    assert_eq!(cache_small.height(), 1);
    let path_small = cache_small
        .find_path((0, 0), (0, 0))
        .expect("Path in 1x1 should exist");
    assert_eq!(path_small.cost(), 0);
    assert_eq!(path_small.length(), 0);

    let grid_rect = vec![vec![0; 25]; 7];
    let mut cache_rect = DensePathCache::new(grid_rect, Box::new(|_, _, _| Some(1)));
    assert_eq!(cache_rect.width(), 25);
    assert_eq!(cache_rect.height(), 7);
    let path_rect = cache_rect
        .find_path((0, 0), (24, 6))
        .expect("Path in rect should exist");
    assert_eq!(path_rect.cost(), 30);
    assert_eq!(path_rect.length(), 30);
}

// ---------------------------------------------------------------------------
// Custom struct elements and Entry/Position assertions
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq, Eq)]
enum Terrain {
    Grass,
    Water,
    Mountain,
}

#[test]
fn custom_grid_data_type() {
    let mut grid = vec![vec![Terrain::Grass; 16]; 16];
    grid[0][1] = Terrain::Water;
    grid[5][5] = Terrain::Mountain;

    let mut cache = DensePathCache::new(
        grid,
        Box::new(|from, _dir, to| {
            // Verify access to positions and values in callback
            let _ = (from.pos, to.pos);
            match to.value {
                Terrain::Grass => Some(1),
                Terrain::Water => Some(5),
                Terrain::Mountain => None,
            }
        }),
    );

    let path = cache
        .find_path((0, 0), (0, 2))
        .expect("Path around water or through water should exist");
    let points: Vec<(usize, usize)> = path.iter().collect();
    assert_eq!(points.first(), Some(&(0, 0)));
    assert_eq!(points.last(), Some(&(0, 2)));

    assert!(cache.find_path((0, 0), (5, 5)).is_none());
}
