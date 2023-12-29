use hierarchical_pathfinding::prelude::*;

#[test]
fn new() {
    let grid = [
        [0, 2, 0, 0, 0],
        [0, 2, 2, 2, 2],
        [0, 1, 0, 0, 0],
        [0, 1, 0, 2, 0],
        [0, 0, 0, 2, 0],
    ];
    let (width, height) = (grid[0].len(), grid.len());
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
fn update_path() {
    let mut grid = [
        [0, 2, 0, 0, 0],
        [0, 2, 2, 2, 2],
        [0, 1, 0, 0, 0],
        [0, 1, 0, 2, 0],
        [0, 0, 0, 2, 0],
    ];
    let (width, height) = (grid[0].len(), grid.len());
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
    let changed_tiles = [(1, 2)];

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

    // Add walls along chunk borders
    let start = (0, 0);
    let goal = (3, 4);
    let path = pathfinding.find_path(start, goal, cost_fn(&grid));
    assert!(path.is_some());

    // Vertical wall
    let changed_tiles: Vec<_> = (0..grid.len()).map(|y| (2, y)).collect();
    grid.iter_mut().for_each(|row| row[2] = 2);
    pathfinding.tiles_changed(&changed_tiles, cost_fn(&grid));

    let path = pathfinding.find_path(start, goal, cost_fn(&grid));
    assert!(path.is_none());

    let goal = (2, 4);
    let path = pathfinding.find_path(start, goal, cost_fn(&grid));
    assert!(path.is_some());

    // Horizontal wall
    let mut changed_tiles = Vec::new();
    {
        let y = 2;
        for x in 0..grid[y].len() {
            grid[y][x] = 2;
            changed_tiles.push((x, y));
        }
    }
    pathfinding.tiles_changed(&changed_tiles, cost_fn(&grid));

    let path = pathfinding.find_path(start, goal, cost_fn(&grid));
    assert!(path.is_none());
}

#[test]
fn skip_first_and_last() {
    let grid = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1], [1, 1, 0, 0]];
    // explanation:
    // grid:
    // . . .|#   // . = empty, # = wall
    // . S 0|1   // S = start
    // . G 2|#   // G = goal
    // -----+-   // chunk border
    // # # 4|5
    //
    //      ^ chunk border
    // numbers 0-3: nodes
    //
    // calculated path will be S -> 0 -> 2 -> G
    // but: old path optimizations would add the shortcut S->2 and 0->G, which would create
    //      an invalid path
    let (width, height) = (grid[0].len(), grid.len());
    fn cost_fn(grid: &[[usize; 4]]) -> impl '_ + Fn((usize, usize)) -> isize {
        move |(x, y)| [1, -1][grid[y][x]]
    }
    let pathfinding = PathCache::new(
        (width, height),
        cost_fn(&grid),
        ManhattanNeighborhood::new(width, height),
        PathCacheConfig {
            chunk_size: 3,
            a_star_fallback: false,
            ..Default::default()
        },
    );
    let start = (1, 1);
    let goal = (1, 2);
    let path = pathfinding.find_closest_goal(start, &[goal, (2, 3)], cost_fn(&grid));
    assert!(path.is_some());
    assert_eq!(path.unwrap().1.resolve(cost_fn(&grid)), vec![goal]);
}
