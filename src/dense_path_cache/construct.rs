use super::*;

use rayon::prelude::*;

impl<T: Send + Sync + 'static> DensePathCache<T> {
    pub(super) fn new_impl(grid: Vec<Vec<T>>, cost_fn: Box<InputCallback<T>>) -> DensePathCache<T> {
        let cost_fn = Box::new(move |a, dir: Dir, grid: &[Vec<T>]| {
            let b = dir.step(a, (grid[0].len(), grid.len()))?;
            let entry_a = Entry {
                value: &grid[a.1][a.0],
                pos: a,
            };
            let entry_b = Entry {
                value: &grid[b.1][b.0],
                pos: b,
            };
            cost_fn(entry_a, dir, entry_b)
        });

        let dirty_chunks = HashSet::new();

        let chunks = Self::construct_chunks(&grid, &cost_fn);

        DensePathCache {
            grid,
            cost_fn,
            dirty_chunks,
            chunks,
        }
    }

    fn construct_chunks(grid: &[Vec<T>], cost_fn: &Callback<T>) -> Vec<Vec<Chunk>> {
        let cw = grid[0].len() / CHUNK_SIZE;
        let ch = grid.len() / CHUNK_SIZE;
        (0..ch)
            .into_par_iter()
            .map(|cy| {
                (0..cw)
                    .into_par_iter()
                    .map(|cx| {
                        Self::process_base_chunk(grid, (cx * CHUNK_SIZE, cy * CHUNK_SIZE), cost_fn)
                    })
                    .collect()
            })
            .collect()
    }

    fn process_base_chunk(grid: &[Vec<T>], (left, top): Point, cost_fn: &Callback<T>) -> Chunk {
        let bounds = (grid[0].len(), grid.len());
        let right = left + CHUNK_SIZE - 1;
        let bottom = top + CHUNK_SIZE - 1;

        let cost_fn = |a: Point, dir: Dir| cost_fn(a, dir, grid);

        let exit_for = |a: Point, dir: Dir| {
            let cost = cost_fn(a, dir)?;
            let mut walk_costs = [None; 4];
            walk_costs[dir as usize] = Some(cost);
            Some(Exit {
                internal_paths: Vec::new(),
                walk_costs,
            })
        };

        let mut chunk = Chunk {
            top_exits: std::array::from_fn(|dx| exit_for((left + dx, top), Dir::Up)),
            bottom_exits: std::array::from_fn(|dx| exit_for((left + dx, bottom), Dir::Down)),
            left_exits: std::array::from_fn(|dy| exit_for((left, top + dy), Dir::Left)),
            right_exits: std::array::from_fn(|dy| exit_for((right, top + dy), Dir::Right)),
        };

        // corners can have more than one side
        for (pos, dir) in [
            ((left, top), Dir::Left),
            ((right, top), Dir::Right),
            ((left, bottom), Dir::Left),
            ((right, bottom), Dir::Right),
        ] {
            if let Some(cost) = cost_fn(pos, dir) {
                let exit = chunk.exit_at_mut(pos).get_or_insert_default();
                exit.walk_costs[dir as usize] = Some(cost);
            }
        }

        // compute internal paths
        let mut all_exit_positions = chunk
            .border_iterator_mut()
            .filter_map(|(pos, exit)| Some((pos, exit.as_mut()?)))
            .collect::<Vec<_>>();

        while let Some((start, exit)) = all_exit_positions.pop()
            && !all_exit_positions.is_empty()
        {
            let paths = dijkstra(start, &all_exit_positions, (left, top), cost_fn);

            exit.internal_paths.push(paths);
        }

        chunk
    }
}

fn dijkstra(
    start: Point,
    targets: &[(Point, &mut Exit)],
    offset: (usize, usize),
    cost_fn: impl Fn(Point, Dir) -> Option<usize>,
) -> HashMap<Point, Arc<PathSegment>> {
    let mut min_cost_to = [[(usize::MAX, start); CHUNK_SIZE]; CHUNK_SIZE]; // (cost, previous_point)

    #[derive(PartialEq, Eq)]
    struct HeapEntry {
        pos: Point,
        cost: usize,
    }
    impl PartialOrd for HeapEntry {
        fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
            Some(self.cmp(other))
        }
    }
    impl Ord for HeapEntry {
        fn cmp(&self, other: &Self) -> std::cmp::Ordering {
            other.cost.cmp(&self.cost) // reverse order for min-heap
        }
    }

    let mut queue = std::collections::BinaryHeap::<HeapEntry>::new();
    min_cost_to[start.1][start.0] = (0, start);
    queue.push(HeapEntry {
        pos: start,
        cost: 0,
    });

    while let Some(entry) = queue.pop() {
        let HeapEntry { pos, cost } = entry;
        if cost > min_cost_to[pos.1][pos.0].0 {
            continue; // we have already found a better path to this point
        }

        for &dir in &[Dir::Up, Dir::Down, Dir::Left, Dir::Right] {
            let Some(next_pos) = dir.step(pos, (CHUNK_SIZE, CHUNK_SIZE)) else {
                continue;
            };
            let Some(next_cost) = cost_fn((pos.0 + offset.0, pos.1 + offset.1), dir) else {
                continue;
            };
            let total_cost = cost + next_cost;
            if total_cost < min_cost_to[next_pos.1][next_pos.0].0 {
                min_cost_to[next_pos.1][next_pos.0] = (total_cost, pos);
                queue.push(HeapEntry {
                    pos: next_pos,
                    cost: total_cost,
                });
            }
        }
    }

    let mut ret = HashMap::new();
    for &(target, _) in targets {
        let (cost, prev) = min_cost_to[target.1][target.0];
        if cost == usize::MAX {
            continue; // can't be reached
        }
        let mut path = vec![];
        let mut current = target;
        while current != start {
            path.push(current);
            let (prev_cost, prev) = min_cost_to[current.1][current.0];
            debug_assert!(prev_cost != usize::MAX);
            current = prev;
        }
        path.reverse();
        let len = path.len();
        let path_segment = PathSegment {
            inner: path::InnerPath::Raw(path),
            cost,
            len,
        };
        ret.insert(target, Arc::new(path_segment));
    }

    ret
}
