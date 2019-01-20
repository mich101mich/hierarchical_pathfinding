use hierarchical_pathfinding::{generics, prelude::*, Point};
use rand::Rng;
use rayon::prelude::*;
use std::collections::HashMap;
use std::time::{Duration, Instant};

const SIZE: usize = 1024;

fn main() {
    let mut rng = rand::thread_rng();

    let def: PathCacheConfig = Default::default();

    let configs: HashMap<&str, PathCacheConfig> = [
        ("default", def),
        (
            "big_chunks",
            PathCacheConfig {
                chunk_size: 64,
                ..def
            },
        ),
    ]
    .into_iter()
    .cloned()
    .collect();

    let mut results: HashMap<&str, Vec<(u128, Option<usize>)>> =
        configs.keys().map(|name| (*name, vec![])).collect();
    results.insert("regular", vec![]);

    let (width, height) = (SIZE, SIZE);
    fn cost_fn<'a>(grid: &'a [Vec<u16>]) -> impl 'a + Fn(Point) -> isize {
        move |(x, y)| (grid[y][x >> 4] >> (x & 0b1111)) as isize * 2 - 1
    }
    let neighborhood = ManhattanNeighborhood::new(width, height);

    let points = [(0, 0), (0, SIZE - 1), (SIZE - 1, 0), (SIZE - 1, SIZE - 1)];

    for _ in 0..10 {
        let mut grid: Vec<Vec<u16>> = Vec::with_capacity(SIZE);

        for _ in 0..SIZE {
            let mut row = Vec::with_capacity(SIZE / 16);
            for _ in 0..SIZE {
                row.push(rng.gen());
            }
            grid.push(row);
        }
        grid[0][0] &= !0b1;

        let get_cost = cost_fn(&grid);

        println!("finished Grid gen");

        for i in (0..4).filter(|x| get_cost(points[*x]) >= 0) {
            for j in (0..4).filter(|x| *x != i) {
                let start_time = Instant::now();
                let path = generics::a_star_search(
                    |p| neighborhood.get_all_neighbors(p),
                    |p, _| get_cost(p) as usize,
                    |p| get_cost(p) >= 0,
                    points[i],
                    points[j],
                    |p| neighborhood.heuristic(p, points[j]),
                );
                let dt = duration_as_nanos(Instant::now() - start_time);
                results
                    .get_mut("regular")
                    .unwrap()
                    .push((dt, path.map(|p| p.cost)));
            }
        }
        println!("{}: {:?}", "regular", results["regular"]);

        for (&name, &cfg) in &configs {
            let mut pathfinding =
                PathCache::new((width, height), cost_fn(&grid), neighborhood, cfg);
            println!("finished PathCache gen");

            let results = results.get_mut(name).unwrap();

            for i in (0..4).filter(|x| get_cost(points[*x]) >= 0) {
                for j in (0..4).filter(|x| *x != i) {
                    let start_time = Instant::now();
                    let path = pathfinding.find_path(points[i], points[j], cost_fn(&grid));
                    let dt = duration_as_nanos(Instant::now() - start_time);
                    results.push((dt, path.map(|p| p.cost())));
                }
            }
            println!("{}: {:?}", name, results);
        }
    }

    let best = results["regular"]
        .iter()
        .map(|(_, cost)| cost)
        .cloned()
        .collect::<Vec<_>>();

    for (name, results) in results {
        let times = results.iter().map(|r| r.0);
        let min_time = times.clone().min().unwrap() as f64 / 1_000_000.0;
        let max_time = times.clone().max().unwrap() as f64 / 1_000_000.0;
        let avg_time = times.sum::<u128>() as f64 / results.len() as f64 / 1_000_000.0;

        assert!(results
            .iter()
            .zip(best.iter())
            .all(|(a, b)| a.1.is_some() == b.is_some()));

        let costs = results
            .iter()
            .filter_map(|r| r.1)
            .zip(best.iter().filter_map(|p| *p))
            .map(|(cost, best)| best as f64 / cost as f64);

        let min_cost = costs.clone().fold(std::f64::INFINITY, |a, b| a.min(b));
        let max_cost = costs.clone().fold(0.0_f64, |a, b| a.max(b));
        let avg_cost = costs.clone().sum::<f64>() / costs.count() as f64;

        println!(
            "{} | {:.3}ms - {:.3}ms; {:.3}ms | {:.2}% - {:.2}%; {:.2}%",
            name, min_time, max_time, avg_time, min_cost, max_cost, avg_cost,
        );
    }
}

fn duration_as_nanos(d: Duration) -> u128 {
    d.as_secs() as u128 * 1_000_000_000 + d.subsec_nanos() as u128
}
