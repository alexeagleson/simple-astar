use bracket_pathfinding::prelude::*;
use criterion::{black_box, criterion_group, criterion_main, Criterion};
use simple_astar::astar;

fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("simple_astar straight line 5 * 5", |b| {
        let grid: Box<[u32]> = Box::new([
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        ]);
        b.iter(|| astar(black_box(0), black_box(24), black_box(&grid), black_box(5)))
    });
    c.bench_function("bracket_astar straight line 5 * 5", |b| {
        let map = Map::new(Box::new(['.'; 25]), 5, 5);
        b.iter(|| {
            a_star_search(black_box(0), black_box(24), black_box(&map));
        });
    });
    
    c.bench_function("simple_astar avoid obstacle 7 * 7", |b| {
        let grid: Box<[u32]> = Box::new([
            1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1,
            1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        ]);
        b.iter(|| astar(black_box(0), black_box(48), black_box(&grid), black_box(7)))
    });
    c.bench_function("bracket_astar avoid obstacle 7 * 7", |b| {
        let map = Map::new(
            Box::new([
                '.', '.', '.', '.', '.', '.', '.', '.', '.', '#', '.', '.', '#', '.', '.', '.',
                '#', '#', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '#',
                '#', '#', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.',
                '.',
            ]),
            7,
            7,
        );
        b.iter(|| {
            a_star_search(black_box(0), black_box(24), black_box(&map));
        });
    });
    c.bench_function("simple_astar avoid obstacle 28 * 28", |b| {
        let grid: Box<[u32]> = Box::new([
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1,
            1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1,
            1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1,
            1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1,
            1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1,
            1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1,
            1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1,
            1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1,
            1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1,
            1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1,
            1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1,
            1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1,
            1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
            1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        ]);
        b.iter(|| astar(black_box(0), black_box(28 * 28 -1), black_box(&grid), black_box(28)))
    });
    c.bench_function("bracket_astar avoid obstacle 28 * 28", |b| {
        let map = Map::new(
            Box::new([
                '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.',
                '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.',
                '.', '.', '#', '#', '.', '#', '.', '.', '.', '#', '#', '.', '#', '.', '.', '.', '#', '#', '.', '#', '.', '.', '.', '#', '#', '.', '#', '.',
                '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.',
                '.', '.', '#', '#', '#', '#', '.', '.', '.', '#', '#', '#', '#', '.', '.', '.', '#', '#', '#', '#', '.', '.', '.', '#', '#', '#', '#', '.',
                '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.',
                '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.',
                '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.',
                '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.',
                '.', '.', '#', '#', '.', '#', '.', '.', '.', '#', '#', '.', '#', '.', '.', '.', '#', '#', '.', '#', '.', '.', '.', '#', '#', '.', '#', '.',
                '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.',
                '.', '.', '#', '#', '#', '#', '.', '.', '.', '#', '#', '#', '#', '.', '.', '.', '#', '#', '#', '#', '.', '.', '.', '#', '#', '#', '#', '.',
                '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.',
                '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.',
                '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.',
                '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.',
                '.', '.', '#', '#', '.', '#', '.', '.', '.', '#', '#', '.', '#', '.', '.', '.', '#', '#', '.', '#', '.', '.', '.', '#', '#', '.', '#', '.',
                '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.',
                '.', '.', '#', '#', '#', '#', '.', '.', '.', '#', '#', '#', '#', '.', '.', '.', '#', '#', '#', '#', '.', '.', '.', '#', '#', '#', '#', '.',
                '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.',
                '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.',
                '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.',
                '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.',
                '.', '.', '#', '#', '.', '#', '.', '.', '.', '#', '#', '.', '#', '.', '.', '.', '#', '#', '.', '#', '.', '.', '.', '#', '#', '.', '#', '.',
                '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.', '.', '.', '#', '.', '.', '#', '.',
                '.', '.', '#', '#', '#', '#', '.', '.', '.', '#', '#', '#', '#', '.', '.', '.', '#', '#', '#', '#', '.', '.', '.', '#', '#', '#', '#', '.',
                '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.',
                '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.', '.',
            ]),
            28,
            28,
        );
        b.iter(|| {
            a_star_search(black_box(0), black_box(28 * 28 -1), black_box(&map));
        });
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);

pub struct Map {
    pub tiles: Box<[char]>,
    pub width: u32,
    pub height: u32,
}

impl Map {
    pub fn new(tiles: Box<[char]>, width: u32, height: u32) -> Self {
        Self {
            tiles,
            width,
            height,
        }
    }

    fn valid_exit(&self, loc: Point, delta: Point) -> Option<usize> {
        let destination = loc + delta;

        if destination.x < 0 || destination.y < 0 {
            return None;
        }

        let idx = self.point2d_to_index(destination);
        if self.in_bounds(destination) && self.tiles[idx] == '.' {
            Some(idx)
        } else {
            None
        }
    }
}

impl BaseMap for Map {
    fn is_opaque(&self, idx: usize) -> bool {
        self.tiles[idx as usize] == '#'
    }

    fn get_available_exits(&self, idx: usize) -> SmallVec<[(usize, f32); 10]> {
        let mut exits = SmallVec::new();
        let location = self.index_to_point2d(idx);

        if let Some(idx) = self.valid_exit(location, Point::new(-1, 0)) {
            exits.push((idx, 1.0))
        }
        if let Some(idx) = self.valid_exit(location, Point::new(1, 0)) {
            exits.push((idx, 1.0))
        }
        if let Some(idx) = self.valid_exit(location, Point::new(0, -1)) {
            exits.push((idx, 1.0))
        }
        if let Some(idx) = self.valid_exit(location, Point::new(0, 1)) {
            exits.push((idx, 1.0))
        }

        if let Some(idx) = self.valid_exit(location, Point::new(-1, -1)) {
            exits.push((idx, 1.4))
        }
        if let Some(idx) = self.valid_exit(location, Point::new(1, -1)) {
            exits.push((idx, 1.4))
        }
        if let Some(idx) = self.valid_exit(location, Point::new(-1, 1)) {
            exits.push((idx, 1.4))
        }
        if let Some(idx) = self.valid_exit(location, Point::new(1, 1)) {
            exits.push((idx, 1.4))
        }

        exits
    }

    fn get_pathing_distance(&self, idx1: usize, idx2: usize) -> f32 {
        DistanceAlg::Pythagoras.distance2d(self.index_to_point2d(idx1), self.index_to_point2d(idx2))
    }
}

impl Algorithm2D for Map {
    fn dimensions(&self) -> Point {
        Point::new(self.width, self.height)
    }
}
