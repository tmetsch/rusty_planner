use std::sync::mpsc;
use std::thread;
use std::vec;

use rusty_planner::dstar_lite;
use rusty_planner::maze;
use rusty_planner::planner;

struct MyMaze {
    maze: maze::Maze,
}

impl planner::ProblemSpace for MyMaze {
    type State = (i32, i32);
    type Iter = vec::IntoIter<((i32, i32), f64)>;

    fn heuristic(&self, one: &Self::State, another: &Self::State) -> f64 {
        ((i32::pow(another.0 - one.0, 2) + i32::pow(another.1 - one.1, 2)) as f64).sqrt()
    }

    fn succ(&self, state: &Self::State) -> Self::Iter {
        let mut res = vec![];
        for item in self.maze.successors[state].iter() {
            res.push((*item, 1.0));
        }
        res.into_iter()
    }

    fn pred(&self, state: &Self::State) -> Self::Iter {
        let mut res = vec![];
        for item in self.maze.predecessors[state].iter() {
            res.push((*item, 1.0));
        }
        res.into_iter()
    }
}

impl planner::Lifelong for MyMaze {}

/// Simple callback function.
fn callback(path: Vec<(i32, i32)>) {
    print!("Start");
    for x in &path {
        print!(" -> {:?}", x);
    }
    println!();
}

fn main() {
    // generate a basic maze...
    let my_maze: maze::Maze = maze::generate(9);
    println!("{}", maze::print_maze(&my_maze));

    // ...wrap it a in a problem space...
    let mut ps = MyMaze { maze: my_maze };
    let start: (i32, i32) = (0, 0);
    let goal: (i32, i32) = (4, 4);

    // ...and solve it.
    let (tx, rx) = mpsc::channel();
    let planner = thread::spawn(move || {
        dstar_lite::solve(&mut ps, start, goal, rx, callback);
    });
    tx.send(((-1, -1), goal)).unwrap();
    planner.join().unwrap();
}
