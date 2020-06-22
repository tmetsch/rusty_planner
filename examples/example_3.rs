use std::sync::mpsc;
use std::thread;
use std::vec;

use rusty_planner::dstar_lite;
use rusty_planner::planner;

/// Simple maze as set of arrays.
struct Maze {
    maze: [[f64; 6]; 6],
}

/// Simple example using basic maze.
impl planner::ProblemSpace for Maze {
    type State = (usize, usize);
    type Iter = vec::IntoIter<(Self::State, f64)>;

    fn heuristic(&self, p: &Self::State, q: &Self::State) -> f64 {
        let x = (p.0 as i32 - q.0 as i32).abs();
        let y = (p.1 as i32 - q.1 as i32).abs();
        ((x).pow(2) as f64 + (y).pow(2) as f64).sqrt()
    }

    fn succ(&self, s: &Self::State) -> Self::Iter {
        let mut res = Vec::new();
        // same row...
        if s.1 > 0 && self.maze[s.0][s.1 - 1] != -1. {
            res.push(((s.0, s.1 - 1), self.maze[s.0][s.1 - 1]));
        }
        if s.1 + 1 < 6 && self.maze[s.0][s.1 + 1] != -1. {
            res.push(((s.0, s.1 + 1), self.maze[s.0][s.1 + 1]));
        }
        // row above & below;
        if s.0 > 0 && self.maze[s.0 - 1][s.1] != -1. {
            res.push(((s.0 - 1, s.1), self.maze[s.0 - 1][s.1]));
        }
        if s.0 + 1 < 6 && self.maze[s.0 + 1][s.1] != -1. {
            res.push(((s.0 + 1, s.1), self.maze[s.0 + 1][s.1]));
        }
        res.into_iter()
    }

    fn pred(&self, s: &Self::State) -> Self::Iter {
        // can move both ways...
        self.succ(s)
    }

    fn update(&mut self, state: &Self::State) {
        // removes obstacle @ (3, 4)
        if state.0 == 3 && state.1 == 4 {
            self.maze[state.0][state.1] = 1.;
        }
    }
}

/// Simple callback printing the path.
fn callback(path: Vec<(usize, usize)>) {
    print!("start");
    for x in &path {
        print!(" -> ({}, {})", x.0, x.1);
    }
    print!("\n");
}

///
/// Simple example with a robot moving trough a maze, in which obstacles get removed.
///
fn main() {
    let (tx, rx) = mpsc::channel();
    let env = [
        [-1., 1., 1., 1., 1., 1.],
        [-1., 1., -1., -1., -1., 2.],
        [-1., 1., 1., -1., -1., 2.],
        [-1., -1., 1., 1., -1., 2.],
        [-1., -1., -1., 1., -1., 2.],
        [1., 2., 2., 2., 2., 2.],
    ];
    let mut maze = Maze { maze: env };

    // Start the planner.
    let plnr = thread::spawn(move || {
        dstar_lite::solve(&mut maze, (5, 0), (0, 5), rx, callback);
    });

    // remote obstacle @ (3, 4) & move robot to coord (5, 2).
    tx.send(((3, 4), (5, 2))).unwrap();
    // signal the planner that we've reached the goal.
    tx.send(((0, 0), (0, 5))).unwrap();
    plnr.join().unwrap();
}