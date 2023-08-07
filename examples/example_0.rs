use std::collections;
use std::num;
use std::vec;

use rusty_planner::iterative_repair;

const WIDTH: i32 = 4;
const HEIGHT: i32 = 4;

struct Queen {
    x: i32,
    y: i32,
}

struct Chess<'a> {
    board: &'a mut collections::HashMap<i32, Queen>,
}

///
/// Super simplified n-queens problem; omitting diagonals.
///
impl<'a> iterative_repair::Problem for Chess<'a> {
    // identifier of the two conflicting queens.
    type Conflict = (i32, i32);
    type Iter = vec::IntoIter<(Self::Conflict, f64)>;

    fn find_conflicts(&self) -> Self::Iter {
        let mut res = Vec::new();
        for (i, q_0) in self.board.iter() {
            for (j, q_1) in self.board.iter() {
                if i == j {
                    break;
                }
                if q_0.x == q_1.x || q_0.y == q_1.y {
                    res.push(((*i, *j), 0.0));
                }
            }
        }
        res.into_iter()
    }

    // conflict between queens can be fixed by moving away; or starting @ 0 if run our of space.
    fn fix_conflict(&mut self, conflict: &Self::Conflict) {
        if self.board[&conflict.0].x == self.board[&conflict.1].x {
            if self.board[&conflict.0].x < WIDTH {
                self.board.get_mut(&conflict.0).unwrap().x += 1;
            } else {
                self.board.get_mut(&conflict.0).unwrap().x += 0;
            }
        } else if self.board[&conflict.0].y == self.board[&conflict.1].y {
            if self.board[&conflict.0].y < HEIGHT {
                self.board.get_mut(&conflict.0).unwrap().y += 1;
            } else {
                self.board.get_mut(&conflict.0).unwrap().y += 0;
            }
        }
    }
}

fn main() -> Result<(), num::ParseIntError> {
    let mut board: collections::HashMap<i32, Queen> = collections::HashMap::new();
    board.insert(0, Queen { x: 0, y: 0 });
    board.insert(1, Queen { x: 1, y: 2 });
    board.insert(2, Queen { x: 2, y: 2 });
    board.insert(3, Queen { x: 3, y: 0 });
    let mut ps = Chess { board: &mut board };

    let res = iterative_repair::solve(&mut ps, 10);
    println!("Solution found: {}.\nIterations: {}.", res.0, res.1);

    for (i, queen) in board {
        println!("Queen {} is now @ ({}, {}).", i, queen.x, queen.y)
    }
    Ok(())
}
