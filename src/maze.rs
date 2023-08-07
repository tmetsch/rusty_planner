use std::collections;

use std::iter::FromIterator;

use rand::prelude::SliceRandom;
use rand::Rng;

///
/// Represents a maze in form of a set of coordinates defining successors and predecessors.
///
pub struct Maze {
    pub size: i32,
    pub successors: collections::HashMap<(i32, i32), Vec<(i32, i32)>>,
    pub predecessors: collections::HashMap<(i32, i32), Vec<(i32, i32)>>,
}

///
/// Generates a Maze.
///
/// For more information see:
/// <https://weblog.jamisbuck.org/2011/1/27/maze-generation-growing-tree-algorithm>.
///
pub fn generate(size: i32) -> Maze {
    let mut maze = Maze {
        size,
        successors: collections::HashMap::new(),
        predecessors: collections::HashMap::new(),
    };
    let mut directions = [(0, -1), (1, 0), (-1, 0), (0, 1)]; // [N, S, E, W]
    let mut cells = vec![(0, 0)];
    let mut rng = rand::thread_rng();

    while !cells.is_empty() {
        let mut index = Some(rng.gen_range(0..cells.len()));
        let curr = cells[index.unwrap_or(0)];
        directions.shuffle(&mut rand::thread_rng());
        for dir in directions {
            let ngbh = (curr.0 + dir.0, curr.1 + dir.1);
            if ngbh.0 < 0 || ngbh.0 >= size || ngbh.1 < 0 || ngbh.1 >= size {
                continue;
            }
            if maze.predecessors.contains_key(&ngbh) {
                continue;
            }
            if let collections::hash_map::Entry::Vacant(e) = maze.successors.entry(curr) {
                e.insert(vec![ngbh]);
                maze.predecessors.insert(curr, vec![ngbh]);
            } else if let Some(val) = maze.successors.get_mut(&curr) {
                val.push(ngbh)
            }
            if let collections::hash_map::Entry::Vacant(e) = maze.predecessors.entry(ngbh) {
                e.insert(vec![curr]);
                maze.successors.insert(ngbh, vec![]);
            }
            cells.push(ngbh);
            index = None;
            break;
        }
        if let Some(index) = index {
            cells.remove(index);
        }
    }
    maze
}

///
/// Returns a visual representation of the maze.
///
pub fn print_maze(maze: &Maze) -> String {
    use std::fmt::Write;
    let mut res = String::new();

    let top_border = "_".repeat((maze.size * 2 - 1) as usize);

    //
    let header: String = Vec::from_iter(0..maze.size)
        .iter()
        .map(|&id| id.to_string() + " ")
        .collect();

    writeln!(res, "   {}", header).unwrap();
    writeln!(res, "   {} ", top_border).unwrap();
    for row in 0..maze.size {
        write!(res, "{} |", row).unwrap();
        for col in 0..maze.size {
            if !maze.successors.contains_key(&(col, row))
                || !maze.predecessors.contains_key(&(col, row))
            {
                continue;
            }
            // check if path to below is blocked...
            if maze.successors[&(col, row)].contains(&(col, row + 1))
                || maze.predecessors[&(col, row)].contains(&(col, row + 1))
            {
                write!(res, " ").unwrap();
            } else {
                write!(res, "_").unwrap();
            }
            // check if right of me the path is blocked...
            if maze.successors[&(col, row)].contains(&(col + 1, row))
                || maze.predecessors[&(col, row)].contains(&(col + 1, row))
            {
                if maze.successors[&(col, row)].contains(&(col, row + 1))
                    || maze.predecessors[&(col, row)].contains(&(col, row + 1))
                    || (maze.successors.contains_key(&(col + 1, row))
                        && maze.successors[&(col + 1, row)].contains(&(col + 1, row + 1)))
                    || (maze.predecessors.contains_key(&(col + 1, row))
                        && maze.predecessors[&(col + 1, row)].contains(&(col + 1, row + 1)))
                {
                    write!(res, " ").unwrap();
                } else {
                    write!(res, "_").unwrap();
                }
            } else {
                write!(res, "|").unwrap();
            }
        }
        writeln!(res).unwrap();
    }
    res
}

#[cfg(test)]
mod tests {
    use super::*;

    // Test for success.

    #[test]
    fn test_generate_for_success() {
        generate(2);
    }

    #[test]
    fn test_print_maze_for_success() {
        let my_maze: Maze = generate(2);
        print_maze(&my_maze);
    }

    // Test for failure.

    // n/a

    // Test for sanity.

    #[test]
    fn test_generate_for_sanity() {
        let my_maze = generate(2);
        assert_eq!(4, my_maze.successors.len());
        assert_eq!(4, my_maze.predecessors.len());
    }

    #[test]
    fn test_print_maze_for_sanity() {
        let my_maze: Maze = generate(2);
        let repr: String = print_maze(&my_maze);
        assert_ne!(0, repr.len())
    }
}
