use std::collections;

use crate::util;

/// Trait to be implemented for iterative repair.
pub trait Problem {
    /// Type defining a conflict.
    type Conflict;
    /// Iterator to loop over the conflicts - uses tuple with 2nd item defining a priority.
    type Iter: Iterator<Item = (Self::Conflict, f64)>;

    /// Function to find all conflicts in the problem space.
    fn find_conflicts(&self) -> Self::Iter;
    /// Routine to fix a particular conflict.
    fn fix_conflict(&mut self, _: &Self::Conflict);
}

///
/// Solve a problem using iterative repair algorithm. Conflicts can be ranked.
///
/// # Example
/// ```
/// use std::vec;
///
/// use rusty_planner::iterative_repair;
///
/// struct Problem {}
///
/// impl iterative_repair::Problem for Problem {
///     type Conflict = (i32, i32);
///     type Iter = vec::IntoIter<(Self::Conflict, f64)>;
///     fn find_conflicts(&self) -> Self::Iter {
///         // TODO: Add code that finds conflicts...
///         vec![].into_iter()
///     }
///     fn fix_conflict(&mut self, conflict: &Self::Conflict) {
///         // TODO: Add code that solves a conflict...
///     }
/// }
///
/// let mut ps = Problem {};
/// iterative_repair::solve(&mut ps, 10);
/// ```
///
pub fn solve<PS: Problem>(ps: &mut PS, steps: i32) -> (bool, i32) {
    let mut found: bool = false;
    let mut iterations: i32 = 0;
    for i in 0..steps {
        iterations = i;
        // TODO: find nicer way to push items on max-heap.
        let mut heap: collections::BinaryHeap<util::HeapEntry<PS::Conflict>> =
            collections::BinaryHeap::new();
        for item in ps.find_conflicts() {
            heap.push(util::HeapEntry {
                state: item.0,
                keys: (item.1, 0.0),
            });
        }
        if !heap.is_empty() {
            let conflict: PS::Conflict = heap.pop().unwrap().state;
            ps.fix_conflict(&conflict);
        // TODO: trace conflicts solved - and if we find repetition of fixes -> break.
        } else {
            found = true;
            break;
        }
    }
    (found, iterations)
}

#[cfg(test)]
mod tests {
    use std::collections;
    use std::vec;

    use crate::iterative_repair;
    use crate::iterative_repair::Problem;

    const SIZE: i32 = 3;

    struct Channel {
        iden: i32,
    }

    // simple example - assume .e.g Wifi Channels in a 3x3 grid -> make sure no overlapping
    // channels are used.
    struct ScheduleProblem {
        channels: collections::HashMap<i32, Channel>,
    }

    impl Problem for ScheduleProblem {
        type Conflict = (i32, i32);
        type Iter = vec::IntoIter<(Self::Conflict, f64)>;

        fn find_conflicts(&self) -> Self::Iter {
            let mut res = Vec::new();
            for (iden, chan) in self.channels.iter() {
                // ngbhs to the right...
                let rem = (iden + 1) % SIZE;
                if rem > 0 {
                    for i in 0..(SIZE - rem) {
                        if chan.iden == self.channels[&(iden + i + 1)].iden {
                            res.push((
                                (*iden, (iden + i + 1)),
                                -1.0 * (chan.iden - self.channels[&(iden + i + 1)].iden) as f64,
                            ));
                        }
                    }
                }
                // ngbh below me...
                if (iden + SIZE) < self.channels.len() as i32 {
                    if chan.iden == self.channels[&(iden + SIZE)].iden {
                        res.push((
                            (*iden, (iden + SIZE)),
                            -1.0 * (chan.iden - self.channels[&(iden + SIZE)].iden) as f64,
                        ));
                    }
                }
            }
            res.into_iter()
        }

        fn fix_conflict(&mut self, conflict: &Self::Conflict) {
            if self.channels[&conflict.0].iden < 16 {
                self.channels.get_mut(&conflict.0).unwrap().iden += 1;
            } else {
                self.channels.get_mut(&conflict.0).unwrap().iden = 0;
            }
        }
    }

    // Test for success.

    #[test]
    fn test_solve_for_success() {
        let mut data = collections::HashMap::new();
        data.insert(0, Channel { iden: 1 });
        data.insert(1, Channel { iden: 4 });
        data.insert(2, Channel { iden: 3 });
        data.insert(3, Channel { iden: 3 });
        data.insert(4, Channel { iden: 4 });
        data.insert(5, Channel { iden: 1 });
        data.insert(6, Channel { iden: 2 });
        data.insert(7, Channel { iden: 1 });
        data.insert(8, Channel { iden: 3 });
        let mut ps = ScheduleProblem { channels: data };
        iterative_repair::solve(&mut ps, 32);
    }
    // Test for failure.

    // TODO: figure out what to do about this...

    // Test for sanity.

    #[test]
    fn test_solve_for_sanity() {
        let mut data = collections::HashMap::new();
        data.insert(0, Channel { iden: 7 });
        data.insert(1, Channel { iden: 12 });
        data.insert(2, Channel { iden: 16 });
        data.insert(3, Channel { iden: 8 });
        data.insert(4, Channel { iden: 3 });
        data.insert(5, Channel { iden: 16 });
        data.insert(6, Channel { iden: 4 });
        data.insert(7, Channel { iden: 4 });
        data.insert(8, Channel { iden: 11 });
        let mut ps = ScheduleProblem { channels: data };
        // 10 steps will do @ only 2 conflicts.
        let res = iterative_repair::solve(&mut ps, 10);
        assert_eq!(res.0, true); // solution is possible...
        assert_eq!(res.1, 2); // two conflicts to repair...
        assert_eq!(ps.find_conflicts().len(), 0);
    }
}
