use std::collections;
use std::sync::mpsc;

use crate::planner;
use crate::util;

fn key(data: &util::StateData, h: f64) -> (f64, f64) {
    let mut k_0 = data.g;
    if data.rhs + h < k_0 {
        k_0 = data.rhs + h;
    }
    let mut k_1 = data.g;
    if data.rhs > k_1 {
        k_1 = data.rhs;
    }
    (k_0, k_1)
}

fn update_state<PS: planner::ProblemSpace>(
    ps: &mut PS,
    s: PS::State,
    start: PS::State,
    goal: PS::State,
    data: &mut collections::HashMap<PS::State, util::StateData>,
    open: &mut collections::BinaryHeap<util::HeapEntry<PS::State>>,
) {
    data.entry(s).or_insert(util::StateData {
        rhs: f64::INFINITY,
        g: f64::INFINITY,
    });
    if s != goal {
        let mut tmp = f64::INFINITY;
        for item in ps.succ(&s) {
            if data.contains_key(&item.0) && item.1 + data[&item.0].g < tmp {
                tmp = item.1 + data[&item.0].g;
            }
        }
        data.get_mut(&s).unwrap().rhs = tmp;
    }
    open.retain(|e| e.state != s);
    if data[&s].g as i64 != data[&s].rhs as i64 {
        open.push(util::HeapEntry::new_entry(
            s,
            key(&data[&s], ps.heuristic(&s, &start)),
        ));
    }
}

fn compute_path<PS: planner::ProblemSpace>(
    ps: &mut PS,
    start: PS::State,
    goal: PS::State,
    data: &mut collections::HashMap<PS::State, util::StateData>,
    open: &mut collections::BinaryHeap<util::HeapEntry<PS::State>>,
) {
    while (!open.is_empty())
        && ((open.peek().unwrap().keys < key(&data[&start], ps.heuristic(&start, &start)))
            || (data[&start].rhs as i64 != data[&start].g as i64))
    {
        let s: util::HeapEntry<PS::State> = open.pop().unwrap();
        if data[&s.state].g > data[&s.state].rhs {
            data.get_mut(&s.state).unwrap().g = data[&s.state].rhs;
        } else {
            data.get_mut(&s.state).unwrap().g = f64::INFINITY;
            update_state(ps, s.state, start, goal, data, open);
        }
        for (s_dash, _) in ps.pred(&s.state) {
            update_state(ps, s_dash, start, goal, data, open);
        }
    }
}

fn publish_path<PS: planner::ProblemSpace>(
    ps: &mut PS,
    start: PS::State,
    goal: PS::State,
    data: &mut collections::HashMap<PS::State, util::StateData>,
    callback: fn(Vec<PS::State>),
) {
    // TODO: check if useful to implement action ids to not only output the states but also the
    //   actions (aka. the path/edges -> the road taken).
    let mut res = Vec::new();
    let mut curr = start;
    while curr != goal {
        let mut min_cost = f64::INFINITY;
        let mut next_state = curr;
        for item in ps.succ(&curr) {
            if data.contains_key(&item.0) && data[&item.0].g < min_cost {
                min_cost = data[&item.0].g;
                next_state = item.0;
            }
        }
        res.push(next_state);
        curr = next_state;
    }
    callback(res);
}

///
/// Find a plan to get from start to goal, given a problem space.
///
/// *Note*: This will run forever! Signal that you reach the goal state to let it terminate.
///
// TODO: "hide" the threading part behind a trait.
pub fn solve<PS: planner::ProblemSpace + planner::Lifelong>(
    ps: &mut PS,
    start: PS::State,
    goal: PS::State,
    rx: mpsc::Receiver<(PS::State, PS::State)>,
    callback: fn(Vec<PS::State>),
) {
    // TODO: add key_modifier parameter.
    let mut start_int: PS::State = start;
    let mut open: collections::BinaryHeap<util::HeapEntry<PS::State>> =
        collections::BinaryHeap::new();
    let mut data: collections::HashMap<PS::State, util::StateData> = collections::HashMap::new();

    data.insert(
        start_int,
        util::StateData {
            g: f64::INFINITY,
            rhs: f64::INFINITY,
        },
    );
    data.insert(
        goal,
        util::StateData {
            g: f64::INFINITY,
            rhs: 0.0,
        },
    );
    open.push(util::HeapEntry::new_entry(
        goal,
        key(&data[&goal], ps.heuristic(&goal, &start_int)),
    ));

    compute_path(ps, start_int, goal, &mut data, &mut open);
    publish_path(ps, start_int, goal, &mut data, callback);
    loop {
        let (u, new_start) = rx.recv().unwrap();
        if new_start == goal {
            break;
        }
        start_int = new_start;
        ps.update(&u);
        update_state(ps, u, start_int, goal, &mut data, &mut open);
        compute_path(ps, start_int, goal, &mut data, &mut open);
        publish_path(ps, start_int, goal, &mut data, callback);
    }
}

#[cfg(test)]
mod tests {
    use std::collections;
    use std::sync::mpsc;
    use std::thread;
    use std::vec;

    use crate::dstar_lite;
    use crate::planner;
    use crate::util;

    struct SimpleGraph {
        ts: i32,
    }

    impl planner::ProblemSpace for SimpleGraph {
        type State = i32;
        type Iter = vec::IntoIter<(i32, f64)>;
        fn heuristic(&self, _: &Self::State, _: &Self::State) -> f64 {
            0.0
        }

        fn succ(&self, s: &Self::State) -> Self::Iter {
            match (*s, self.ts) {
                (0, _) => vec![(1, 1.0)].into_iter(),
                (1, _) => vec![(2, 1.0), (3, 1.0)].into_iter(),
                (2, 0) => vec![(4, 1.0)].into_iter(),
                (2, _) => vec![(4, 7.0)].into_iter(),
                _ => vec![(4, 5.0)].into_iter(),
            }
        }

        fn pred(&self, s: &Self::State) -> Self::Iter {
            match (*s, self.ts) {
                (1, _) => vec![(0, 1.0)].into_iter(),
                (2, _) => vec![(1, 1.0)].into_iter(),
                (3, _) => vec![(1, 1.0)].into_iter(),
                _ => vec![(2, 1.0), (3, 5.0)].into_iter(),
            }
        }
    }

    impl planner::Lifelong for SimpleGraph {
        fn update(&mut self, _: &Self::State) {
            self.ts += 1;
        }
    }

    fn callback(_: vec::Vec<i32>) {}

    fn callback_test(res: vec::Vec<i32>) {
        assert_eq!(res, [1, 2, 4]);
    }

    // Test for success.

    #[test]
    fn test_key_for_success() {
        let s_d = util::StateData { g: 1.0, rhs: 1.0 };
        dstar_lite::key(&s_d, 1.0);
    }

    #[test]
    fn test_update_state_for_success() {
        let mut ps = SimpleGraph { ts: 0 };
        let start = 0;
        let goal = 4;

        let mut data = collections::HashMap::new();
        data.insert(
            start,
            util::StateData {
                g: f64::INFINITY,
                rhs: f64::INFINITY,
            },
        );
        data.insert(
            goal,
            util::StateData {
                g: f64::INFINITY,
                rhs: 0.0,
            },
        );

        let mut open = collections::BinaryHeap::new();
        open.push(util::HeapEntry::new_entry(
            goal,
            dstar_lite::key(&data[&goal], 0.0),
        ));

        let s = 1;
        dstar_lite::update_state(&mut ps, s, start, goal, &mut data, &mut open);
    }

    #[test]
    fn test_compute_path_for_success() {
        let mut ps = SimpleGraph { ts: 0 };
        let start = 0;
        let goal = 4;

        let mut data = collections::HashMap::new();
        data.insert(
            start,
            util::StateData {
                g: f64::INFINITY,
                rhs: f64::INFINITY,
            },
        );
        data.insert(
            goal,
            util::StateData {
                g: f64::INFINITY,
                rhs: 0.0,
            },
        );

        let mut open = collections::BinaryHeap::new();
        open.push(util::HeapEntry::new_entry(
            goal,
            dstar_lite::key(&data[&goal], 0.0),
        ));

        dstar_lite::compute_path(&mut ps, start, goal, &mut data, &mut open);
    }

    #[test]
    fn test_publish_path_for_success() {
        let mut ps = SimpleGraph { ts: 0 };
        let start = 0;
        let goal = 1;
        let mut data = collections::HashMap::new();
        data.insert(start, util::StateData { g: 1.0, rhs: 1.0 });
        data.insert(goal, util::StateData { g: 0.0, rhs: 0.0 });

        dstar_lite::publish_path(&mut ps, start, goal, &mut data, callback)
    }

    #[test]
    fn test_solve_for_success() {
        let mut ps = SimpleGraph { ts: 0 };
        let (tx, rx) = mpsc::channel();
        let plnr = thread::spawn(move || {
            dstar_lite::solve(&mut ps, 0, 4, rx, callback);
        });
        tx.send((-1, 4)).unwrap();
        plnr.join().unwrap();
    }

    // Test for failure.

    // TODO: figure out what to do about this...

    // Test for sanity.

    #[test]
    fn test_key_for_sanity() {
        let sd_0 = util::StateData { g: 1.0, rhs: 1.0 };
        let sd_1 = util::StateData { g: 10.0, rhs: 1.0 };
        let sd_2 = util::StateData { g: 1.0, rhs: 10.0 };
        assert_eq!(dstar_lite::key(&sd_0, 1.0), (1.0, 1.0));
        assert_eq!(dstar_lite::key(&sd_1, 1.0), (2.0, 10.0));
        assert_eq!(dstar_lite::key(&sd_2, 1.0), (1.0, 10.0));
    }

    #[test]
    fn test_update_state_for_sanity() {
        let mut ps = SimpleGraph { ts: 0 };
        let start = 0;
        let goal = 4;

        let mut data = collections::HashMap::new();
        // data.insert(start, util::StateData { g: f64::INFINITY, rhs: f64::INFINITY });
        data.insert(goal, util::StateData { g: 1.0, rhs: 0.0 });
        let mut open = collections::BinaryHeap::new();

        // s not visited...
        let s = 1;
        dstar_lite::update_state(&mut ps, s, start, goal, &mut data, &mut open);
        assert_eq!(data.contains_key(&s), true);

        // s != goal
        let s = 2;
        dstar_lite::update_state(&mut ps, s, start, goal, &mut data, &mut open);
        assert_eq!(data[&s].rhs, 2.0); // g(goal) == 1.0 + cost of 1.0

        // s should have been added to open queue...
        let mut contains = false;
        for item in open.iter() {
            if item.state == s {
                contains = true;
            }
        }
        assert_eq!(contains, true);
    }

    #[test]
    fn test_publish_path_for_sanity() {
        let mut ps = SimpleGraph { ts: 0 };
        let start = 0;
        let goal = 4;
        let mut data = collections::HashMap::new();
        data.insert(start, util::StateData { g: 4.0, rhs: 4.0 });
        data.insert(1, util::StateData { g: 3.0, rhs: 3.0 });
        data.insert(2, util::StateData { g: 2.0, rhs: 2.0 });
        data.insert(
            3,
            util::StateData {
                g: f64::INFINITY,
                rhs: f64::INFINITY,
            },
        );
        data.insert(goal, util::StateData { g: 0.0, rhs: 0.0 });

        dstar_lite::publish_path(&mut ps, start, goal, &mut data, callback_test)
    }

    #[test]
    fn test_compute_path_for_sanity() {
        let mut ps = SimpleGraph { ts: 0 };
        let start = 0;
        let goal = 4;

        let mut data = collections::HashMap::new();
        data.insert(
            start,
            util::StateData {
                g: f64::INFINITY,
                rhs: f64::INFINITY,
            },
        );
        data.insert(
            goal,
            util::StateData {
                g: f64::INFINITY,
                rhs: 0.0,
            },
        );

        let mut open = collections::BinaryHeap::new();
        open.push(util::HeapEntry::new_entry(
            goal,
            dstar_lite::key(&data[&goal], 0.0),
        ));

        dstar_lite::compute_path(&mut ps, start, goal, &mut data, &mut open);

        assert_eq!(data[&0].g, 3.0); // 3 steps to goal possible ...
        assert_eq!(data[&1].g, 2.0); // 2 steps ...
        assert_eq!(data[&2].g, 1.0);
        assert_eq!(data[&3].g, f64::INFINITY); // should not have expanded in A* search ...
        assert_eq!(data[&4].g, 0.0);
    }

    #[test]
    fn test_solve_for_sanity() {
        let mut ps = SimpleGraph { ts: 0 };
        let (tx, rx) = mpsc::channel();
        let plnr = thread::spawn(move || {
            dstar_lite::solve(&mut ps, 0, 4, rx, callback);
        });
        tx.send((2, 1)).unwrap();
        tx.send((-1, 4)).unwrap();
        plnr.join().unwrap();
    }
}
