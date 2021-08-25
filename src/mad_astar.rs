use std::cmp;
use std::collections;
use std::thread;
use std::time;

use crate::agent;
use crate::planner;
use crate::util;

struct StateValues {
    g_val: f64,
    h_val: f64,
}

fn process_message<PS: planner::ProblemSpace>(
    ps: &PS,
    s: PS::State,
    para: StateValues,
    goal: PS::State,
    data: &mut collections::HashMap<PS::State, StateValues>,
    open: &mut collections::BinaryHeap<util::HeapEntry<PS::State>>,
    closed: &mut collections::HashMap<PS::State, f64>,
) {
    // TODO: once open has contains method - remove this!
    let mut in_open: bool = false;
    for item in open.iter() {
        if item.state == s {
            in_open = true;
            break;
        }
    }
    if !(in_open && closed.contains_key(&s)) || data[&s].g_val > para.g_val {
        open.push(util::HeapEntry::new_entry(s, (para.g_val, 0.0)));
        let h_val = cmp::max(ps.heuristic(&s, &goal) as i64, para.h_val as i64);
        data.insert(
            s,
            StateValues {
                g_val: para.g_val,
                h_val: h_val as f64,
            },
        );
    }
}

fn expand<A: agent::Agent, PS: planner::ProblemSpace + planner::SharedStates>(
    ps: &PS,
    agent: &A,
    s: PS::State,
    goal: PS::State,
    data: &mut collections::HashMap<PS::State, StateValues>,
    open: &mut collections::BinaryHeap<util::HeapEntry<PS::State>>,
    closed: &mut collections::HashMap<PS::State, f64>,
) -> bool {
    // if we've found the goal --> tell others.
    if s == goal {
        agent.broadcast(&ps.serialize(0, &s, vec![data[&s].g_val, data[&s].h_val]));
        closed.insert(s, data[&s].g_val + ps.heuristic(&s, &goal));
        return true;
    }

    // if we've found a public state --> tell others.
    if ps.is_public(&s)
        && (!closed.contains_key(&s)
            || closed.get(&s).unwrap() > &(data[&s].g_val + ps.heuristic(&s, &goal)))
    {
        agent.broadcast(&ps.serialize(0, &s, vec![data[&s].g_val, data[&s].h_val]))
    }

    // Add to closed list.
    closed.insert(s, data[&s].g_val + ps.heuristic(&s, &goal));

    // check successors.
    for item in ps.succ(&s) {
        let s_dash: PS::State = item.0;
        let g_val: f64 = data[&s].g_val + item.1;
        let h_val: f64 = ps.heuristic(&s_dash, &goal);
        let f_val: f64 = g_val + h_val;
        if data.contains_key(&s_dash) {
            // in case another action already extended this one only keep better path!
            if data[&s_dash].g_val > g_val {
                data.insert(s_dash, StateValues { g_val, h_val });
            }
        } else {
            data.insert(s_dash, StateValues { g_val, h_val });
        }
        let mut old_f_val = -1.0;
        if closed.contains_key(&s_dash) {
            old_f_val = *closed.get(&s_dash).unwrap();
        }
        if !closed.contains_key(&s_dash) || (old_f_val > 0.0 && f_val < old_f_val) {
            open.push(util::HeapEntry::new_entry(s_dash, (f_val, 0.0)))
        }
    }
    false
}

fn traceback<A: agent::Agent, PS: planner::ProblemSpace + planner::SharedStates>(
    agent: &A,
    ps: &PS,
    start: PS::State,
    goal: PS::State,
    closed: collections::HashMap<PS::State, f64>,
) -> Vec<PS::State> {
    let mut res = Vec::new();
    let mut done = false;
    let mut curr: PS::State = goal;

    while !done {
        let preds = ps.pred(&curr);
        if preds.count() == 0 {
            let revd: Vec<String> = agent.retrieve();
            if !revd.is_empty() {
                for msg in revd {
                    let (msg_id, p_state, _) = ps.deserialize(msg);
                    if msg_id == 1 {
                        curr = p_state;
                    }
                }
            } else {
                thread::sleep(time::Duration::from_millis(250));
                continue;
            }
        }

        // find next shortest step to take...
        let mut min_cost = f64::INFINITY;
        let mut next_state = curr;
        for pred in ps.pred(&curr) {
            if closed.contains_key(&pred.0) && (closed[&pred.0] + pred.1) < min_cost {
                min_cost = closed[&pred.0] + pred.1;
                next_state = pred.0;
            }
        }
        res.push(next_state);
        curr = next_state;

        // if we found a public state tell ngbh. sys.
        if ps.is_public(&curr) {
            agent.broadcast(&ps.serialize(1, &curr, vec![]));
            done = true;
        }

        // we are done if we found the start too.
        if curr == start {
            done = true;
        }
    }
    res.reverse(); // traceback -> so let's reverse...
    res
}

///
/// Find a plan for multiple collaborative systems/agents.
///
pub fn solve<A: agent::Agent, PS: planner::ProblemSpace + planner::SharedStates>(
    agent: &A,
    ps: &PS,
    start: PS::State,
    goal: PS::State,
) -> Vec<PS::State> {
    let mut done: bool = false;

    // Open and closed state lists.
    let mut open: collections::BinaryHeap<util::HeapEntry<PS::State>> =
        collections::BinaryHeap::new();
    let mut closed: collections::HashMap<PS::State, f64> = collections::HashMap::new();
    let mut data: collections::HashMap<PS::State, StateValues> = collections::HashMap::new();

    // insert start into closed...
    data.insert(
        start,
        StateValues {
            g_val: 0.0,
            h_val: ps.heuristic(&start, &goal),
        },
    );
    open.push(util::HeapEntry::new_entry(start, (0.0, 0.0)));

    while !done {
        let revd: Vec<String> = agent.retrieve();
        if !revd.is_empty() {
            for msg in revd {
                let s: (u8, PS::State, Vec<f64>) = ps.deserialize(msg);
                if s.1 == goal {
                    done = true;
                    break;
                }
                process_message(
                    ps,
                    s.1,
                    StateValues {
                        g_val: s.2[0],
                        h_val: s.2[1],
                    },
                    goal,
                    &mut data,
                    &mut open,
                    &mut closed,
                );
            }
            if done {
                break;
            }
        }
        if !open.is_empty() {
            let s: util::HeapEntry<PS::State> = open.pop().unwrap();
            done = expand(ps, agent, s.state, goal, &mut data, &mut open, &mut closed);
        }
        thread::sleep(time::Duration::from_millis(250));
    }
    traceback(agent, ps, start, goal, closed)
}

#[cfg(test)]
mod tests {
    use std::collections;
    use std::vec;

    use crate::agent;
    use crate::mad_astar;
    use crate::planner;
    use crate::util;

    struct SimpleExample {}

    impl planner::ProblemSpace for SimpleExample {
        type State = i32;
        type Iter = vec::IntoIter<(i32, f64)>;

        fn heuristic(&self, _: &Self::State, _: &Self::State) -> f64 {
            0.0
        }

        fn succ(&self, state: &Self::State) -> Self::Iter {
            match *state {
                0 => vec![(1, 0.7), (2, 1.0)].into_iter(),
                1 => vec![(3, 1.0), (2, 0.1)].into_iter(),
                _ => vec![(3, 0.2)].into_iter(),
            }
        }

        fn pred(&self, state: &Self::State) -> Self::Iter {
            match *state {
                0 => vec![].into_iter(),
                3 => vec![(1, 1.0), (2, 0.2)].into_iter(),
                5 => vec![(4, 1.0)].into_iter(),
                6 => vec![(5, 1.0)].into_iter(),
                _ => vec![].into_iter(),
            }
        }
    }

    impl planner::SharedStates for SimpleExample {
        fn is_public(&self, state: &Self::State) -> bool {
            let mut res = false;
            if *state == 2 || *state == 1 || *state == 4 {
                res = true;
            }
            res
        }

        fn serialize(&self, msg_type: u8, state: &Self::State, para: Vec<f64>) -> String {
            let mut string_list: Vec<String> = vec![msg_type.to_string(), state.to_string()];
            for item in para {
                string_list.push(item.to_string());
            }
            string_list.join(";")
        }

        fn deserialize(&self, msg: String) -> (u8, Self::State, Vec<f64>) {
            let split = msg.split(';');
            let data = split.collect::<Vec<&str>>();

            let msg_type = data[0].parse::<u8>().unwrap();
            let state: i32 = data[1].parse::<i32>().unwrap();

            (msg_type, state, vec![])
        }
    }

    struct SimpleAgent {
        msgs: vec::Vec<String>,
    }

    impl agent::Agent for SimpleAgent {
        fn retrieve(&self) -> Vec<String> {
            self.msgs.clone()
        }

        fn broadcast(&self, _: &str) {}
    }

    // Test for success.

    #[test]
    fn test_process_message_for_success() {
        let ps = SimpleExample {};
        let vals = mad_astar::StateValues {
            g_val: 1.0,
            h_val: 1.0,
        };
        let mut data: collections::HashMap<i32, mad_astar::StateValues> =
            collections::HashMap::new();
        let mut open: collections::BinaryHeap<util::HeapEntry<i32>> =
            collections::BinaryHeap::new();
        let mut closed: collections::HashMap<i32, f64> = collections::HashMap::new();

        mad_astar::process_message(&ps, 1, vals, 3, &mut data, &mut open, &mut closed);
    }

    #[test]
    fn test_expand_for_success() {
        let ps = SimpleExample {};
        let agent = SimpleAgent {
            msgs: vec![String::from("foo")],
        };

        let mut data: collections::HashMap<i32, mad_astar::StateValues> =
            collections::HashMap::new();
        let mut open: collections::BinaryHeap<util::HeapEntry<i32>> =
            collections::BinaryHeap::new();
        let mut closed: collections::HashMap<i32, f64> = collections::HashMap::new();

        let s = 1;
        data.insert(
            s,
            mad_astar::StateValues {
                g_val: 1.0,
                h_val: 0.3,
            },
        );
        mad_astar::expand(&ps, &agent, s, 3, &mut data, &mut open, &mut closed);
    }

    #[test]
    fn test_traceback_for_success() {
        let ps = SimpleExample {};
        let agent = SimpleAgent { msgs: vec![] };
        let mut closed: collections::HashMap<i32, f64> = collections::HashMap::new();

        closed.insert(0, 1.0);
        closed.insert(1, 1.0);
        closed.insert(2, 1.0);

        mad_astar::traceback(&agent, &ps, 0, 3, closed);
    }

    #[test]
    fn test_solve_for_success() {
        let ps = SimpleExample {};
        let agent = SimpleAgent { msgs: vec![] };
        mad_astar::solve(&agent, &ps, 0, 3);
    }

    // Test for failure.

    // Test for sanity.

    #[test]
    fn test_process_message_for_sanity() {
        let ps = SimpleExample {};
        let vals = mad_astar::StateValues {
            g_val: 1.0,
            h_val: 1.0,
        };
        let mut data: collections::HashMap<i32, mad_astar::StateValues> =
            collections::HashMap::new();
        let mut open: collections::BinaryHeap<util::HeapEntry<i32>> =
            collections::BinaryHeap::new();
        let mut closed: collections::HashMap<i32, f64> = collections::HashMap::new();

        // not previously seen this state --> should update g and h val.
        let s = 1;
        mad_astar::process_message(&ps, s, vals, 3, &mut data, &mut open, &mut closed);
        assert_eq!(data[&s].g_val, 1.0);
        assert_eq!(data[&s].h_val, 1.0);

        // now state is in closed & open --> should not change anything.
        closed.insert(s, 1.0);
        let vals_1 = mad_astar::StateValues {
            g_val: 1.0,
            h_val: 1.0,
        };
        mad_astar::process_message(&ps, s, vals_1, 3, &mut data, &mut open, &mut closed);
        assert_eq!(data[&s].g_val, 1.0);
        assert_eq!(data[&s].h_val, 1.0);

        // newly received state has a better g_val --> should update g_val
        let vals = mad_astar::StateValues {
            g_val: 0.5,
            h_val: 1.0,
        };
        mad_astar::process_message(&ps, s, vals, 3, &mut data, &mut open, &mut closed);
        assert_eq!(data[&s].g_val, 0.5);
        assert_eq!(data[&s].h_val, 1.0);

        // newly received state has a worse g_val --> should NOT update g_val
        let vals = mad_astar::StateValues {
            g_val: 2.0,
            h_val: 1.0,
        };
        mad_astar::process_message(&ps, s, vals, 3, &mut data, &mut open, &mut closed);
        assert_eq!(data[&s].g_val, 0.5);
        assert_eq!(data[&s].h_val, 1.0);
    }

    #[test]
    fn test_expand_for_sanity() {
        let ps = SimpleExample {};
        let agent = SimpleAgent {
            msgs: vec![String::from("foo")],
        };

        let mut data: collections::HashMap<i32, mad_astar::StateValues> =
            collections::HashMap::new();
        let mut open: collections::BinaryHeap<util::HeapEntry<i32>> =
            collections::BinaryHeap::new();
        let mut closed: collections::HashMap<i32, f64> = collections::HashMap::new();

        // expand start state.
        data.insert(
            0,
            mad_astar::StateValues {
                g_val: 0.0,
                h_val: 0.0,
            },
        );
        let res = mad_astar::expand(&ps, &agent, 0, 3, &mut data, &mut open, &mut closed);
        assert_eq!(res, false);

        // found a public state.
        let res = mad_astar::expand(&ps, &agent, 1, 3, &mut data, &mut open, &mut closed);
        assert_eq!(res, false);
        assert_eq!(data[&1].g_val, 0.7);

        // found a better path.
        mad_astar::expand(&ps, &agent, 2, 3, &mut data, &mut open, &mut closed);
        assert_eq!(data[&2].g_val as i64, 0.8 as i64);

        // already expanded...
        mad_astar::expand(&ps, &agent, 1, 3, &mut data, &mut open, &mut closed);
        assert_eq!(data[&2].g_val as i64, 0.8 as i64);

        // found the goal.
        let res = mad_astar::expand(&ps, &agent, 3, 3, &mut data, &mut open, &mut closed);
        assert_eq!(res, true);
        assert_eq!(closed.contains_key(&3), true);
    }

    #[test]
    fn test_traceback_for_sanity() {
        let ps = SimpleExample {};
        let agent = SimpleAgent { msgs: vec![] };
        let mut closed: collections::HashMap<i32, f64> = collections::HashMap::new();

        closed.insert(4, 0.0);
        closed.insert(5, 1.0);
        closed.insert(6, 2.0);

        // simple traceback: start to finish!
        let res = mad_astar::traceback(&agent, &ps, 4, 6, closed);
        assert_eq!(res, vec![4, 5]);

        // goal state unknown - trigger by other agent
        let agent = SimpleAgent {
            msgs: vec!["1;6".to_string()],
        };
        let mut closed: collections::HashMap<i32, f64> = collections::HashMap::new();
        closed.insert(4, 0.0);
        closed.insert(5, 1.0);
        closed.insert(6, 2.0);
        let res = mad_astar::traceback(&agent, &ps, 4, 7, closed);
        assert_eq!(res, vec![4, 5]);
    }

    #[test]
    fn test_solve_for_sanity() {
        let ps = SimpleExample {};
        let agent = SimpleAgent { msgs: vec![] };

        // simple example.
        let res = mad_astar::solve(&agent, &ps, 0, 3);
        assert_eq!(res, vec![2]);
    }
}
