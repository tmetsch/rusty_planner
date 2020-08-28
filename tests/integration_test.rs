extern crate rusty_planner;

use std::sync::mpsc;
use std::thread;
use std::time;
use std::vec;

#[test]
fn test_simple_example() {
    struct Example {}

    impl rusty_planner::planner::ProblemSpace for Example {
        type State = i32;
        type Iter = vec::IntoIter<(i32, f64)>;

        fn heuristic(&self, _: &Self::State, _: &Self::State) -> f64 {
            1.0
        }

        fn succ(&self, state: &Self::State) -> Self::Iter {
            match *state {
                0 => vec![(1, 1.0)].into_iter(),
                1 => vec![(2, 1.0), (3, 1.0)].into_iter(),
                2 => vec![(3, 2.0), (4, 1.0)].into_iter(),
                3 => vec![(4, 10.0)].into_iter(),
                _ => vec![].into_iter(),
            }
        }

        fn pred(&self, state: &Self::State) -> Self::Iter {
            match *state {
                1 => vec![(0, 1.0)].into_iter(),
                2 => vec![(1, 1.0)].into_iter(),
                3 => vec![(1, 1.0), (2, 2.0)].into_iter(),
                4 => vec![(2, 1.0), (3, 10.0)].into_iter(),
                _ => vec![].into_iter(),
            }
        }
    }

    // Using default :-)
    impl rusty_planner::planner::Lifelong for Example {}

    fn callback(path: Vec<i32>) {
        assert_eq!(path, [1, 2, 4]);
    }

    let mut example = Example {};

    // Anytime Dynamic A*
    rusty_planner::any_dyn_astar::solve(&example, 0, 4, callback);

    // D* Lite
    let (tx, rx) = mpsc::channel();
    let plnr = thread::spawn(move || {
        rusty_planner::dstar_lite::solve(&mut example, 0, 4, rx, callback);
    });
    tx.send((-1, 4)).unwrap();
    plnr.join().unwrap();
}

#[test]
fn test_multi_agent_example() {
    struct PaperExample {
        agent_id: u8,
    }

    #[derive(Copy, Clone, Eq, Hash, PartialEq, Debug)]
    struct StateVector {
        v1: u8,
        v2: u8,
        v3: u8,
        v4: u8,
    }

    // This example is taken from the paper describing MAD-A*.
    impl rusty_planner::planner::ProblemSpace for PaperExample {
        type State = StateVector;
        type Iter = vec::IntoIter<(Self::State, f64)>;

        fn heuristic(&self, _: &Self::State, _: &Self::State) -> f64 {
            0.0
        }

        fn succ(&self, state: &Self::State) -> Self::Iter {
            let mut res = vec![];
            if self.agent_id == 1 {
                if state.v1 == 0 {
                    res.push((
                        StateVector {
                            v1: 1,
                            v2: state.v2,
                            v3: state.v3,
                            v4: state.v4,
                        },
                        1.0,
                    ))
                }
                if state.v1 == 1 {
                    res.push((
                        StateVector {
                            v1: 2,
                            v2: state.v2,
                            v3: state.v3,
                            v4: state.v4,
                        },
                        1.0,
                    ))
                }
                if state.v2 == 0 {
                    res.push((
                        StateVector {
                            v1: state.v1,
                            v2: 1,
                            v3: state.v3,
                            v4: state.v4,
                        },
                        1.0,
                    ))
                }
                if state.v2 == 1 {
                    res.push((
                        StateVector {
                            v1: state.v1,
                            v2: 2,
                            v3: state.v3,
                            v4: state.v4,
                        },
                        1.0,
                    ))
                }
                if state.v4 == 0 && state.v1 == 2 && state.v2 == 2 {
                    res.push((
                        StateVector {
                            v1: state.v1,
                            v2: state.v2,
                            v3: state.v3,
                            v4: 1,
                        },
                        1.0,
                    ))
                }
            } else if self.agent_id == 2 {
                if state.v4 == 1 && state.v3 == 2 {
                    res.push((
                        StateVector {
                            v1: state.v1,
                            v2: state.v2,
                            v3: state.v3,
                            v4: 2,
                        },
                        1.0,
                    ))
                }
                if state.v3 == 0 {
                    res.push((
                        StateVector {
                            v1: state.v1,
                            v2: state.v2,
                            v3: 1,
                            v4: state.v4,
                        },
                        1.0,
                    ))
                }
                if state.v3 == 1 {
                    res.push((
                        StateVector {
                            v1: state.v1,
                            v2: state.v2,
                            v3: 2,
                            v4: state.v4,
                        },
                        1.0,
                    ))
                }
            } else {
                panic!("Whoops! Only supporting 2 agents...")
            }
            res.into_iter()
        }

        fn pred(&self, state: &Self::State) -> Self::Iter {
            // This is cheating - but good enough for a test...
            match (*state, self.agent_id) {
                (
                    StateVector {
                        v1: 2,
                        v2: 2,
                        v3: 2,
                        v4: 2,
                    },
                    2,
                ) => vec![(
                    StateVector {
                        v1: 2,
                        v2: 2,
                        v3: 2,
                        v4: 1,
                    },
                    1.0,
                )]
                .into_iter(),
                (
                    StateVector {
                        v1: 2,
                        v2: 2,
                        v3: 2,
                        v4: 1,
                    },
                    2,
                ) => vec![(
                    StateVector {
                        v1: 2,
                        v2: 2,
                        v3: 1,
                        v4: 1,
                    },
                    1.0,
                )]
                .into_iter(),
                (
                    StateVector {
                        v1: 2,
                        v2: 2,
                        v3: 1,
                        v4: 1,
                    },
                    2,
                ) => vec![(
                    StateVector {
                        v1: 2,
                        v2: 2,
                        v3: 0,
                        v4: 1,
                    },
                    1.0,
                )]
                .into_iter(),
                (
                    StateVector {
                        v1: 2,
                        v2: 2,
                        v3: 0,
                        v4: 1,
                    },
                    1,
                ) => vec![(
                    StateVector {
                        v1: 2,
                        v2: 2,
                        v3: 0,
                        v4: 0,
                    },
                    1.0,
                )]
                .into_iter(),
                (
                    StateVector {
                        v1: 2,
                        v2: 2,
                        v3: 0,
                        v4: 0,
                    },
                    1,
                ) => vec![
                    (
                        StateVector {
                            v1: 2,
                            v2: 1,
                            v3: 0,
                            v4: 0,
                        },
                        1.0,
                    ),
                    (
                        StateVector {
                            v1: 1,
                            v2: 2,
                            v3: 0,
                            v4: 0,
                        },
                        1.0,
                    ),
                ]
                .into_iter(),
                (
                    StateVector {
                        v1: 2,
                        v2: 1,
                        v3: 0,
                        v4: 0,
                    },
                    1,
                ) => vec![
                    (
                        StateVector {
                            v1: 2,
                            v2: 0,
                            v3: 0,
                            v4: 0,
                        },
                        1.0,
                    ),
                    (
                        StateVector {
                            v1: 1,
                            v2: 1,
                            v3: 0,
                            v4: 0,
                        },
                        1.0,
                    ),
                ]
                .into_iter(),
                (
                    StateVector {
                        v1: 1,
                        v2: 2,
                        v3: 0,
                        v4: 0,
                    },
                    1,
                ) => vec![
                    (
                        StateVector {
                            v1: 1,
                            v2: 1,
                            v3: 0,
                            v4: 0,
                        },
                        1.0,
                    ),
                    (
                        StateVector {
                            v1: 0,
                            v2: 2,
                            v3: 0,
                            v4: 0,
                        },
                        1.0,
                    ),
                ]
                .into_iter(),
                (
                    StateVector {
                        v1: 2,
                        v2: 0,
                        v3: 0,
                        v4: 0,
                    },
                    1,
                ) => vec![(
                    StateVector {
                        v1: 1,
                        v2: 0,
                        v3: 0,
                        v4: 0,
                    },
                    1.0,
                )]
                .into_iter(),
                (
                    StateVector {
                        v1: 1,
                        v2: 1,
                        v3: 0,
                        v4: 0,
                    },
                    1,
                ) => vec![
                    (
                        StateVector {
                            v1: 1,
                            v2: 0,
                            v3: 0,
                            v4: 0,
                        },
                        1.0,
                    ),
                    (
                        StateVector {
                            v1: 0,
                            v2: 1,
                            v3: 0,
                            v4: 0,
                        },
                        1.0,
                    ),
                ]
                .into_iter(),
                (
                    StateVector {
                        v1: 0,
                        v2: 2,
                        v3: 0,
                        v4: 0,
                    },
                    1,
                ) => vec![(
                    StateVector {
                        v1: 0,
                        v2: 1,
                        v3: 0,
                        v4: 0,
                    },
                    1.0,
                )]
                .into_iter(),
                (
                    StateVector {
                        v1: 1,
                        v2: 0,
                        v3: 0,
                        v4: 0,
                    },
                    1,
                ) => vec![(
                    StateVector {
                        v1: 0,
                        v2: 0,
                        v3: 0,
                        v4: 0,
                    },
                    1.0,
                )]
                .into_iter(),
                (
                    StateVector {
                        v1: 0,
                        v2: 1,
                        v3: 0,
                        v4: 0,
                    },
                    1,
                ) => vec![(
                    StateVector {
                        v1: 0,
                        v2: 0,
                        v3: 0,
                        v4: 0,
                    },
                    1.0,
                )]
                .into_iter(),
                _ => vec![].into_iter(),
            }
        }
    }

    impl rusty_planner::planner::SharedStates for PaperExample {
        fn is_public(&self, state: &Self::State) -> bool {
            let mut res: bool = false;
            if state.v1 == 2 && state.v2 == 2 && state.v3 == 0 && state.v4 == 1 {
                res = true
            }
            res
        }

        fn serialize(&self, msg_type: u8, state: &Self::State, para: Vec<f64>) -> String {
            let mut string_list: Vec<String> = vec![
                msg_type.to_string(),
                state.v1.to_string(),
                state.v2.to_string(),
                state.v3.to_string(),
                state.v4.to_string(),
            ];
            for item in para {
                string_list.push(item.to_string());
            }
            string_list.join(";")
        }

        fn deserialize(&self, msg: String) -> (u8, Self::State, Vec<f64>) {
            let split = msg.split(';');
            let data = split.collect::<Vec<&str>>();

            let msg_type = data[0].parse::<u8>().unwrap();
            let state = StateVector {
                v1: data[1].parse::<u8>().unwrap(),
                v2: data[2].parse::<u8>().unwrap(),
                v3: data[3].parse::<u8>().unwrap(),
                v4: data[4].parse::<u8>().unwrap(),
            };
            let mut para: Vec<f64> = vec![];
            if data.len() > 5 {
                para.insert(0, data[5].parse::<f64>().unwrap());
                para.insert(1, data[6].parse::<f64>().unwrap());
            }

            (msg_type, state, para)
        }
    }

    // Setup the multi-agent system-of-systems.
    let agent_1 = rusty_planner::agent::ZeroAgent::new(String::from("tcp://127.0.0.1:8001"));
    let agent_2 = rusty_planner::agent::ZeroAgent::new(String::from("tcp://127.0.0.1:8002"));
    agent_1.add_peer(String::from("tcp://127.0.0.1:8002"));
    agent_1.activate();
    agent_2.activate();

    // Prep the problem space for the agents.
    let ps_1 = PaperExample { agent_id: 1 };
    let ps_2 = PaperExample { agent_id: 2 };
    let start = StateVector {
        v1: 0,
        v2: 0,
        v3: 0,
        v4: 0,
    };
    let goal = StateVector {
        v1: 2,
        v2: 2,
        v3: 2,
        v4: 2,
    };

    let mut ready:bool = false;
    while !ready {
        if agent_1.get_n_peers() == 2 && agent_2.get_n_peers() == 2 {
            ready = true;
        }
        thread::sleep(time::Duration::from_millis(500));
    }

    thread::spawn(move || {
        rusty_planner::mad_astar::solve(&agent_1, &ps_1, start, goal);
        thread::sleep(time::Duration::from_secs(1));
        agent_1.send_msg("tcp://127.0.0.1:8001", &rusty_planner::agent::Msg::Kill());
    });
    let th2 = thread::spawn(move || {
        let res = rusty_planner::mad_astar::solve(&agent_2, &ps_2, start, goal);
        thread::sleep(time::Duration::from_secs(1));
        agent_2.send_msg("tcp://127.0.0.1:8002", &rusty_planner::agent::Msg::Kill());
        res
    });

    // Second Agent should have found goal and initial steps...
    let res2 = th2.join().unwrap();
    assert_eq!(
        vec![
            StateVector {
                v1: 2,
                v2: 2,
                v3: 0,
                v4: 1
            },
            StateVector {
                v1: 2,
                v2: 2,
                v3: 1,
                v4: 1
            },
            StateVector {
                v1: 2,
                v2: 2,
                v3: 2,
                v4: 1
            }
        ],
        res2
    );
}
