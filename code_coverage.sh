#!/usr/bin/env bash

cargo clean
rustup default nightly

export CARGO_INCREMENTAL=0
export RUSTFLAGS="-Zprofile -Ccodegen-units=1 -Copt-level=0 -Clink-dead-code -Coverflow-checks=off -Zpanic_abort_tests -Cpanic=abort"
export RUSTDOCFLAGS="-Cpanic=abort"

cargo build
cargo test

if ! [ -x "$(command -v grcov)" ];
then
  cargo install grcov
fi

grcov ./target/debug/ -s . -t html --llvm --branch --ignore-not-existing -o ./target/debug/coverage/

rustup default stable
