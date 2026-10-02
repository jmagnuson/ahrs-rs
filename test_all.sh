#!/bin/bash
set -euo pipefail

cargo test
cargo test --features simd,field_access
cargo test --release --no-default-features --features simd,field_access
cargo clippy --all-targets --features simd,field_access -- -D warnings
cargo fmt --check
cargo run --example simd --features simd
cargo +nightly-2026-10-01 test --all-features
cargo +nightly-2026-10-01 check --all-targets --all-features
cargo +1.89.0 test --features simd,field_access
cargo +1.89.0 check --lib --no-default-features --target thumbv7em-none-eabihf
cargo +1.89.0 check --lib --no-default-features --features simd --target thumbv7em-none-eabihf
