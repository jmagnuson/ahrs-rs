# Generic SIMD support for ahrs

Reviewed 2026-10-02, at `2bd8c8a`, against the scalar baseline `4a4fb76`.

Use one implementation of each algorithm over Simba's SIMD traits and one
unified `Ahrs` interface returning per-lane status for both scalar and SIMD
values. Scalar f32/f64 values are one-lane SIMD values with bool masks. Ship
stable SIMD through Simba's `Wide*` wrappers first.
Portable SIMD can remain an optional nightly backend. No upstream fork is needed.

## Findings

1. **The local nalgebra fix is necessary, but still unpublished in the release
   checked.** The sibling checkout's `1ab7bfc1` changes the normalization validity
   mask from `norm <= min_norm` to `norm > min_norm`. Published nalgebra 0.35.0
   still uses the reversed mask in both `simd_try_normalize` and
   `simd_try_normalize_mut`. The mutable method also selects the wrong lanes to
   normalize. A standalone reproduction against the published crate confirms
   that a nonzero vector gets a false mask and a zero vector gets a true mask
   with NaN coordinates. Removing the patch and upgrading dependencies alone
   causes five of the seven original scalar tests to fail. Two error tests pass
   for the wrong reason, so checking only `is_err()` is insufficient.
2. **`.option()` discards lane information.** Simba's `SimdOption::option()`
   returns `Some` only when every lane is valid. The sensor checks therefore
   reject the whole batch when one sensor lane is zero. This can be an intentional
   atomic batch contract, but it is not independent filter processing.
3. **Madgwick's fallback is incorrect for mixed gradients.** At both corrective
   step checks in `src/madgwick.rs`, one zero-gradient lane makes every lane use
   `update_gyro`. A mixed-lane reproduction confirms that a nonzero-gradient
   lane differs from its independent scalar filter. Splatting the same input
   into every lane, as the current test does, cannot detect this.
4. **Mahony's `N: PartialOrd` excludes the intended SIMD types.** Neither Simba's
   portable SIMD wrapper nor its wide wrappers implement scalar `PartialOrd`.
   Nothing in the algorithm needs that bound. `SimdRealField` supplies lane
   comparisons. The additional `SimdComplexField` bound is redundant.
5. **The current tests cannot run as committed.** Stable `cargo test` fails on the
   unconditional nightly feature and ungated portable SIMD types. Nightly with
   `portable_simd` gets further but fails because `UnitQuaternion` has no
   `simd_relative_eq` method. The sibling approx patch is unused by the locked
   dependency graph. Comparing extracted scalar lanes avoids needing it.
6. **CI enables a nightly feature on stable and beta.** Its `--all-features`
   commands need separate stable and portable SIMD configurations.

## Upstream changes and dependency choice

The [nalgebra changelog](https://github.com/dimforge/nalgebra/blob/v0.35.0/CHANGELOG.md)
lists 0.35.0 (2026-05-24), a move to Simba 0.10, and Rust 1.89 as its minimum.
Version 0.34.2 contains unrelated fixes; it is not evidence that normalization
was repaired. The released 0.35.0 source was inspected directly after download.

The downloaded Simba 0.10.2 changelog records:

- 0.9.1 made `Wide*` array conversion and mapping APIs public and improved mask
  reductions. Stable SIMD was therefore already viable on the old branch.
- 0.10.0 upgraded `wide` to 1.x, added one-lane portable aliases, and adapted to
  recent nightly mask API changes.
- 0.10.1 updated wide comparisons/selection, made math respect `libm_force`, and
  removed a libc call from wide splats.
- 0.10.2 lowered the wide minimum to 1.5 after 1.6 was yanked.

See the [published Simba package](https://docs.rs/crate/simba/0.10.2) and
[SIMD types and traits](https://docs.rs/simba/latest/simba/simd/index.html).
The prototype resolved wide 1.7.1, whose declared minimum is also Rust 1.89.

Recommend nalgebra `0.35`, Simba `0.10.2`, and ordinary approx `0.5`; remove both
local path patches. Update the direct Simba dependency together with nalgebra
to avoid two incompatible generations of SIMD traits. Declare Rust 1.89 and
release this dependency/API change as ahrs 0.10. If retaining nalgebra 0.34 is a
hard compatibility requirement, the same architecture works with Simba 0.9.1;
the upgrade is useful maintenance, not the normalization fix.

Keep backend features separate: `simd = ["simba/wide"]` for stable and no_std
wide types; `portable_simd = ["std", "simba/portable_simd"]` for nightly.
The generic kernels themselves should not require either backend feature.
Use explicit dependency `std` feature forwarding rather than forwarding
`default`, and preserve the existing libm policy initially. Benchmark its
effect before changing it. Raw `wide::f64x4` and `std::simd::Simd` are not the
Simba trait implementations: examples must use the Simba wrappers.

## API and semantics

SIMD lanes represent independent sensor streams/filter instances. Vector3's
three coordinates remain x/y/z; each coordinate contains the same stream's
lane. Consecutive samples of one stream cannot be processed independently
because each sample depends on the previous state.

Change `Ahrs::update` and `update_imu` to return `AhrsUpdate<'_, N>` for
all supported numeric types. Keep `update_gyro` returning `&UnitQuaternion<N>`:
it has no sensor validation failures. There is no separate SIMD trait.

```rust
pub struct AhrsUpdate<'a, N: Scalar + SimdValue> {
    pub quaternion: &'a UnitQuaternion<N>,
    pub accelerometer_norm_zero: N::SimdBool,
    pub magnetometer_norm_zero: N::SimdBool,
}
```

`is_ok()` and `is_err()` return masks of successful and failed lanes. A scalar
caller receives ordinary bool values. Provide `into_result()` only for
`N: SimdValue<SimdBool = bool>`, preserving conventional scalar `?`/`unwrap`
handling after an explicit conversion. This is a deliberate breaking return
change for 0.10. Lane count and mask type come from `N`; no independent const
lane count or `[Error; N::LANES]` array is needed.

For 6DOF updates the magnetometer error mask is always false. For 9DOF,
preserve accelerometer error precedence within each lane: `accel_error = !a_ok`,
`mag_error = a_ok & !m_ok`, `update_ok = a_ok & m_ok`.

Valid lanes advance; failed lanes preserve their quaternion and, for Mahony,
their integral error exactly. A zero Madgwick gradient is a successful gyro-only
update in that lane. Gyro-only updates remain unconditional and preserve
Mahony's integral error. Do not silently substitute IMU processing for a zero
magnetometer; that would change the existing error contract.

Use `N: SimdRealField + Copy, N::Element: RealField` on algorithm kernels. This
combination was compiled for both algorithms with wide values. The element bound
is needed by nalgebra quaternion operations; `Element: Scalar` alone does not
suffice. Keep `Copy` for this primitive SIMD scope; converting all arithmetic to
support arbitrary non-Copy fields is separate work. Use structural bounds for
equality, hash, and accessors instead of unnecessarily requiring real arithmetic.

## Implementation sequence

1. **Restore a reproducible scalar build.** Remove path patches and experimental
   commented bounds/assertions. Upgrade the dependency pair, migrate scalar
   callers through `into_result()`, and gate backend tests. Preserve the equations
   and reference values.
2. **Add a private normalization helper.** Compute `norm`, `valid = norm > 0`,
   and `safe_norm = norm.select(valid, one)`; divide by `safe_norm`, then select
   zero for invalid lanes. Return the value and validity mask. An `SVector<N, D>`
   helper handles accelerometer, magnetometer, and the corrective step.
   This avoids zero division before selection and bypasses nalgebra's broken
   fallible normalization methods. Do not invert upstream masks as a workaround:
   that would break if upstream fixes them. Preserve the zero threshold; do not
   introduce an arbitrary epsilon. Guarantee parity for finite inputs with
   representable norms; a broader NaN/infinity/overflow policy needs a separate
   error/API decision.
3. **Compute and commit candidate state.** Each unified update computes a
   proposed quaternion, proposed integral state where applicable, and error
   masks without mutating the filter during calculation. Commit candidate state
   with `select(update_ok, old)` and return `AhrsUpdate`. Scalar and SIMD calls
   execute the same code, without allocating or extracting lanes in production.
4. **Make Madgwick correction lane-local.** Replace the early whole-batch
   `update_gyro` return with a safely normalized gradient that is zero in
   zero-gradient lanes. The existing quaternion derivative then naturally gives
   gyro-only integration in those lanes. Mask commits on sensor validity.
5. **Mask both Mahony state fields.** Compute the integral candidate, use it in
   feedback, and commit integral error and quaternion with the same validity
   mask. Remove scalar `PartialOrd`. Retain existing integral behavior, including
   accumulation when `ki == 0`; changing that is outside SIMD support.
6. **Document, verify, and benchmark.** Include a four-stream example with
   different inputs and one failure, backend/feature requirements, parameter
   splatting and packing, and status inspection. Bench throughput per stream
   including packing costs against four scalar filters. Start with existing
   matrices; a Matrix4x6/Vector3 rewrite of the padded gradients is an optional
   later optimization supported by measurements.

## Acceptance checks

- Existing scalar reference tests, plus exact error variants, error precedence,
  and unchanged state on error for both filters and both sensor update methods.
- Scalar f32/f64 and wide f32/f64 parity for all three methods over repeated
  updates, distinct starting rotations, sample periods, and gains per lane.
  Mahony tests must include nonzero integral gain and inspect integral state.
- Mixed zero accelerometer/magnetometer lanes, all-invalid batches, signed zero,
  and recovery on later valid samples. Valid lanes match independent scalar
  filters; invalid lanes retain every state field exactly.
- Mixed zero/nonzero gradients in both Madgwick sensor methods. Verify fallback
  and correction in the same batch, finite coordinates, and unit norms.
- Extract lanes and use scalar approx assertions with suitable f32/f64
  tolerances. Compare rotations with quaternion sign equivalence where useful;
  do not require SIMD approximate-equality implementations upstream.
- Stable/beta test scalar, field_access, and wide configurations with explicit
  features. Stable checks no-default-features with and without wide; test on an
  actual no_std target to catch feature-unification surprises from dev dependencies.
- Nightly alone tests portable_simd and all-features. Pin a tested recent nightly
  for reproducibility and periodically test the latest one.
- Run formatting, clippy, doctests, and examples for supported configurations;
  publish only after the dependency graph has no local patches.

## Initial review validation

During the initial review, production code and commits were left intact.
Experiments were made in
`/tmp/ahrs-simd-review` using registry dependencies, not sibling forks:

| Experiment | Result |
| --- | --- |
| Current checkout, stable `cargo test` | Compile failure: nightly feature and ungated portable types |
| Current checkout, nightly + portable_simd | Compile failure: nonexistent `simd_relative_eq` |
| Published nalgebra 0.35.0 normalization reproduction | Confirmed reversed scalar and mixed-lane masks and zero-vector NaNs |
| Current algorithm source + published dependency upgrade + original scalar tests | 2 pass, 5 fail |
| Safe helper + simplified bounds + original scalar tests | All 7 pass |
| Same prototype, wide f64, both filters/all methods, 10 updates per method | Matches 4 independent scalar filters within 1e-12 |
| Same prototype, no-default-features + explicit wide feature | All 7 scalar tests and both backend probes pass on the host |
| Same prototype, mixed Madgwick gradients | Confirmed unwanted whole-batch gyro fallback |
| Simba 0.10.2 portable backend with installed nightly 2025-09-22 | Upstream compile failure: newer mask APIs unavailable on this old compiler |

These initial probes validated the arithmetic bounds and normalization approach.
An upstream normalization repair would be useful, but does not block ahrs support
or require a published fork.

## Implementation and validation

Rebased onto origin/master at `0b04dfe`, which released ahrs 0.9.0 with
nalgebra 0.35/Simba 0.10 and updated rand to 0.10 and Criterion to 0.8.
Keep those upstream dependency changes and its `RngExt` benchmark import.
The breaking unified return type is now prepared for ahrs 0.10.0.
Re-ran `test_all.sh` after the rebase; every check passed with the upstream
rand/Criterion versions. Algorithm source and tests were unchanged by the rebase.

The unified API is implemented in `src/ahrs.rs` and re-exported at the crate root.
Both algorithms use the same generic update methods for scalar and SIMD inputs.
`src/normalize.rs` handles safe normalization; sensor validity selects committed
quaternion/integral state and zero gradients contribute zero correction.
Dependencies now use published crates and the package version is 0.10.0.

`tests/generic.rs` exercises scalar f32/f64, WideF32x4/WideF32x8/WideF64x4,
and portable f64x1/f64x4/f32x4. Each backend checks both filters and all methods,
24 updates per method, distinct lane parameters/rotations, exact error precedence,
all-invalid batches, signed zeros, later recovery, finite unit quaternions,
and mixed zero/nonzero gradients. With field_access, Mahony integral state is
compared to independent scalar filters and checked for exact preservation.
The seven original scalar reference/error tests remain in `tests/unit.rs`.

The README and runnable `examples/simd.rs` show four streams with one failed lane.
Scalar examples and benchmarks use `.into_result()` for conventional Result
handling. CI separates stable SIMD from portable SIMD, checks the minimum Rust
version, and builds without std for thumbv7em-none-eabihf.

Validation commands (using registry dependencies):

```sh
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
```

All listed checks passed locally. Formatting and clippy also passed on Rust
1.89.0. The nightly suite passes 15 integration tests and 5 doctests; the stable
wide suite passes 12 integration tests and 5 doctests. CI is configured to repeat
these checks on its toolchain matrix; no remote CI run was initiated.

The four-stream benchmarks compare four scalar filters with wide filters using
prepacked inputs and scalar inputs packed during each iteration. Short host
measurements (10 samples, 1-second warmup and measurement; default target flags)
before the upstream rebase produced these estimated times per four stream
updates:

| Filter/method | Scalar | Wide prepacked | Wide with packing |
| --- | ---: | ---: | ---: |
| Madgwick update | 327 ns | 261 ns | 274 ns |
| Madgwick update_imu | 196 ns | 164 ns | 177 ns |
| Mahony update | 275 ns | 167 ns | 179 ns |
| Mahony update_imu | 173 ns | 119 ns | 130 ns |

These are local smoke measurements, not portable speed guarantees. Packing costs
are included in the last column. Reproduce with:

```sh
cargo bench --features simd --bench ahrs -- four_streams --sample-size 10 --measurement-time 1 --warm-up-time 1
```

No matrix-equation rewrite was needed for this support. Broader non-finite input
handling and performance tuning remain separate work.
