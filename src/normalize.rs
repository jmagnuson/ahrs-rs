use nalgebra::{SVector, Scalar, SimdRealField, SimdValue};

/// Normalizes each nonzero lane without dividing zero lanes by zero.
///
/// Keep this local: nalgebra's fallible SIMD normalization currently returns
/// an inverted validity mask. Computing validity also avoids relying on its
/// behavior when that upstream bug is fixed.
pub(crate) fn normalize<N, const D: usize>(v: &SVector<N, D>) -> (SVector<N, D>, N::SimdBool)
where
    N: SimdRealField + Copy,
    N::Element: Scalar,
{
    let norm = v.norm();
    let valid = norm.simd_gt(N::zero());
    let denominator = norm.select(valid, N::one());
    let normalized = v.unscale(denominator).select(valid, SVector::zeros());
    (normalized, valid)
}
