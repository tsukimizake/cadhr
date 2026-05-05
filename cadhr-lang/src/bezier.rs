pub const DEFAULT_STEPS: usize = 16;

pub fn evaluate_quadratic(
    start: (f64, f64),
    control: (f64, f64),
    end: (f64, f64),
    steps: usize,
) -> Vec<(f64, f64)> {
    (1..=steps)
        .map(|i| {
            let t = i as f64 / steps as f64;
            let u = 1.0 - t;
            let x = u * u * start.0 + 2.0 * u * t * control.0 + t * t * end.0;
            let y = u * u * start.1 + 2.0 * u * t * control.1 + t * t * end.1;
            (x, y)
        })
        .collect()
}

pub fn evaluate_cubic(
    start: (f64, f64),
    cp1: (f64, f64),
    cp2: (f64, f64),
    end: (f64, f64),
    steps: usize,
) -> Vec<(f64, f64)> {
    (1..=steps)
        .map(|i| {
            let t = i as f64 / steps as f64;
            let u = 1.0 - t;
            let x = u * u * u * start.0
                + 3.0 * u * u * t * cp1.0
                + 3.0 * u * t * t * cp2.0
                + t * t * t * end.0;
            let y = u * u * u * start.1
                + 3.0 * u * u * t * cp1.1
                + 3.0 * u * t * t * cp2.1
                + t * t * t * end.1;
            (x, y)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quadratic_endpoints() {
        let pts = evaluate_quadratic((0.0, 0.0), (5.0, 10.0), (10.0, 0.0), 16);
        assert_eq!(pts.len(), 16);
        assert_close!(pts.last().unwrap().0, 10.0);
        assert_close!(pts.last().unwrap().1, 0.0);
    }

    #[test]
    fn test_cubic_endpoints() {
        let pts = evaluate_cubic((0.0, 0.0), (5.0, 10.0), (10.0, 10.0), (10.0, 0.0), 16);
        assert_eq!(pts.len(), 16);
        assert_close!(pts.last().unwrap().0, 10.0);
        assert_close!(pts.last().unwrap().1, 0.0);
    }

    #[test]
    fn test_quadratic_midpoint() {
        // t=0.5: 0.25*start + 0.5*control + 0.25*end
        let pts = evaluate_quadratic((0.0, 0.0), (0.0, 10.0), (10.0, 0.0), 2);
        assert_close!(pts[0].0, 2.5);
        assert_close!(pts[0].1, 5.0);
    }
}
