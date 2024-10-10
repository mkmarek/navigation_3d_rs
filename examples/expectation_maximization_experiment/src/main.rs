mod expectation_maximization_1d;

use expectation_maximization_1d::expectation_maximization_1d;
use plotly::{Bar, Plot};
use rand::Rng;
use rand_distr::{Distribution, Normal};

fn generate_values(mean: f64, std_dev: f64, n: usize) -> Vec<f64> {
    let mut rng = rand::thread_rng();
    let normal = Normal::new(mean, std_dev).unwrap();

    normal.sample_iter(&mut rng).take(n).collect()
}

fn shuffle<T>(values: &mut [T]) {
    let mut rng = rand::thread_rng();

    for i in 0..values.len() {
        let j = rng.gen_range(0..values.len());
        values.swap(i, j);
    }
}

fn main() {
    let mut plot = Plot::new();

    let gauss_1 = generate_values(50.0, 2.0, 200);
    let gauss_2 = generate_values(20.0, 5.0, 200);
    let gauss_4 = generate_values(70.0, 8.0, 500);

    let mut points = gauss_1;
    points.extend(gauss_2);
    points.extend(gauss_4);

    shuffle(&mut points);

    let x = (0..points.len()).collect();
    let mut y = vec![0_i32; points.len()];

    for p in &points {
        let p_i32 = p.round() as usize;
        y[p_i32] += 1;
    }

    expectation_maximization_1d(&points, 3, 200);

    let bar = Bar::new(x, y);
    plot.add_trace(bar);

    //plot.show();
}
