use std::f64::consts::TAU;

fn probability_density_function_of_gaussian(x: f64, mean: f64, std_dev: f64) -> f64 {
    let a = (std_dev * TAU.sqrt()).recip();
    let b = -(x - mean).powi(2) / (2.0 * std_dev.powi(2));

    a * b.exp()
}

pub fn expectation_maximization_1d(values: &[f64], n_gaussians: usize, max_steps: usize) {
    // Initialization step
    let mut gaussians = Vec::new();
    let min_value = values.iter().cloned().fold(f64::INFINITY, f64::min);
    let max_value = values.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let interval = (max_value - min_value) / n_gaussians as f64;

    for k in 0..n_gaussians {
        let mean = min_value + k as f64 * interval + interval / 2.0; // Mean
        let std_dev = (interval / 2.0).sqrt(); // Standard deviation

        gaussians.push((mean, std_dev));
    }

    let mut prior_estimates = vec![1.0 / n_gaussians as f64; n_gaussians];

    let mut steps = 0;
    loop {
        let initial_gaussians = gaussians.clone();

        // Calculate probabilities of each value belonging to each Gaussian
        let mut probabilities = Vec::new();

        for i in 0..values.len() {
            let value = values[i];

            let mut total = 0.0;
            for k in 0..n_gaussians {
                let (mean, std_dev) = gaussians[k];

                let a = prior_estimates[k]
                    * probability_density_function_of_gaussian(value, mean, std_dev);

                total += a;

                probabilities.push(a);
            }

            for k in 0..n_gaussians {
                probabilities[i * n_gaussians + k] /= total;
            }
        }

        // Update prior estimates
        for k in 0..n_gaussians {
            prior_estimates[k] = 0.0;
            for i in 0..values.len() {
                prior_estimates[k] += probabilities[i * n_gaussians + k] / values.len() as f64;
            }
        }

        // Update means
        for k in 0..n_gaussians {
            let mut mean = 0.0;
            let mut denominator = 0.0;

            for i in 0..values.len() {
                mean += values[i] * probabilities[i * n_gaussians + k];
                denominator += probabilities[i * n_gaussians + k];
            }

            gaussians[k].0 = mean / denominator;
        }

        // Update standard deviations
        for k in 0..n_gaussians {
            let mut std_dev = 0.0;
            let mut denominator = 0.0;

            for i in 0..values.len() {
                std_dev +=
                    (values[i] - gaussians[k].0).powi(2) * probabilities[i * n_gaussians + k];
                denominator += probabilities[i * n_gaussians + k];
            }

            gaussians[k].1 = (std_dev / denominator).sqrt();
        }

        println!(
            "Step: {}, Gaussians: {:?}, Probabilities: {:?}",
            steps, gaussians, prior_estimates
        );

        steps += 1;

        if steps >= max_steps {
            break;
        }

        let gaussian_deltas = gaussians
            .iter()
            .zip(initial_gaussians.iter())
            .map(|((mean, std_dev), (initial_mean, initial_std_dev))| {
                (mean - initial_mean)
                    .abs()
                    .max((std_dev - initial_std_dev).abs())
            })
            .reduce(f64::max)
            .expect("No gaussians");

        if gaussian_deltas < 1e-6 {
            break;
        }
    }
}
