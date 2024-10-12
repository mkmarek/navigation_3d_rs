pub fn hungarian(cost: &[&[f32]]) -> Vec<(usize, usize, f32)> {
    let j = cost.len();
    let w = cost[0].len();

    assert!(
        j <= w,
        "The number of rows must be less than or equal to the number of columns"
    );

    let mut job = vec![None; w + 1];
    let mut ys = vec![0.0; j];
    let mut yt = vec![0.0; w + 1];

    for j_curr in 0..j {
        let mut w_curr = w;
        job[w_curr] = Some(j_curr);

        // min reduced cost over edges from Z to worker w
        let mut min_to = vec![f32::INFINITY; w + 1];
        let mut prv = vec![None; w + 1];
        let mut in_z = vec![false; w + 1];

        while let Some(j) = job[w_curr] {
            in_z[w_curr] = true;
            let mut delta = f32::INFINITY;
            let mut w_next = 0;

            for w_ in 0..w {
                if !in_z[w_] {
                    let diff = cost[j][w_] - ys[j] - yt[w_];
                    if diff < min_to[w_] {
                        min_to[w_] = diff;
                        prv[w_] = Some(w_curr);
                    }
                    if min_to[w_] < delta {
                        delta = min_to[w_];
                        w_next = w_;
                    }
                }
            }

            for w_ in 0..=w {
                if in_z[w_] {
                    if let Some(j_) = job[w_] {
                        ys[j_] += delta;
                    } else {
                        panic!("No job found for worker {} when in_z[{}] is true. This shouldn't happen", w_, w_);
                    }
                    yt[w_] -= delta;
                } else {
                    min_to[w_] -= delta;
                }
            }

            w_curr = w_next;
        }

        loop {
            if w_curr == w {
                break;
            }

            let w_ = prv[w_curr].unwrap();
            job[w_curr] = job[w_];

            w_curr = w_;
        }
    }

    let result = job[..j]
        .iter()
        .enumerate()
        .filter_map(|(j, w)| w.as_ref().map(|w| (j, *w, cost[j][*w])))
        .collect();

    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hungarian() {
        let cost = vec![
            vec![8.0, 5.0, 9.0],
            vec![4.0, 2.0, 4.0],
            vec![7.0, 3.0, 8.0],
        ];

        let result = hungarian(&cost.iter().map(|row| row.as_slice()).collect::<Vec<_>>());

        assert_eq!(result, vec![(0, 0, 8.0), (1, 2, 4.0), (2, 1, 3.0)]);
    }
}
