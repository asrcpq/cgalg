use crate::V2;

// all collisions here does not include contant

pub fn lineseg_collision(a: [V2; 2], b: [V2; 2]) -> bool {
	// u * a01 + a0 = v * b01 + b0
	// a01 u + b10 v = b0 - a0 = ab
	let a01 = a[1] - a[0];
	let b10 = b[0] - b[1];
	let ab = b[0] - a[0];
	let d = a01[0] * b10[1] - a01[1] * b10[0];
	if d == 0.0 {
		return false
	}
	let mut na = ab[0] * b10[1] - b10[0] * ab[1];
	na /= d;
	if na <= 0f32 || na >= 1f32 { return false }
	let mut nb = a01[0] * ab[1] - ab[0] * a01[1];
	nb /= d;
	nb > 0f32 && nb < 1f32
}

pub fn point_in_triangle(p: V2, t: [V2; 3]) -> bool {
	let x = p - t[0];
	let y1 = t[1] - t[0];
	let y2 = t[2] - t[0];
	// y1 u + y2 v = x
	let d = y1[0] * y2[1] - y2[0] * y1[1];
	if d == 0f32 { return false }
	let mut na = x[0] * y2[1] - y2[0] * x[1];
	na /= d;
	if na <= 0f32 { return false }
	let mut nb = y1[0] * x[1] - x[0] * y1[1];
	nb /= d;
	nb > 0f32 && na + nb < 1f32
}

pub fn triangle_collision(t1: [V2; 3], t2: [V2; 3]) -> bool {
	// step1 check 2 sides of t1 collides with 3 sides of t2
	if lineseg_collision([t1[0], t1[1]], [t2[0], t2[1]]) { return true }
	if lineseg_collision([t1[0], t1[1]], [t2[0], t2[2]]) { return true }
	if lineseg_collision([t1[0], t1[1]], [t2[1], t2[2]]) { return true }
	if lineseg_collision([t1[0], t1[2]], [t2[0], t2[1]]) { return true }
	if lineseg_collision([t1[0], t1[2]], [t2[0], t2[2]]) { return true }
	if lineseg_collision([t1[0], t1[2]], [t2[1], t2[2]]) { return true }

	// step2 check inner point
	if point_in_triangle(t2[0], t1) { return true }
	if point_in_triangle(t2[1], t1) { return true }
	if point_in_triangle(t2[2], t1) { return true }
	if point_in_triangle(t1[0], t2) { return true }
	false
}

#[cfg(test)]
mod test {
	use super::*;

	fn point_in_triangle_permutated(p: V2, t: [V2; 3]) -> bool {
		let mut result = None;
		for i in 0..3 {
			let ret = point_in_triangle(
				p,
				core::array::from_fn(|idx| t[(idx + i) % 3]),
			);
			match result {
				None => {
					result = Some(ret);
					continue
				}
				Some(t) => {
					assert_eq!(t, ret);
				}
			}
		}
		result.unwrap()
	}

	#[test]
	fn test_point_in_triangle() {
		let p1 = V2::new(0.5, 0.3);
		let p2 = V2::new(0.0, 0.5);
		let t = [
			V2::new(0.0, 0.0),
			V2::new(1.0, 1.0),
			V2::new(5.0, 0.0),
		];
		assert!(point_in_triangle_permutated(p1, t));
		assert!(!point_in_triangle_permutated(p2, t));

		let p1 = V2::new(2.0, 0.0);
		let p2 = V2::new(0.0, 2.0);
		let p3 = V2::new(1.0, 0.1);
		let t = [
			V2::new(0.0, 0.0),
			V2::new(1.0, 0.0),
			V2::new(5.0, 2.0),
		];
		assert!(!point_in_triangle_permutated(p1, t));
		assert!(!point_in_triangle_permutated(p2, t));
		assert!(point_in_triangle_permutated(p3, t));
	}

	fn triangle_collision_permutated(t1: [V2; 3], t2: [V2; 3]) -> bool {
		let mut result = None;
		for i in 0..3 {
			for j in 0..3 {
				let ret = triangle_collision(
					core::array::from_fn(|idx| t1[(idx + i) % 3]),
					core::array::from_fn(|idx| t2[(idx + j) % 3]),
				);
				match result {
					None => {
						result = Some(ret);
						continue
					}
					Some(t) => {
						assert_eq!(t, ret);
					}
				}
			}
		}
		result.unwrap()
	}

	#[test]
	fn test_triangle_collision() {
		// ◁ ▷
		let t1 = [
			V2::new(-1.0, -1.0),
			V2::new(-1.0, 1.0),
			V2::new(-2.0, 0.0),
		];
		let t2 = [
			V2::new(1.0, -1.0),
			V2::new(1.0, 1.0),
			V2::new(2.0, 0.0),
		];
		assert!(!triangle_collision_permutated(t1, t2));

		// ⧄
		let t1 = [
			V2::new(0.01, 0.0),
			V2::new(1.0, 0.0),
			V2::new(1.0, 0.99),
		];
		let t2 = [
			V2::new(0.0, 0.0),
			V2::new(0.0, 1.0),
			V2::new(1.0, 1.0),
		];
		assert!(!triangle_collision_permutated(t1, t2));


		// regular collision
		let t1 = [
			V2::new(-2.0, 0.0),
			V2::new(0.0, 2.0),
			V2::new(2.0, 0.0),
		];
		let t2 = [
			V2::new(0.0, -1.0),
			V2::new(0.0, 1.0),
			V2::new(1.0, 0.0),
		];
		assert!(triangle_collision_permutated(t1, t2));

		// inside
		let t1 = [
			V2::new(0.0, 0.0),
			V2::new(0.0, 2.0),
			V2::new(2.0, 0.0),
		];
		let t2 = [
			V2::new(0.5, 0.5),
			V2::new(0.5, 1.5),
			V2::new(1.5, 0.5),
		];
		assert!(triangle_collision_permutated(t1, t2));
	}

	#[test]
	fn test_lineseg_collision() {
		// =
		let l1 = [
			V2::new(0.0, 0.0),
			V2::new(0.0, 1.0),
		];
		let l2 = [
			V2::new(1.0, 0.0),
			V2::new(1.0, 1.0),
		];
		assert!(!lineseg_collision(l1, l2));

		// x
		let l1 = [
			V2::new(0.0, 1.0),
			V2::new(1.0, 0.0),
		];
		let l2 = [
			V2::new(0.0, 0.0),
			V2::new(1.0, 1.0),
		];
		assert!(lineseg_collision(l1, l2));

		// λ
		let l1 = [
			V2::new(0.0, 0.0),
			V2::new(0.499, 0.499),
		];
		let l2 = [
			V2::new(0.0, 1.0),
			V2::new(1.0, 0.0),
		];
		assert!(!lineseg_collision(l1, l2));

		// ∠
		let l1 = [
			V2::new(0.001, 0.001),
			V2::new(1.0, 1.0),
		];
		let l2 = [
			V2::new(0.0, 0.0),
			V2::new(1.0, 0.0),
		];
		assert!(!lineseg_collision(l1, l2));
	}
}
