pub fn concat<T: Copy + Default, const A: usize, const B: usize>(
    a: &[T; A],
    b: &[T; B],
) -> [T; A + B] {
    let mut whole: [T; A + B] = [Default::default(); A + B];
    let (one, two) = whole.split_at_mut(A);
    one.copy_from_slice(a);
    two.copy_from_slice(b);
    whole
}
