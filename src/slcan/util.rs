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

pub fn pad_left<const N: usize>(a: &[u8]) -> Result<[u8; N], ()> {
    if a.len() > N {
        return Err(());
    }
    let start_idx = N - a.len();

    let mut arr = [b'0'; N];
    arr[start_idx..].copy_from_slice(a);
    Ok(arr)
}
