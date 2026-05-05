#[macro_export]
macro_rules! debug_log {
    ($($arg:tt)*) => {
        eprintln!("[debug] {}", format!($($arg)*))
    };
}