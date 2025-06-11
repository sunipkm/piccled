// build.rs
use std::process::Command;
fn main() {
    let git_hash = Command::new("git")
        .args(["rev-parse", "HEAD"])
        .output()
        .ok()
        .and_then(|s| String::from_utf8(s.stdout).ok())
        .unwrap_or("00000".into());
    println!("cargo:rustc-env=GIT_HASH={}", git_hash);
}
